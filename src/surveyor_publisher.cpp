/*
Copyright (c) 2010 Rene Ladan. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <ros/time.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <opencv/cv.h>
#include <opencv/highgui.h> /* cvDecodeImageM() */

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <string.h> /* memcpy(), memset(), strerror() */
#include <unistd.h> /* close() */

extern "C" {
#include <errno.h>
}

#define BUFSIZE 1024
#define COUNT 30

sensor_msgs::CameraInfo cam[2];

/*
 * Data structure to represent the master socket.
 */
struct master_socket
{
	int socket; /* handle */
	struct sockaddr_in address; /* address/port info */
};

///////////////////////// hardware-specific stuff //////////////////////////////

int open_TCP_client(std::string address, int port)
{
	struct sockaddr_in client_address;
	int client_socket_desc;

	client_socket_desc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (client_socket_desc == -1)
	{
		ROS_ERROR("open_TCP_client: socket: %s", strerror(errno));
		return -1;
	}

	memset(&client_address, 0, sizeof(struct sockaddr_in));
	client_address.sin_family = AF_INET;
	client_address.sin_port = htons(port);
	client_address.sin_addr.s_addr = inet_addr(address.c_str());
	if (connect(client_socket_desc, (struct sockaddr *)&client_address, sizeof(struct sockaddr_in)) == -1)
	{
		close(client_socket_desc);
		ROS_ERROR("open_TCP_client: connect: %s", strerror(errno));
		return -1;
	}

	return client_socket_desc;
}

int receive_jpeg_frame(int socket, char *buffer)
{
	/*
	 * recv() always returns chunks of 1024 bytes or less, so the JPEG picture
	 * needs to be reassembled.
	 */

	bool imageReady = false;
	int len = 0;
	int res;
	char buf[BUFSIZE];

	while (!imageReady)
	{
		res = recv(socket, buf, BUFSIZE, 0);
		if (res < 1)
		{
			ROS_WARN("receive_jpeg_frame: recv: %s", strerror(errno));
			return -1;
		}

		memcpy(buffer + len, buf, res);
		len += res;
		if (buf[res-2] == -1 && buf[res-1] == -39)
			imageReady = true; /* magic marker 0xffd9 found */
	}

	return len;
}

int get_surveyor_frame(int socket, char *buffer)
{
	char message[2] = {'I', 0};
	char sig[6];
	int len = -1;
	struct timeval tv;

	// send 'I' command (grab JPEG frame) until a frame is returned
	tv.tv_sec = 1; // maybe make configurable? 0.5s seems too short
	tv.tv_usec = 0;
	if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)))
	{
		ROS_WARN("get_surveyor_frame: setsockopt 1s: %s", strerror(errno));
		return -1;
	}

	while (len == -1)
	{
		if (send(socket, message, strlen(message), 0) != strlen(message))
		{
			ROS_WARN("get_surveyor_frame: send: %s", strerror(errno));
			return -1;
		}

		len = receive_jpeg_frame(socket, buffer);
	}

	// remove timeout from socket
	tv.tv_sec = tv.tv_usec = 0;
	if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)))
	{
		ROS_WARN("get_surveyor_frame: setsockopt rm: %s", strerror(errno));
		return -1;
	}

	len -= 10; // strip length of Surveyor header

	// check validity of captured frames
	strcpy(sig, "##IMJ");
	if (strncmp(sig, buffer, 5))
	{
		ROS_WARN("corrupt image header, expected %s", sig);
		return -1;
	}
	return len;
}

int set_surveyor_resolution(int socket, int format)
{
	char message[2] = {0, 0};
	char buf[BUFSIZE];

	switch (format) {
	case 3:
		message[0] = 'a';
		break;
	case 5:
		message[0] = 'b';
		break;
	case 7:
		message[0] = 'c';
		break;
	case 9:
		message[0] = 'd';
		break;
	}
	if (send(socket, message, strlen(message), 0) != strlen(message))
	{
		ROS_WARN("set_surveyor_resolution: send: %s", strerror(errno));
		return -1;
	}

	// read back answer
	if (recv(socket, buf, BUFSIZE, 0) < 1)
	{
		ROS_WARN("set_surveyor_resolution: recv: %s", strerror(errno));
		return -1;
	}
	if (buf[0] != '#' || buf[1] != message[0])
	{
		ROS_WARN("set_surveyor_resolution: data mismatch, expected #%c", message[0]);
		return -1;
	}

	return format;
}

int set_surveyor_quality(int socket, int quality)
{
	char message[3] = {'q', 0, 0};
	char buf[BUFSIZE];

	message[1] = quality + '0';
	if (send(socket, message, strlen(message), 0) != strlen(message))
	{
		ROS_WARN("set_surveyor_quality: send: %s", strerror(errno));
		return -1;
	}

	// read back answer
	if (recv(socket, buf, BUFSIZE, 0) < 1)
	{
		ROS_WARN("set_surveyor_quality: recv: %s", strerror(errno));
		return -1;
	}
	if (strncmp("##quality - ", buf, 12) || buf[12] != quality + '0') // the protocol description is wrong!
	{
		ROS_WARN("set_surveyor_quality: data mismatch, expected ##quality - %c", quality + '0');
		return -1;
	}

	return quality;
}

int set_automask(int socket, int automask)
{
	char message[4] = {'v', 'a', 0, 0};
	char buf[BUFSIZE];

	message[2] = automask + '0';
	if (send(socket, message, strlen(message), 0) != strlen(message))
	{
		ROS_WARN("set_automask: send: %s", strerror(errno));
		return -1;
	}

	// read back answer
	if (recv(socket, buf, BUFSIZE, 0) < 1)
	{
		ROS_WARN("set_automask: recv: %s", strerror(errno));
		return -1;
	}
	if (strncmp("##va", buf, 4) || buf[4] != automask + '0')
	{
		ROS_WARN("set_automask: data mismatch, expected ##va%c", automask + '0');
		return -1;
	}
	return automask;
}

/////////////////////////// service callbacks /////////////////////////////////

bool set_caminfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
	ROS_INFO("camera_info called, NOP");
	res.success = true;
	res.status_message = "nop";
	return res.success;
}

bool set_caminfo_0(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
	ROS_INFO("left camera_info called");
	memcpy(&(cam[0]), (sensor_msgs::CameraInfo*)&(req.camera_info), sizeof(sensor_msgs::CameraInfo));
	res.success = true;
	res.status_message = "stored";
	return res.success;
}

bool set_caminfo_1(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
	ROS_INFO("right camera_info callled");
	memcpy(&(cam[1]), (sensor_msgs::CameraInfo*)&(req.camera_info), sizeof(sensor_msgs::CameraInfo));
	res.success = true;
	res.status_message = "stored";
	return res.success;
}

/////////////////////// main(), like every C/C++ program ///////////////////////

int main(int argc, char *argv[])
{
	int i;
	int numcams;
	int format;
	int quality;
	int automask; // 0 = off, 1 = agc, 2 = awb, 4 = aec, 7 = on
	int srv_socket_desc[2];
	long length[2];
	char imageBuf[2][240*BUFSIZE];
	int port[2];
	std::string ip;
	bool sendframes;

	ros::init(argc, argv, "surveyor_publisher");
	ros::NodeHandle nh("~");

	nh.param("numcams", numcams, 2);
	if (numcams < 1 || numcams > 2)
	{
		ROS_ERROR("Invalid number of cameras chosen (%i), must be 1 or 2", numcams);
		return 1;
	}
	nh.param("format", format, 5);
	if (format != 3 && format != 5 && format != 7 && format != 9)
	{
		ROS_ERROR("Invalid resolution chosen (%i), must be one of 3, 5, 7, or 9", format);
		return 1;
	}
	nh.param("quality", quality, 3); // 8 (low) .. 1 (high)
	if (quality < 1 || quality > 8)
	{
		ROS_ERROR("Invalid quality chosen (%i), must be 1-8", quality);
		return 1;
	}
	nh.param("auto", automask, 7);
	if (automask < 0 || automask > 7)
	{
		ROS_ERROR("Invalid autosettings chosen (%i), must be 0-7", automask);
		return 1;
	}
	nh.param("ip", ip, std::string("192.168.2.111"));
	nh.param("lport", port[0], 10001);
	nh.param("rport", port[1], 10002);
	ROS_INFO("numcams=%i format=%i quality=%i ip=%s lport=%i rport=%i", numcams, format, quality, ip.c_str(), port[0], port[1]);
	for (i = 0; i < numcams; i++)
	{
		srv_socket_desc[i] = open_TCP_client(ip, port[i]);
		if (srv_socket_desc[i] == -1)
		{
			ROS_ERROR("No connection to srv #%i", i);
			return 1;
		}
		ROS_INFO("Connection to srv #%i established", i);
		if (set_surveyor_resolution(srv_socket_desc[i], format) == -1)
		{
			ROS_WARN("Resolution failed for srv #%i", i);
			for (i = 0; i < numcams; i++)
				close(srv_socket_desc[i]);
			return 1;
		}
		ROS_INFO("Resolution of srv #%i set", i);
		if (set_surveyor_quality(srv_socket_desc[i], quality) == -1)
		{
			ROS_WARN("Quality failed for srv #%i", i);
			for (i = 0; i < numcams; i++)
				close(srv_socket_desc[i]);
			return 1;
		}
		ROS_INFO("Quality of srv #%i set", i);
		if (set_automask(srv_socket_desc[i], automask) == -1)
		{
			ROS_WARN("Autosettings failed for srv #%i", i);
			for (i = 0; i < numcams; i++)
				close(srv_socket_desc[i]);
			return 1;
		}
		ROS_INFO("Autosettings of srv #%i set", i);
	}
	ROS_WARN("Camera not yet calibrated, call the set_camera_info services");

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub[2];
	ros::Publisher pubci[2];
	pub[0] = it.advertise("surveyor/left/image_raw", 1);
	pubci[0] = nh.advertise<sensor_msgs::CameraInfo>("surveyor/left/camera_info", 1);
	ros::ServiceServer sci_service[2];
	ros::ServiceServer sci_dummy;
	sci_dummy = nh.advertiseService("surveyor/set_camera_info", set_caminfo);
	sci_service[0] = nh.advertiseService("surveyor/left/set_camera_info", set_caminfo_0);
	if (numcams == 2)
	{
		pub[1] = it.advertise("surveyor/right/image_raw", 1);
		pubci[1] = nh.advertise<sensor_msgs::CameraInfo>("surveyor/right/camera_info", 1);
		sci_service[1] = nh.advertiseService("surveyor/right/set_camera_info", set_caminfo_1);
	}

	ROS_INFO("Service(s) set up");

	ros::Time t_prev(ros::Time::now());
	int count = 0;
	long unsigned numBytes = 0;

	while (nh.ok()) {
		ros::spinOnce(); // listen for service requests

		if (count++ % COUNT == 0) {
			ros::Time t(ros::Time::now());
			ros::Duration d(t - t_prev);
			ROS_INFO("%.1f fps, average %.0f bytes/frame", float(COUNT)/d.toSec(), float(numBytes)/float(numcams)/COUNT);
			t_prev = t;
			numBytes = 0;
		}
		// first get the picture(s) from the surveyor ...
		sendframes = true;

		for (i = 0; sendframes && i < numcams; i++)
		{
			length[i] = get_surveyor_frame(srv_socket_desc[i], imageBuf[i]);
			if (length[i] == -1) {
				sendframes = false;
				ROS_WARN("Getting frame failed for #%i", i);
			}
		}

		// ... then publish the picture(s) for better sync
		ros::Time pt(ros::Time::now());
		for (i = 0; sendframes && i < numcams; i++)
		{
			cam[i].header.stamp = pt;
			cam[i].header.seq = count;
			cam[i].header.frame_id = "1"; // 0 = no frame, 1 = global
			switch (format) {
			case 3:
				cam[i].width = 160;
				cam[i].height = 120;
				break;
			case 5:
				cam[i].width = 320;
				cam[i].height = 240;
				break;
			case 7:
				cam[i].width = 640;
				cam[i].height = 480;
				break;
			case 9:
				cam[i].width = 1280;
				cam[i].height = 1024;
				break;
			}
			// initially the entire image is a ROI
			cam[i].roi.x_offset = 0;
			cam[i].roi.y_offset = 0;
			cam[i].roi.height = cam[i].height;
			cam[i].roi.width = cam[i].width;
			memcpy(&(cam[i]), (sensor_msgs::CameraInfo*)&(cam[i]), sizeof(sensor_msgs::CameraInfo));
			numBytes += length[i];
			// publish camera info
			pubci[i].publish(cam[i]);
			// publish the picture itself
			sensor_msgs::Image msg;
			msg.header = cam[i].header;
			msg.width = cam[i].width;
			msg.height = cam[i].height;
			msg.encoding = "bgr8";
			msg.is_bigendian = 0;
			msg.step = msg.width*3;
			CvMat *jpeg = cvCreateMatHeader(1, length[i], CV_8UC1);
			cvSetData(jpeg, imageBuf[i]+10, length[i]);
			CvMat *bitmap = cvDecodeImageM(jpeg, CV_LOAD_IMAGE_COLOR);
			msg.data.resize(msg.height*msg.step);
			memcpy((char*)(&msg.data[0]), (char*)bitmap->data.ptr, msg.height*msg.step);
			pub[i].publish(msg);
			cvReleaseMat(&bitmap);
			cvReleaseMat(&jpeg);
		}
	}

	for (i = 0; i < numcams; i++)
		close(srv_socket_desc[i]);
	return 0;
}
