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
#include <geometry_msgs/Twist.h>
#include <opencv/cv.h>
#include <opencv/highgui.h> /* cvDecodeImageM() */

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <math.h> /* round(), fabs() */
#include <string.h> /* memcpy(), memset(), strerror() */
#include <unistd.h> /* close() */

extern "C" {
#include <errno.h>
}

#define BUFSIZE 1024 /* Max. message length from the surveyor, do not modify */
#define COUNT 30
#define SERVO_TIMEOUT 0 /* 1 = 10 ms, timeout for servo velocity (M command), max = 255, 0 = infinite */
#define VMAX (1.0) /* Max. linear velocity of one wheel (m/s), i.e. velocity with command 'M',0x7F,0x7F */
#define WHEEL_DISTANCE (0.088) /* (m) */

sensor_msgs::CameraInfo g_cam[2];
geometry_msgs::Twist g_twist;

// ROS parameters
/* Socket timeout (s) */
double g_socket_timeout;
/* Servo velocity correction, if greater than 1, vl will be greater, so that the robot will turn right */
double g_vl_correction;

///////////////////////// hardware-specific stuff //////////////////////////////

int flush_socket(int socket)
{
	struct timeval tv;

	// Set a timeout, so that we don't block if there is nothing.
	tv.tv_sec = 0;
	tv.tv_usec = 100000;
	if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)))
	{
		ROS_WARN("flush_socket: setsockopt 100ms: %s", strerror(errno));
		return -1;
	}

	int res;
	char buf[BUFSIZE];
	do
	{
		res = recv(socket, buf, BUFSIZE, 0);
	} while(res > 0);

	// remove timeout from socket
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)))
	{
		ROS_WARN("flush_socket: setsockopt no_timeout: %s", strerror(errno));
		return -1;
	}

	return 0;
}

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

	if (flush_socket(client_socket_desc) == -1)
	{
		return -1;
	}
	return client_socket_desc;
}

/* Read a frame and returns it, if any, otherwise return -1
 */
int receive_jpeg_frame(int socket, char *buffer)
{
	/*
	 * If the camera is ready, recv() returns chunks of 1024 bytes or less, so
	 * the JPEG picture
	 * needs to be reassembled.
	 */

	bool imageReady = false;
	int len = 0;
	int res;
	char buf[BUFSIZE];


	// Set a timeout, so that we don't block if there is no frame ready.
	struct timeval tv;
	tv.tv_sec = (__time_t)g_socket_timeout;
	tv.tv_usec = (__suseconds_t)((g_socket_timeout - tv.tv_sec) * 1e6);
	if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)))
	{
		ROS_WARN("get_surveyor_frame: error on setsockopt SO_RCVTIMEO (set timeout): %s", strerror(errno));
		return -1;
	}

	while (!imageReady)
	{
		res = recv(socket, buf, BUFSIZE, 0);
		if (res < 1)
		{
			return -1;
		}

		memcpy(buffer + len, buf, res);
		len += res;
		if (buf[res-2] == -1 && buf[res-1] == -39)
			imageReady = true; /* magic marker 0xffd9 found */
	}

	// remove timeout from socket
	tv.tv_sec = tv.tv_usec = 0;
	if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)))
	{
		ROS_WARN("get_surveyor_frame: error on setsockopt SO_RCVTIMEO (reset timeout): %s", strerror(errno));
		return -1;
	}

	return len;
}

int get_surveyor_frame(int socket, char *buffer)
{
	char message[2] = {'I', 0};
	char sig[6];
	int len = -1;
	// send one 'I' command (grab JPEG frame).
	if (send(socket, message, strlen(message), 0) != strlen(message))
	{
		ROS_WARN("get_surveyor_frame: error on send: %s", strerror(errno));
		return -1;
	}

	len = receive_jpeg_frame(socket, buffer);

	if (len != -1)
	{
		len -= 10; // strip length of Surveyor header

		// check validity of captured frames
		strcpy(sig, "##IMJ");
		if (strncmp(sig, buffer, 5))
		{
			ROS_WARN("corrupt image header, expected %s", sig);
			flush_socket(socket);
			return -1;
		}
	}
	return len;
}

/* Limit and convert a real-world velocity to a 8-bit binary
 *
 * Limit and convert a real-world velocity to its 8-bit binary representation
 * for the 'M' command.
 */
unsigned char convert_v(double v)
{
	if (v > VMAX)
	{
		v = VMAX;
	}
	else if (v < -VMAX)
	{
		v = -VMAX;
	}

	return (unsigned char)((round((v + VMAX) / 2.0 / VMAX * 0xFF)) + 0x80);
}

void compute_servo(const geometry_msgs::Twist& twist, unsigned char& vleft, unsigned char& vright)
{
	double vl = twist.linear.x - twist.angular.z * WHEEL_DISTANCE / 2.0;
	double vr = twist.linear.x + twist.angular.z * WHEEL_DISTANCE / 2.0;

	// Apply the direction correction.
	vl *= g_vl_correction;

	ROS_DEBUG("v = %.3f, w = %.3f", twist.linear.x, twist.angular.z);
	ROS_DEBUG("vl = %.3f, vr = %.3f", vl, vr);
	const double throttle_factor = fmax(fabs(vr) / VMAX, fabs(vl) / VMAX);

	if (throttle_factor > 1)
	{
		vl /= throttle_factor;
		vr /= throttle_factor;
	}

	ROS_DEBUG("vl_throttle = %.3f, vr_throttle = %.3f", vl, vr);

	vleft = convert_v(vl);
	vright = convert_v(vr);

	ROS_DEBUG("vleft = 0x%02x, vright = 0x%02x", vleft, vright);
}

bool set_servo(int socket, unsigned char vleft, unsigned char vright)
{
	unsigned char message[] = {'M', 0, 0, 0, 0};
	message[1] = vleft;
	message[2] = vright;
	message[3] = (unsigned char) SERVO_TIMEOUT;

	if (send(socket, message, 4, 0) != 4)
	{
		ROS_WARN("set_servo: error on send: %s", strerror(errno));
		return false;
	}

	// read back answer
	char buf[BUFSIZE];
	if (recv(socket, buf, BUFSIZE, 0) < 1)
	{
		ROS_WARN("set_servo: recv: %s", strerror(errno));
		return false;
	}
	if (buf[0] != '#' || buf[1] != message[0])
	{
		ROS_WARN("set_servo: data mismatch, expected #%c, got 0x%0x%0x", message[0], buf[0], buf[1]);
		return false;
	}
	return true;
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

/////////////////////////// Message callbacks /////////////////////////////////

void twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
	g_twist = *twist;
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
	memcpy(&(g_cam[0]), (sensor_msgs::CameraInfo*)&(req.camera_info), sizeof(sensor_msgs::CameraInfo));
	res.success = true;
	res.status_message = "stored";
	return res.success;
}

bool set_caminfo_1(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
	ROS_INFO("right camera_info called");
	memcpy(&(g_cam[1]), (sensor_msgs::CameraInfo*)&(req.camera_info), sizeof(sensor_msgs::CameraInfo));
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
	bool received_image[] = {false, true};

	ros::init(argc, argv, "surveyor");
	ros::NodeHandle nh("~");

	nh.param("numcams", numcams, 1);
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
	nh.param("ip", ip, std::string("169.254.0.10"));
	nh.param("lport", port[0], 10001);
	nh.param("rport", port[1], 10002);
	nh.param("socket_timeout", g_socket_timeout, 0.2);
	nh.param("vl_correction", g_vl_correction, 1.0);
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
	image_transport::Publisher image_publisher[2];
	ros::Publisher info_publisher[2];
	ros::Subscriber twist_subscriber = nh.subscribe("cmd_vel", 1, twist_callback);
	image_publisher[0] = it.advertise("surveyor/left/image_raw", 1);
	info_publisher[0] = nh.advertise<sensor_msgs::CameraInfo>("surveyor/left/camera_info", 1);
	ros::ServiceServer sci_service[2];
	ros::ServiceServer sci_dummy;
	sci_dummy = nh.advertiseService("surveyor/set_camera_info", set_caminfo);
	sci_service[0] = nh.advertiseService("surveyor/left/set_camera_info", set_caminfo_0);
	if (numcams == 2)
	{
		received_image[1] = false;
		image_publisher[1] = it.advertise("surveyor/right/image_raw", 1);
		info_publisher[1] = nh.advertise<sensor_msgs::CameraInfo>("surveyor/right/camera_info", 1);
		sci_service[1] = nh.advertiseService("surveyor/right/set_camera_info", set_caminfo_1);
	}

	ROS_INFO("Service(s) set up");

	ros::Time t_prev(ros::Time::now());
	int count = 0;
	long unsigned numBytes = 0;

	ros::Rate main_loop(100);
	while (nh.ok())
	{
		ros::spinOnce(); // listen for service requests

		nh.getParamCached("socket_timeout", g_socket_timeout);
		nh.getParamCached("vl_correction", g_vl_correction);

		if (count++ % COUNT == 0)
		{
			ros::Time t(ros::Time::now());
			ros::Duration d(t - t_prev);
			ROS_INFO("%.1f fps, average %.0f bytes/frame", float(COUNT)/d.toSec(), float(numBytes)/float(numcams)/COUNT);
			t_prev = t;
			numBytes = 0;
		}

		// Try once to get the picture(s) from the surveyor ...
		for (i = 0; i < numcams; i++)
		{
			length[i] = get_surveyor_frame(srv_socket_desc[i], imageBuf[i]);
			if (length[i] != -1)
			{
				received_image[i] = true;
			}
		}

		// ... then publish the picture(s) for better sync
		if (received_image[0] && received_image[1])
		{
			ros::Time pt(ros::Time::now());
			for (i = 0; i < numcams; i++)
			{
				g_cam[i].header.stamp = pt;
				g_cam[i].header.seq = count;
				g_cam[i].header.frame_id = "1"; // 0 = no frame, 1 = global
				switch (format)
				{
					case 3:
						g_cam[i].width = 160;
						g_cam[i].height = 120;
						break;
					case 5:
						g_cam[i].width = 320;
						g_cam[i].height = 240;
						break;
					case 7:
						g_cam[i].width = 640;
						g_cam[i].height = 480;
						break;
					case 9:
						g_cam[i].width = 1280;
						g_cam[i].height = 1024;
						break;
				}
				// initially the entire image is a ROI
				g_cam[i].roi.x_offset = 0;
				g_cam[i].roi.y_offset = 0;
				g_cam[i].roi.height = g_cam[i].height;
				g_cam[i].roi.width = g_cam[i].width;
				memcpy(&(g_cam[i]), (sensor_msgs::CameraInfo*)&(g_cam[i]), sizeof(sensor_msgs::CameraInfo));
				numBytes += length[i];
				// publish camera info
				info_publisher[i].publish(g_cam[i]);
				// publish the picture itself
				sensor_msgs::Image msg;
				msg.header = g_cam[i].header;
				msg.width = g_cam[i].width;
				msg.height = g_cam[i].height;
				msg.encoding = "bgr8";
				msg.is_bigendian = 0;
				msg.step = msg.width*3;
				CvMat *jpeg = cvCreateMatHeader(1, length[i], CV_8UC1);
				cvSetData(jpeg, imageBuf[i]+10, length[i]);
				CvMat *bitmap = cvDecodeImageM(jpeg, CV_LOAD_IMAGE_COLOR);
				msg.data.resize(msg.height*msg.step);
				memcpy((char*)(&msg.data[0]), (char*)bitmap->data.ptr, msg.height*msg.step);
				image_publisher[i].publish(msg);
				cvReleaseMat(&bitmap);
				cvReleaseMat(&jpeg);
				received_image[i] = false;
			}
		}

		// Compute and send servo velocities.
		unsigned char vleft;
		unsigned char vright;
		compute_servo(g_twist, vleft, vright);
		set_servo(srv_socket_desc[0], vleft, vright);

		main_loop.sleep();
	}

	for (i = 0; i < numcams; i++)
		close(srv_socket_desc[i]);
	return 0;
}
