/*The MIT License (MIT)

Copyright (c) 2014 HeXiang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/
#include "ros/ros.h"

#include "xsens_reader/cmdargsreader.h"
#include "xsens_reader/transformation3.hh"
#include "xsens_reader/xsensDriver.h"


#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <string>
#include <sstream>

using std::cout;
using namespace std;

sensor_msgs::Imu xsens_msg;


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "xsens_reader");
// %EndTag(INIT)%
  std::string device;
  int getAccelerationData = 1;
  int getGyroData = 1;
  int getMagneticData = 1;
  int doAMD = 1;
  int Id = 0;
  //char *temp_argv = "/dev/ttyUSB0";
  //argv = &temp_argv;

	if (argc < 2){
		std::cerr << "Usage: " << argv[0] << std::endl;
		std::cerr << "                         -d | -device <pathToDevice>  {path to IMU}" << std::endl;
		std::cerr << "                         -u | -upsidedown {if the imu is mounted upside down} " << std::endl;
		exit(1);
	}

	CmdArgsReader commandline(argc, argv);
	bool upsidedown = false;
	commandline.getStringValue("-d","-device",&device);
	commandline.getFlagValue("-u","-upsidedown",&upsidedown);
        if (commandline.printUnusedArguments()){
		exit(1);
	}
	cout<<"configure done!"<<endl;
	qc_imu_imu_message msg;

	int hasErrors = 0;

	Xsens xsens(device.c_str());
	xsens.init(XSENS_ORIENTATION_QUATERNION_FORMAT, getAccelerationData, getGyroData, getMagneticData);
	xsens.useAMD(doAMD);

	int k =0;


  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%


  ros::Publisher xsens_pub = n.advertise<sensor_msgs::Imu>("xsens_data", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(100);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
/*
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
*/
// %EndTag(FILL_MESSAGE)%
	xsens.fillMessage(msg);


	xsens_msg.orientation.x = msg.q0;
	xsens_msg.orientation.y = msg.q1;
	xsens_msg.orientation.z = msg.q2;
	xsens_msg.orientation.w = msg.q3;

	xsens_msg.angular_velocity.x = msg.gyroX;
	xsens_msg.angular_velocity.y = msg.gyroY;
	xsens_msg.angular_velocity.z = msg.gyroZ;

	xsens_msg.linear_acceleration.x = msg.accX;
	xsens_msg.linear_acceleration.y = msg.accY;
	xsens_msg.linear_acceleration.z = msg.accZ;
// %Tag(ROSCONSOLE)%
//    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

	stringstream ss;
	ss <<  Id++;
	xsens_msg.header.frame_id = ss.str();

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%

    xsens_pub.publish(xsens_msg);
// %EndTag(PUBLISH)%
    if(!(Id % 100)) cerr<<".";
// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

