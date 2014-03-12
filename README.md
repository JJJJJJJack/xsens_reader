xsens_reader
============

Node based on ROS(hydro) reading MTi data. Tested on Ubuntu 12.04LTS with the 3rd production of xsens MTi.


Preparation
------------
1.Install ROS(hydro).

2.Follow the instruction bellow.

Installation
------------
Caution: Only for hydro user
         I'm quite sorry that I'm still working on xsens_reader package compatible with other version of ROS.
Open a terminal:

        $ sudo cd ~/catkin_ws/src
        $ git clone https://github.com/JJJJJJJack/xsens_reader.git
        $ cd ..
        $ catkin_make
        $ catkin_make install

Run
------------
Open a terminal:

         $ sudo su
         $ cd ~/catkin_ws
         $ source devel/setup.bash
         $ roscore
         
Open another terminal:

         $ sudo su
         $ cd ~/catkin_ws
         $ source devel
         $ rosrun xsens_reader xsens_reader -d /dev/ttyUSB0

Note
------------
Try USB1 USB2 if you can't connect to the port.

xsens_reader publishes the sensor_msgs::Imu message of "/xsens_data".

$ rostopic echo /xsens_data  #to show the message of MTi.
