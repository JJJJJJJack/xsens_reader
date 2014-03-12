## The MIT License (MIT)

## Copyright (c) 2014 HeXiang

## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:

## The above copyright notice and this permission notice shall be included in all
## copies or substantial portions of the Software.

## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
## SOFTWARE.
cmake_minimum_required(VERSION 2.8.3)
project(xsens_reader)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_executable(xsens_reader 
                  src/xsens_reader.cpp 
                  src/xsens_reader/cmdargsreader.cpp
                  src/xsens_reader/MTComm.cpp
                  src/xsens_reader/timeval_ops.c
                  src/xsens_reader/xsensDriver.cpp)
 target_link_libraries(xsens_reader ${catkin_LIBRARIES})

