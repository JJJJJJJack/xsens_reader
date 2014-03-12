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
#ifndef _QC_IMU_MESSAGES_H
#define _QC_IMU_MESSAGES_H


struct qc_imu_imu_message {
	float accX;    ///< acceleration in x direction
	float accY;    ///< acceleration in y direction
	float accZ;    ///< acceleration in z direction
	float q0;      ///< Quaternion describing the rotation, w elem
	float q1;      ///< Quaternion describing the rotation, x elem
	float q2;      ///< Quaternion describing the rotation, y elem
	float q3;      ///< Quaternion describing the rotation, z elem
	float magX;    ///< magnetic field vector
	float magY;    ///< magnetic field vector
	float magZ;    ///< magnetic field vector
	float gyroX;   ///< gyro rotvel in radians/s
	float gyroY;   ///< gyro rotvel in radians/s
	float gyroZ;   ///< gyro rotvel in radians/s
	long timestamp_sec;
	long timestamp_usec;
};

#define QC_IMU_IMU_MESSAGE_NAME "qc_imu_imu_message"
#define QC_IMU_IMU_MESSAGE_FMT  "{float, float, float, float, float, float, float, float, float, float, float, float, float, long, long}"
#define QC_IMU_IMU_MESSAGE_TYPE IPC_VARIABLE_LENGTH


#endif // _QC_IMU_MESSAGES_H
