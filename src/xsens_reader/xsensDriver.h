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
#ifndef XSENS_H
#define XSENS_H


#include <string>
#include "timeval_ops.h"
#include "qc_imu_messages.h"
//#include <ipcInterfaces/qc_imu_interface.h>
#include "MTComm.h"

enum XSENS_ORIENTATION_FORMAT {
   XSENS_NO_ORIENTATION_DATA,
   XSENS_ORIENTATION_QUATERNION_FORMAT,
   XSENS_ORIENTATION_EULER_FORMAT,
   XSENS_ORIENTATION_MATRIX_FORMAT,
};

class Xsens {
   //-----STATIC-----
   protected: static const char* className;

   public:
      //-----CONSTRUCTOR&DESTRUCTOR-----
      Xsens(const char* device="/dev/ttyUSB0"); // testMode=true: No real xsens-device neccessary. Sends just zeros.
      ~Xsens();
      //-----METHODS-----
      void init(XSENS_ORIENTATION_FORMAT orientationFormat=XSENS_ORIENTATION_QUATERNION_FORMAT, bool getAccelerationData=false, bool getGyroData=false, bool getMagneticData=false);
      void useAMD(bool active=true); // Use or don't use AMD (Adapt to Magnetic Disturbances filter).

      ////////FIXME//////////
      void fillMessage(qc_imu_imu_message& msg, bool updateData = true);

      bool hadReadError() {return _readError;}
   
   protected:
      //-----METHODS-----
      void getNewData();

      //-----VARIABLES-----
      std::string _device;
      bool _readError;
      CMTComm mtcomm;
      bool _doAMD;
      XSENS_ORIENTATION_FORMAT _orientationFormat;
      bool _getAccelerationData, _getGyroData, _getMagneticData;
      int _outputFormat, _outputMode;
      float fdata[18];
      short datalen;
      unsigned char data[MAXMSGLEN];

			struct timeval lastDataTime;
			struct timeval startTime;
      //double lastDataTime;
      //double startTime;
      
      unsigned short samplecounter;
      int _lastSampleCounter;
      
};

#endif
