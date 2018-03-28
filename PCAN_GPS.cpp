/***************************************************************************
  This is a library for the Peak PCAN GPS pressure sensor

  The sensor uses CAN to communicate.

 ***************************************************************************/
#include "Arduino.h"
#include "FlexCAN.h"
#include "PCAN_GPS.h"


/***************************************************************************
 Public FUNCTIONS
 ***************************************************************************/


PCAN_GPS::PCAN_GPS()
  : _enable(-1), _bus(0), _txAlt(0), _rxAlt(0)
{ }

PCAN_GPS::PCAN_GPS(int8_t enablePin)
  : _enable(enablePin),_bus(0), _txAlt(0), _rxAlt(0)
{ }

PCAN_GPS::PCAN_GPS(int8_t enablePin, int8_t bus)
  : _enable(enablePin), _bus(bus),_txAlt(0), _rxAlt(0)
{ }

PCAN_GPS::PCAN_GPS(int8_t enablePin, int8_t bus, int8_t txAlt, int8_t rxAlt)
  : _enable(enablePin), _bus(bus), _txAlt(txAlt), _rxAlt(rxAlt)
{ }


bool PCAN_GPS::begin(uint32_t baud) {
  static struct CAN_filter_t defaultMask;

  if (_bus == 0)
  {
    Can0.begin(baud, defaultMask, _txAlt, _rxAlt);
  }
  else if (_bus==1)
  {
    Can1.begin(baud, defaultMask, _txAlt, _rxAlt);
  }
  else { return false; }

  if (_enable) {
    pinMode(_enable, OUTPUT);
    digitalWrite(_enable, HIGH);
  }

  if (_bus == 0)
  {
    Can0.attachObj(&canClass);
    canClass.attachGeneralHandler();
  }
  else if (_bus==1)
  {
    Can1.attachObj(&canClass);
    canClass.attachGeneralHandler();
  }
  return true;
}

int PCAN_GPS::setDout(bool Dout)
{
  uint8_t gpsPower;
  if (canClass._gpsPowerStatus) { gpsPower = 0x02; }
  else { gpsPower = 0x00; }

  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_OUT_IO;
  msg.len = 1;

  if (Dout) { msg.buf[0] = 0x01 | gpsPower; }
  else { msg.buf[0] = 0x00 | gpsPower; }
  return write(msg);
}

int PCAN_GPS::setGpsPower(bool gpsPower)
{
  uint8_t Dout;
  if (canClass._DoutStatus) { Dout = 0x01; }
  else { Dout = 0x00; }

  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_OUT_IO;
  msg.len = 1;
  
  if (gpsPower) { msg.buf[0] = 0x02 | Dout; }
  else { msg.buf[0] = 0x00 | Dout; }
  return write(msg);
}

int PCAN_GPS::devicePowerOff(bool powerOff)
{
  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_POWER_OFF;
  msg.len = 1;
  
  if (powerOff) { msg.buf[0] = 0x01; }
  else { msg.buf[0] = 0x00; }
  return write(msg);
}

int PCAN_GPS::setGyroScale(int gyroScale)           // 0 = +-250°/s, 1 = +-500°/s, 2 = +-2000°/s, 
{
  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_GYRO_SCALE;
  msg.len = 1;
  
  if (gyroScale == 2) { msg.buf[0] = 0x02; }
  else if (gyroScale == 1) { msg.buf[0] = 0x01; }
  else if (gyroScale == 0) { msg.buf[0] = 0x00; }
  else { return -1; }
  return write(msg);
}

int PCAN_GPS::setAccScale(int accScale)             // 1 = +-2 G, 2 = +-4 G, 3 = +-8 G, 4 = +-16 G
{
  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_ACC_SCALE;
  msg.len = 1;
  
  if (accScale == 4) { msg.buf[0] = 0x04; }
  else if (accScale == 3) { msg.buf[0] = 0x03; }
  else if (accScale == 2) { msg.buf[0] = 0x02; }
  else if (accScale == 1) { msg.buf[0] = 0x01; }
  else { return -1; }
  return write(msg);
}

int PCAN_GPS::saveConfig()
{
  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_SAVE_CONFIG;
  msg.len = 1;
  msg.buf[0] = 0x01;
  return write(msg);
}

int PCAN_GPS::setRtc(int year, int month, int day, int dayOfWeek, int hour, int minute, int second)
                                          // dayOfWeek: 0 = Mon, 1 = Tue, 2 = Wed, 3 = Thu, 4 = Fri. 5 = Sat, 6 = Sun
{
  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_RTC_SETTIME;
  msg.len = 1;

  msg.buf[0] = second;
  msg.buf[1] = minute;
  msg.buf[2] = hour;
  msg.buf[3] = dayOfWeek;
  msg.buf[4] = day;
  msg.buf[5] = month;
  uint32_t uintYear = year;
  msg.buf[6] = uintYear & 0x00FF;
  msg.buf[7] = uintYear >> 8;
  return write(msg);
}

int PCAN_GPS::setRtcFromGps()
{
  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_RTC_SETGPSTIME;
  msg.len = 1;
  msg.buf[0] = 0x01;
  return write(msg);
}

int PCAN_GPS::accFastCalibration(int targetX, int targetY, int targetZ)   //target values: 0 = 0 G, 1 = +1 G, 2 = -1 G
{
  CAN_message_t msg;
  msg.ext = 0;
  msg.id = CANID_ACC_CALIBRATION;
  msg.len = 4;
  uint8_t bufX, bufY, bufZ;
  bufX = bufY = bufZ = 0x00;
  if ( generateAccCalibrationBuffer(targetX, bufX) && generateAccCalibrationBuffer(targetY, bufY) && generateAccCalibrationBuffer(targetZ, bufZ) ) 
  {
    msg.buf[0] = bufX;
    msg.buf[1] = bufY;
    msg.buf[2] = bufZ;
    msg.buf[3] = 0x80;
  }
  return write(msg);
}

void CanClass::storeFrame(CAN_message_t &frame)
{
  switch (frame.id)
  {
    case CANID_ACC: {
      _accX = float( (int16_t)(frame.buf[1] << 8) | frame.buf[0] ) * 0.00391*9.80665; // * pow(2,_accRange-1);
      _accY = float( (int16_t)(frame.buf[3] << 8) | frame.buf[2] ) * 0.00391*9.80665; // * pow(2,_accRange-1);
      _accZ = float( (int16_t)(frame.buf[5] << 8) | frame.buf[4] ) * 0.00391*9.80665; // * pow(2,_accRange-1);

      _temp = float((int8_t)frame.buf[6]) * 0.5 + 24.0;              
      _vertAxis = frame.buf[7] & 0x03;
      _orientation = (frame.buf[7] >> 2) & 0x07;
      break; 
    }
    case CANID_MAG: {
      _magX = float( (int16_t)(frame.buf[1] << 8) | frame.buf[0] ) * 0.3;
      _magY = float( (int16_t)(frame.buf[3] << 8) | frame.buf[2] ) * 0.3;
      _magZ = float( (int16_t)(frame.buf[5] << 8) | frame.buf[4] ) * 0.3;
      break; 
    }
    case CANID_ROT_A: {
      _rotX = int32ToFloat( first4ByteToInt32(frame) );
      _rotY = int32ToFloat( last4ByteToInt32(frame) );
      break; 
    }
    case CANID_ROT_B: {
      _rotZ = int32ToFloat( first4ByteToInt32(frame) );
      break; 
    }
    case CANID_GPS_STATUS: {
      _antennaStatus = frame.buf[0];
      _numSatelits = frame.buf[1];
      _navMethod = frame.buf[2];
      break; 
    }
    case CANID_GPS_COURSESPEED: {
      _course = int32ToFloat( first4ByteToInt32(frame) );
      _speed = int32ToFloat( last4ByteToInt32(frame) );
      break; 
    }
    case CANID_GPS_LONGITUDE: {
      float minutes = int32ToFloat( first4ByteToInt32(frame) );
      float degrees = float( (int16_t)(frame.buf[5] << 8) | frame.buf[4] );
      _longitude = degrees + minutes / 60.0;
      _indEW = frame.buf[6];
      break; 
    }
    case CANID_GPS_LATITUDE: {
      float minutes = int32ToFloat( first4ByteToInt32(frame) );
      float degrees = float( (uint16_t)(frame.buf[5] << 8) | frame.buf[4] );
      _latitude = degrees + minutes / 60.0;
      _indNS = frame.buf[6];
      break; 
    }
    case CANID_GPS_ALTITUDE: {
      _altitude = int32ToFloat( first4ByteToInt32(frame) );
      break; 
    }
    case CANID_GPS_DILUTION_A: {
      _posDilution = int32ToFloat( first4ByteToInt32(frame) );
      _horizDilution = int32ToFloat( last4ByteToInt32(frame) );
      break; 
    }
    case CANID_GPS_DILUTION_B: {
      _vertDilution = int32ToFloat( first4ByteToInt32(frame) );
      break; 
    }
    case CANID_GPS_DATETIME: {
      _gpsYear = frame.buf[0];
      _gpsMonth = frame.buf[1];
      _gpsDay = frame.buf[2];
      _gpsHour = frame.buf[3];
      _gpsMinute = frame.buf[4];
      _gpsSecond = frame.buf[5];
      break; 
    }
    case CANID_GPS_IO: {
      uint8_t buf0 = frame.buf[0];
      _Din1Status = buf0 & 0x01;
      _Din2Status = (buf0 >> 1) & 0x01;
      _DoutStatus = (buf0 >> 2) & 0x01;
      _SDpresent = (buf0 >> 3) & 0x01;
      _gpsPowerStatus = (buf0 >> 4) & 0x01;
      _deviceID = (buf0 >> 5) & 0x07;
      break; 
    }
    case CANID_RTC_DATETIME: {
      _rtcSecond = frame.buf[0];
      _rtcMinute = frame.buf[1];
      _rtcHour = frame.buf[2];
      _rtcDayOfWeek = frame.buf[3];
      _rtcDay = frame.buf[4];
      _rtcMonth = frame.buf[5];
      _rtcYear = (frame.buf[7] << 8) | frame.buf[6];
      break; 
    }
  }
}

void CanClass::gotFrame(CAN_message_t &frame, int mailbox)
{
    storeFrame(frame);
}

float int32ToFloat(int32_t int32Number)
{
  // return *((float *) &uint32Number);   // alterniative
  float floatNumber;
  char* intPointer = (char*)&int32Number;
  char* floatPointer = (char*)&floatNumber;
  memcpy(floatPointer, intPointer, sizeof(int32Number));
  return floatNumber;
}

int32_t first4ByteToInt32(CAN_message_t &frame)
{
  return (frame.buf[3] << 24) | (frame.buf[2] << 16) | (frame.buf[1] << 8) | frame.buf[0];
}

int32_t last4ByteToInt32(CAN_message_t &frame)
{
  return (frame.buf[7] << 24) | (frame.buf[6] << 16) | (frame.buf[5] << 8) | frame.buf[4];
}

float median(float array[]) {
    // Allocate an array of the same size and sort it.
    int size = sizeof(array);
    float* sorted = new float[size];
    for (int j = 0; j < size; ++j) {
        sorted[j] = array[j];
    }
    for (int j = size - 1; j > 0; --j) {
        for (int k = 0; k < j; ++k) {
            if (sorted[k] > sorted[k+1]) {
                float temp = sorted[k];
                sorted[k] = sorted[k+1];
                sorted[k+1] = temp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    float median = 0.0;
    if ((size % 2) == 0) {
        median = (sorted[size/2] + sorted[(size/2) - 1])/2.0;
    } else {
        median = sorted[size/2];
    }
    delete [] sorted;
    return median;
}

float mean(float array[]) {
    int size = sizeof(array);
    float sum = 0.0;
    for (int j = 0; j < size; ++j) {
        sum += array[j];
    }
    return sum/((float)size);
}


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

int PCAN_GPS::read(CAN_message_t &msg)
{
  if (_bus == 0)
  {
    return Can0.read(msg);
  }
  else if (_bus==1)
  {
    return Can1.read(msg);
  }
  return -1;
}

int PCAN_GPS::write(CAN_message_t &msg)
{
  if (_bus == 0)
  {
    return Can0.write(msg);
  }
  else if (_bus==1)
  {
    return Can1.write(msg);
  }
  return -1;
}

int PCAN_GPS::generateAccCalibrationBuffer(int target, uint8_t &uint8)
{
  if (target == 0) { uint8 = 0x00; }
  else if (target == 1) { uint8 = 0x01; }
  else if (target == 2) { uint8 = 0x02; }
  else { return -1; }
  return 1;
}