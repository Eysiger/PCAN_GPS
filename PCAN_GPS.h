/***************************************************************************
  This is a library for the Peak PCAN GPS pressure sensor

  The sensor uses CAN to communicate.

 ***************************************************************************/
#ifndef __PCAN_GPS_H__
#define __PCAN_GPS_H__

#include "Arduino.h"
#include "FlexCAN.h"
#include <vector>

/*=========================================================================
    CAN IDs
    -----------------------------------------------------------------------*/
enum
    {
      CANID_ACC				= 0x600,
      CANID_MAG				= 0x601,
      CANID_ROT_A			= 0x610,
      CANID_ROT_B			= 0x611,

      CANID_GPS_STATUS		= 0x620,
      CANID_GPS_COURSESPEED	= 0x621,
      CANID_GPS_LONGITUDE	= 0x622,
      CANID_GPS_LATITUDE	= 0x623,
      CANID_GPS_ALTITUDE	= 0x624,
      CANID_GPS_DILUTION_A	= 0x625,
      CANID_GPS_DILUTION_B	= 0x626,
      CANID_GPS_DATETIME	= 0x627,
      CANID_GPS_IO 			= 0x630,

      CANID_RTC_DATETIME	= 0x640,

      CANID_OUT_IO			= 0x650,
      CANID_POWER_OFF		= 0x651,
      CANID_GYRO_SCALE		= 0x652,
      CANID_ACC_SCALE		= 0x653,
      CANID_SAVE_CONFIG		= 0x654,
      CANID_RTC_SETTIME		= 0x655,
      CANID_RTC_SETGPSTIME	= 0x656,
      CANID_ACC_CALIBRATION = 0x657,
    };


class CanClass : public CANListener
{
public:
	void gotFrame(CAN_message_t &frame, int mailbox); 	//overrides the parent version so we can actually do something
	void storeFrame(CAN_message_t &frame);

	float _accX, _accY, _accZ;

	float _temp;							
	int _vertAxis, _orientation;

	float _magX, _magY, _magZ;

	float _rotX, _rotY, _rotZ;

	int _antennaStatus, _numSatelits, _navMethod;

	float _course, _speed;
	float _longitude, _latitude, _altitude;
	int _indEW, _indNS;

	float _horizDilution, _posDilution, _vertDilution;

	int _gpsYear, _gpsMonth, _gpsDay, _gpsHour, _gpsMinute, _gpsSecond;

	bool _Din1Status, _Din2Status, _DoutStatus, _SDpresent, _gpsPowerStatus;
	int8_t _deviceID;

	int _rtcYear, _rtcMonth, _rtcDayOfWeek, _rtcDay, _rtcHour, _rtcMinute, _rtcSecond;

	int _accRange, _gyroRange;
};

class PCAN_GPS
{
  public:
    PCAN_GPS();
    PCAN_GPS(int8_t speedPin, bool highSpeedMode);
    PCAN_GPS(int8_t speedPin, bool highSpeedMode, int8_t bus);
    PCAN_GPS(int8_t speedPin, bool highSpeedMode, int8_t bus, int8_t txAlt, int8_t rxAlt);

    bool begin(uint32_t baud = 500000);

    void getAcc(float &accX, float &accY, float &accZ)	// in m/s^2
    {	accX = canClass._accX; accY = canClass._accY; accZ = canClass._accZ;	}
    void getTemp(float &temp)							// in °C
    {	temp = canClass._temp;	}
    void getVertAxis(int &vertAxis)						// 0 = undefined, 1 = X Axis, 2 = Y Axis, 3 = Z Axis
    {	vertAxis = canClass._vertAxis;	}
    void getOrientation(int &orientation)				// 0 = flat, 1 = flat upside down
    													// 2 = landscape left, 3 = landscape right
    													// 4 = portrait, 5 = portrait upside down
    {	orientation = canClass._orientation;	}

    void getMag(float &magX, float &magY, float &magZ)
    {	magX = canClass._magX; magY = canClass._magY; magZ = canClass._magZ;	}

    void getRot(float &rotX, float &rotY, float &rotZ)
    {	rotX = canClass._rotX; rotY = canClass._rotY; rotZ = canClass._rotZ;	}

    void gpsAntennaStatus(int &antStatus)								// 0 = INIT, 1 = Dont Know, 2 = OK, 3 = Short, 4 = Open
    {	antStatus = canClass._antennaStatus;	}
    void gpsNumSatelits(int &num)
    {	num = canClass._numSatelits;	}
    void gpsNavMethod(int &method)									// 0 = INIT, 1 = None, 2 = 2D, 3 = 3D
    {	method = canClass._navMethod;	}

    void gpsCourse(float &course)
    {	course = canClass._course;	}
    void gpsSpeed(float &speed)
    {	speed = canClass._speed;	}
    void gpsCourseAndSpeed(float &course, float &speed)
    {	course = canClass._course; speed = canClass._speed;	}

    void gpsLongitude(float &longitude)
    {	longitude = canClass._longitude;	}
    void gpsIndEW(int &indEW)							// 0 = INIT, 69 = East, 87 = West
    {	indEW = canClass._indEW;	}
    void gpsLatitude(float &latitude)
    {	latitude = canClass._latitude;	}
    void gpsIndNS(int &indNS)							// 0 = INIT, 78 = North, 83 = South
    {	indNS = canClass._indNS;	}

    void gpsAltitude(float &altitude)
    {	altitude = canClass._altitude;	}

    void gps3DPos(float &latitude, float &longitude, float &altitude)
    {	latitude = canClass._latitude; longitude = canClass._longitude; altitude = canClass._altitude;	}

    void gpsHorizDilution(float &horizDilution)
    {	horizDilution = canClass._horizDilution;	}
    void gpsPosDilution(float &posDilution)
    {	posDilution = canClass._posDilution;	}
    void gpsVertDilution(float &vertDilution)
    {	vertDilution = canClass._vertDilution; }

    void gpsDilution(float &horizDilution, float &vertDilution, float &posDilution)
    {	horizDilution = canClass._horizDilution; vertDilution = canClass._vertDilution; posDilution = canClass._posDilution;	}

    void gpsYear(int &year)
    {	year = canClass._gpsYear;	}
    void gpsMonth(int &month)
    {	month = canClass._gpsMonth;	}
    void gpsDay(int &day)
    {	day = canClass._gpsDay;	}
    void gpsDate(int &year, int &month, int &day)
    {	year = canClass._gpsYear; month = canClass._gpsMonth; day = canClass._gpsDay;	}

    void gpsHour(int &hour)
    {	hour = canClass._gpsHour;	}
    void gpsMinute(int &minute)
    {	minute = canClass._gpsMinute;	}
    void gpsSecond(int &second)
    {	second = canClass._gpsSecond;	}
    void gpsTime(int &hour, int &minute, int &second)
    {	hour = canClass._gpsHour; minute = canClass._gpsMinute; second = canClass._gpsSecond;	}

    void gpsDateAndTime(int &year, int &month, int &day, int &hour, int &minute, int &second)
    {	year = canClass._gpsYear; month = canClass._gpsMonth; day = canClass._gpsDay; hour = canClass._gpsHour; minute = canClass._gpsMinute; second = canClass._gpsSecond;	}

    bool Din1Status()
    {	return canClass._Din1Status;	}
    bool Din2Status()
    {	return canClass._Din2Status;	}
    bool DoutStatus()
    {	return canClass._DoutStatus;	}
    bool SDpresent()
    {	return canClass._SDpresent;	}
    bool gpsPowerStatus()
    {	return canClass._gpsPowerStatus;	}
    int8_t deviceID()
    {	return canClass._deviceID;	}

    void rtcSecond(int &second)
    {	second = canClass._rtcSecond;	}
    void rtcMinute(int &minute)
    {	minute = canClass._rtcMinute;	}
    void rtcHour(int &hour)
    {	hour = canClass._rtcHour;	}
    void rtcTime(int &hour, int &minute, int &second)
    {	second = canClass._rtcSecond; minute = canClass._rtcMinute; hour = canClass._rtcHour;	}

    void rtcDayOfWeek(int &dayOfWeek)					// 0 = Mon, 1 = Tue, 2 = Wed, 3 = Thu, 4 = Fri. 5 = Sat, 6 = Sun
    {	dayOfWeek = canClass._rtcDayOfWeek;	}

    void rtcDay(int &day)
    {	day = canClass._rtcDay;	}
    void rtcMonth(int &month)
    {	month = canClass._rtcMonth;	}
	void rtcYear(int &year)
	{	year = canClass._rtcYear;	}   
    void rtcDate(int &year, int &month, int &day)
    {	day = canClass._rtcDay; month = canClass._rtcMonth; year = canClass._rtcYear;	}

    void rtcDateAndTime(int &year, int &month, int &day, int &hour, int &minute, int &second)
    {	second = canClass._rtcSecond; minute = canClass._rtcMinute; hour = canClass._rtcHour; day = canClass._rtcDay; month = canClass._rtcMonth; year = canClass._rtcYear;	}

    int setDout(bool Dout);
    int setGpsPower(bool gpsPower);
    int devicePowerOff(bool powerOff);

    int setGyroScale(int gyroScale);					// 0 = +-250°/s, 1 = +-500°/s, 2 = +-2000°/s, 

    int setAccScale(int accScale);						// 1 = +-2 G, 2 = +-4 G, 3 = +-8 G, 4 = +-16 G

    int saveConfig();

    int setRtc(int year, int month, int day, int dayOfWeek, int hour, int minute, int second);
    													// dayOfWeek: 0 = Mon, 1 = Tue, 2 = Wed, 3 = Thu, 4 = Fri. 5 = Sat, 6 = Sun
    int setRtcFromGps();

    int accFastCalibration(int targetX, int targetY, int targetZ);	//target values: 0 = 0 G, 1 = +1 G, 2 = -1 G


  private:
  	int read(CAN_message_t &msg);
    int write(CAN_message_t &msg);

    int generateAccCalibrationBuffer(int target, uint8_t &uint8);

 	CanClass canClass;

	int8_t _speedPin;
    bool _highSpeedMode;
	int8_t _bus;
	uint8_t _txAlt;
	uint8_t _rxAlt;
};

float int32ToFloat(int32_t int32Number);
int32_t first4ByteToInt32(CAN_message_t &frame);
int32_t last4ByteToInt32(CAN_message_t &frame);

float median(std::vector< float > vector);
float mean(std::vector< float > vector);

#endif