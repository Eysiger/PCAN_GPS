#include <PCAN_GPS.h>

// Initialize PCAN_GPS class object(enablePin, bus, txAlt, rxAlt)
PCAN_GPS pcan_gps(28,0,1,1);

// variables required for sensor readout
float accX, accY, accZ, rotX, rotY, rotZ, magX, magY, magZ, temp;
int vertAxis, orientation;

int years, months, days, hours, minutes, seconds, dayOfWeek;
float gpsLatitude, gpsLongitude, gpsAltitude, gpsCourse, gpsSpeed;
float horizDilution, posDilution, vertDilution;
int indEW, indNS;


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // start the PCAN_GPS driver that reads & stores CAN messages
  if (!pcan_gps.begin(500000)) {          // baud rate
    Serial.println("PCAN_GPS failed to initialize");
    while (1);
  }
  Serial.println("PCAN_GPS initialized.");
  
  // Check if Antenna is ok
  Serial.println("Check if Antenna is ok (0 = INIT, 1 = Dont Know, 2 = OK, 3 = Short, 4 = Open): ");
  while (pcan_gps.gpsAntennaStatus() != 2) {    // 0 = INIT, 1 = Dont Know, 2 = OK, 3 = Short, 4 = Open
    Serial.print(pcan_gps.gpsAntennaStatus());
    Serial.print(" ");
    delay(5000);
  }
  Serial.println(pcan_gps.gpsAntennaStatus());
  Serial.println("Antenna Status ok");

  // Wait until GPS State is OK
  Serial.println("Looking for GPS satelites...");
  Serial.println("Navigation Method (0 = INIT, 1 = None, 2 = 2D, 3 = 3D): ");
  while (pcan_gps.gpsNavMethod() != 3) {    // wait for connection to GPS Satellites.
    Serial.print(pcan_gps.gpsNavMethod());
    Serial.print(" ");
    delay(5000);
  }
  Serial.println(pcan_gps.gpsNavMethod());
  Serial.print("GPS Status ok, connected to ");
  Serial.print(pcan_gps.gpsNumSatelits());
  Serial.println(" satelites.");

  // set internal real time clock (RTC) with the time of the gps signal
  Serial.println("Set RTC of PCAN_GPS from GPS signal.");
  if (!pcan_gps.setRtcFromGps()) {
    Serial.println("RTC of PCAN_GPS couldn't be set from GPS.");
    while (1);
  }
  Serial.println("----------------");

  // // Available functions to configure the PCAN_GPS

  // // Set Dout, GPS power and device power off
  // if (pcan_gps.setDout(true)) { Serial.println("Dout set to true."); }
  // else { Serial.println("Failed to set Dout!"); }
  // if (pcan_gps.setGpsPower(true)) { Serial.println("GPS power set to true."); }
  // else { Serial.println("Failed to set GPS power!"); }
  // if (pcan_gps.devicePowerOff(true)) { Serial.println("Set device power off."); }
  // else { Serial.println("Failed to set device power off!"); }

  // Gyroscope scale selection
  int gyroScale = 0;                        // 0 = +-250°/s, 1 = +-500°/s, 2 = +-2000°/s,
  if (pcan_gps.setGyroScale(gyroScale)) 
  {
    switch (gyroScale)
    {
      case 0:
        Serial.println("Gyroscope scale set to +-250 deg/s.");
        break;
      case 1:
        Serial.println("Gyroscope scale set to +-500 deg/s.");
        break;
      case 2:
        Serial.println("Gyroscope scale set to +-2000 deg/s.");
        break;
    }
  }
  else {  Serial.println("Failed to set gyroscope scale!"); }

  // Accelerometer scale selection
  int accScale = 1;                         // 1 = +-2 G, 2 = +-4 G, 3 = +-8 G, 4 = +-16 G
  if (pcan_gps.setAccScale(accScale)) 
  {
    switch (accScale)
    {
      case 1:
        Serial.println("Accelerometer scale set to +-2 G.");
        break;
      case 2:
        Serial.println("Accelerometer scale set to +-4 G.");
        break;
      case 3:
        Serial.println("Accelerometer scale set to +-8 G.");
        break;
      case 4:
        Serial.println("Accelerometer scale set to +-16 G.");
        break;
    }
  }
  else {  Serial.println("Failed to set accelerometer scale!"); }             

  // // Set real time clock (RTC), dayOfWeek: 0 = Mon, 1 = Tue, 2 = Wed, 3 = Thu, 4 = Fri. 5 = Sat, 6 = Sun
  // if (pcan_gps.setRtc(years,months,days,dayOfWeek,hours,minutes,seconds)) { Serial.println("Set time of real time clock (RTC)."); } 
  // else { Serial.println("Failed to set real time clock (RTC)!"); }

  // // Accelerometer calibration, target values: 0 = 0 G, 1 = +1 G, 2 = -1 G
  // if (pcan_gps.accFastCalibration(0, 0, 1)) { // else { Serial.println("Calibrated the accelerometer!"); }; }
  // else { Serial.println("Failed to calibrate the accelerometer!"); }
  
  // // Required to store configuration for use after power off
  // if (pcan_gps.saveConfig()) { Serial.println("Saved the configuration."); }
  // else { Serial.println("Failed to save the configuration!"); }
   
  Serial.println("----------------");
}

void loop() {
  // Functions to obtain sensor values
  pcan_gps.getAcc(accX,accY,accZ);          // in m/s^2
  pcan_gps.getTemp(temp);                   // in °C
  pcan_gps.getVertAxis(vertAxis);           // 0 = undefined, 1 = X Axis, 2 = Y Axis, 3 = Z Axis
  pcan_gps.getOrientation(orientation);     // 0 = flat, 1 = flat upside down
                                            // 2 = landscape left, 3 = landscape right
                                            // 4 = portrait, 5 = portrait upside down
                              
  pcan_gps.getRot(rotX,rotY,rotZ);          // in °/s
  pcan_gps.getMag(magX,magY,magZ);          // in micro Tesla

  //  pcan_gps.gpsLongitude(gpsLongitude);
  //  pcan_gps.gpsLatitude(gpsLatitude);
  //  pcan_gps.gpsAltitude(gpsAltitude);
  pcan_gps.gps3DPos(gpsLatitude, gpsLongitude, gpsAltitude);        // in [°, °, m]

  //  pcan_gps.gpsCourse(gpsCourse);
  //  pcan_gps.gpsSpeed(gpsSpeed);
  pcan_gps.gpsCourseAndSpeed(gpsCourse, gpsSpeed);                  // in [°, km/h]

  //  pcan_gps.gpsHorizDilution(horizDilution);
  //  pcan_gps.gpsVertDilution(vertDilution);
  //  pcan_gps.gpsPosDilution(posDilution);
  pcan_gps.gpsDilution(horizDilution, vertDilution, posDilution);   // values <6 are ok, 1 is optimal

  pcan_gps.gpsIndEW(indEW);              // 0 = INIT, 69 = East, 87 = West
  pcan_gps.gpsIndNS(indNS);              // 0 = INIT, 78 = North, 83 = South

  //  pcan_gps.rtcSecond(seconds);
  //  pcan_gps.rtcMinute(minutes);
  //  pcan_gps.rtcHour(hours);
  //  pcan_gps.rtcTime(hours, minutes, seconds);
  pcan_gps.rtcDayOfWeek(dayOfWeek);       // 0 = Mon, 1 = Tue, 2 = Wed, 3 = Thu, 4 = Fri. 5 = Sat, 6 = Sun
  //  pcan_gps.rtcDay(days);
  //  pcan_gps.rtcMonth(months);
  //  pcan_gps.rtcYear(years);
  //  pcan_gps.rtcDate(years, months, days);
  pcan_gps.rtcDateAndTime(years, months, days, hours, minutes, seconds);

  //  pcan_gps.gpsYear(years);
  //  pcan_gps.gpsMonth(months);
  //  pcan_gps.gpsDay(days);
  //  pcan_gps.gpsDate(years, months, days);
  //  pcan_gps.gpsHour(hours);
  //  pcan_gps.gpsMinute(minutes);
  //  pcan_gps.gpsSecond(seconds);
  //  pcan_gps.gpsTime(hours, minutes, seconds);
  //  pcan_gps.gpsDateAndTime(years, months, days, hours, minutes, seconds);

  bool din1Status = pcan_gps.Din1Status();
  bool din2Status = pcan_gps.Din2Status();
  bool doutStatus = pcan_gps.DoutStatus();
  bool sdPresent = pcan_gps.SDpresent();
  bool gpsPower = pcan_gps.gpsPowerStatus();
  int id = pcan_gps.deviceID();

  // print sensor values in a compact form
  String acc = "acc [m/s^2]: " + String(accX) + ", " + String(accY) + ", " + String(accZ) + "\t\t\t ";
  String rot = "rot [deg/s]: " + String(rotX) + ", " + String(rotY) + ", " + String(rotZ) + "\t ";
  String mag = "mag [micro Tesla]: " + String(magX) + ", " + String(magY) + ", " + String(magZ);
  Serial.print(acc);
  Serial.print(rot);
  Serial.println(mag);

  String gps1 = "gps (lat,long,alt): " + String(gpsLatitude) + ", " + String(gpsLongitude) + ", " + String(gpsAltitude) + "\t\t ";
  String gps2 = "course, speed: " + String(gpsCourse) + ", " + String(gpsSpeed) + "\t\t ";
  String gps3 = "dilution (horiz,vert,pos): " + String(horizDilution) + ", " + String(vertDilution) + ", " + String(posDilution) + "\t ";
  String gps4 = "date&time: " + String(days) + "." + String(months) + "." + String(years) + " " + String(hours) + ":" + String(minutes) + "." + String(seconds) + " day of week: " + String(dayOfWeek);
  Serial.print(gps1);
  Serial.print(gps2);
  Serial.print(gps3);
  Serial.println(gps4);

  String gps5 = "East/West (69 = E, 87 = W): " + String(indEW) + "\t\t\t North/South (78 = N, 83 = S): " + String(indNS) + "\t ";
  String temperature = "temperatrue [degC]: " + String(temp) + "\t\t\t ";
  String vert = "vertical axis (0 = undef, 1 = X Axis, 2 = Y Axis, 3 = Z Axis): " +  String(vertAxis) + "\t\t\t ";
  String ori = "orientation (0 = flat, 1 = flat upside down, 2 = landscape left, 3 = landscape righ, 4 = portrait, 5 = portrait upside down): " + String(orientation) + "\t ";
  String io = "din1Status: " + String(din1Status) + ", din2Status: " + String(din2Status) + ", doutStatus: " + String(doutStatus) + ", sdPresent: " + String(sdPresent) + ", gps Power: " + String(gpsPower) + ", device ID: " + String(id); 
  Serial.print(gps5);
  Serial.print(temperature);
  Serial.println(io);
  Serial.print(vert);
  Serial.println(ori);
  
  Serial.println("");

  delay(100);
}
