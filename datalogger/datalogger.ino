#include <TinyGPS++.h>
#include "Wire.h"
#include <SD.h>
#include "I2Cdev.h"
#include "MPU60X0.h"
#include <HMC5883L.h>
#include <SeeedOLED.h>

// *****************************************
// *************-- INIT --******************
// *****************************************
// *****************************************

// ***************************
// ******** MPU 6050 *********
// ***************************

int16_t ax, ay, az;
int16_t gx, gy, gz;

MPU60X0 accelgyro;

// ***************************
// ******** HC-SR04 *********
// ***************************

int maximumRange = 400; // distance Maximale acceptée (en cm)
int minimumRange = 0;   // distance Minimale acceptée (en cm)
long duration, distance; // Durée utilisé pour calculer la distance

int16_t dist;

#define echoPin 6 // broche Echo 
#define trigPin 7 // broche Trigger (declenchement)

// ***************************
// ******** HMC5883L *********
// ***************************

HMC5883L compass;

float degree;
int error = 0;

// ***************************
// **** GPS RELATED LIB ******
// ***************************

TinyGPSPlus gps;

String GPStime;
String GPSsat;
double GPSlat;
double GPSlong;
double GPSalt;
double GPSdeg;
double GPSkmph;

// ***************************
// *** SD RELATED LIB ****
// ***************************

const int chipSelect = 53;
char filename[16];

// *****************************************
// *****************************************


//OLED
unsigned long previousMillis1 = 0;        // will store last time LED was updated
long time1 = 500;           // milliseconds of on-time

//HCSR04
unsigned long previousMillis2 = 0;        // will store last time LED was updated
long time2 = 250;           // milliseconds of on-time

//SD
unsigned long previousMillis3 = 10000;        // will store last time LED was updated
long time3 = 500;           // milliseconds of on-time

//MPU6050
unsigned long previousMillis4 = 0;        // will store last time LED was updated
long time4 = 250;           // milliseconds of on-time

//MAGNETO
unsigned long previousMillis5 = 0;        // will store last time LED was updated
long time5 = 250;           // milliseconds of on-time



// *****************************************
// ************-- SETUP --******************
// *****************************************
// *****************************************


void setup()
{

  //SERIAL
  Serial.begin(115200);


  // ***************************
  // ********** OLED ***********
  // ***************************

  Wire.begin();
  SeeedOled.init();  //initialze SEEED OLED display
  DDRB |= 0x21;
  PORTB |= 0x21;

  SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
  SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
  SeeedOled.setPageMode();           //Set addressing mode to Page Mode
  SeeedOled.setTextXY(0, 0);         //Set the cursor to Xth Page, Yth Column
  SeeedOled.putString("Initializing..."); //Print the String

  // ***************************
  // ******** HC-SR04 **********
  // ***************************

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // ***************************
  // *** MPU 6050 & OTHER ? ****
  // ***************************

  accelgyro.initialize();
  magneto_initialize();

  // ***************************
  // ***** SD RELATED SET ******
  // ***************************

  pinMode(53, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }

  //Serial.println("card initialized.");
  //char filename[16]; // make it long enough to hold your longest file name, plus a null terminator
  int n = 0;
  snprintf(filename, sizeof(filename), "data%03d.txt", n); // includes a three-digit sequence number in the file name
  while (SD.exists(filename)) {
    n++;
    snprintf(filename, sizeof(filename), "data%03d.txt", n);
  }
  File dataFile = SD.open(filename, FILE_WRITE);

  dataFile.println("millis,time,sat,lat,long,alt,heading,spd,ax_bi,ay_bi,az_bi,gx_bi,gy_bi,gz_bi,dist");
  
  //Serial.println(n);
  //Serial.println(filename);
  dataFile.close();
  //now filename[] contains the name of a file that doesn't exist



// *****************************************
// *************-- LOOP --******************
// *****************************************
// *****************************************

  //GPS
  Serial1.begin(9600);
  
    SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
  
}

void loop()
{

  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();

  readNMEA();

  if (currentMillis - previousMillis1 >= time1)
  {
    oled();
    previousMillis1 = currentMillis;  // Remember the time

  }

  if (currentMillis - previousMillis2 >= time2)
  {
    hcsr04();
    previousMillis2 = currentMillis;  // Remember the time

  }

  if (currentMillis - previousMillis3 >= time3)
  {
    serial_data();
    previousMillis3 = currentMillis;  // Remember the time

  }

  if (currentMillis - previousMillis4 >= time4)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    previousMillis4 = currentMillis;  // Remember the time

  }

  if (currentMillis - previousMillis5 >= time5)
  {
    magneto_getDegree();
    previousMillis5 = currentMillis;  // Remember the time

  }

}


static void oled() {

  String logOledString1;
  logOledString1 += "";
  logOledString1 +=  String(millis());

  String logOledString2;
  logOledString2 += "Sat visible :";
  logOledString2 += GPSsat;

    String logOledString3;
  logOledString3 += "Ultrasound :";
  logOledString3 += dist;

  int str_len1 = logOledString1.length() + 1; 
  char char_array1[str_len1];
  logOledString1.toCharArray(char_array1, str_len1);

    int str_len2 = logOledString2.length() + 1; 
  char char_array2[str_len2];
  logOledString2.toCharArray(char_array2, str_len2);

     int str_len3 = logOledString3.length() + 1; 
  char char_array3[str_len3];
  logOledString3.toCharArray(char_array3, str_len3);
  

  SeeedOled.setTextXY(0, 0);         //Set the cursor to Xth Page, Yth Column
  SeeedOled.putString(char_array1); //Print the String
  
  SeeedOled.setTextXY(1, 0);         //Set the cursor to Xth Page, Yth Column
  SeeedOled.putString(char_array2); //Print the String

    SeeedOled.setTextXY(2, 0);         //Set the cursor to Xth Page, Yth Column
  SeeedOled.putString(char_array3); //Print the String
  

}

static void hcsr04() {

  // Envoi une impulsion de 10 micro seconde sur la broche "trigger"
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  // Attend que la broche Echo passe au niveau HAUT
  // retourne la durée
  duration = pulseIn(echoPin, HIGH);

  //Calculer la distance (en cm, basé sur la vitesse du son).
  distance = duration / 58.2;

  // Si la distance mesurée est HORS des valeurs acceptables
  if (distance >= maximumRange || distance <= minimumRange) {
    /* Envoyer une valeur négative sur la liaison série.
       Activer la LED pour indiquer que l'erreur */
    //Serial.println("-1");
    dist = 0;
    //digitalWrite(LEDPin, HIGH);
  }
  else {
    /* Envoyer la distance vers l'ordinateur via liaison série.
       Eteindre la LED pour indiquer une lecture correcte. */
    //Serial.println(distance);
    dist = distance;
    //digitalWrite(LEDPin, LOW);
  }

}

static void   magneto_initialize() {

  compass = HMC5883L();

  //Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(1.3); // Set the scale of the compass.
  //if (error != 0) // If there is an error, print it out.
  //Serial.println(compass.GetErrorText(error));

  //Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  //if (error != 0) // If there is an error, print it out.
  //Serial.println(compass.GetErrorText(error));

}

static void magneto_getDegree() {

  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();

  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0457;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  // Output the data via the serial port.
  degree = headingDegrees;

  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);

}

static void serial_data() {
  char tmp[10];

  String logDataString = String(millis());
  logDataString += ",";
  logDataString +=  GPStime;
  logDataString += ",";
  logDataString += GPSsat;
  logDataString += ",";
  dtostrf(GPSlat, 1, 6, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(GPSlong, 1, 6, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(GPSalt, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(GPSdeg, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(GPSkmph, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(ax, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(ay, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(az, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(gx, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(gy, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(gz, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(degree, 1, 2, tmp);
  logDataString += tmp;
  logDataString += ",";
  dtostrf(dist, 1, 2, tmp);
  logDataString += tmp;



  Serial.println(logDataString);

  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println(logDataString);
    dataFile.close();
  }
  else {
    Serial.println("error");
  }

}

static void readNMEA()
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());

    //Serial.println(gps.location.lat());
    //Serial.println(printInt(gps.satellites.value(), gps.satellites.isValid(), 5));
    GPStime = printDateTime(gps.date, gps.time);
    GPSsat = printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    GPSlat = gps.location.lat();
    GPSlong = gps.location.lng();
    GPSalt = gps.altitude.meters();
    GPSdeg = gps.course.deg();
    GPSkmph = gps.speed.kmph();

  } while (millis() - start < 0);
}




const char * printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid) {
    sprintf(sz, "%ld", val);
    return sz;
  }

}



const char * printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if ((d.isValid()) && (t.isValid()) )
  {
    char sz[64];
    sprintf(sz, "%02d-%02d-%02d %02d:%02d:%02d", d.month(), d.day(), d.year(), t.hour(), t.minute(), t.second());
    return sz;
  }

}





