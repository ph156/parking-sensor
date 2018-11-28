#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <LSM303.h>
#include <FreeRTOS_ARM.h>
#include <Adafruit_GPS.h>

/***************************************GPS setup***************************************/
//hardware serial port
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

uint32_t timer = millis();
/***************************************************************************************/

/**********************************Magnetometer setup***********************************/
LSM303 mag;
const byte interruptPin = 51;  //interrupt pin for lsm303
int count = 0;   //count of magnetometer interrupts
/***************************************************************************************/


/****************************************HC-SR04****************************************/
const int trigPin = 5;
const int echoPin = 6;
long duration;
float distance;
float Mdistance;
float minDistance = 0.3; //1 ft
float maxDistance = 0.76; //2.5 ft
bool carDetected = false;

/***************************************************************************************/


QueueHandle_t MagX_Queue_Handle;
QueueHandle_t MagY_Queue_Handle;
QueueHandle_t MagZ_Queue_Handle;

//Initalize thresholds for x,y,z
uint16_t xThreshold = 0;
uint16_t yThreshold = 0;
uint16_t zThreshold = 0;

void setup() {

  portBASE_TYPE s1;

  pinMode(interruptPin, INPUT_PULLUP); //  set interruptPin to an input with the internal pull-up resistor enabled

  // Initialize SerialUSB
  Serial.begin(115200);
  while (!Serial);


  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  Serial.println("Find location with GPS");
  GPSinfo();

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input






  //Create queues with 100 fields for each axis of magnetometer data
  MagX_Queue_Handle = xQueueCreate(100, sizeof(int16_t));
  MagY_Queue_Handle = xQueueCreate(100, sizeof(int16_t));
  MagZ_Queue_Handle = xQueueCreate(100, sizeof(int16_t));

  //Enable interrupt pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), collectData, FALLING);

  s1 =  xTaskCreate(processMagData, NULL, 512, NULL, 1, NULL);

  //start task scheduler
  vTaskStartScheduler();
  while (1);
}

//collect data from magnetometer from the Z axis
void collectData(void) {
  uint8_t item = 1;
  int z = xQueueSendFromISR(MagZ_Queue_Handle, &(item), NULL);
  count ++;

}


//Return # of times interrupt is triggered
static void processMagData(void* arg) {
  //  while (1) {
  //    sensor.readMagData();
  //    int z = xQueueReceive(MagZ_Queue_Handle, &(sensor.magData.z), 5000);
  //
  //    if (!z) {
  //    }
  //    else {
  //      SerialUSB.print("Num of interrupts: ");
  //      SerialUSB.println(count);
  //    }
  //    vTaskDelay(500);
  //  }
}



void loop() {

}


void GPSinfo() {
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}

void readDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = (duration * 0.0228) / 2; // Distance with Cm
  Mdistance = distance / 100;     // Distance with m
  if (Mdistance > minDistance && Mdistance > maxDistance) {
    carDetected = true;
    Serial.println("Car detected");
  }
  else {
    carDetected = false;
    Serial.println("No car detected");
  }

  //// Prints the distance
  //Serial.print("Distance: ");
  //Serial.print(Mdistance);
  //Serial.print(" m");
  //Serial.println();
  //delay(50);
}
