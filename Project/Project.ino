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

  Serial.println("Smart Parking Sensor");


  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  Serial.println("Find location with GPS");
  GPSinfo();


  //    initialize magnetometer
  Wire.begin();
  mag.init();
  mag.enableDefault();

  //Calibrate mag interrupt thresholds
  calibration();
  Serial.print ("Z Threshold: ");
  Serial.println (zThreshold);
  Serial.println ();


  //Enable interrupts based on calibrated threshold values
  mag.enableInterrupt(zThreshold);

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
  while (1) {
    mag.readMag();
    int z = xQueueReceive(MagZ_Queue_Handle, &(mag.m.z), 5000);

    if (!z) {
    }
    else {
      SerialUSB.print("Num of interrupts: ");
      SerialUSB.println(count);
    }
    vTaskDelay(500);
  }
}



void loop() {

}


void GPSinfo() {
  Serial.print("Getting Location... ");

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  //    if (GPS.fix) {
  Serial.print("Location: ");
  Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
  Serial.print(", ");
  Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
  //    }


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
}

void calibration() {
  Serial.println("Calibrating... ");
  int16_t magz[150];
  long sumz, sumdiffz, stdvz, avgz;
  sumz = 0;
  sumdiffz = 0;
  stdvz = 0;
  avgz = 0;
  for (int i = 0; i < 150; i++) {
    mag.readMag();
    magz[i] = mag.m.z;
    sumz += (long)mag.m.z;
    Serial.print("Z value: ");
    Serial.println(magz[i]);
    delay(100);

  }
  avgz = (sumz / 150);
  Serial.print("Average Z: " );
  Serial.println(avgz);
  for (int i = 0; i < 150; i++) {
    sumdiffz += sq(magz[i] - avgz);
  }
  stdvz = sqrt(sumdiffz / 150);
  zThreshold = abs(avgz) + stdvz * 5;
  Serial.println("Calibration Finished");
  Serial.println();

}
