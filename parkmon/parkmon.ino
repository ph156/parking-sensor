/*
 * The parkmon project is to make a parking monitor using Arduino Due board as the main control board
 * and uses the magnetometer  and ultrosound priximeter to measure a parking space is empty or occupied.
 * It also uses GPS for position location and time sync, Real-time clock for time and interrupt.
 * There is also a temperature and humidity sensor to monitor surrounding.
 * It uses RF95 modules, one as server and one as client. the client is the monitor, it transmit 
 * the infomation back to servor. So serve has record of the parking space occupancy information.
 * 
 * It uses many of the public available library for Arduino Due with the appropriate breakout board
 * Adafruit_AM2320_sensor_library
 * Adafruit_GPS
 * Adafruit_LSM303DLHC
 * Adafruit_Unified_Sensor
 * FreeRTOS_ARM
 * RadioHead
 * RTCDue
 *  
 * the following header file indicate the library we used, during development,
 * although we do not  need all for final.
 * 
 * Adafruit_Sensor.h sensors from Adafruit product
 * Adafruit_AM2320.h for AM2320 chip. temperature and humidity.
 * Adafruit_GPS.h    for GPS board
 * LSM303.h
 * 
 * RH_RF95.h         for RF communication, RadioHead library
 * RTCDue.h          for Real-time clock 
 * 
 * FreeRTOS_ARM.h    for the Free RTOS operating system.
 * 
 * Adafruit wweb site got examples and info about many libraries to help we get started.
 * 
 * We used I2C and SPI serial communications between boards.
 * 
 * 2018-12-10
 * 
 *    
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_AM2320.h"
#include "LSM303.h"
#include <FreeRTOS_ARM.h>
#include <Adafruit_GPS.h>
#include <RTCDue.h>
#include <RH_RF95.h>

#include "parking_monitor_id.h"

/***************************************GPS setup***************************************/
//hardware serial port
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
//#define GPSECHO true
#define GPSECHO false
char     gpsString[50]={ ""};

/***************************************************************************************/

/**********************************Magnetometer setup***********************************/
LSM303 mag;
const byte interruptPin = 51;  //interrupt pin for lsm303 //not really used

/***************************************************************************************/
void   CollectInfo();
void   Init_RTT();
void   GPSinfo();
void   MeasureDistance();
void   RfSendDataToServer(int code);
void   calibration();

/****************************************HC-SR04****************************************/
const int trigPin = 5;
const int echoPin = 6;
long  duration;
float distance;
float minDistance = 10.; //   10 cm   
float maxDistance = 76.; //  76 cm  2.5 ft
bool  carDetected = false; 
char  slotOccupied[2]="E"; // E is empty, O is occupied

/*********************frreRTOS******************************************************************/

//Initialize thresholds for x,y,z
int16_t xThreshold = 0;
int16_t yThreshold = 0;
int16_t zThreshold = 0;
/***************************************************************************************/
// Select the Slowclock source
RTCDue rtc(RC);
//RTCDue rtc(XTAL);
const char* daynames[]={"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

/***************************************************************************************/
Adafruit_AM2320 am2320 = Adafruit_AM2320();
// Singleton instance of the radio driver
RH_RF95 rf95;
const int ledPin   = 13;      // the number of the LED pin
const int enGPSPin =  4;      // the number of GPS module enable pin
const int enRFPin  =  8;      // the number of RF  module enable pin

float temperature, humidity;
uint8_t  tx_data[240];
int   num_bytes;
char  pm_id[8]; // mostly only 3 char digit
volatile uint32_t count = 0;
/***************************************************************************************/
volatile uint32_t loopCount;
int  reset_mode;
int  in_backup_mode;
int  wakeup_sr;
#define   SUPC_KEY                    ( 0x0A5 )
#define ALARM_in_SECONDS_default      120
//#define ALARM_in_SECONDS_default      60 /* for testing, faster */

/***************************************************************************************/
void setup() {
  in_backup_mode = 0; //initially not in backup mode
  reset_mode = (RSTC->RSTC_SR>> 8) & 0x7;  // 3 bit
 // Initialize SerialUSB
    Serial.begin(115200);
//   while (!Serial);
  
    Init_RTT();

    if ( reset_mode & 0x01)
    {
      // after backup reset
      in_backup_mode = 1;
    }
    else
    {
      in_backup_mode = 0;
      GPBR->SYS_GPBR[0] = 0;
      GPBR->SYS_GPBR[1] = 'X';
    }

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // save power
//  pinMode(enGPSPin, OUTPUT);   // did not impact power
//  pinMode(enRFPin, OUTPUT);
//  digitalWrite(enRFPin, HIGH);  // enable RF
// assign id
  strncpy(pm_id,PARKING_MONITOR_ID, 6);
  rtc.begin();

/*
 * ---------------------------------------------------------------------
 */
  if ( in_backup_mode == 0)
  {
//    digitalWrite(enGPSPin, HIGH);  // start hi, enable  for GPS
 
//    pinMode(interruptPin, INPUT_PULLUP); //  set interruptPin to an input with the internal pull-up resistor enabled
 
 
    loopCount = 0;

    Serial.println("============ New Boot =============");
    Serial.println("Smart Parking Sensor");
    Serial.print("The code is built on: ");
    Serial.print( __DATE__ );
    Serial.print(" at ");
    Serial.println( __TIME__ );
    Serial.print("This Monitor is ID: ");
    Serial.println( pm_id);
    // Set the initial time
    rtc.setTime(__TIME__);
    // Set the date
    rtc.setDate(__DATE__);
     rtc.disableAlarmTime ();
     rtc.disableAlarmDate ();
     rtc.detachAlarm ();
 

    GPSSerial.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  }
  else
  {
      Serial.println("======== wake up ====");
  }

    //    initialize magnetometer
    Wire.begin();
    mag.init();
    mag.enableDefault();

  //Calibrate mag interrupt thresholds
//  calibration();
//  Serial.print ("Z Threshold: ");
//  Serial.println (zThreshold);
//  Serial.println ();
//  Serial.println("Find location with GPS");
  if ( in_backup_mode == 0)
  {
    GPSinfo();
  }
//  digitalWrite(enGPSPin, LOW);  //disable   GPS


  //Enable interrupts based on calibrated threshold values
  //mag.enableInterrupt(zThreshold);
  // it turns out the break out board LSM303DLHC do not  interupt on magnetic sensor.
  // the LSM303C does.
  // temperature and huminity sensor Am2320; it uses I2C bus
   Serial.println("Adafruit AM2320 start");
   am2320.begin();

/*
 * --------------------------------------------------------------------
 */

// for ultrasonic distance sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

// start rf
  delay(500);
  if (!rf95.init())
  {
    Serial.println("RF init failed");
  }
  else
  {
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    Serial.println("RF95 ParkMon(sender) start...");
  } 

  //Enable interrupt pin
//  attachInterrupt(digitalPinToInterrupt(interruptPin), collectData, FALLING);

 } //setup()
 
void loop() {
/*
 * ----------------------------------------------------
 */

   CollectInfo();

  if ( in_backup_mode == 0)
  {
      temperature = am2320.readTemperature();
      humidity = am2320.readHumidity();
     
      RfSendDataToServer(3);
  }
  if ( loopCount == 720 )
  {
      temperature = am2320.readTemperature();
      humidity = am2320.readHumidity();
     
      RfSendDataToServer(2);
  }
 //SetAlarm_RTT(180);
//  Serial.print("Time: ");
//  digitprint(rtc.getHours(), 2);
//  Serial.print(":");
//  digitprint(rtc.getMinutes(), 2);
//  Serial.print(":");
//  digitprint(rtc.getSeconds(), 2);
//  Serial.println("");
    Serial.print("wakeup in sec: ");
    Serial.println( ALARM_in_SECONDS_default);

    delay(2000); // to finish RF  etc
//    digitalWrite(enRFPin, LOW);  // disable RF

    
    if (( reset_mode != 1 ) || (GPBR->SYS_GPBR[0] < 10) ) // use 10 sample to demo backup mode for now
    {
      NVIC_EnableIRQ        ( RTT_IRQn ); 
     // pmc_enable_backupmode();
      //to enter backup mode in alternate way  
      SUPC->SUPC_CR = SUPC_CR_VROFF_STOP_VREG | SUPC_CR_KEY(SUPC_KEY); 
    }
    else
    {
        int ctrl;
        ctrl = SysTick->CTRL;
        SysTick->CTRL = (ctrl & ~3);  // clear TICKINT & ENABLE
    
        pmc_enable_waitmode();
    
        SysTick->CTRL = (ctrl | 3);  // re-set TICKINT & ENABLE
        delay(1000);
    }
 loopCount++;
  
} //loop
int buff[10] ={0};

void   CollectInfo()
{
 
  unsigned int ctrl;
    Serial.print("procesMag ");
    GPBR->SYS_GPBR[0] += 1;
    Serial.print("in while : ");
    Serial.println(reset_mode );
//    if ( ( reset_mode != 1 ) && (GPBR->SYS_GPBR[0] > 10)) {
 // to handle not make it a as "dead brick"
  //  }
    
    mag.readMag();
    Serial.print("Mz: ");
//    Serial.print("Mz, x ,y: ");
    Serial.println(mag.m.z);
//    Serial.print(mag.m.z);
//    Serial.print(", Mx: ");
//    Serial.print(mag.m.x);
//    Serial.print(", My: ");
//    Serial.println(mag.m.y);
  

//only read prox if magnetic sensor see different value.
//    if ( (mag.m.z > zThreshold )
//    if ( ( abs(mag.m.z) > abs(zThreshold) )
      {
        MeasureDistance();
      }
    if ((char) GPBR->SYS_GPBR[1] !=  slotOccupied[0] )
    {
      GPBR->SYS_GPBR[1] = slotOccupied[0];
      RfSendDataToServer(1);
    }

} 


void GPSinfo() {

  int i = 0;
  Serial.print("Getting Location... ");
  uint32_t timer = millis();

  char c = GPS.read();
  while( millis() - timer < 20000)
  {
    if (GPSSerial.available()) {
      c = GPS.read();
//      Serial.write(c);
   }
   // make sure to a few data line, so at least one is complete
   if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))
        continue;
      else
      {
        i++;
        if( i > 3 )
          break;        
      }
    }
 
 }  // for loop


  // sync to gps time
  rtc.setDate(GPS.day, GPS.month, ( 2000 + GPS.year));
  rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);

 
  Serial.print("Location: ");
  sprintf(gpsString,"lat=%10.4f%c,lon=%11.4f%c",GPS.latitude,GPS.lat,GPS.longitude,GPS.lon);
  Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
  Serial.print(", ");
  Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
 
  Serial.println(gpsString);
  pinMode(enGPSPin, OUTPUT);
  digitalWrite(enGPSPin, LOW);  // disable GPS now, save power
}

/* 
 *  the following two function is from web library example.
 *  No use in our logic operation. but used for debugging  print.
 */

void digitprint(int value, int lenght){
  for (int i = 0; i < (lenght - numdigits(value)); i++){
    Serial.print("0");
  }
  Serial.print(value);
}

int numdigits(int i){
  int digits;
  if (i < 10)
    digits = 1;
  else
    digits = (int)(log10((double)i)) + 1;
  return digits;
}

void MeasureDistance() {
  // Clears the trigPin
  int i, okKont = 0;
  uint8_t kontOut = 0;
  float sum = 0.0;
  for ( i = 0; i < 3; i++)
  {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
    
      // Calculating the distance
       distance = (duration * 0.0343) *0.5; // Distance with Cm, use multiplication instead of division
    
       if (distance >= 150 || distance <= 0){
          kontOut++;
       }
       else {
          sum += distance;
          okKont++;
          if( okKont == 2) break;
       }
     delay(300);  
  }

  
  if (kontOut != 3) 
  {
    distance = sum/(okKont);
    if ( (distance > minDistance ) && (distance < maxDistance) )
    {
       carDetected = true;
       Serial.println("Car detected");
     //  Serial.print(distance);
     //   Serial.println(" cm");
      slotOccupied[0] = 'O';

    }
      else {
      slotOccupied[0] = 'E';
      carDetected = false;
      Serial.println("No car detected");
   }
 }
 else {
    carDetected = false;
    slotOccupied[0] = 'E';
    Serial.println("No car detected");
 }
}

#define AVERAGE_SAMPLES 150

void calibration() {
  Serial.println("Calibrating... ");
  int16_t sign = 1;
  int16_t magz[150];
  int16_t magx[150];
  int16_t magy[150];
  long sumz, sumdiffz, stdvz, avgz;
  long sumx, sumdiffx, stdvx, avgx;
  long sumy, sumdiffy, stdvy, avgy;
  sumz = 0;
  sumdiffz = 0;
  stdvz = 0;
  avgz = 0;
  
  sumx = 0;
  sumdiffx = 0;
  stdvx = 0;
  avgx = 0;
  
  sumy = 0;
  sumdiffy = 0;
  stdvy = 0;
  avgy = 0;
  
  for (int i = 0; i < 150; i++) {
    mag.readMag();
    magz[i] = mag.m.z;
    magx[i] = mag.m.x;
    magy[i] = mag.m.y;
    sumz += (long)mag.m.z;
    sumx += (long)mag.m.x;
    sumy += (long)mag.m.y;
//    Serial.print("Z value: ");
//    Serial.print(magz[i]);
//    Serial.println(magz[i]);
//    Serial.print(", X value: ");
//    Serial.print(magx[i]);
//    Serial.print(", Y value: ");
//    Serial.println(magy[i]);
    delay(100);

  }
  avgz = (sumz / 150);
  avgx = (sumx / 150);
  avgy = (sumy / 150);
//  Serial.print("Average Z: " );
//  Serial.print(avgz);
// // Serial.println(avgz);
//  Serial.print("; Average X: " );
//  Serial.print(avgx);
//  Serial.print("; Average Y: " );
//  Serial.println(avgy);
  for (int i = 0; i < 150; i++) {
    sumdiffz += sq(magz[i] - avgz);
    sumdiffx += sq(magx[i] - avgx);
    sumdiffy += sq(magy[i] - avgy);
  }
  stdvz = sqrt(sumdiffz / 150);
//    Serial.print("stdvz: ");
//    Serial.println(stdvz);

  if ( avgz < 0)
  {
   sign = -1;
  }
    zThreshold = (avgz) + sign*stdvz * 5;

//    Serial.print("zThreshold: ");
//    Serial.println(zThreshold);

  stdvx = sqrt(sumdiffx / 150);
//    Serial.print("stdvx: ");
//    Serial.println(stdvx);
// 
     sign = 1;
  if ( avgx < 0)
  {
   sign = -1;
  }
  xThreshold = (avgx) + sign*stdvx * 5;
//    Serial.print("xThreshold: ");
//    Serial.println(xThreshold);

  stdvy = sqrt(sumdiffy / 150);
    //Serial.print("stdvy: ");
//    Serial.println(stdvy);
  
    sign = 1;
  if ( avgy < 0)
  {
   sign = -1;
  }
  yThreshold = (avgy) + sign*stdvy * 5;
//    Serial.print("yThreshold: ");
//    Serial.println(yThreshold);

    
  Serial.println("Calibration Finished");
  Serial.println();

}

void RfSendDataToServer(int code)
{
      count++;

  switch ( code )
  {
    case  1:  
      sprintf((char*)tx_data,"ID=%s,P=%s\n",pm_id,slotOccupied);
      break;
    case  2:   
      sprintf((char*)tx_data,"ID=%s,P=%s,T=%6.2f,Hum=%5.2f;\n",pm_id,slotOccupied,temperature, humidity);
      break;
    case  3:
      sprintf((char*)tx_data,"ID=%s,P=%s,T=%6.2f,Hum=%5.2f;%s\n",pm_id,slotOccupied,temperature, humidity, gpsString);
      break;
    default:
      break; 
  
  }
  Serial.println((char*)tx_data);

  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  rf95.send(tx_data, sizeof(tx_data));
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("got reply: ");
//      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf95_server running?");
  }
  delay(1000);
//  delay(400);
}

void SetAlarm_RTT( int in_sec)
{
    RTT->RTT_AR = RTT_AR_ALMV(in_sec);  
}
void Init_RTT()
{
    RTT->RTT_MR = RTT_MR_RTTRST
                | RTT_MR_RTPRES (0x8000)  /* prescaler clock T = 1 second = 32k/32k */
                | RTT_MR_ALMIEN;  
               
    RTT->RTT_AR = RTT_AR_ALMV(ALARM_in_SECONDS_default); 
               
    NVIC_DisableIRQ       ( RTT_IRQn    ); 
    NVIC_ClearPendingIRQ  ( RTT_IRQn    ); 
    NVIC_SetPriority      ( RTT_IRQn, 0 ); 
    NVIC_EnableIRQ        ( RTT_IRQn    ); 
    SUPC->SUPC_WUMR |= SUPC_WUMR_RTTEN;  // enable wakeup by RTT
}
void RTT_Handler () 
{
   wakeup_sr  =   RTT->RTT_SR;
}
