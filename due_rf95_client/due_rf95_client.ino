// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

Adafruit_AM2320 am2320 = Adafruit_AM2320();

// Singleton instance of the radio driver
RH_RF95 rf95;
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W
//RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95 

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

void setup() 
{
  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
//  pinMode(4, OUTPUT);
//  digitalWrite(4, HIGH);

  Serial.begin(115200);
  while (!Serial) ; // Wait for serial port to be available
  // temperature and huminity sensor Am2320
    Serial.println("Adafruit AM2320 start");
  am2320.begin();

  if (!rf95.init())
  {
    Serial.println("init failed");
  }
  else
  {
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    Serial.println("RF95 client(sender) start");
  }
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
//  driver.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true. 
  // Failure to do that will result in extremely low transmit powers.
//  driver.setTxPower(14, true);
}

float temperature, humidity;
uint8_t  tx_data[240];
int   num_bytes;
void loop()
{
  temperature = am2320.readTemperature();
  humidity = am2320.readHumidity();
//  Serial.print("Temp: "); Serial.println(am2320.readTemperature());
//  Serial.print("Hum: "); Serial.println(am2320.readHumidity());
//  Serial.print("Temp: "); Serial.println(temperature);
//  Serial.print("Hum: "); Serial.println(humidity);

  sprintf((char*)tx_data,"Bytes=%3d,T=%6.2f,Hum=%5.2f;\n",30,temperature, humidity);
  num_bytes = strlen((const char*)tx_data);
  sprintf((char*)tx_data,"Bytes=%3d,T=%6.2f,Hum=%5.2f;\n",num_bytes,temperature, humidity);
  //Serial.println("1234567890123456789012345678901234567890");
  //Serial.println((char*)tx_data);
 
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
//  uint8_t data[] = "Hello World!";
//  rf95.send(data, sizeof(data));
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
      Serial.println((char*)buf);
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
  delay(3000);
//  delay(400);
}