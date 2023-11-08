/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file BasicConnectivityTest.ino
 * Use this to test connectivity with your DW1000 from Arduino.
 * It performs an arbitrary setup of the chip and prints some information.
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - make real check of connection (e.g. change some values on DW1000 and verify)
 */

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

void setup() {
  // DEBUG monitoring
  Serial.begin(9600);
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDeviceAddress(5);    //sets short address to 05  (5 in hex)
  DW1000.setNetworkId(10);       //sets network ID to 0A (10 in hex)
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // wait a bit
  delay(1000);
}

void loop() {
  //Standard loop for printing out device info. We will manipulate it below
  delay(10000);
  printDeviceInfo();
  //end standard loop

  //First, we manipulate the EUI (Extended Unique Identifier)
  DW1000.setEUI("82:17:5B:D5:A9:9A:E2:9C");
  Serial.println("   === EUI changed to ...9C ===");
  printDeviceInfo();
  delay(1000);
  //Second manipulation
  DW1000.setEUI("17:54:3A:2A:66:1C:FF:3A");
  Serial.println("   === EUI changed to ...3A ===");
  printDeviceInfo();
  delay(1000);

  //Now we adjust the Network ID and Device Address.
  DW1000.setDeviceAddress(8);
  DW1000.setNetworkId(15);
  DW1000.writeNetworkIdAndDeviceAddress();   //set functions do NOT write to DW1000, need this function separately
  Serial.println("   === New Device Address = 08 , New Network ID = 0F ===");
  printDeviceInfo();
  delay(1000);

  //Now we mess with our channel setting
  byte chan = 0x01;
  DW1000.setChannel(chan);     //set from channel 5 to channel 1
  DW1000.writeChannelControlRegister();
  Serial.println("   === Channel set from 5 to 1 === ");
  printDeviceInfo();
  delay(1000);

  //Now, let's change modes from Longdata_Range_Lowpower to Longdata_Range_Accuracy (110kbps, prf 64mhz, preamble 2048)
  DW1000.setNewDeviceSettings_Kuipers();
  Serial.println("   === Mode changed to Shortdata_Fast_Accuracy ===");
  printDeviceInfo();
  delay(10000);
  
}

void printDeviceInfo()
{
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Extended Unique ID (EUI): "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
}
