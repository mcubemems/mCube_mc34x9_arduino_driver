/*****************************************************************************

   Copyright (c) 2020 mCube, Inc.  All rights reserved.

   This source is subject to the mCube Software License.
   This software is protected by Copyright and the information and source code
   contained herein is confidential. The software including the source code
   may not be copied and the information contained herein may not be used or
   disclosed except with the written permission of mCube Inc.

   All other rights reserved.
 *****************************************************************************/

/**
   @file    MC34X9_demo.ino
   @author  mCube
   @date    13 January 2020
   @brief   Arduino example code for accelerometer mc34X9 series.
   @see     http://www.mcubemems.com
*/

#include <MC34X9.h>

/*** bSpi: I2C/SPI bus selection.  SPI: 0, I2C: 1 ***/
const uint8_t bSpi = 1;

// Chip Select & Address
uint8_t chipSelect = 0;
const uint8_t SPIChipSelectPin = 10; // SPI chipSelectPin
const uint8_t I2CAddress = 0x4c; // I2C address

// Arduino interrupt pin
const int INTERRUPT_PIN = 8;

// *** FIFO control ***/
int FIFO_THRE_SIZE = 30;
// FIFO Interrupt
const bool enableFifoThrINT = false;
// For FIFO feature, enable FIFO interrupt will automatically enable FIFO feature
bool enableFIFO = false;

// *** Motion control ***/
// Enabling motion feature below also enables corresponded motion interrupt
const bool enableTILT = false;
const bool enableFLIP = false;
const bool enableANYM = true;
const bool enableSHAKE = false;
const bool enableTILT_35 = false;

// Determine if enable interrupt
const bool interruptEnabled = enableFifoThrINT || enableTILT || enableFLIP || enableANYM || enableSHAKE || enableTILT_35;

MC34X9 MC34X9_acc = MC34X9();

void setup()
{
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("mCube Accelerometer MC34X9:");

  // Define SPI chip select or I2C address
  if (!bSpi) {
    chipSelect = SPIChipSelectPin;
  } else {
    chipSelect = I2CAddress;
  }

  // Init MC34X9 Object
  if (!MC34X9_acc.start(bSpi, chipSelect)) {
    // Invalid chip ID
    // Block here
    while (true);
    return;
  }

  // Check chip setup
  checkRange();
  checkSamplingRate();
  Serial.println();

  //Test read
  readAndOutput();

  // Enable feature & interrput
  enableFIFO = enableFIFO || enableFifoThrINT;
  if (enableFIFO) {
    sensorFIFO();
  }
  if (enableTILT || enableFLIP || enableANYM || enableSHAKE || enableTILT_35) {
    // Checker: These modes can only be enabled separately.
    int counter = 0;
    if (enableTILT)
      counter++;
    if (enableFLIP)
      counter++;
    if (enableANYM)
      counter++;
    if (enableSHAKE)
      counter++;
    if (enableTILT_35)
      counter++;
    if (counter > 1) {
      // Detected: These modes can only be enabled separately.
      Serial.println("Error: Enable too many motion feature.");
      Serial.println("Error: These modes can only be enabled separately.");
      // Block here
      while (true);
    }
    sensorMotion();
  }
}

void sensorFIFO()
{
  //Enable FIFO and interrupt
  MC34X9_acc.stop();
  MC34X9_acc.SetSampleRate(MC34X9_SR_50Hz);
  MC34X9_acc.SetFIFOCtrl(MC34X9_FIFO_CTL_ENABLE, MC34X9_FIFO_MODE_WATERMARK, FIFO_THRE_SIZE);
  MC34X9_acc.SetFIFOINTCtrl(false, false, enableFifoThrINT); //Enable FIFO threshold interrupt
  MC34X9_acc.wake();

  Serial.println("Sensor FIFO enable.");
}

void sensorMotion() {
  //Enable motion feature and motion interrupt
  MC34X9_acc.stop();
  MC34X9_acc.SetSampleRate(MC34X9_SR_DEFAULT_1000Hz);
  MC34X9_acc.SetMotionCtrl(enableTILT, enableFLIP, enableANYM, enableSHAKE, enableTILT_35);
  MC34X9_acc.SetINTCtrl(enableTILT, enableFLIP, enableANYM, enableSHAKE, enableTILT_35);
  MC34X9_acc.wake();

  Serial.println("Sensor motion enable.");
}

void checkRange()
{
  switch (MC34X9_acc.GetRangeCtrl())
  {
    case MC34X9_RANGE_16G:
      Serial.println("Range: +/- 16 g");
      break;
    case MC34X9_RANGE_12G:
      Serial.println("Range: +/- 12 g");
      break;
    case MC34X9_RANGE_8G:
      Serial.println("Range: +/- 8 g");
      break;
    case MC34X9_RANGE_4G:
      Serial.println("Range: +/- 4 g");
      break;
    case MC34X9_RANGE_2G:
      Serial.println("Range: +/- 2 g");
      break;
    default:
      Serial.println("Range: +/- ?? g");
      break;
  }
}

void checkSamplingRate()
{
  Serial.println("Low Power Mode SR");
  switch (MC34X9_acc.GetSampleRate())
  {
    case MC34X9_SR_25Hz:
      Serial.println("Output Sampling Rate: 25 Hz");
      break;
    case MC34X9_SR_50Hz:
      Serial.println("Output Sampling Rate: 50 Hz");
      break;
    case MC34X9_SR_62_5Hz:
      Serial.println("Output Sampling Rate: 62.5 Hz");
      break;
    case MC34X9_SR_100Hz:
      Serial.println("Output Sampling Rate: 100 Hz");
      break;
    case MC34X9_SR_125Hz:
      Serial.println("Output Sampling Rate: 125 Hz");
      break;
    case MC34X9_SR_250Hz:
      Serial.println("Output Sampling Rate: 250 Hz");
      break;
    case MC34X9_SR_500Hz:
      Serial.println("Output Sampling Rate: 500 Hz");
      break;
    case MC34X9_SR_DEFAULT_1000Hz:
      Serial.println("Output Sampling Rate: 1000 Hz");
      break;
    default:
      Serial.println("Output Sampling Rate: ?? Hz");
      break;
  }
}

void readAndOutput() {
  // Read the raw sensor data count
  MC34X9_acc_t rawAccel = MC34X9_acc.readRawAccel();

  // Output Count
  Serial.print("X:\t"); Serial.print(rawAccel.XAxis); Serial.print("\t");
  Serial.print("Y:\t"); Serial.print(rawAccel.YAxis); Serial.print("\t");
  Serial.print("Z:\t"); Serial.print(rawAccel.ZAxis); Serial.print("\t");
  Serial.println("counts");

  // Display the results (acceleration is measured in m/s^2)
  Serial.print("X: \t"); Serial.print(rawAccel.XAxis_g); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(rawAccel.YAxis_g); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(rawAccel.ZAxis_g); Serial.print("\t");
  Serial.println("m/s^2");

  Serial.println("---------------------------------------------------------");
  return;
}

// Interrupt checker: read interrupt register and determine if interrupt happen
bool interruptChecker() {
  // Init interrupt table
  bool retCode = false;
  MC34X9_interrupt_event_t evt_mc34X9 = {0};
  MC34X9_fifo_interrupt_event_t fifo_evt_mc34X9 = {0};

  // Read interrupt table
  MC34X9_acc.FIFOINTHandler(&fifo_evt_mc34X9);
  MC34X9_acc.INTHandler(&evt_mc34X9);

  // Whether there is interrupt
  uint8_t* iter = (uint8_t*) &evt_mc34X9;
  for (int i = 0; i < sizeof (MC34X9_interrupt_event_t); i++, iter++) {
    if ((*iter) & 0x01)
      retCode = true;
  }
  iter = (uint8_t*) &fifo_evt_mc34X9;
  for (int i = 0; i < sizeof (MC34X9_fifo_interrupt_event_t); i++, iter++) {
    if ((*iter) & 0x01)
      retCode = true;
  }

  if (retCode)
    Serial.print("Get interrupt: ");
  if (enableFIFO) {
    if (fifo_evt_mc34X9.bFIFO_EMPTY) {
      Serial.print("FIFO empty. ");
    }
    if (fifo_evt_mc34X9.bFIFO_FULL) {
      Serial.print("FIFO full. ");
    }
    if (fifo_evt_mc34X9.bFIFO_THRESH) {
      Serial.print("FIFO threshold. ");
    }
  }
  if (evt_mc34X9.bTILT) {
    Serial.print("Tilt. ");
  }
  if (evt_mc34X9.bFLIP) {
    Serial.print("Flip. ");
  }
  if (evt_mc34X9.bANYM) {
    Serial.print("Any Motion. ");
  }
  if (evt_mc34X9.bSHAKE) {
    Serial.print("Shake. ");
  }
  if (evt_mc34X9.bTILT_35) {
    Serial.print("Tilt 35. ");
  }
  if (retCode)
    Serial.println();
  return retCode;
}

// Function for enabled interrupt mode
void interruptLoop() {
  if (interruptChecker()) {
    // When interrupt happen
    int count = 0;
    if (enableFIFO) {
      while (!(MC34X9_acc.IsFIFOEmpty()))
      {
        // Read and output all data in FIFO
        readAndOutput();
      }
    } else {
      readAndOutput();
    }
  }
  return;
}

// Arduino declare function
void loop()
{
  if (interruptEnabled) {
    interruptLoop();
    delay(1);
  } else {
    // without enable interrupt
    // read and output data periodically
    readAndOutput();
    delay(10);
  }
  return;
}
