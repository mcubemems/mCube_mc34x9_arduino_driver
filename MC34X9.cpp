/******************************************************************************

   Copyright (c) 2020 mCube, Inc.  All rights reserved.

   This source is subject to the mCube Software License.
   This software is protected by Copyright and the information and source code
   contained herein is confidential. The software including the source code
   may not be copied and the information contained herein may not be used or
   disclosed except with the written permission of mCube Inc.

   All other rights reserved.
 *****************************************************************************/

/**
   @file    MC34X9.c
   @author  mCube
   @date    13 January 2020
   @brief   Driver interface header file for accelerometer mc34X9 series.
   @see     http://www.mcubemems.com
*/

#include "MC34X9.h"

#define MC34X9_CFG_MODE_DEFAULT                 MC34X9_MODE_STANDBY
#define MC34X9_CFG_SAMPLE_RATE_DEFAULT    MC34X9_SR_DEFAULT_1000Hz
#define MC34X9_CFG_RANGE_DEFAULT                MC34X9_RANGE_8G

#define mcube_printf(arg) \
  Serial.println(arg);

#define mcube_printf_number(arg) \
  Serial.println(arg, HEX);

uint8_t CfgRange, CfgFifo;

uint8_t MC34X9::readRegister8(uint8_t reg) {
  uint8_t value;
  mcube_read_regs(M_bSpi, M_chip_select, reg, &value, 1);
  return value;
}

void MC34X9::writeRegister8(uint8_t reg, uint8_t value) {
  mcube_write_regs(M_bSpi, M_chip_select, reg, &value, 1);
  return;
}

//Initialize the MC34X9 sensor and set as the default configuration
bool MC34X9::start(bool bSpi, uint8_t chip_select)
{
  /** 0 = SPI, 1 = I2C */
  M_bSpi = bSpi;
  M_chip_select = chip_select;
  if (!bSpi) {
    mcube_printf("SPI mode");
    // Chip select pin
    pinMode(chip_select, OUTPUT);
    digitalWrite(chip_select, HIGH);
    // Initialize SPI
    m_drv_spi_init(E_M_DRV_INTERFACE_SPIMODE_HS);
  } else {
    mcube_printf("I2C mode");
    // Initialize I2C
    m_drv_i2c_init();
  }

  //Init Reset
  reset();
  SetMode(MC34X9_MODE_STANDBY);

  /* Check I2C connection */
  uint8_t id = readRegister8(MC34X9_REG_PROD);
  if (id != MC34X9_CHIP_ID)
  {
    /* No MC34X9 detected ... return false */
    mcube_printf("No MC34X9 detected!");
    mcube_printf("Chip ID: ");
    mcube_printf_number(id);
    return false;
  }

  //Range: 8g
  SetRangeCtrl(MC34X9_CFG_RANGE_DEFAULT);
  //Sampling Rate: 50Hz by default
  SetSampleRate(MC34X9_CFG_SAMPLE_RATE_DEFAULT);
  //Mode: Active
  SetMode(MC34X9_MODE_CWAKE);

  delay(50);

  return true;
}

void MC34X9::wake()
{
  //Set mode as wake
  SetMode(MC34X9_MODE_CWAKE);
}

void MC34X9::stop()
{
  //Set mode as Sleep
  SetMode(MC34X9_MODE_STANDBY);
}

//Initial reset
void MC34X9::reset()
{
  // Stand by mode
  writeRegister8(MC34X9_REG_MODE, MC34X9_MODE_STANDBY);

  delay(10);

  // power-on-reset
  writeRegister8(0x1c, 0x40);

  delay(50);

  // Disable interrupt
  writeRegister8(0x06, 0x00);
  delay(10);
  // 1.00x Aanalog Gain
  writeRegister8(0x2B, 0x00);
  delay(10);
  // DCM disable
  writeRegister8(0x15, 0x00);

  delay(50);
}

//Set the operation mode
void MC34X9::SetMode(MC34X9_mode_t mode)
{
  uint8_t value;

  value = readRegister8(MC34X9_REG_MODE);
  value &= 0b11110000;
  value |= mode;

  writeRegister8(MC34X9_REG_MODE, value);
}

//Set the range control
void MC34X9::SetRangeCtrl(MC34X9_range_t range)
{
  uint8_t value;
  CfgRange = range;
  SetMode(MC34X9_MODE_STANDBY);
  value = readRegister8(MC34X9_REG_RANGE_C);
  value &= 0b00000111;
  value |= (range << 4) & 0x70;
  writeRegister8(MC34X9_REG_RANGE_C, value);
}

//Set the sampling rate
void MC34X9::SetSampleRate(MC34X9_sr_t sample_rate)
{
  uint8_t value;
  SetMode(MC34X9_MODE_STANDBY);
  value = readRegister8(MC34X9_REG_SR);
  value &= 0b00000000;
  value |= sample_rate;
  writeRegister8(MC34X9_REG_SR, value);
}

// Set Motion feature
void MC34X9::SetMotionCtrl(bool tilt_ctrl,
                           bool flip_ctl,
                           bool anym_ctl,
                           bool shake_ctl,
                           bool tilt_35_ctl) {
  uint8_t CfgMotion = 0;

  if (tilt_ctrl || flip_ctl) {
    _M_DRV_MC34X6_SetTilt_Flip();
    CfgMotion |= (((tilt_ctrl || flip_ctl) & 0x01) << MC34X9_TILT_FEAT);
  }

  if (anym_ctl) {
    _M_DRV_MC34X6_SetAnym();
    CfgMotion |= ((anym_ctl & 0x01) << MC34X9_ANYM_FEAT);
  }

  if (shake_ctl) {
    _M_DRV_MC34X6_SetShake();
    // Also enable anyMotion feature
    CfgMotion |= ((shake_ctl & 0x01) << MC34X9_ANYM_FEAT) | ((shake_ctl & 0x01) << MC34X9_SHAKE_FEAT);
  }

  if (tilt_35_ctl) {
    _M_DRV_MC34X6_SetTilt35();
    // Also enable anyMotion feature
    CfgMotion |= ((tilt_35_ctl & 0x01) << MC34X9_ANYM_FEAT) | ((tilt_35_ctl & 0x01) << MC34X9_TILT35_FEAT);
  }

  writeRegister8(MC34X9_REG_MOTION_CTRL, CfgMotion);
}

//Set FIFO feature
void MC34X9::SetFIFOCtrl(MC34X9_fifo_ctl_t fifo_ctl,
                         MC34X9_fifo_mode_t fifo_mode,
                         uint8_t fifo_thr)
{
  if (fifo_thr > 31)  //maximum threshold
    fifo_thr = 31;

  SetMode(MC34X9_MODE_STANDBY);

  CfgFifo = (MC34X9_COMB_INT_ENABLE << 3) | ((fifo_ctl << 5) | (fifo_mode << 6)) ;

  writeRegister8(MC34X9_REG_FIFO_CTRL, CfgFifo);

  uint8_t CfgFifoThr = fifo_thr;
  writeRegister8(MC34X9_REG_FIFO_TH, CfgFifoThr);
}

void MC34X9::SetGerneralINTCtrl() {
  // Gerneral Interrupt setup
  uint8_t CfgGPIOINT = (((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 2) // int1
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 3)// int1
                        | ((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 6)// int2
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 7));// int2

  writeRegister8(MC34X9_REG_GPIO_CTRL, CfgGPIOINT);
}

//Set interrupt control register
void MC34X9::SetINTCtrl(bool tilt_int_ctrl,
                        bool flip_int_ctl,
                        bool anym_int_ctl,
                        bool shake_int_ctl,
                        bool tilt_35_int_ctl)
{
  SetMode(MC34X9_MODE_STANDBY);

  uint8_t CfgINT = (((tilt_int_ctrl & 0x01) << 0)
                    | ((flip_int_ctl & 0x01) << 1)
                    | ((anym_int_ctl & 0x01) << 2)
                    | ((shake_int_ctl & 0x01) << 3)
                    | ((tilt_35_int_ctl & 0x01) << 4)
                    | ((MC34X9_AUTO_CLR_ENABLE & 0x01) << 6));
  writeRegister8(MC34X9_REG_INTR_CTRL, CfgINT);

  SetGerneralINTCtrl();
}

//Set FIFO interrupt control register
void MC34X9::SetFIFOINTCtrl(bool fifo_empty_int_ctl,
                            bool fifo_full_int_ctl,
                            bool fifo_thr_int_ctl)
{
  SetMode(MC34X9_MODE_STANDBY);

  CfgFifo = CfgFifo
            | (((fifo_empty_int_ctl & 0x01) << 0)
               | ((fifo_full_int_ctl & 0x01) << 1)
               | ((fifo_thr_int_ctl & 0x01) << 2));

  writeRegister8(MC34X9_REG_FIFO_CTRL, CfgFifo);

  SetGerneralINTCtrl();
}

//Interrupt handler (clear interrupt flag)
void MC34X9::INTHandler(MC34X9_interrupt_event_t *ptINT_Event)
{
  uint8_t value;

  value = readRegister8(MC34X9_REG_INTR_STAT);

  ptINT_Event->bTILT           = ((value >> 0) & 0x01);
  ptINT_Event->bFLIP           = ((value >> 1) & 0x01);
  ptINT_Event->bANYM           = ((value >> 2) & 0x01);
  ptINT_Event->bSHAKE          = ((value >> 3) & 0x01);
  ptINT_Event->bTILT_35        = ((value >> 4) & 0x01);

  value &= 0x40;
  writeRegister8(MC34X9_REG_INTR_STAT, value);
}

//FIFO Interrupt handler (clear interrupt flag)
void MC34X9::FIFOINTHandler(MC34X9_fifo_interrupt_event_t *ptFIFO_INT_Event)
{
  uint8_t value;

  value = readRegister8(MC34X9_REG_FIFO_INTR);

  ptFIFO_INT_Event->bFIFO_EMPTY           = ((value >> 0) & 0x01);
  ptFIFO_INT_Event->bFIFO_FULL            = ((value >> 1) & 0x01);
  ptFIFO_INT_Event->bFIFO_THRESH          = ((value >> 2) & 0x01);
}

//Get the range control
MC34X9_range_t MC34X9::GetRangeCtrl(void)
{
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister8(MC34X9_REG_RANGE_C);
  mcube_printf("In GetRangeCtrl(): ");
  mcube_printf_number(value);
  value &= 0x70;
  return (MC34X9_range_t) (value >> 4);
}

//Get the output sampling rate
MC34X9_sr_t MC34X9::GetSampleRate(void)
{
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister8(MC34X9_REG_SR);
  mcube_printf("In GetCWakeSampleRate(): ");
  mcube_printf_number(value);
  value &= 0b00011111;
  return (MC34X9_sr_t) (value);
}

//Is FIFO empty
bool MC34X9::IsFIFOEmpty(void)
{
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister8(MC34X9_REG_FIFO_STAT);
  value &= 0x01;
  //Serial.println("FIFO_Status");
  //Serial.println(value, HEX);

  if (value ^ 0x01)
    return false;	//Not empty
  else {
    return true;  //Is empty
  }
}

//Read the raw counts and SI units measurement data
MC34X9_acc_t MC34X9::readRawAccel(void)
{
  //{2g, 4g, 8g, 16g, 12g}
  float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
  // 16bit
  float faResolution = 32768.0f;

  byte rawData[6];
  // Read the six raw data registers into data array
  mcube_read_regs(M_bSpi, M_chip_select, MC34X9_REG_XOUT_LSB, rawData, 6);
  x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
  y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
  z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

  AccRaw.XAxis = (short) (x);
  AccRaw.YAxis = (short) (y);
  AccRaw.ZAxis = (short) (z);
  AccRaw.XAxis_g = (float) (x) / faResolution * faRange[CfgRange];
  AccRaw.YAxis_g = (float) (y) / faResolution * faRange[CfgRange];
  AccRaw.ZAxis_g = (float) (z) / faResolution * faRange[CfgRange];

  return AccRaw;
}

// ***BUS***
/** I2C init function */
int m_drv_i2c_init(void)
{
  Wire.begin();
  return 0;
}

/** SPI init function */
int m_drv_spi_init(e_m_drv_interface_spimode_t spi_hs_mode)
{
  //Set active-low CS low to start the SPI cycle
  SPI.begin();
  const int SPI_SPEED_4M = 4000000;
  SPI.beginTransaction(SPISettings(SPI_SPEED_4M, MSBFIRST, SPI_MODE3));
  return 0;
}

/** I2C/SPI read function */
/** bSpi : I2C/SPI bus selection.        SPI: 0,       I2C: 1           */
/** chip_select : Chip selection.        SPI: CS pins, I2C: I2C address */
/** reg : Sensor registers. */
/** value : read value.*/
/** size : data length */
uint8_t mcube_read_regs(bool bSpi, uint8_t chip_select, uint8_t reg,  \
                        uint8_t * value, uint8_t size)
{
  for (uint8_t i = 0; i < size; i++) {
    value[i] = _readRegister8(bSpi, chip_select, reg + i);
  }
  return 0;
}

/** I2C/SPI write function */
/** bSpi : I2C/SPI bus selection.        SPI: 0,       I2C: 1           */
/** chip_select : Chip selection.        SPI: CS pins, I2C: I2C address */
/** reg : Sensor registers. */
/** value : Write value.*/
/** size : data length */
uint8_t mcube_write_regs(bool bSpi, uint8_t chip_select, uint8_t reg,       \
                         uint8_t *value, uint8_t size)
{
  for (uint8_t i = 0; i < size; i++) {
    _writeRegister8(bSpi, chip_select, reg + i, value[i]);
  }
  return 0;
}

// Read 8-bit from register
uint8_t _readRegister8(bool bSpi, uint8_t chip_select, uint8_t reg)
{
  uint8_t value;
  /** 0 = SPI, 1 = I2C */
  if (!bSpi) { //Reads an 8-bit register with the SPI port.
    /** SPI read function */
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chip_select, LOW);
    //Send the register address
    SPI.transfer(reg | 0x80);
    SPI.transfer(0x00);
    //Read the value from the register
    value = SPI.transfer(0x00);
    //Raise CS
    digitalWrite(chip_select, HIGH);
  } else { //Reads an 8-bit register with the SPI port.
    /** I2C read function */
    Wire.requestFrom(chip_select, 1, reg, 1, true);
    value = Wire.read();
  }

  return value;
}

// Write 8-bit to register
void _writeRegister8(bool bSpi, uint8_t chip_select, uint8_t reg, uint8_t value)
{
  /** 0 = SPI, 1 = I2C */
  if (!bSpi) {
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chip_select, LOW);
    //Send the register address
    SPI.transfer(reg);
    //Send value to write into register
    SPI.transfer(value);
    //Raise CS
    digitalWrite(chip_select, HIGH);
  } else {
    Wire.beginTransmission(chip_select);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }
  return;
}
// ***MC34X9 dirver motion part***
void M_DRV_MC34X6_SetTFThreshold(uint16_t threshold) {
  uint8_t _bFTThr[2] = {0};

  _bFTThr[0] = (threshold & 0x00ff);
  _bFTThr[1] = ((threshold & 0x7f00) >> 8 );


  // set threshold
  MC34X9_acc_writeRegister8(MC34X9_REG_TF_THRESH_LSB, _bFTThr[0]);
  MC34X9_acc_writeRegister8(MC34X9_REG_TF_THRESH_MSB, _bFTThr[1]);
}

void M_DRV_MC34X6_SetTFDebounce(uint8_t debounce) {
  // set debounce
  MC34X9_acc_writeRegister8(MC34X9_REG_TF_DB, debounce);
}

void M_DRV_MC34X6_SetANYMThreshold(uint16_t threshold) {
  uint8_t _bANYMThr[2] = {0};

  _bANYMThr[0] = (threshold & 0x00ff);
  _bANYMThr[1] = ((threshold & 0x7f00) >> 8 );

  // set threshold
  MC34X9_acc_writeRegister8(MC34X9_REG_AM_THRESH_LSB, _bANYMThr[0]);
  MC34X9_acc_writeRegister8(MC34X9_REG_AM_THRESH_MSB, _bANYMThr[1]);
}

void M_DRV_MC34X6_SetANYMDebounce(uint8_t debounce) {
  MC34X9_acc_writeRegister8(MC34X9_REG_AM_DB, debounce);
}

void M_DRV_MC34X6_SetShakeThreshold(uint16_t threshold) {
  uint8_t _bSHKThr[2] = {0};

  _bSHKThr[0] = (threshold & 0x00ff);
  _bSHKThr[1] = ((threshold & 0xff00) >> 8 );

  // set threshold
  MC34X9_acc_writeRegister8(MC34X9_REG_SHK_THRESH_LSB, _bSHKThr[0]);
  MC34X9_acc_writeRegister8(MC34X9_REG_SHK_THRESH_MSB, _bSHKThr[1]);
}

void M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(uint16_t threshold, uint8_t shakeCount) {

  uint8_t _bSHKP2PDuration[2] = {0};

  _bSHKP2PDuration[0] = (threshold & 0x00ff);
  _bSHKP2PDuration[1] = ((threshold & 0x0f00) >> 8);
  _bSHKP2PDuration[1] |= ((shakeCount & 0x7) << 4);

  // set peak to peak duration and count
  MC34X9_acc_writeRegister8(MC34X9_REG_PK_P2P_DUR_THRESH_LSB, _bSHKP2PDuration[0]);
  MC34X9_acc_writeRegister8(MC34X9_REG_PK_P2P_DUR_THRESH_MSB, _bSHKP2PDuration[1]);
}

void M_DRV_MC34X6_SetTILT35Threshold(uint16_t threshold) {
  M_DRV_MC34X6_SetTFThreshold(threshold);
}

void M_DRV_MC34X6_SetTILT35Timer(uint8_t timer) {
  uint8_t value;

  value = MC34X9_acc_readRegister8(MC34X9_REG_TIMER_CTRL);
  value &= 0b11111000;
  value |= MC34X9_TILT35_2p0;

  MC34X9_acc_writeRegister8(MC34X9_REG_TIMER_CTRL, timer);
}

// Tilt & Flip
void _M_DRV_MC34X6_SetTilt_Flip() {
  // set threshold
  M_DRV_MC34X6_SetTFThreshold(s_bCfgFTThr);
  // set debounce
  M_DRV_MC34X6_SetTFDebounce(s_bCfgFTDebounce);
  return;
}

// AnyMotion
void _M_DRV_MC34X6_SetAnym() {
  // set threshold
  M_DRV_MC34X6_SetANYMThreshold(s_bCfgANYMThr);

  // set debounce
  M_DRV_MC34X6_SetANYMDebounce(s_bCfgANYMDebounce);
  return;
}

// Shake
void _M_DRV_MC34X6_SetShake() {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym();

  // Config shake
  // set threshold
  M_DRV_MC34X6_SetShakeThreshold(s_bCfgShakeThr);

  // set peak to peak duration and count
  M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(s_bCfgShakeP2PDuration, s_bCfgShakeCount);
  return;
}

// Tilt 35
void _M_DRV_MC34X6_SetTilt35() {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym();

  // Config Tilt35
  // set threshold
  M_DRV_MC34X6_SetTILT35Threshold(s_bCfgTILT35Thr);

  //set timer
  M_DRV_MC34X6_SetTILT35Timer(MC34X9_TILT35_2p0);
  return;
}
