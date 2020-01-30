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
   @file    MC34X9.h
   @author  mCube
   @date    13 January 2020
   @brief   Driver interface header file for accelerometer mc34X9 series.
   @see     http://www.mcubemems.com
*/

#ifndef MC34X9_h
#define MC34X9_h


/******************************************************************************
 *** INFORMATION
 *****************************************************************************/
#define M_DRV_MC34X9_VERSION    "1.0.0"

#include "Arduino.h"

#include <Wire.h> // I2C header file
#include <SPI.h> // SPI header file

//SPI pin definition
//SS  : SCS, Active-low CS�Xchip select
//MOSI: SDA ICSP-4, MOSI�Xmaster out slave in
//MISO: SDO ICSP-1, MISO�Xmaster in slave out
//SCK : SCL ICSP-3, SCK - SPI clock
// pins used for the connection with the sensor
// other information you can refer to the Arduino SPI library

/******************************************************************************
 *** CONFIGURATION
 *****************************************************************************/

/******************************************************************************
*** Motion threshold and debounce config
 *****************************************************************************/
#define s_bCfgFTThr               100
#define s_bCfgFTDebounce          50

#define s_bCfgANYMThr             200
#define s_bCfgANYMDebounce        100

#define s_bCfgShakeThr            300
#define s_bCfgShakeP2PDuration    10
#define s_bCfgShakeCount          1

#define s_bCfgTILT35Thr           20
#define s_bCfgTILT35Timer         MC34X9_TILT35_2p0

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC34X9_RETCODE_SUCCESS                 (0)
#define MC34X9_RETCODE_ERROR_BUS               (-1)
#define MC34X9_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC34X9_RETCODE_ERROR_STATUS            (-3)
#define MC34X9_RETCODE_ERROR_SETUP             (-4)
#define MC34X9_RETCODE_ERROR_GET_DATA          (-5)
#define MC34X9_RETCODE_ERROR_IDENTIFICATION    (-6)
#define MC34X9_RETCODE_ERROR_NO_DATA           (-7)
#define MC34X9_RETCODE_ERROR_WRONG_ARGUMENT    (-8)
#define MC34X9_FIFO_DEPTH                        32
#define MC34X9_REG_MAP_SIZE                      64

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define MC34X9_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define MC34X9_INTR_C_IAH_ACTIVE_LOW         (0x00)
#define MC34X9_INTR_C_IAH_ACTIVE_HIGH        (0x01)

#define MC34X9_AUTO_CLR_DISABLE              (0x00)
#define MC34X9_AUTO_CLR_ENABLE               (0x01)
/******************************************************************************
 *** Register Map
 *****************************************************************************/
#define MC34X9_REG_DEV_STAT         (0x05)
#define MC34X9_REG_INTR_CTRL        (0x06)
#define MC34X9_REG_MODE             (0x07)
#define MC34X9_REG_SR               (0x08)
#define MC34X9_REG_MOTION_CTRL      (0x09)
#define MC34X9_REG_FIFO_STAT        (0x0A)
#define MC34X9_REG_FIFO_RD_P        (0x0B)
#define MC34X9_REG_FIFO_WR_P        (0x0C)
#define MC34X9_REG_XOUT_LSB         (0x0D)
#define MC34X9_REG_XOUT_MSB         (0x0E)
#define MC34X9_REG_YOUT_LSB         (0x0F)
#define MC34X9_REG_YOUT_MSB         (0x10)
#define MC34X9_REG_ZOUT_LSB         (0x11)
#define MC34X9_REG_ZOUT_MSB         (0x12)
#define MC34X9_REG_STATUS           (0x13)
#define MC34X9_REG_INTR_STAT        (0x14)
#define MC34X9_REG_PROD             (0x18)
#define MC34X9_REG_RANGE_C          (0x20)
#define MC34X9_REG_XOFFL            (0x21)
#define MC34X9_REG_XOFFH            (0x22)
#define MC34X9_REG_YOFFL            (0x23)
#define MC34X9_REG_YOFFH            (0x24)
#define MC34X9_REG_ZOFFL            (0x25)
#define MC34X9_REG_ZOFFH            (0x26)
#define MC34X9_REG_XGAIN            (0x27)
#define MC34X9_REG_YGAIN            (0x28)
#define MC34X9_REG_ZGAIN            (0x29)
#define MC34X9_REG_FIFO_CTRL        (0x2D)
#define MC34X9_REG_FIFO_TH          (0x2E)
#define MC34X9_REG_FIFO_INTR        (0x2F)
#define MC34X9_REG_FIFO_CTRL_SR2    (0x30)
#define MC34X9_REG_COMM_CTRL        (0x31)
#define MC34X9_REG_GPIO_CTRL        (0x33)
#define MC34X9_REG_TF_THRESH_LSB    (0x40)
#define MC34X9_REG_TF_THRESH_MSB    (0x41)
#define MC34X9_REG_TF_DB            (0x42)
#define MC34X9_REG_AM_THRESH_LSB    (0x43)
#define MC34X9_REG_AM_THRESH_MSB    (0x44)
#define MC34X9_REG_AM_DB            (0x45)
#define MC34X9_REG_SHK_THRESH_LSB   (0x46)
#define MC34X9_REG_SHK_THRESH_MSB   (0x47)
#define MC34X9_REG_PK_P2P_DUR_THRESH_LSB    (0x48)
#define MC34X9_REG_PK_P2P_DUR_THRESH_MSB    (0x49)
#define MC34X9_REG_TIMER_CTRL       (0x4A)

#define MC34X9_NULL_ADDR            (0)

#define MC34X9_CHIP_ID (0xA4)

#define s_bCfgFTThr 200
#define s_bCfgFTDebounce 50

struct MC34X9_acc_t
{
  short XAxis;
  short YAxis;
  short ZAxis;
  float XAxis_g;
  float YAxis_g;
  float ZAxis_g;
} ;

typedef enum
{
  MC34X9_GAIN_5_24X      = 0b0011,
  MC34X9_GAIN_3_89X      = 0b0010,
  MC34X9_GAIN_DEFAULT_1X = 0b0000,
  MC34X9_GAIN_0_5X       = 0b0100,
  MC34X9_GAIN_0_33X      = 0b1100,
}   MC34X9_gain_t;

typedef enum
{
  MC34X9_MODE_SLEEP    = 0b000,
  MC34X9_MODE_CWAKE      = 0b001,
  MC34X9_MODE_RESERVED   = 0b010,
  MC34X9_MODE_STANDBY  = 0b011,
}   MC34X9_mode_t;

typedef enum
{
  MC34X9_RANGE_2G    = 0b000,
  MC34X9_RANGE_4G    = 0b001,
  MC34X9_RANGE_8G    = 0b010,
  MC34X9_RANGE_16G   = 0b011,
  MC34X9_RANGE_12G   = 0b100,
  MC34X9_RANGE_END,
}   MC34X9_range_t;

typedef enum
{
  MC34X9_SR_25Hz            = 0x10,
  MC34X9_SR_50Hz            = 0x11,
  MC34X9_SR_62_5Hz          = 0x12,
  MC34X9_SR_100Hz           = 0x13,
  MC34X9_SR_125Hz           = 0x14,
  MC34X9_SR_250Hz           = 0x15,
  MC34X9_SR_500Hz           = 0x16,
  MC34X9_SR_DEFAULT_1000Hz  = 0x17,
  MC34X9_SR_END,
}   MC34X9_sr_t;

typedef enum
{
  MC34X9_TILT_FEAT = 0,
  MC34X9_ANYM_FEAT = 2,
  MC34X9_SHAKE_FEAT = 3,
  MC34X9_TILT35_FEAT = 4,
}   MC34X9_motion_feature_t;

typedef enum
{
  MC34X9_FIFO_CTL_DISABLE = 0,
  MC34X9_FIFO_CTL_ENABLE,
  MC34X9_FIFO_CTL_END,
}   MC34X9_fifo_ctl_t;

typedef enum
{
  MC34X9_FIFO_MODE_NORMAL = 0,
  MC34X9_FIFO_MODE_WATERMARK,
  MC34X9_FIFO_MODE_END,
}   MC34X9_fifo_mode_t;

typedef enum
{
  MC34X9_COMB_INT_DISABLE = 0,
  MC34X9_COMB_INT_ENABLE,
  MC34X9_COMB_INT_END,
}   MC34X9_fifo_int_t;

typedef struct
{
  uint8_t    bTILT;
  uint8_t    bFLIP;
  uint8_t    bANYM;
  uint8_t    bSHAKE;
  uint8_t    bTILT_35;
  uint8_t    bRESV;
  uint8_t 	 bAUTO_CLR;
  uint8_t    bACQ;
}   MC34X9_interrupt_event_t;

typedef struct
{
  uint8_t    bFIFO_EMPTY;
  uint8_t    bFIFO_FULL;
  uint8_t    bFIFO_THRESH;
}   MC34X9_fifo_interrupt_event_t;

/* general accel methods */
class MC34X9 {
  public:
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);
    // Setup and begin measurements
    bool start(bool bSpi, uint8_t chip_select);
    // Start measurement
    void wake();
    // End measurement
    void stop();
    // Sensor reset
    void reset();
    void SetMode(MC34X9_mode_t mode);
    void SetRangeCtrl(MC34X9_range_t range);
    void SetSampleRate(MC34X9_sr_t sample_rate);
    void SetFIFOCtrl(MC34X9_fifo_ctl_t fifo_ctl,
                     MC34X9_fifo_mode_t fifo_mode,
                     uint8_t fifo_thr);
    void SetMotionCtrl(bool tilt_int_ctrl,
                       bool flip_int_ctl,
                       bool anym_int_ctl,
                       bool shake_int_ctl,
                       bool tilt_35_int_ctl);
    void SetINTCtrl(bool tilt_int_ctrl,
                    bool flip_int_ctl,
                    bool anym_int_ctl,
                    bool shake_int_ctl,
                    bool tilt_35_int_ctl);
    void SetFIFOINTCtrl(bool fifo_empty_int_ctl,
                        bool fifo_full_int_ctl,
                        bool fifo_thr_int_ctl);
    void SetGerneralINTCtrl();
    void INTHandler(MC34X9_interrupt_event_t *ptINT_Event);
    void FIFOINTHandler(MC34X9_fifo_interrupt_event_t *ptFIFO_INT_Event);
    MC34X9_range_t GetRangeCtrl(void);
    MC34X9_sr_t GetSampleRate(void);
    bool IsFIFOEmpty(void);
    MC34X9_acc_t readRawAccel(void);

  private:
    bool M_bSpi;
    uint8_t M_chip_select;
    short x, y, z;
    // Raw Accelerometer data
    MC34X9_acc_t AccRaw;
};

// ***I2C/SPI BUS***
// Use object to do Bus communication
extern MC34X9 MC34X9_acc;
#define MC34X9_acc_readRegister8(reg) \
        MC34X9_acc.readRegister8(reg);

#define MC34X9_acc_writeRegister8(reg, value) \
        MC34X9_acc.writeRegister8(reg, value);
typedef enum
{
  /** SPI run under 2MHz when normal mode enable. */
  E_M_DRV_INTERFACE_SPIMODE_NORMAL = 0,
  /** SPI bus could over 2MHz after enable high speed mode. */
  E_M_DRV_INTERFACE_SPIMODE_HS,
  E_M_DRV_INTERFACE_SPIMODE_END,
}   e_m_drv_interface_spimode_t;

// Bus Protocol init
int m_drv_i2c_init(void);
int m_drv_spi_init(e_m_drv_interface_spimode_t spi_hs_mode);

// Bus function
uint8_t mcube_write_regs(bool bSpi, uint8_t chip_select, uint8_t reg, \
                         uint8_t *value, uint8_t size);
uint8_t mcube_read_regs( bool bSpi, uint8_t chip_select, uint8_t reg, \
                         uint8_t *value, uint8_t size);

uint8_t _readRegister8(bool bSpi, uint8_t chip_select, uint8_t reg);
void _writeRegister8(bool bSpi, uint8_t chip_select, uint8_t reg, uint8_t value);

// ***MC34X9 dirver motion part***
typedef enum
{
  MC34X9_TILT35_1p6           = 0b000,
  MC34X9_TILT35_1p8           = 0b001,
  MC34X9_TILT35_2p0           = 0b010,  
  MC34X9_TILT35_2p2           = 0b011,
  MC34X9_TILT35_2p4           = 0b100,
  MC34X9_TILT35_2p6           = 0b101,
  MC34X9_TILT35_2p8           = 0b110,
  MC34X9_TILT35_3p0           = 0b111
} MC34X9_TILT35_DURATION_TIMER_t;

void M_DRV_MC34X6_SetTFThreshold(uint16_t threshold);
void M_DRV_MC34X6_SetTFDebounce(uint8_t debounce);
void M_DRV_MC34X6_SetANYMThreshold(uint16_t threshold);
void M_DRV_MC34X6_SetANYMDebounce(uint8_t debounce);
void M_DRV_MC34X6_SetShakeThreshold(uint16_t threshold);
void M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(uint16_t threshold, uint8_t shakeCount);
void M_DRV_MC34X6_SetTILT35Threshold(uint16_t threshold);
void M_DRV_MC34X6_SetTILT35Timer(uint8_t timer);

void _M_DRV_MC34X6_SetTilt_Flip();
void _M_DRV_MC34X6_SetAnym();
void _M_DRV_MC34X6_SetShake();
void _M_DRV_MC34X6_SetTilt35();
#endif
