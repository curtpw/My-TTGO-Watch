/****************************************************************************
 *   linuxthor 2020
 ****************************************************************************/
 
/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include "config.h"

#include "hardware/wifictl.h"
#include "hardware/display.h"


#include "wifimon_app.h"
#include "wifimon_app_main.h"

#include "gui/mainbar/app_tile/app_tile.h"
#include "gui/mainbar/main_tile/main_tile.h"
#include "gui/mainbar/mainbar.h"
#include "gui/statusbar.h"
#include "gui/keyboard.h"
#include "gui/widget_styles.h"
#include "gui/widget_factory.h"

#include "hardware/sound.h"
#include "gui/sound/piep.h"
#include "gui/sound/test_c_mouth.h"
#include "hardware/motor.h"
#include "hardware/powermgm.h"
//#include "app/alarm_clock/alarm_in_progress.h"


//core
#include <Arduino.h>
#include <math.h>
#include <lwip/sockets.h>
#include "esp_wifi.h"

//CURT
/********************************** TENSORFLOW ***********************************/
#include <EloquentTinyML.h>
// sine_model.h contains the array you exported from the previous step with xxd or tinymlgen
#include "sine_model.h"
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 1
// in future projects you may need to tweak this value: it's a trial and error process
#define TENSOR_ARENA_SIZE 2*1024

Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;

/******************* MLX90615 VAR ********************/
//MLX90615 Thermopile IR Thermometer Addresses
#define MLX90615_I2CADDR          0x2A //default
#define MLX90615_I2CADDR1          0x2A //default
#define MLX90615_I2CADDR2          0x2B //default
#define MLX90615_I2CADDR3          0x2C //default
//#define MLX90615_I2CADDR          0x00  //master
// RAM
#define MLX90615_RAWIR1           0x04
#define MLX90615_RAWIR2           0x05
#define MLX90615_TA               0x26
#define MLX90615_TOBJ1            0x27
#define MLX90615_TOBJ2            0x28
// EEPROM
#define MLX90615_TOMAX            0x20
#define MLX90615_TOMIN            0x21
#define MLX90615_PWMCTRL          0x22
#define MLX90615_TARANGE          0x23
#define MLX90615_EMISS            0x24
#define MLX90615_CONFIG           0x25
#define MLX90615_ADDR             0x0E
//VAR
float TDeviceM15[3] = {0.0}; 
float TObjectM15[3] = {0.0};
float TDeviceAvMLX15 = 0.0;

/******************* BMI055 VAR ********************/
//    I2C ADDRESS/BITS
    #define BMG160_DEFAULT_ADDRESS_GYRO                         (0x68)
    #define BMG160_ADDRESS_GYRO_UPDATED                         (0x69)
    #define BMG160_DEFAULT_CHIP_ID                              (0x0F)
//    CONVERSION DELAY (in mS)
    #define BMG160_CONVERSIONDELAY                              (100)
//    GYROSCOPE REGISTERS
    #define BMG160_REG_GYRO_CHIP_ID                             (0x00)      // Chip Identification Number Register
    #define BMG160_REG_GYRO_RATE_X_LSB                          (0x02)      // X-Axis Gyroscope Low Data Register
    #define BMG160_REG_GYRO_RATE_X_MSB                          (0x03)      // X-Axis Gyroscope High Data Register
    #define BMG160_REG_GYRO_RATE_Y_LSB                          (0x04)      // Y-Axis Gyroscope Low Data Register
    #define BMG160_REG_GYRO_RATE_Y_MSB                          (0x05)      // Y-Axis Gyroscope High Data Register
    #define BMG160_REG_GYRO_RATE_Z_LSB                          (0x06)      // Z-Axis Gyroscope Low Data Register
    #define BMG160_REG_GYRO_RATE_Z_MSB                          (0x07)      // Z-Axis Gyroscope High Data Register
    #define BMG160_REG_GYRO_TEMP                                (0x08)      // Temperature Data Register
    #define BMG160_REG_GYRO_INTR_STATUS                         (0x09)      // Interrupt Status Register
    #define BMG160_REG_GYRO_INTR_STATUS1                        (0x0A)      // Interrupt Status 1 Register
    #define BMG160_REG_GYRO_INTR_STATUS2                        (0x0B)      // Interrupt Status 2 Register
    #define BMG160_REG_GYRO_INTR_STATUS3                        (0x0C)      // Interrupt Status 3 Register
    #define BMG160_REG_GYRO_FIFO_STATUS                         (0x0E)      // FIFO Status Register
    #define BMG160_REG_GYRO_RANGE                               (0x0F)      // Angular Rate Range Measurement Register
    #define BMG160_REG_GYRO_BANDWIDTH                           (0x10)      // Selection of the Angular Rate Data Filter Bandwidth Register
    #define BMG160_REG_GYRO_LPM1                                (0x11)      // Main Power Mode Register
    #define BMG160_REG_GYRO_LPM2                                (0x12)      // Fast Power-Up and External Trigger Configuration Settings Register
    #define BMG160_REG_GYRO_RATE_HODR                            (0x13)      // Angular Rate Data Acquisition and Data Output Format Register
    #define BMG160_REG_GYRO_BGW_SOFTRESET                       (0x14)      // User Triggered SoftReset Register
    #define BMG160_REG_GYRO_BGW_INT_EN_0                        (0x15)      // Interrupt Control Register
    #define BMG160_REG_GYRO_BGW_INT_EN_1                        (0x16)      // Interrupt Pin Configuration Register
    #define BMG160_REG_GYRO_BGW_INT_MAP_0                       (0x17)      // Interrupt Signal Mapped INT1 Control Register
    #define BMG160_REG_GYRO_BGW_INT_MAP_1                       (0x18)      // Interrupt Signal Mapped INT1/INT2 Control Register
    //#define BMG160_REG_GYRO_BGW_INT_MAP_0                       (0x19)      // Interrupt Signal Mapped INT2 Control Register
    #define BMG160_REG_GYRO_INTERRUPT_DATA_SRC_SLCT             (0x1A)      // Interrupt with Selectable Data Source Definition Register
    #define BMG160_REG_GYRO_INTERRUPT_DATA_SRC_ANY              (0x1B)      // Fast Offset Compensation with Any Motion Threshold Definition Register
    #define BMG160_REG_GYRO_AWAKE_SAMPLE_DURATION               (0x1C)      // Awake and Sample Duration Register
    #define BMG160_REG_GYRO_FIFO_WATER_MARK_INTR                (0x1E)      // FIFO Water Mark Interrupt Register
    #define BMG160_REG_GYRO_INT_RST_LATCH                       (0x21)      // Interrupt Reset and Mode Selection Register
    #define BMG160_REG_GYRO_HIGH_THS_X                          (0x22)      // High Rate Threshold and Hysteresis Seettings for X-Axis Register
    #define BMG160_REG_GYRO_HIGH_DUR_X                          (0x23)      // High Rate Duration Settings X-Axis Register
    #define BMG160_REG_GYRO_HIGH_THS_Y                          (0x24)      // High Rate Threshold and Hysteresis Seettings for Y-Axis Register
    #define BMG160_REG_GYRO_HIGH_DUR_Y                          (0x25)      // High Rate Duration Settings Y-Axis Register
    #define BMG160_REG_GYRO_HIGH_THS_Z                          (0x26)      // High Rate Threshold and Hysteresis Seettings for Z-Axis Register
    #define BMG160_REG_GYRO_HIGH_DUR_Z                          (0x27)      // High Rate Duration Settings Z-Axis Register
    #define BMG160_REG_GYRO_SOC                                 (0x31)      // Slow Offset Cancellation Settings Register
    #define BMG160_REG_GYRO_A_FOC                               (0x32)      // Fast Offset Cancellation Settings Register
    #define BMG160_REG_GYRO_TRIM_NVM_CTRL                       (0x33)      // NVM Control Settings Register
    #define BMG160_REG_GYRO_BGW_SPI3_WDT                        (0x34)      // Digital Interface Settings Register
    #define BMG160_REG_GYRO_OFC1                                (0x36)      // Offset Compensation Settings Register
    #define BMG160_REG_GYRO_OFC2                                (0x37)      // Offset Compensation Settings for X-Axis Register
    #define BMG160_REG_GYRO_OFC3                                (0x38)      // Offset Compensation Settings for Y-Axis Register
    #define BMG160_REG_GYRO_OFC4                                (0x39)      // Offset Compensation Settings for Z-Axis Register
    #define BMG160_REG_GYRO_TRIM_GP0                            (0x3A)      // NVM General Purpose Data Register
    #define BMG160_REG_GYRO_TRIM_GP1                            (0x3B)      // NVM General Purpose Data Register
    #define BMG160_REG_GYRO_BIST                                (0x3C)      // Built in Self-Test (BIST) Register
    #define BMG160_REG_GYRO_FIFO_CONFIG_0                       (0x3D)      // FIFO Watermark Level Register
    #define BMG160_REG_GYRO_FIFO_CONFIG_1                       (0x3E)      // FIFO Configuration Settings Register
    #define BMG160_REG_GYRO_FIFO_DATA                           (0x3F)      // FIFO Data Readout Register
//    GYROSCOPE RANGE SELCTION REGISTER DESCRIPTION
    #define BMG160_REG_GYRO_G_RANGE_MASK                        (0x07)      // Angular Rate and Resolution
    #define BMG160_REG_GYRO_G_RANGE_2000                        (0x00)      // Full Scale: ± 2000 °/s
    #define BMG160_REG_GYRO_G_RANGE_1000                        (0x01)      // Full Scale: ± 1000 °/s
    #define BMG160_REG_GYRO_G_RANGE_500                         (0x02)      // Full Scale: ± 500 °/s
    #define BMG160_REG_GYRO_G_RANGE_250                         (0x03)      // Full Scale: ± 250 °/s
    #define BMG160_REG_GYRO_G_RANGE_125                         (0x04)      // Full Scale: ± 125 °/s
//    GYROSCOPE BANDWIDTH REGISTER DESCRIPTION
    #define BMG160_REG_GYRO_BANDWIDTH_MASK                      (0x07)      // Selection of the Bandwidth for the GYROeration Data
    #define BMG160_REG_GYRO_BANDWIDTH_2000_UNFILTERED           (0x00)      // ODR: 2000 Hz, Filter Bandwidth: Unfiltered (523 Hz)
    #define BMG160_REG_GYRO_BANDWIDTH_2000_230HZ                (0x01)      // ODR: 2000 Hz, Filter Bandwidth: 230 Hz
    #define BMG160_REG_GYRO_BANDWIDTH_1000_116HZ                (0x02)      // ODR: 1000 Hz, Filter Bandwidth: 116 Hz
    #define BMG160_REG_GYRO_BANDWIDTH_400_47HZ                  (0x03)      // ODR: 400 Hz, Filter Bandwidth: 47 Hz
    #define BMG160_REG_GYRO_BANDWIDTH_200_23HZ                  (0x04)      // ODR: 200 Hz, Filter Bandwidth: 23 Hz
    #define BMG160_REG_GYRO_BANDWIDTH_100_12HZ                  (0x05)      // ODR: 100 Hz, Filter Bandwidth: 12 Hz
    #define BMG160_REG_GYRO_BANDWIDTH_200_64HZ                  (0x06)      // ODR: 200 Hz, Filter Bandwidth: 64 Hz
    #define BMG160_REG_GYRO_BANDWIDTH_100_32HZ                  (0x07)      // ODR: 100 Hz, Filter Bandwidth: 32 Hz

    #define GYRO_RANGE_2000                        (0x00)      // Full Scale: ± 2000 °/s
    #define GYRO_RANGE_1000                        (0x01)      // Full Scale: ± 1000 °/s
    #define GYRO_RANGE_500                         (0x03)      // Full Scale: ± 500 °/s

    #define GYRO_BANDWIDTH_200_23HZ                (0x04)      // ODR: 200 Hz, Filter Bandwidth: 23 Hz

uint8_t bmg_i2cAddress = BMG160_DEFAULT_ADDRESS_GYRO;
uint8_t bmg_conversionDelay;
uint8_t bmg_gyrorange;
uint8_t bmg_gyrobandwidth;
int16_t gyroX, gyroY, gyroZ;



//BMA2X2 accelerometer

#define MAX_READ                 4
#define MAX_WRITE                4
#define FRAME_SIZE               6
#define BMA_RANGE_2G             0x03
#define BMA_RANGE_4G             0x05
#define BMA_RANGE_8G             0x08
#define BMA_RANGE_16G            0x0C
#define BMA_BANDWIDTH_7_81HZ     0x08
#define BMA_BANDWIDTH_15_63HZ    0x09
#define BMA_BANDWIDTH_31_25HZ    0x0A
#define BMA_BANDWIDTH_62_5HZ     0x0B
#define BMA_BANDWIDTH_125HZ      0x0C
#define BMA_BANDWIDTH_250HZ      0x0D
#define BMA_BANDWIDTH_500HZ      0x0E
#define BMA_BANDWIDTH_1000HZ     0x0F
#define BMA_REG_FIFO_STATUS      0x0E
#define BMA_REG_PMU_RANGE        0x0F
#define BMA_REG_PMU_BW           0x10
#define BMA_REG_PMU_LPW          0x11
#define BMA_REG_PMU_LOW_POWER    0x12
#define BMA_REG_FIFO_CONFIG_1    0x3E
#define BMA_REG_FIFO_DATA        0x3F
#define BMA_ACCD_X_LSB      (0x02)
#define BMA_ACCD_X_MSB      (0x03)
#define BMA_ACCD_Y_LSB      (0x04)
#define BMA_ACCD_Y_MSB      (0x05)
#define BMA_ACCD_Z_LSB      (0x06)
#define BMA_ACCD_Z_MSB      (0x07)

#define BMA_ADDRESS              (0x18)

uint32_t bma_offset;
uint8_t bma_FifoStatus;
int32_t bma_currentResolution;
uint32_t bma_currentInterval;
//int32_t accelX, accelY, accelZ;
int16_t accelX, accelY, accelZ;




/********************************************************************************************************/
/******************** APDS9500 STUFF * APDS9500 STUFF * APDS9500 STUFF **********************************/
/********************************************************************************************************/


 #define APDS9500_R_RegBankSet                  0xEF // 0x00 register bank 0, 0x01 register bank 1

 /* Bank 0*/
 #define APDS9500_PartID_L                      0x00 // Low  byte of Part ID
 #define APDS9500_PartID_H                      0x01 // High byte of Part ID
 #define APDS9500_VersionID                     0x02 // High byte of Part ID

 /* Cursor Mode Controls */
 #define APDS9500_R_CursorUseTop                0x32
 #define APDS9500_R_PositionFilterStartSizeTh_L 0x33
 #define APDS9500_R_PositionFilterStartSizeTh_H 0x34
 #define APDS9500_R_ProcessFilterStartSizeTh_L  0x35
 #define APDS9500_R_ProcessFilterStartSizeTh_H  0x36
 #define APDS9500_R_CursorClampLeft             0x37
 #define APDS9500_R_CursorClampRight            0x38
 #define APDS9500_R_CursorClampUp               0x39
 #define APDS9500_R_CursorClampDown             0x3A
 #define APDS9500_R_CursorClampCenterX_L        0x3B
 #define APDS9500_R_CursorClampCenterX_H        0x3C
 #define APDS9500_R_CursorClampCenterY_L        0x3D
 #define APDS9500_R_CursorClampCenterY_H        0x3E
 #define APDS9500_R_Cursor_ObjectSizeTh         0x8B
 #define APDS9500_R_PositionResolution          0x8C

 /* CURT ADD PROXIMITY CONTROLLS */
 #define R_Prox_UB                              0x69 //7:0 0xC8 R/W Proximity up bound
 #define R_Prox_LB                              0x6A //7:0 0x40 R/W Proximity low bound
 #define S_State                                0x6B  // 7:0 - R PS approach state 1: Approach, (S_AvgY ≥ R_Pox_UB) 0: Not approach, (S_AvgY ≤ R_Pox_LB) (Only functional at proximity detection mode)
 #define S_AvgY                                 0x6C  //0 S_AvgY 0x6C 7:0 - R Proximity object average brightness
  
 /* Interrupt Controls */
 #define APDS9500_R_MCU_Int_Flag                0x40
 #define APDS9500_R_Int1_En                     0x41
 #define APDS9500_R_Int2_En                     0x42
 #define APDS9500_Int_Flag_1                    0x43
 #define APDS9500_Int_Flag_2                    0x44

 /* AE/AG Controls */
 #define APDS9500_R_AELedOff_UB                 0x46
 #define APDS9500_R_AELedOff_LB                 0x47
 #define APDS9500_R_AE_Exposure_UB_L            0x48
 #define APDS9500_R_AE_Exposure_UB_H            0x49
 #define APDS9500_R_AE_Exposure_LB_L            0x4A
 #define APDS9500_R_AE_Exposure_LB_H            0x4B
 #define APDS9500_R_AE_Gain_UB                  0x4C
 #define APDS9500_R_AE_Gain_LB                  0x4D
 #define APDS9500_R_AE_Gain_Step                0x4E
 #define APDS9500_R_AE_Gain_Default             0x4F
 #define APDS9500_R_Exp_Sel                     0x50
 #define APDS9500_R_Manual                      0x51
 #define APDS9500_AG_Stage_GG                   0x54
 #define APDS9500_Reg_ExposureNum_L             0x55
 #define APDS9500_Reg_ExposureNum_H             0x56
 #define APDS9500_Reg_global                    0x57
 #define APDS9500_AE_LED_Off_YAvg               0x58
 #define APDS9500_AE_Dec_Inc                    0x59
 #define APDS9500_AE_Normal_Factor              0x5A

   //CURT ADD LED POWER BOOST
 #define R_LED1_DAC_UB                          0x50     //4:0 default 0x14 R/W LED1 upper bound

 /* GPIO Setting*/
 #define APDS9500_InputMode_GPIO_0_1            0x80
 #define APDS9500_InputMode_GPIO_2_3            0x81
 #define APDS9500_InputMode_INT                 0x82

 /* Gesture Mode Controls */
 #define APDS9500_R_LightThd                    0x83
 #define APDS9500_R_GestureStartTh_L            0x84
 #define APDS9500_R_GestureStartTh_H            0x85
 #define APDS9500_R_GestureEndTh_L              0x86
 #define APDS9500_R_GestureEndTh_H              0x87
 #define APDS9500_R_ObjectMinZ                  0x88
 #define APDS9500_R_ObjectMaxZ                  0x89
 #define APDS9500_R_ProcessResolution           0x8C
 #define APDS9500_R_TimeDelayNum                0x8D 
 #define APDS9500_R_Disable45Degree             0x8E 
 #define APDS9500_R_XtoYGain                    0x8F 
 #define APDS9500_R_NoMotionCountThd            0x90 
 #define APDS9500_R_NoObjectCountThd            0x91 
 #define APDS9500_R_NormalizedImageWidth        0x92 
 #define APDS9500_R_XDirectionThd               0x93 
 #define APDS9500_R_YDirectionThd               0x94 
 #define APDS9500_R_ZDirectionThd               0x95 
 #define APDS9500_R_ZDirectionXYThd             0x96 
 #define APDS9500_R_ZDirectionAngleThd          0x97 
 #define APDS9500_R_RotateAngleThd              0x98 
 #define APDS9500_R_RotateConti                 0x99 
 #define APDS9500_R_RotateXYThd                 0x9A 
 #define APDS9500_R_RotateZThd                  0x9B 
 #define APDS9500_R_Filter                      0x9C 
 #define APDS9500_R_DistThd                     0x9D 
 #define APDS9500_R_GestureDetEn                0x9F 
 #define APDS9500_R_FilterImage                 0xA5 
 #define APDS9500_R_DiffAngleThd                0xA9 
 #define APDS9500_ObjectCenterX_L               0xAC 
 #define APDS9500_ObjectCenterX_H               0xAD 
 #define APDS9500_ObjectCenterY_L               0xAE 
 #define APDS9500_ObjectCenterY_H               0xAF 
 #define APDS9500_ObjectAvgY                    0xB0 
 #define APDS9500_ObjectSize_L                  0xB1 
 #define APDS9500_ObjectSize_H                  0xB2 
 #define APDS9500_Gx                            0xB3 
 #define APDS9500_Gy                            0xB4
 #define APDS9500_Gz                            0xB5
 #define APDS9500_GestureResult                 0xB6
 #define APDS9500_WaveCount                     0xB7
 #define APDS9500_NoObjectCount                 0xB8
 #define APDS9500_NoMotionCount                 0xB9
 #define APDS9500_LightCount                    0xBA
 #define APDS9500_LightAcc_L                    0xBB
 #define APDS9500_LightAcc_H                    0xBC
 #define APDS9500_TimeAcc_L                     0xBD
 #define APDS9500_TimeAcc_H                     0xBE
 #define APDS9500_AngleAcc_L                    0xC7
 #define APDS9500_AngleAcc_H                    0xC8
 #define APDS9500_XGainValue                    0xCA
 #define APDS9500_YGainValue                    0xCB
 #define APDS9500_R_YtoZSum                     0xCC
 #define APDS9500_R_YtoZFactor                  0xCD
 #define APDS9500_R_FilterLength                0xCE
 #define APDS9500_R_WaveThd                     0xCF
 #define APDS9500_R_AbortCountThd               0xD0
 #define APDS9500_R_AbortLength                 0xD1
 #define APDS9500_R_WaveEnH                     0xD2
 #define APDS9500_PositionFilterCenterX_L       0xD3 //Low byte of horizontal object center after IIR fi lter for cursor mode
 #define APDS9500_PositionFilterCenterXY_H      0xD4 //High byte of horizontal object center after IIR fi lter for cursor mode
 #define APDS9500_PositionFilterCenterY_L       0xD5
 #define APDS9500_PositionFilterAvgY_L          0xD6
 #define APDS9500_PositionFilterAvgY_H          0xD7
 #define APDS9500_PositionFilterSize_L          0xD8
 #define APDS9500_PositionFilterSize_H           0xD9  //curt fix
 #define APDS9500_ProcessFilterAvgY_H           0xD9
 #define APDS9500_ProcessFilterCenterX_L        0xDA
 #define APDS9500_ProcessFilterCenterXY_H       0xDB
 #define APDS9500_ProcessFilterCenterY_L        0xDC
 #define APDS9500_ProcessFilterSize_L           0xDD
 #define APDS9500_ProcessFilterAvgY_L           0xDE
 #define APDS9500_AbortIntervalCount_L          0xDF

 //CURT ADD
 #define R_ObjectMinZ                           0x88 //4:0 0x05 (def) R/W Z direction minimum threshold
 #define R_ObjectMaxZ                           0x89 //5:0 0x18 R/W Z direction maximum threshold
 
 /* Bank 1 */
 
 /* Image size settings */
 #define APDS9500_Cmd_HSize                     0x00
 #define APDS9500_Cmd_VSize                     0x01
 #define APDS9500_Cmd_HStart                    0x02
 #define APDS9500_Cmd_VStart                    0x03
 #define APDS9500_Cmd_HV                        0x04
 
 /* Lens Shading */ 
 #define APDS9500_R_LensShadingComp_EnH         0x25
 #define APDS9500_R_Offest_X                    0x26
 #define APDS9500_R_Offest_Y                    0x27
 #define APDS9500_R_LSC                         0x28
 #define APDS9500_R_LSFT                        0x29
   
 #define APDS9500_R_global                      0x42
 #define APDS9500_R_ggh                         0x44
 
/* Sleep Mode Parameters */
 #define APDS9500_R_IDLE_TIME_L                 0x65
 #define APDS9500_R_IDLE_TIME_H                 0x66
 #define APDS9500_R_IDLE_TIME_SLEEP_1_L         0x67
 #define APDS9500_R_IDLE_TIME_SLEEP_1_H         0x68
 #define APDS9500_R_IDLE_TIME_SLEEP_2_L         0x69
 #define APDS9500_R_IDLE_TIME_SLEEP_2_H         0x6A
 #define APDS9500_R_Object_TIME_1_L             0x6B
 #define APDS9500_R_Object_TIME_1_H             0x6C
 #define APDS9500_R_Object_TIME_2_L             0x6D
 #define APDS9500_R_Object_TIME_2_H             0x6E
 #define APDS9500_R_TG_INIT_TIME                0x6F
 #define APDS9500_R_TG_POWERON_WAKEUP_TIME      0x71
 #define APDS9500_R_TG_EnH                      0x72
 #define APDS9500_R_Auto_SLEEP_Mode             0x73
 #define APDS9500_R_Wake_Up_Sig_Sel             0x74

 /* Image Controls */
 #define APDS9500_R_SRAM_Read_EnH               0x77

 /* I2C Address */
 #define APDS9500_ADDRESS                   0x73

 /* Camera Image Out Over SPI */
  #define R_SPIOUT_EnH                      0x7E  //bank 1, write high (0x00) to enable image out over SPI

 bool intFlag = false;
 uint8_t intFlag1 = 0, intFlag2 = 0, gestResult, getEnabled;
 uint8_t myLed = 13;
 uint8_t intPin_APDS9500 = 5;

 //APDS data global var

 //gesture related
 float objectSize = 0.0;
 float objectX = 0.0;
 float objectY = 0.0;
 float objectAvSize = 0.0;
 float proxBrightness = 0.0;
 float movementGXYZ[3] = {0.0};
 float lightAccumulation = 0.0;
 float angleAccumulation = 0.0;
 float XAngleGainValue = 0.0;
 float YAngleGainValue = 0.0;

 //cursor related
 float cursorCenterX = 0.0;
 float cursorCenterY = 0.0;
 float cursorAverageY = 0.0;
 float cursorSize = 0.0;




/********************************************************************************************************/
/************************ MLX90615 THERMOPILE FUNCTIONS *************************************************/
/********************************************************************************************************/
float read16(uint8_t a, int sensorNum) {
  uint8_t _addr = MLX90615_I2CADDR;

  if(sensorNum == 0) _addr = MLX90615_I2CADDR;   //custom addresses
  else if(sensorNum == 1)  _addr = MLX90615_I2CADDR1;   
  else if(sensorNum == 2)  _addr = MLX90615_I2CADDR2;
  else if(sensorNum == 3)  _addr = MLX90615_I2CADDR3;
  
  uint16_t ret;
  Wire.beginTransmission(_addr);                  // start transmission to device 
  Wire.write(a); delay(1);                        // sends register address to read from
  Wire.endTransmission(false);                    // end transmission
  Wire.requestFrom(_addr, (uint8_t)3); delay(1);  // send data n-bytes read
  ret = Wire.read();// delay(1);                    // receive DATA
  ret |= Wire.read() << 8;// delay(1);              // receive DATA
  uint8_t pec = Wire.read(); delay(1);
  return (float)ret;
}

float readTemp(uint8_t reg, int sensorNum) {
  float temp;
  temp = read16(reg, sensorNum);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

double readObjectTempF(int sensorNum) {
  return (readTemp(MLX90615_TOBJ1, sensorNum) * 9 / 5) + 32;
}

double readAmbientTempF(int sensorNum) {
  return (readTemp(MLX90615_TA, sensorNum) * 9 / 5) + 32;
}

double readObjectTempC(int sensorNum) {
  return readTemp(MLX90615_TOBJ1, sensorNum);
}

double readAmbientTempC(int sensorNum) {
  return readTemp(MLX90615_TA, sensorNum);
}


/********************************************************************************************************/
/************************************* BMI055 FUNCTIONS *************************************************/
/********************************************************************************************************/

static uint8_t i2cread(void){ return Wire.read(); }

static void i2cwrite(uint8_t x){ Wire.write((uint8_t)x); }

/****** Writes 8-bits to the specified destination register ***************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(i2cAddress);
    i2cwrite((uint8_t)reg);
    i2cwrite((uint8_t)(value));
    Wire.endTransmission();
}

/******** Reads 8-bits to the specified destination register *************/
static uint8_t readRegister(uint8_t i2cAddress, uint8_t reg)
{
    Wire.beginTransmission(i2cAddress);
    i2cwrite((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress, (uint8_t)1);
    return (uint8_t)(i2cread());
}

// Sets up the Hardware
bool BMG160_begin()
{
 //   Wire.begin();
 //   bmg_i2cAddress = i2cAddress;
    uint8_t chipid = readRegister(bmg_i2cAddress, BMG160_REG_GYRO_CHIP_ID);
    if (chipid != BMG160_DEFAULT_CHIP_ID)
        return false;
    
    // Set up the sensor for Gyroscope
    // setUpSensor();
    
    return true;
}

// Sets the Full Scale Range of the Gyroscope Outputs
void BMG160_setGyroRange(uint8_t gyrorange)
{
    bmg_gyrorange = gyrorange;
}

// Sets the Selection of the Bandwidth for the Gyroscope Data
void BMG160_setGyroBandwidth(uint8_t gyrobandwidth)
{
    bmg_gyrobandwidth = gyrobandwidth;
}

// Sets up the Sensor for Gyroscope
void BMG160_setUpSensor(void)
{
    // Set Up the Configuration for the Gyroscope Angular Rate Range Register
    // Full Scale Range of the Gyroscope Outputs
    uint8_t range = bmg_gyrorange;
    
    // Write the configuration to the Gyroscope Angular Rate Range Register
    writeRegister(bmg_i2cAddress, BMG160_REG_GYRO_RANGE, range);
    
    // Wait for the configuration to complete
    delay(bmg_conversionDelay);
    
    // Set Up the Configuration for the Gyroscope Angular Rate Data Filter Bandwidth Register
    // Set the Selection of the Bandwidth for the Gyroscope Data
    uint8_t bandwidth = bmg_gyrobandwidth;
    
    // Write the configuration to the Gyroscope Angular Rate Data Filter Bandwidth Register
    writeRegister(bmg_i2cAddress, BMG160_REG_GYRO_BANDWIDTH, bandwidth);
    
    // Wait for the configuration to complete
    delay(bmg_conversionDelay);

}


//      Reads the 3 axes of the Gyroscope
//      The value is expressed in 16 bit as two’s complement
void BMG160_Measure_Gyroscope()
{
    // Read the Gyroscope
    uint8_t xGyroLo, xGyroHi, yGyroLo, yGyroHi, zGyroLo, zGyroHi;
    
    // Read the Data
    // Reading the Low X-Axis Gyroscope Data Register
    xGyroLo = readRegister(bmg_i2cAddress, BMG160_REG_GYRO_RATE_X_LSB);
    // Reading the High X-Axis Gyroscope Data Register
    xGyroHi = readRegister(bmg_i2cAddress, BMG160_REG_GYRO_RATE_X_MSB);
    // Conversion of the result
    // 16-bit signed result for X-Axis Gyroscope Data of BMG160
    //bmg_gyroData.X = (int16_t)((xGyroHi << 8) | xGyroLo);
    gyroX = (int16_t)((xGyroHi << 8) | xGyroLo);
    
    // Reading the Low Y-Axis Gyroscope Data Register
    yGyroLo = readRegister(bmg_i2cAddress, BMG160_REG_GYRO_RATE_Y_LSB);
    // Reading the High Y-Axis Gyroscope Data Register
    yGyroHi = readRegister(bmg_i2cAddress, BMG160_REG_GYRO_RATE_Y_MSB);
    // Conversion of the result
    // 16-bit signed result for Y-Axis Gyroscope Data of BMG160
    //bmg_gyroData.Y = (int16_t)((yGyroHi << 8) | yGyroLo);
    gyroY = (int16_t)((yGyroHi << 8) | yGyroLo);
    
    // Reading the Low Z-Axis Gyroscope Data Register
    zGyroLo = readRegister(bmg_i2cAddress, BMG160_REG_GYRO_RATE_Z_LSB);
    // Reading the High Z-Axis Gyroscope Data Register
    zGyroHi = readRegister(bmg_i2cAddress, BMG160_REG_GYRO_RATE_Z_MSB);
    // Conversion of the result
    // 16-bit signed result for Z-Axis Gyroscope Data of BMG160
    //bmg_gyroData.Z = (int16_t)((zGyroHi << 8) | zGyroLo);
    gyroZ = (int16_t)((zGyroHi << 8) | zGyroLo);
}


// Sets up the Sensor for Accelerometer
/*
void BMA2X2_setUpSensor(void)
{
    // Set Up the Configuration for the Gyroscope Angular Rate Range Register
    // Full Scale Range of the Gyroscope Outputs
    uint8_t range = bmg_gyrorange;

    //Read accelerometer ID
    uint8_t chipid = readRegister(BMA_ADDRESS, BMG160_REG_GYRO_CHIP_ID);
    
    // Write the configuration accelerometer range
    writeRegister(BMA_ADDRESS, BMA_REG_PMU_RANGE, BMA_RANGE_2G);

    // Write the configuration accelerometer bandwidth
    writeRegister(BMA_ADDRESS, BMA_REG_PMU_BW, bandwidth);
    
    // Wait for the configuration to complete
    delay(bmg_conversionDelay);
    
    // Set Up the Configuration for the Gyroscope Angular Rate Data Filter Bandwidth Register
    // Set the Selection of the Bandwidth for the Gyroscope Data
    uint8_t bandwidth = bmg_gyrobandwidth;
    
    // Write the configuration to the Gyroscope Angular Rate Data Filter Bandwidth Register
    writeRegister(bmg_i2cAddress, BMG160_REG_GYRO_BANDWIDTH, bandwidth);
    
    // Wait for the configuration to complete
    delay(bmg_conversionDelay);

}
*/

void BMA2X2_setRange(uint8_t range)
{
    switch (range)
    {
        case 0x03: //2G
                        bma_currentResolution = 98;
                        break;
        case 0x05: //4G
                        bma_currentResolution = 195;
                        break;
        case 0x08: //8G
                        bma_currentResolution = 391;
                        break;
        case 0x0C: //16G
                        bma_currentResolution = 781;
                        break;
        default:
                        bma_currentResolution = 98;
                        break;
    }
    writeRegister(BMA_ADDRESS, BMA_REG_PMU_RANGE, range);
}

void BMA2X2_setBandwidth(uint8_t bandwidth)
{
    switch (bandwidth)
    {
        case 0x08: //BANDWIDTH_7_81HZ
                                bma_currentInterval = 128;
                                break;
        case 0x09: //BANDWIDTH_15_63HZ
                                bma_currentInterval = 64;
                                break;
        case 0x0A: //BANDWIDTH_31_25HZ
                                bma_currentInterval = 32;
                                break;
        case 0x0B: //BANDWIDTH_62_5HZ
                                bma_currentInterval = 16;
                                break;
        case 0x0C: //BANDWIDTH_125HZ
                                bma_currentInterval = 8;
                                break;
        case 0x0D: //BANDWIDTH_250HZ
                                bma_currentInterval = 4;
                                break;
        case 0x0E: //BANDWIDTH_500HZ
                                bma_currentInterval = 2;
                                break;
        case 0x0F: //BANDWIDTH_1000HZ
        default:
                                bma_currentInterval = 1;
                                break;
    }

    writeRegister(BMA_ADDRESS, BMA_REG_PMU_BW, bandwidth);
}

void BMA2X2_setFifo()
{
    /*  FIFO_BYPASS         = 0x00,
        FIFO_FIFO           = (1 << 6),
        FIFO_STREAM         = (2 << 6)     */
    writeRegister(BMA_ADDRESS, BMA_REG_FIFO_CONFIG_1, 0x00);
}

void BMA2X2_getFifoStatus()
{
    //getCallback = callback;
    //i2c.read(BMA2X2::ADDRESS, REG_FIFO_STATUS, memoryRead, 1, this, &BMA2X2::getFifoStatusDone);
    bma_FifoStatus = readRegister(BMA_ADDRESS, BMA_REG_FIFO_STATUS);
}

void BMA2X2_getSampleFromBuffer()
{
    int16_t x;
    int16_t y;
    int16_t z;

    Wire.beginTransmission(BMA_ADDRESS);
    i2cwrite(BMA_REG_FIFO_DATA);
    Wire.endTransmission();
    Wire.requestFrom(BMA_ADDRESS,6,true);  // request a total of 14 registers
    x=Wire.read()<<8|Wire.read();  // (ACCEL_XOUT_H) & (ACCEL_XOUT_L)    
    y=Wire.read()<<8|Wire.read();  // (ACCEL_YOUT_H) & (ACCEL_YOUT_L)
    z=Wire.read()<<8|Wire.read();  // (ACCEL_ZOUT_H) & (ACCEL_ZOUT_L)

    /*  convert raw values to mg */
    accelX = (x * bma_currentResolution) / 100;
    accelY = (y * bma_currentResolution) / 100;
    accelZ = (z * bma_currentResolution) / 100;
}

void BMA2X2_Measure_Accelerometer()
{
    // Read the Accelerometer
    uint8_t xAccLo, xAccHi, yAccLo, yAccHi, zAccLo, zAccHi;
    
    // Read the Data
    // Reading the Low X-Axis Accelerometer Data Register
    xAccLo = readRegister(BMA_ADDRESS, BMA_ACCD_X_LSB);
    // Reading the High X-Axis Gyroscope Data Register
    xAccHi = readRegister(BMA_ADDRESS, BMA_ACCD_X_MSB);
    // Conversion of the result
    // 16-bit signed result for X-Axis Accelerometer Data of BMA2X2
    //bmg_gyroData.X = (int16_t)((xGyroHi << 8) | xGyroLo);
    accelX = (int16_t)((xAccHi << 8) | xAccLo);
    
    // Reading the Low Y-Axis Accelerometer Data Register
    yAccLo = readRegister(BMA_ADDRESS, BMA_ACCD_Y_LSB);
    // Reading the High Y-Axis Accelerometer Data Register
    yAccHi = readRegister(BMA_ADDRESS, BMA_ACCD_Y_MSB);
    // Conversion of the result
    // 16-bit signed result for Y-Axis Accelerometer Data of BMA2X2
    //bmg_gyroData.Y = (int16_t)((yGyroHi << 8) | yGyroLo);
    accelY = (int16_t)((yAccHi << 8) | yAccLo);
    
    // Reading the Low Z-Axis Accelerometer Data Register
    zAccLo = readRegister(BMA_ADDRESS, BMA_ACCD_Z_LSB);
    // Reading the High Z-Axis Accelerometer Data Register
    zAccHi = readRegister(BMA_ADDRESS, BMA_ACCD_Z_MSB);
    // Conversion of the result
    // 16-bit signed result for Z-Axis Accelerometer Data of BMA2X2
    //bmg_gyroData.Z = (int16_t)((zGyroHi << 8) | zGyroLo);
    accelZ = (int16_t)((zAccHi << 8) | zAccLo);
}




/**************************************************************************************************************
 **************************************** WRITEBYTE (APDS9500 I2C) *********************************************
 **************************************************************************************************************/
// I2C read/write functions for the MPU9250 sensors
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
{
//  uint8_t temp[2];
//  temp[0] = subAddress;
//   temp[1] = data;
//    Wire.transfer(address, &temp[0], 2, NULL, 0); 
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}
/*
uint8_t readByte(uint8_t address, uint8_t subAddress) 
{
        //  uint8_t temp[1];
         // Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        //  return temp[0];
            uint8_t data; // `data` will store the register data   
            Wire.beginTransmission(address);         // Initialize the Tx buffer
            Wire.write(subAddress);                  // Put slave register address in Tx buffer
            Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
            Wire.requestFrom(address, 1);            // Read one byte from slave register address 
            data = Wire.read();                      // Fill Rx buffer with result
            return data;                             // Return data read from slave register
}
*/

/**************************************************************************************************************
 **************************************** READBYTES (APDS9500 I2C) ********************************************
 **************************************************************************************************************/
 void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) 
{
//    Wire.transfer(address, &subAddress, 1, dest, count); 
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address 
    while (Wire.available()) {dest[i++] = Wire.read(); } // Put read results in the Rx buffer
}
        
/**************************************************************************************************************
 **************************************** READBYTE (APDS9500 I2C) *********************************************
 **************************************************************************************************************/
uint8_t readByte(uint8_t address, uint8_t subAddress) 
{
//  uint8_t temp[1];
// Wire.transfer(address, &subAddress, 1, &temp[0], 1);
//  return temp[0];
    uint8_t data; // `data` will store the register data   
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, 1);            // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}


/**************************************************************************************************************
 ************************************** FILTER APDS-9500 SENSOR DATA *******************************************
 **************************************************************************************************************/  
 
float filterApds8(uint8_t sensorValue, uint8_t sensorMax, float sensorAverage, bool debug){
  //debug makes the average the most recent value so we can see what raw sensor values look like
  if(debug){ return (float)sensorValue;}
  
  uint8_t roundSensorAverage = (uint8_t)sensorAverage;
  uint8_t difference;

  //clamp to 0 and max
  if(sensorValue > sensorMax){  
    sensorValue = sensorMax;
  } else if(sensorValue < 0){
    sensorValue = 0;
  }

  //find difference
  if(sensorValue >= roundSensorAverage){  
    difference = sensorValue - roundSensorAverage;
  } else {
    difference = roundSensorAverage - sensorValue;
  }

  //throw away most recent value if too far from average but keep some of the time incase valid
  if( ( (difference > (sensorMax - (sensorMax / 20))) ) && random(100) > 60){  
    return sensorAverage;
  } else {
    //return most recent value added into average
    return ((sensorAverage*3 + (float)sensorValue) / 4) ;
  }
}

float filterApds16(uint16_t sensorValue, uint16_t sensorMax, float sensorAverage, bool debug){
  //debug makes the average the most recent value so we can see what raw sensor values look like
  if(debug){ return (float)sensorValue;}
  
  uint16_t roundSensorAverage = (uint16_t)sensorAverage;
  uint16_t difference;

  //clamp to 0 and max
  if(sensorValue > sensorMax){  
    sensorValue = sensorMax;
  } else if(sensorValue < 0){
    sensorValue = 0;
  }

  //find difference
  if(sensorValue >= roundSensorAverage){  
    difference = sensorValue - roundSensorAverage;
  } else {
    difference = roundSensorAverage - sensorValue;
  }

  //throw away most recent value if too far from average but keep some of the time incase valid
  if( ( (difference > (sensorMax - (sensorMax / 20))) ) && random(100) > 60){  
    return sensorAverage;
  } else {
    //return most recent value added into average
    return ((sensorAverage*3 + (float)sensorValue) / 4) ;
  }
}

/**************************************************************************************************************
 ************************************** READ APDS-9500 SENSOR DATA ********************************************
 **************************************************************************************************************/  

 void readAPDS9500(){
    /*
     if(intFlag == true){ //we could be reading data from interupts, but right now we're just polling
        Serial.println("!!! APDS INT FLAG !!!");
         intFlag = false;
     }
     intFlag1 = readByte(APDS9500_ADDRESS, APDS9500_Int_Flag_1);
     intFlag2 = readByte(APDS9500_ADDRESS, APDS9500_Int_Flag_2);
     gestResult = readByte(APDS9500_ADDRESS, APDS9500_GestureResult);
     delayMicroseconds(100);
     */

     //PROXIMITY BRIGHTNESS
     uint8_t proxBrightness_temp = readByte(APDS9500_ADDRESS, APDS9500_ObjectAvgY);
    //log_i("proxBrightness_temp: %d", proxBrightness_temp);

     //float filterApds8(uint8_t sensorValue, uint8_t sensorMax, float sensorAverage, bool debug)
     proxBrightness = filterApds8(proxBrightness_temp, 255, proxBrightness, false);
     delayMicroseconds(100);
      
     // GESTURE OBJECT SIZE
     uint8_t objectSize_L  = readByte(APDS9500_ADDRESS, APDS9500_ObjectSize_L); //max 900
     uint8_t objectSize_H  = readByte(APDS9500_ADDRESS, APDS9500_ObjectSize_H);
     uint16_t objectSize_temp = (objectSize_H<<8) | objectSize_L;
     objectSize = filterApds16(objectSize_temp, 900, objectSize, false);
     delayMicroseconds(100);

     // GESTURE OBJECT CENTER X POS
     uint8_t objectX_L  = readByte(APDS9500_ADDRESS, APDS9500_ObjectCenterX_L);
     uint8_t objectX_H  = readByte(APDS9500_ADDRESS, APDS9500_ObjectCenterX_H);
     uint16_t objectX_temp = (objectX_H<<8) | objectX_L;
     //float filterApds16(uint16_t sensorValue, uint16_t sensorMax, float sensorAverage, bool debug)
     objectX = filterApds16(objectX_temp, 4500, objectX, false);
     delayMicroseconds(100);
     
     // GESTURE OBJECT CENTER Y POS
     uint8_t objectY_L  = readByte(APDS9500_ADDRESS, APDS9500_ObjectCenterY_L);
     uint8_t objectY_H  = readByte(APDS9500_ADDRESS, APDS9500_ObjectCenterY_H);
     uint16_t objectY_temp = (objectY_H<<8) | objectY_L;
     //float filterApds16(uint16_t sensorValue, uint16_t sensorMax, float sensorAverage, bool debug)
     objectY = filterApds16(objectY_temp, 4500, objectY, false);
     delayMicroseconds(100);
  
     // AVERAGE GESTURE OBJECT SIZE
     uint8_t objectAvSize_temp = readByte(APDS9500_ADDRESS, APDS9500_ObjectCenterY_H);  //max 255
     //float filterApds8(uint8_t sensorValue, uint8_t sensorMax, float sensorAverage, bool debug)
     objectAvSize = filterApds8(objectAvSize_temp, 255, objectAvSize, false);
     delayMicroseconds(100);  
     
     // X/Y/X MOVEMENT FOR GESTURE DETECTION
     uint8_t movementGX_temp = readByte(APDS9500_ADDRESS, APDS9500_Gx); 
     uint8_t movementGY_temp = readByte(APDS9500_ADDRESS, APDS9500_Gy); 
     uint8_t movementGZ_temp = readByte(APDS9500_ADDRESS, APDS9500_Gz); 
     movementGXYZ[0] = filterApds8(movementGX_temp, 255, movementGXYZ[0], false);
     movementGXYZ[1] = filterApds8(movementGY_temp, 255, movementGXYZ[1], false);
     movementGXYZ[2] = filterApds8(movementGZ_temp, 255, movementGXYZ[2], false);
     delayMicroseconds(100);

     // CURSOR HORIZONTAL CENTER horizontal object center after IIR fi lter for cursor mode
     uint8_t cursorCenterX_L  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterCenterX_L);
     uint8_t cursorCenterX_H  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterCenterXY_H);
     uint16_t cursorCenterX_temp = (cursorCenterX_H<<8) | cursorCenterX_L;
     cursorCenterX = filterApds16(cursorCenterX_temp, 30000, cursorCenterX, false);
     delayMicroseconds(100);

     // CURSOR VERTICAL CENTER vertical object center after IIR fi lter for cursor mode
     uint8_t cursorCenterY_L  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterCenterY_L);
     uint8_t cursorCenterY_H  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterCenterXY_H);
     uint16_t cursorCenterY_temp = (cursorCenterY_H<<8) | cursorCenterY_L;
     cursorCenterY = filterApds16(cursorCenterY_temp, 30000, cursorCenterY, false);
     delayMicroseconds(100);

     // CURSOR BRIGHTNESS object brightness after IIR filter for cursor mode
     uint8_t cursorAverageY_L  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterAvgY_L);
     uint8_t cursorAverageY_H  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterAvgY_H);
     uint16_t cursorAverageY_temp = (cursorAverageY_H<<8) | cursorAverageY_L;
     cursorAverageY = filterApds16(cursorAverageY_temp, 30000, cursorAverageY, false);
     delayMicroseconds(100);

     // CURSOR SIZE object size after IIR fi lter for cursor mode
     uint8_t cursorSize_L  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterSize_L);
     uint8_t cursorSize_H  = readByte(APDS9500_ADDRESS, APDS9500_PositionFilterSize_H);
     uint16_t cursorSize_temp = (cursorSize_H<<8) | cursorSize_L;
     cursorSize = filterApds16(cursorSize_temp, 30000, cursorSize, false);
     delayMicroseconds(100);

     //GESTURE OBJECT BRIGHTNESS ACCUMULATION
     uint8_t LightAcc_L  = readByte(APDS9500_ADDRESS, APDS9500_LightAcc_L);
     uint8_t LightAcc_H  = readByte(APDS9500_ADDRESS, APDS9500_LightAcc_H);
     uint16_t lightAccumulation_temp = (LightAcc_H<<8) | LightAcc_L;
     lightAccumulation = filterApds16(lightAccumulation_temp, 30000, lightAccumulation, false);
     delayMicroseconds(100);

     //GESTURE OBJECT ANGLE ACCUMULATION
     uint8_t AngleAcc_L  = readByte(APDS9500_ADDRESS, APDS9500_AngleAcc_L);
     uint8_t AngleAcc_H  = readByte(APDS9500_ADDRESS, APDS9500_AngleAcc_H);
     uint16_t angleAccumulation_temp = (LightAcc_H<<8) | LightAcc_L;
     angleAccumulation = filterApds16(angleAccumulation_temp, 3000, angleAccumulation, false);
     delayMicroseconds(100);

     //45 DEGREE GESTURE X DIRECTION PARAMETER degree gesture detection x direction parameter
     uint8_t XAngleGainValue_temp = readByte(APDS9500_ADDRESS, APDS9500_XGainValue);
     XAngleGainValue = filterApds8(XAngleGainValue_temp, 255, XAngleGainValue, false);
     delayMicroseconds(100);

     //45 DEGREE GESTURE Y DIRECTION PARAMETER degree gesture detection x direction parameter
     uint8_t YAngleGainValue_temp = readByte(APDS9500_ADDRESS, APDS9500_YGainValue);
     YAngleGainValue = filterApds8(YAngleGainValue_temp, 255, YAngleGainValue, false);
     delayMicroseconds(100);    
 }



 


/**************************************************************************************************************
 *************************************** wifimon_sniffer_packet_handler ********************************************
 **************************************************************************************************************/  
void wifimon_sniffer_packet_handler( void* buff, wifi_promiscuous_pkt_type_t type );
static wifi_country_t wifi_country = {.cc="CN", .schan = 1, .nchan = 13}; 

lv_obj_t *wifimon_app_main_tile = NULL;
lv_obj_t *chart = NULL;
lv_obj_t *channel_select = NULL; 
lv_chart_series_t *ser1 = NULL;
lv_chart_series_t *ser2 = NULL;
lv_chart_series_t *ser3 = NULL;
lv_task_t *_wifimon_app_task = NULL;
int wifimon_display_timeout = 0;

lv_task_t * _Play_Target_Sound_task = nullptr;  //curt add

LV_IMG_DECLARE(exit_dark_48px);
LV_IMG_DECLARE(wifimon_app_32px);
LV_FONT_DECLARE(Ubuntu_72px);

static void exit_wifimon_app_main_event_cb( lv_obj_t * obj, lv_event_t event );
static void wifimon_sniffer_set_channel( uint8_t channel );
static void wifimon_app_task( lv_task_t * task );
static void wifimon_activate_cb( void );
static void wifimon_hibernate_cb( void );

static void Play_Target_Sound_task( lv_task_t * task );   //curt add
static void wifimon_test_play_sound( void );


uint8_t level = 0, channel = 1;
int data = 0, mgmt = 0, misc = 0; 


void wifimon_sniffer_packet_handler( void* buff, wifi_promiscuous_pkt_type_t type ) {
    switch( type ) {
        case WIFI_PKT_MGMT: 
            mgmt++;
            break;
        case WIFI_PKT_DATA:
            data++; 
            break; 
        default:  
            misc++;
            break;
    }
}

/**************************************************************************************************************
 *************************************** wifimon_app_main_setup ********************************************
 **************************************************************************************************************/  
static void wifimon_sniffer_set_channel( uint8_t channel ) {
#ifdef NATIVE_64BIT

#else
    esp_wifi_set_channel( channel, WIFI_SECOND_CHAN_NONE );
#endif
    log_i("set wifi channel: %d", channel );
}

static void wifimon_channel_select_event_handler( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case LV_EVENT_VALUE_CHANGED: {
            char buf[32];
            lv_roller_get_selected_str( obj, buf, sizeof( buf ) );
            wifimon_sniffer_set_channel( atoi(buf) );
            break;
        }
    }
}


 //curt add
static void Play_Target_Sound_task( lv_task_t * task ){   //curt add
        sound_set_enabled_config( true );
    sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    sound_play_progmem_wav( piep_wav, piep_wav_len ); 
    motor_vibe(100); 
}

static void wifimon_test_play_sound( void ) {
        sound_set_enabled_config( true );
    sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    sound_play_progmem_wav(test_c_mouth_wav, test_c_mouth_wav_len);
}

/***********************************************************************************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 **********************************************************************************************************/  

void wifimon_app_main_setup( uint32_t tile_num ) {
    log_i("----------------- CURT -------- wifimon_app_main_setup__START $$$ TENSORFLOW $$$");
    //********************************************* CURT ADD TF ***********************************************************************************************************
    ml.begin(sine_model);

    //********************************************* CURT ADD BMI055 ***********************************************************************************************************
    // The Angular Rate Range Measurement and Selection of the Angular Rate Data Filter Bandwidth
    // can be changed via the following function:
    BMG160_setGyroRange(GYRO_RANGE_2000);                   // ± 2000 °/s
    // bmg.setGyroRange(GYRO_RANGE_1000);                   // ± 1000 °/s
    // bmg.setGyroRange(GYRO_RANGE_500);                    // ± 500 °/s
    // bmg.setGyroRange(GYRO_RANGE_250);                    // ± 250 °/s
    // bmg.setGyroRange(GYRO_RANGE_125);                    // ± 125 °/s

    BMG160_setGyroBandwidth(GYRO_BANDWIDTH_200_23HZ);       // ODR: 200 Hz, Filter Bandwidth: 23 Hz
    // bmg.setGyroBandwidth(GYRO_BANDWIDTH_2000_UNFILTERED);   // ODR: 2000 Hz, Filter Bandwidth: Unfiltered (523 Hz)
    // bmg.setGyroBandwidth(GYRO_BANDWIDTH_2000_230HZ);     // ODR: 2000 Hz, Filter Bandwidth: 230 Hz
    // bmg.setGyroBandwidth(GYRO_BANDWIDTH_1000_116HZ);     // ODR: 1000 Hz, Filter Bandwidth: 116 Hz
    // bmg.setGyroBandwidth(GYRO_BANDWIDTH_400_47HZ);       // ODR: 400 Hz, Filter Bandwidth: 47 Hz
    // bmg.setGyroBandwidth(GYRO_BANDWIDTH_100_12HZ);       // ODR: 100 Hz, Filter Bandwidth: 12 Hz
    // bmg.setGyroBandwidth(GYRO_BANDWIDTH_200_64HZ);       // ODR: 200 Hz, Filter Bandwidth: 64 Hz
    // bmg.setGyroBandwidth(GYRO_BANDWIDTH_100_32HZ);       // ODR: 100 Hz, Filter Bandwidth: 32 Hz

    BMG160_begin();
    BMG160_setUpSensor();

    BMA2X2_setRange(BMA_RANGE_2G);
    BMA2X2_setBandwidth(BMA_BANDWIDTH_1000HZ);
    BMA2X2_setFifo();



/********************************************************************************************************/
/************************ APDS 9500 INITIALIZATION AND DIAGNOSTICS **************************************/
/********************************************************************************************************/

  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select bank 0, wake up APDS9500 I2C
  delay(15);

  uint8_t partID_L  = readByte(APDS9500_ADDRESS, APDS9500_PartID_L);
  uint8_t partID_H  = readByte(APDS9500_ADDRESS, APDS9500_PartID_H);
  uint8_t versionID = readByte(APDS9500_ADDRESS, APDS9500_VersionID);
  //Serial.println("APDS9500 Gesture Sensor");
  //Serial.print(" Part ID = 0x"); Serial.print(partID_L, HEX);  
  //Serial.print(" and 0x"); Serial.println(partID_H, HEX);
  //Serial.println(" Part ID should be 0x20 and 0x76!");
  //Serial.println("  ");
  //Serial.print("#1 Version ID = 0x0"); Serial.println(versionID, HEX); 

  //if(partID_L == 0x20 && partID_H == 0x76)
  //{
    //Serial.println("APDS9500 ID confirmed!");
  //}
  //delay(10);
  delay(5);

  /* Initialize Gesture Sensor */
  // Choose bank 0
  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select bank 0

  // Define cursor limits
  writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampLeft, 0x07);    // min horiz value
  writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampRight,0x17);    // max horiz value
  writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampUp,0x03);       // min vert value CURT CHANGE
  writeByte(APDS9500_ADDRESS, APDS9500_R_Int2_En,0x01);             // enable interrupt on proximity
  //CURT ADD CLAMP DOWN
  writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampDown,0x18);       // max vert value default is 12

  //CURT ADD PROXIMITY SETTINGS
  writeByte(APDS9500_ADDRESS, R_Prox_UB, 0xFF);                              //default  , 7:0 0xC8 R/W Proximity up bound
  writeByte(APDS9500_ADDRESS, R_Prox_LB, 0x15);                               // default //7:0 0x40 R/W Proximity low bound



  // Auto exposure/Auto gain Controls
  writeByte(APDS9500_ADDRESS, APDS9500_R_AELedOff_UB, 0x2D);        // exposure time upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AELedOff_LB, 0x0F);        // exposure time lower bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_UB_L, 0x3C);   // low byte auto exposure upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_UB_H, 0x00);   // high byte auto exposure upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_LB_L, 0x1E);   // low byte auto exposure lower bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Gain_LB, 0x20);         // auto gain upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_Manual, 0x10);             // enable auto exposure
  writeByte(APDS9500_ADDRESS, 0x5E, 0x10);                          // don't know
  writeByte(APDS9500_ADDRESS, 0x60, 0x27);                          // don't know
  // Set up Interrupt
  writeByte(APDS9500_ADDRESS, APDS9500_InputMode_GPIO_0_1, 0x42);   // set GPIO0 as OUTPUT, GPIO1 as INPUT
  writeByte(APDS9500_ADDRESS, APDS9500_InputMode_GPIO_2_3, 0x44);   // set GPIO2 as INPUT, GPIO3 as INPUT
  writeByte(APDS9500_ADDRESS, APDS9500_InputMode_INT, 0x04);        // set INT as INPUT
  // Detection thresholds
  writeByte(APDS9500_ADDRESS, APDS9500_R_Cursor_ObjectSizeTh, 0x01); // object size threshold for cursor mode
  writeByte(APDS9500_ADDRESS, APDS9500_R_NoMotionCountThd, 0x06);    // no motion counter threshold
  
  writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionThd, 0x0A);       // gesture detection z threshold
  //writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionThd, 0x15);       // gesture detection z threshold   ** CURT CHANGE
  
  writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionXYThd, 0x0C);     // gesture detection x and y thresholds
  
  writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionAngleThd, 0x05);  // angle threshold for forward and backward detection
  writeByte(APDS9500_ADDRESS, APDS9500_R_RotateXYThd, 0x14);         // rotation detection threshold
  writeByte(APDS9500_ADDRESS, APDS9500_R_Filter, 0x3F);              // filter weight and frame position threshold
  writeByte(APDS9500_ADDRESS, APDS9500_R_FilterImage, 0x19);         // use pixel brightness for weak average filter
  writeByte(APDS9500_ADDRESS, APDS9500_R_YtoZSum, 0x19);             // z-direction mapping parameter
  writeByte(APDS9500_ADDRESS, APDS9500_R_YtoZFactor, 0x0B);          // z-direction mapping parameter
  writeByte(APDS9500_ADDRESS, APDS9500_R_FilterLength, 0x03);        // filter length for cursor object center
  writeByte(APDS9500_ADDRESS, APDS9500_R_WaveThd, 0x64);             // wave gesture counter and angle thresholds
  writeByte(APDS9500_ADDRESS, APDS9500_R_AbortCountThd, 0x21);       // abort gesture counter threshold

  // Change to Bank 1
  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x01);          // select bank 1

//CURT ADD LED POWER BOOST OVERIRDE
  writeByte(APDS9500_ADDRESS, R_LED1_DAC_UB, 0x20);

  // Image size settings  
  writeByte(APDS9500_ADDRESS, APDS9500_Cmd_HStart, 0x0F);            // horizontal starting point
  writeByte(APDS9500_ADDRESS, APDS9500_Cmd_VStart, 0x10);            // vertical starting point
  writeByte(APDS9500_ADDRESS, APDS9500_Cmd_HV, 0x02);                // vertical flip
  writeByte(APDS9500_ADDRESS, APDS9500_R_LensShadingComp_EnH, 0x01); // enable lens shading compensation
  writeByte(APDS9500_ADDRESS, APDS9500_R_Offest_Y, 0x39);            // vertical offset of lens, set to 55
  writeByte(APDS9500_ADDRESS, APDS9500_R_LSC, 0x7F);                 // Lens shading coefficient, set to 127
  writeByte(APDS9500_ADDRESS, APDS9500_R_LSFT, 0x08);                // shift amount, initialize to 10
  writeByte(APDS9500_ADDRESS, 0x3E,0xFF);                            // don't know
  writeByte(APDS9500_ADDRESS, 0x5E,0x3D);                            // don't know
  /* Sleep mode parameters */
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_L, 0x96);         // idle time low byte = 150 which is set for ~120 fps
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_SLEEP_1_L, 0x97); // idle time for weak sleep, set for report rate ~ 120 Hz
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_SLEEP_2_L, 0xCD); // idle time for deep sleep, low byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_SLEEP_2_H, 0x01); // idle time for deep sleep, high byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_Object_TIME_2_L, 0x2C);     // deep sleep enter time, low byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_Object_TIME_2_H, 0x01);     // deep sleep enter time, high byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_TG_EnH, 0x01);              // enable time gating
  writeByte(APDS9500_ADDRESS, APDS9500_R_Auto_SLEEP_Mode, 0x35);     // no object weak and deep sleep, object wake
  writeByte(APDS9500_ADDRESS, APDS9500_R_Wake_Up_Sig_Sel, 0x00);     // interrupt on time gate start

  /* ENABLE CAMERA IMAGE OUT OVER SPI, CURT ADD */ 
//writeByte(APDS9500_ADDRESS, R_SPIOUT_EnH, 0x00);
  
  /* Start sensor */
  writeByte(APDS9500_ADDRESS, APDS9500_R_SRAM_Read_EnH, 0x01);       //SRAM read enable

  // Change back to bank 0 for data read
  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select bank 0

  //getEnabled = readByte(APDS9500_ADDRESS, APDS9500_R_GestureDetEn);
  //if(getEnabled & 0x10) Serial.println("ROTATE gesture detection enabled");
  //if(getEnabled & 0x20) Serial.println("BACKWARD and FORWARD gesture detection enabled");
  //if(getEnabled & 0x40) Serial.println("UP and DOWN gesture detection enabled");
  //if(getEnabled & 0x80) Serial.println("LEFT and RIGHT gesture detection enabled");

  //getEnabled = readByte(APDS9500_ADDRESS, APDS9500_R_WaveEnH);
  //if(getEnabled & 0x80) Serial.println("WAVE gesture detection enabled");
  //  disable wave gesture
  //  writeByte(APDS9500_ADDRESS, APDS9500_R_WaveEnH, getEnabled & ~(0x80) );
  //  getEnabled = readByte(APDS9500_ADDRESS, APDS9500_R_WaveEnH);
  //  if(getEnabled & 0x80) Serial.println("WAVE gesture detection enabled");
    // CURT ADD !!!!!!!!!!!!!!!!
//sound_play_progmem_wav(piep_wav, piep_wav_len);
//sound_play_progmem_wav(piep_wav, 12318);


    log_i("----------------- CURT -------- wifimon_app_main_setup");
    wifimon_app_main_tile = mainbar_get_tile_obj( tile_num );
    /**
     * add chart widget
     */
    chart = lv_chart_create( wifimon_app_main_tile, NULL );
    lv_obj_set_size( chart, lv_disp_get_hor_res( NULL ), lv_disp_get_ver_res( NULL ) - THEME_ICON_SIZE );
    lv_obj_align( chart, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0 );
    lv_chart_set_type( chart, LV_CHART_TYPE_LINE );  
    lv_chart_set_point_count( chart, 32 );
    lv_obj_set_style_local_bg_opa( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_OPA_50 );
    lv_obj_set_style_local_bg_grad_dir( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_GRAD_DIR_VER );
    lv_obj_set_style_local_bg_main_stop( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 255 );
    lv_obj_set_style_local_bg_grad_stop( chart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 0 );
    /**
     * add chart series
     */
    ser1 = lv_chart_add_series( chart, LV_COLOR_RED );
    ser2 = lv_chart_add_series( chart, LV_COLOR_GREEN );
    ser3 = lv_chart_add_series( chart, LV_COLOR_YELLOW );
    /**
     * add exit button
     */
    lv_obj_t * exit_btn = wf_add_exit_button( wifimon_app_main_tile, exit_wifimon_app_main_event_cb );
    lv_obj_align( exit_btn, wifimon_app_main_tile, LV_ALIGN_IN_BOTTOM_LEFT, THEME_ICON_PADDING, -THEME_ICON_PADDING );
    /**
     * add channel select roller
     */
    channel_select = lv_roller_create(wifimon_app_main_tile, NULL);
    lv_roller_set_options( channel_select, "IMU\nTherm\nLidar\nIR\nTemp\nLight\nRGB\nAcc\nGyro\nDist\nProx\nAct\nResp", LV_ROLLER_MODE_INIFINITE );
    lv_roller_set_visible_row_count( channel_select, 5 );
    lv_obj_align( channel_select, NULL, LV_ALIGN_IN_TOP_LEFT, THEME_ICON_PADDING, THEME_ICON_PADDING );
    lv_obj_set_event_cb( channel_select, wifimon_channel_select_event_handler );
    /**
     * add chart series label
     */
    lv_obj_t * chart_series_label = lv_label_create( wifimon_app_main_tile, NULL );
    lv_label_set_long_mode( chart_series_label, LV_LABEL_LONG_BREAK );
    lv_label_set_recolor( chart_series_label, true );
    lv_label_set_align( chart_series_label, LV_LABEL_ALIGN_RIGHT );       
    lv_label_set_text( chart_series_label, "#ffff00 - xxxx#\n#ff0000 - yyyy#\n#11ff00 - zzzz#"); 
    lv_obj_set_width( chart_series_label, 70 );
    lv_obj_align( chart_series_label, NULL, LV_ALIGN_IN_TOP_RIGHT, -THEME_ICON_PADDING, THEME_ICON_PADDING );

    mainbar_add_tile_activate_cb( tile_num, wifimon_activate_cb );
    mainbar_add_tile_hibernate_cb( tile_num, wifimon_hibernate_cb );
}

/**************************************************************************************************************
 *************************************** exit_wifimon_app_main_event_cb ********************************************
 **************************************************************************************************************/  

static void exit_wifimon_app_main_event_cb( lv_obj_t * obj, lv_event_t event ) {
    switch( event ) {
        case( LV_EVENT_CLICKED ):     mainbar_jump_back();
                                      break;
    }
}

/**************************************************************************************************************
 *************************************** wifimon_hibernate_cb ********************************************
 **************************************************************************************************************/  
static void wifimon_hibernate_cb( void ) {
    log_i("----------------- CURT -------- wifimon_hibernate_cb");
    if(_wifimon_app_task != NULL) {
        lv_task_del(_wifimon_app_task);
        _wifimon_app_task = NULL;
    }  

    //sound_set_enabled_config( true );   //CURT ADD
    //sound_play_spiffs_mp3("/gui/sound/eyes.mp3");
    
    esp_wifi_set_promiscuous( false ); 

    wifictl_off();
    /**
     * restore display timeout time
     */
    display_set_timeout( wifimon_display_timeout );
}

/**************************************************************************************************************
 *************************************** wifimon_activate_cb ********************************************
 **************************************************************************************************************/  
static void wifimon_activate_cb( void ) {
    log_i("----------------- CURT -------- wifimon_activate_cb");
 //   alarm_in_progress_start_alarm();

    sound_set_enabled_config( true );
    sound_play_spiffs_mp3("/gui/sound/eyes.mp3");

    sound_set_enabled_config( true );   
    sound_play_progmem_wav(test_c_mouth_wav, test_c_mouth_wav_len);
    /**
     * restart wifi
     */
    wifictl_off();
    /**
     * setup promiscuous mode
     */

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init( &cfg );
    esp_wifi_set_country( &wifi_country );
    esp_wifi_set_mode( WIFI_MODE_NULL ); 
    esp_wifi_start();
    esp_wifi_set_promiscuous( true );
    esp_wifi_set_promiscuous_rx_cb( &wifimon_sniffer_packet_handler );
    lv_roller_set_selected( channel_select, 0, LV_ANIM_OFF );
    wifimon_sniffer_set_channel( 1 );

    /**
     * start stats fetch task
     */
    _wifimon_app_task = lv_task_create( wifimon_app_task, 100, LV_TASK_PRIO_MID, NULL );
    /**
     * save display timeout time
     */
    wifimon_display_timeout = display_get_timeout();
    display_set_timeout( DISPLAY_MAX_TIMEOUT );
}

/**************************************************************************************************************
 *************************************** wifimon_app_task ********************************************
 *  *************************************** wifimon_app_task ********************************************
 *  *************************************** wifimon_app_task ********************************************
 *  *************************************** wifimon_app_task ********************************************
 *  *************************************** wifimon_app_task ********************************************
 **************************************************************************************************************/  
static void wifimon_app_task( lv_task_t * task ) {
    /**
     * limit scale
     */

    // ------------- CURT ADD ----------------------
    log_i("----------------- CURT -------- wifimon_app_task");
        TTGOClass * ttgo = TTGOClass::getWatch();

    Accel acc;
    ttgo->bma->getAccel(acc);
    log_i("acc.x: %d", acc.x);
    log_i("acc.y: %d", acc.y);

    //int16_t x = acc.x * MOUSE_SENSIVITY;
    //int16_t y = acc.y * MOUSE_SENSIVITY;
    mgmt = ((acc.x + 1000) / 20);
    data = ((acc.y + 1000) / 20);

    if( mgmt < 0 ) mgmt = 0; 
    if( data < 0 ) data = 0; 
    if( misc < 0 ) misc = 0; 
    if( mgmt > 100 ) mgmt = 100; 
    if( data > 100 ) data = 100; 
    if( misc > 100 ) misc = 100; 

  //********************************************* CURT ADD MLX90615 ***********************************************************************************************************
    TObjectM15[0] = readObjectTempF(1);
    TObjectM15[1] = readObjectTempF(2);
    TObjectM15[2] = readObjectTempF(3);

    TDeviceM15[0] = readAmbientTempF(1);
    TDeviceM15[1] = readAmbientTempF(2);
    TDeviceM15[2] = readAmbientTempF(3);
    TDeviceAvMLX15 = (TDeviceM15[0] + TDeviceM15[1] + TDeviceM15[2]) / 3;

    log_i("T1: %f", TObjectM15[0]);
    log_i("T2: %f", TObjectM15[1]);
    log_i("T3: %f", TObjectM15[2]);
    log_i("TD: %f", TDeviceAvMLX15);


//********************************************* CURT ADD APDS9500 ***************************************************************************************************************
 //   uint8_t partID_L  = readByte(APDS9500_ADDRESS, APDS9500_PartID_L);
 //   uint8_t partID_H  = readByte(APDS9500_ADDRESS, APDS9500_PartID_H);
 //   log_i("APDS partID_L: %d", partID_L);
 //   log_i("APDS partID_H: %d", partID_H);

    readAPDS9500();
    log_i("APDS objectX: %f", objectX);
    log_i("APDS objectY: %f", objectY);
    log_i("APDS objectSize: %f", objectSize);


  //********************************************* CURT ADD BMI055 ***********************************************************************************************************
    BMG160_Measure_Gyroscope();


    log_i("BMI gX: %d", gyroX);
    log_i("BMI gY: %d", gyroY);
    log_i("BMI gZ: %d", gyroZ);

    //BMA2X2_getSampleFromBuffer();
    BMA2X2_Measure_Accelerometer();

    log_i("BMI aX: %d", accelX);
    log_i("BMI aY: %d", accelY);
    log_i("BMI aZ: %d", accelZ);


  //********************************************* CURT ADD TF TEST ***********************************************************************************************************
    for (float i = 0; i < 10; i++) {
        // pick x from 0 to PI
        float x = 3.14 * i / 10;
        float y = sin(x);
        float input[1] = { x };
        float predicted = ml.predict(input);

        log_i("sin: %f", x);
        log_i("= %f", y);
        log_i("predicted: %f", predicted);
    }

    //********************************************* CURT ADD AUDIO ***********************************************************************************************************
    // CURT ADD !!!!!!!!!!!!!!!!
    if(mgmt > 95){
        wifimon_test_play_sound();
    }
    if(data > 95){
        wifimon_test_play_sound();
    }

    /**
     * scan i2c devices
     */
    for( uint8_t address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        if ( Wire.endTransmission() == 0 )
            log_i("I2C device at: 0x%02x", address );

    }
    // ------------------ END CURT ADD ------------------


  //  if( mgmt > 100 ) mgmt = 100; 
   // if( data > 100 ) data = 100; 
   // if( misc > 100 ) misc = 100; 
    /**
     * add seria data
     */
    lv_chart_set_next(chart, ser1, mgmt);
    lv_chart_set_next(chart, ser2, data);
    lv_chart_set_next(chart, ser3, misc);
    /**
     * refresh chart
     */
    lv_chart_refresh(chart);
    /**
     * reset packet counter
     */
    data = 0;
    mgmt = 0;
    misc = 0; 
}









