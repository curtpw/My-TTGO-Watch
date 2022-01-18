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
#include <stdint.h>
#include <lwip/sockets.h>
#include "esp_wifi.h"

//CURT
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              TENSORFLOW VARIABLES                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

#include <EloquentTinyML.h>
// sine_model.h contains the array you exported from the previous step with xxd or tinymlgen
#include "sine_model.h"
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 1
// in future projects you may need to tweak this value: it's a trial and error process
#define TENSOR_ARENA_SIZE 2*1024

Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;


/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              MLX90615 VARIABLES                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

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

float read16(uint8_t a, int sensorNum);
float readTemp(uint8_t reg, int sensorNum);
double readObjectTempF(int sensorNum);
double readAmbientTempF(int sensorNum);
double readObjectTempC(int sensorNum);
double readAmbientTempC(int sensorNum);




/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              MLX90641 VARIABLES                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

    #define I2C_BUFFER_LENGTH 32



    void MLX90641_I2CInit(void);
   // int MLX90641_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nWordsRead, uint16_t *data);
    int MLX90641_I2CRead(uint8_t _deviceAddress, uint16_t startAddress, uint16_t nWordsRead, uint16_t *data);
    int MLX90641_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);
    void MLX90641_I2CFreqSet(int freq);


void MLX90641_I2CInit()
{   
   // i2c.stop();
}



//int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
//{
   
    /*
    char cmd[2] = {0,0};
    char i2cData[1664] = {0};
    uint16_t *p;
    
    p = data;
     cmd[0] = startAddress >> 8;
    cmd[1] = startAddress & 0x00FF;
    
    uint8_t sa;                           
    int ack = 0;                               
    int cnt = 0;
    int i = 0;
    sa = (slaveAddr << 1);
    
    i2c.stop();
    wait_us(5);    
    ack = i2c.write(sa, cmd, 2, 1);
    
    if (ack != 0x00)
    {
        return -1;
    }
             
    sa = sa | 0x01;
    ack = i2c.read(sa, i2cData, 2*nMemAddressRead, 0);
    
    if (ack != 0x00)
    {
        return -1; 
    }          
    i2c.stop();   
    
    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i]*256 + (uint16_t)i2cData[i+1];
    }*/
   /* 
    uint16_t bytesRemaining = nMemAddressRead * 2;
    uint16_t dataSpot = 0; //Start at beginning of array
    //Setup a series of chunked I2C_BUFFER_LENGTH byte reads
    while (bytesRemaining > 0) {
        Wire.beginTransmission(slaveAddr);
        Wire.write(startAddress >> 8); //MSB
        Wire.write(startAddress & 0xFF); //LSB
        if (Wire.endTransmission(false) != 0) { //Do not release bus
            //Serial.println("No ack read");
            return (0); //Sensor did not ACK
        }

        uint16_t numberOfBytesToRead = bytesRemaining;
        if (numberOfBytesToRead > I2C_BUFFER_LENGTH) {
            numberOfBytesToRead = I2C_BUFFER_LENGTH;
        }

        Wire.requestFrom((uint8_t)slaveAddr, numberOfBytesToRead);
        if (Wire.available()) {
            for (uint16_t x = 0 ; x < numberOfBytesToRead / 2; x++) {
                //Store data into array
                data[dataSpot] = Wire.read() << 8; //MSB
                data[dataSpot] |= Wire.read(); //LSB

                dataSpot++;
            }
        }else
            return -1;

        bytesRemaining -= numberOfBytesToRead;

        startAddress += numberOfBytesToRead / 2;
    }

    return 0;   
*/
//int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)

//int MLX90641_I2CRead(uint8_t _deviceAddress, unsigned int startAddress, unsigned int nWordsRead, uint16_t *data)
int MLX90641_I2CRead(uint8_t _deviceAddress, uint16_t startAddress, uint16_t nWordsRead, uint16_t *data)
{
    // nWordsRead must be <= 32767
    Wire.beginTransmission(_deviceAddress);
    Wire.write(startAddress >> 8); //MSB
    Wire.write(startAddress & 0xFF); //LSB
    Wire.endTransmission(false);
    i2c_err_t error = Wire.readTransmission(_deviceAddress, (uint8_t*) data, nWordsRead*2);
    if(error != 0){//problems
        Serial.printf("Block read from sensor(0x%02X) at address=%d of %d uint16_t's failed=%d(%s)\n",
          _deviceAddress,startAddress,nWordsRead,error,Wire.getErrorText(error));
    }
    else { // reverse byte order, sensor Big Endian, ESP32 Little Endian
        for(auto a = 0; a<nWordsRead; a++){
            data[a] = ((data[a] & 0xff)<<8) | (( data[a]>>8)&0xff);
        }
    }
    return 0;
}
//} 









/*

int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
  */ 
    /*
    char cmd[2] = {0,0};
    char i2cData[1664] = {0};
    uint16_t *p;
    
    p = data;
     cmd[0] = startAddress >> 8;
    cmd[1] = startAddress & 0x00FF;
    
    uint8_t sa;                           
    int ack = 0;                               
    int cnt = 0;
    int i = 0;
    sa = (slaveAddr << 1);
    
    i2c.stop();
    wait_us(5);    
    ack = i2c.write(sa, cmd, 2, 1);
    
    if (ack != 0x00)
    {
        return -1;
    }
             
    sa = sa | 0x01;
    ack = i2c.read(sa, i2cData, 2*nMemAddressRead, 0);
    
    if (ack != 0x00)
    {
        return -1; 
    }          
    i2c.stop();   
    
    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i]*256 + (uint16_t)i2cData[i+1];
    }*/
    /*
    uint16_t bytesRemaining = nMemAddressRead * 2;
    uint16_t dataSpot = 0; //Start at beginning of array
    //Setup a series of chunked I2C_BUFFER_LENGTH byte reads
    while (bytesRemaining > 0) {
        Wire.beginTransmission(slaveAddr);
        Wire.write(startAddress >> 8); //MSB
        Wire.write(startAddress & 0xFF); //LSB
        if (Wire.endTransmission(false) != 0) { //Do not release bus
            //Serial.println("No ack read");
            return (0); //Sensor did not ACK
        }

        uint16_t numberOfBytesToRead = bytesRemaining;
        if (numberOfBytesToRead > I2C_BUFFER_LENGTH) {
            numberOfBytesToRead = I2C_BUFFER_LENGTH;
        }

        Wire.requestFrom((uint8_t)slaveAddr, numberOfBytesToRead);
        if (Wire.available()) {
            for (uint16_t x = 0 ; x < numberOfBytesToRead / 2; x++) {
                //Store data into array
                data[dataSpot] = Wire.read() << 8; //MSB
                data[dataSpot] |= Wire.read(); //LSB

                dataSpot++;
            }
        }else
            return -1;

        bytesRemaining -= numberOfBytesToRead;

        startAddress += numberOfBytesToRead / 2;
    }

    return 0;   
} 

*/

void MLX90641_I2CFreqSet(int freq)
{
//    i2c.frequency(1000*freq);
    Wire.setClock((long)1000 * freq);
}

int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    /*
    uint8_t sa;
    int ack = 0;
    char cmd[4] = {0,0,0,0};
    static uint16_t dataCheck;
    

    sa = (slaveAddr << 1);
    cmd[0] = writeAddress >> 8;
    cmd[1] = writeAddress & 0x00FF;
    cmd[2] = data >> 8;
    cmd[3] = data & 0x00FF;

    i2c.stop();
    wait_us(5);    
    ack = i2c.write(sa, cmd, 4, 0);
    
    if (ack != 0x00)
    {
        return -1;
    }         
    i2c.stop();   
    
    MLX90641_I2CRead(slaveAddr,writeAddress,1, &dataCheck);
    
    if ( dataCheck != data)
    {
        return -2;
    }    */
    Wire.beginTransmission((uint8_t)slaveAddr);
    Wire.write(writeAddress >> 8); //MSB
    Wire.write(writeAddress & 0xFF); //LSB
    Wire.write(data >> 8); //MSB
    Wire.write(data & 0xFF); //LSB
    if (Wire.endTransmission() != 0) {
        //Sensor did not ACK
        //Serial.println("Error: Sensor did not ack");
        return (-1);
    }

    uint16_t dataCheck;
    MLX90641_I2CRead(slaveAddr, writeAddress, 1, &dataCheck);
    if (dataCheck != data) {
        //Serial.println("The write request didn't stick");
        return -2;
    }
    return 0;
}


#define SCALEALPHA 0.000001
    
  typedef struct
    {
        int16_t kVdd;
        int16_t vdd25;
        float KvPTAT;
        float KtPTAT;
        uint16_t vPTAT25;
        float alphaPTAT;
        int16_t gainEE;
        float tgc;
        float cpKv;
        float cpKta;
        uint8_t resolutionEE;
        uint8_t calibrationModeEE;
        float KsTa;
        float ksTo[8];
        int16_t ct[8];
        uint16_t alpha[192]; 
        uint8_t alphaScale;   
        int16_t offset[2][192];    
        int8_t kta[192];
        uint8_t ktaScale;    
        int8_t kv[192];
        uint8_t kvScale;
        float cpAlpha;
        int16_t cpOffset;
        float emissivityEE; 
        uint16_t brokenPixels[2];
    } paramsMLX90641;
    
    int MLX90641_DumpEE(uint8_t slaveAddr, uint16_t *eeData);
    int MLX90641_CheckEEPROMValid(uint16_t *eeData); 

    int MLX90641_GetFrameData(uint8_t slaveAddr, uint16_t *frameData);
    int MLX90641_ExtractParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
    float MLX90641_GetVdd(uint16_t *frameData, const paramsMLX90641 *params);
    float MLX90641_GetTa(uint16_t *frameData, const paramsMLX90641 *params);
    void MLX90641_GetImage(uint16_t *frameData, const paramsMLX90641 *params, float *result);
    void MLX90641_CalculateTo(uint16_t *frameData, const paramsMLX90641 *params, float emissivity, float tr, float *result);
    int MLX90641_SetResolution(uint8_t slaveAddr, uint8_t resolution);
    int MLX90641_GetCurResolution(uint8_t slaveAddr);
    int MLX90641_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate);   
    int MLX90641_GetRefreshRate(uint8_t slaveAddr);  
    int MLX90641_GetSubPageNumber(uint16_t *frameData);
    float MLX90641_GetEmissivity(const paramsMLX90641 *mlx90641);
    void MLX90641_BadPixelsCorrection(uint16_t *pixels, float *to, paramsMLX90641 *params);


void MLX90641_ExtractVDDParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractPTATParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractGainParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractTgcParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractEmissivityParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractResolutionParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractKsTaParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractKsToParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractAlphaParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractOffsetParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
void MLX90641_ExtractCPParameters(uint16_t *eeData, paramsMLX90641 *mlx90641);
int MLX90641_ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90641 *mlx90641);
int MLX90641_HammingDecode(uint16_t *eeData);  

 /********************************************************************************************************/
/******************** MLX90641 STUFF *  MLX90641 STUFF * MLX90641 STUFF ***********************************/
/********************************************************************************************************/
const byte MLX90641_address = 0x33; //Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8 //Default shift for MLX90641 in open air

uint16_t eeMLX90641[832];
float MLX90641To[192];
uint16_t MLX90641Frame[242];
paramsMLX90641 MLX90641;
int errorno = 0;
float mlx90641ReducedTo[48] = {0.0};       //reduce to 8x6, 1/2 each side

bool MLX41_SETUP_FAIL = false;


/**************************************************************************************************************
 *************************************** READ MLX90641 SENSOR DATA ********************************************
 **************************************************************************************************************/  
 void readMLX90641(){
    long startTime = millis();
    float vdd = 0;
    float Ta = 0;
    
    for (byte x = 0 ; x < 2 ; x++) {
        int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);

        float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
        float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);

        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;

        MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
    }
    long stopTime = millis();

    //reduction local var
    float currentReducedM41[48] = {0.0};  //8x6
    int currentIndexM41 = 0;

    for (int y = 0 ; y < 12 ; y++) {
        for(int x = 0; x < 16; x++) {
          currentIndexM41 = y*12 + x*16;
          currentReducedM41[currentIndexM41 / 4] = currentReducedM41[currentIndexM41 / 4] + MLX90641To[currentIndexM41];  
          
  //        Serial.print(MLX90641To[ (y*12 + x) ], 2);
 //         Serial.print("\t");
        }
//        Serial.println("");
    }
//    Serial.println("");

    log_i("MLX90641: %f", MLX90641To[3]); //test

    for(int c = 0; c < 48; c++){
      currentReducedM41[c] = currentReducedM41[c] / 4;  //average each chunk of latest MLX90640 data
      mlx90641ReducedTo[c] = (mlx90641ReducedTo[c] + currentReducedM41[c]) / 2; //average with past data ie low pass filter
    }
 }

/**************************************************************************************************************
 ************************************** MLX90641 ISCONNECTED CHECK ********************************************
 **************************************************************************************************************/  
 
//Returns true if the MLX90641 is detected on the I2C bus
boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX90641_address);
    if (Wire.endTransmission() != 0) {
        return (false);    //Sensor did not ACK
    }
    return (true);
}






/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*                BMI055 VARIABLES                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

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



/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*                APDS9500 VARIABLES                   */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

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



void writeByte(uint8_t address, uint8_t subAddress, uint8_t data); 
 void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest); 
 uint8_t readByte(uint8_t address, uint8_t subAddress); 
 float filterApds8(uint8_t sensorValue, uint8_t sensorMax, float sensorAverage, bool debug);
 void readAPDS9500();



 /*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              VL53L0X VARIABLES                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

enum regAddr
    {
      SYSRANGE_START                              = 0x00,

      SYSTEM_THRESH_HIGH                          = 0x0C,
      SYSTEM_THRESH_LOW                           = 0x0E,

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
      SYSTEM_RANGE_CONFIG                         = 0x09,
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

      RESULT_INTERRUPT_STATUS                     = 0x13,
      RESULT_RANGE_STATUS                         = 0x14,

      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

      MSRC_CONFIG_CONTROL                         = 0x60,

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

      SYSTEM_HISTOGRAM_BIN                        = 0x81,
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
      IDENTIFICATION_MODEL_ID                     = 0xC0,
      IDENTIFICATION_REVISION_ID                  = 0xC2,

      OSC_CALIBRATE_VAL                           = 0xF8,

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

      ALGO_PHASECAL_LIM                           = 0x30,
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
    };

    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

    uint8_t last_status; // status of last I2C transmission

      //VL53L0X Distance
    float   data_VL53L0X_distance = 600.0; //default off target cut-off is 600mm

   // VL53L0X(void);

    void VL53L0X_setAddress(uint8_t new_addr);
    //inline uint8_t getAddress(void) { return address; }

    bool VL53L0X_init(bool io_2v8 = true);

    void VL53L0X_writeReg(uint8_t reg, uint8_t value);
    void VL53L0X_writeReg16Bit(uint8_t reg, uint16_t value);
    void VL53L0X_writeReg32Bit(uint8_t reg, uint32_t value);
    uint8_t VL53L0X_readReg(uint8_t reg);
    uint16_t VL53L0X_readReg16Bit(uint8_t reg);
    uint32_t VL53L0X_readReg32Bit(uint8_t reg);

    void VL53L0X_writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
    void VL53L0X_readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

    bool VL53L0X_setSignalRateLimit(float limit_Mcps);
    float VL53L0X_getSignalRateLimit(void);

    bool VL53L0X_setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t VL53L0X_getMeasurementTimingBudget(void);

    bool VL53L0X_setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
    uint8_t VL53L0X_getVcselPulsePeriod(vcselPeriodType type);

    void VL53L0X_startContinuous(uint32_t period_ms = 0);
    void VL53L0X_stopContinuous(void);
    uint16_t VL53L0X_readRangeContinuousMillimeters(void);
    uint16_t VL53L0X_readRangeSingleMillimeters(void);

    //inline void VL53L0X_setTimeout(uint16_t timeout) { io_timeout = timeout; }
   // inline uint16_t VL53L0X_getTimeout(void) { return io_timeout; }
    bool VL53L0X_timeoutOccurred(void);

 // private:
    // TCC: Target CentreCheck
    // MSRC: Minimum Signal Rate Check
    // DSS: Dynamic Spad Selection

    struct SequenceStepEnables
    {
      boolean tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts
    {
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    uint8_t address;
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;

    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t measurement_timing_budget_us;

    bool VL53L0X_getSpadInfo(uint8_t * count, bool * type_is_aperture);

    void VL53L0X_getSequenceStepEnables(SequenceStepEnables * enables);
    void VL53L0X_getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

    bool VL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte);

    static uint16_t decodeTimeout(uint16_t value);
    static uint16_t encodeTimeout(uint16_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks); 

    // Defines /////////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0b0101001

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)



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


/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/

/*                                             APP SETUP                                                    */

/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/
/************************************************************************************************************/
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/

/***********************************************************************************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 *************************************** wifimon_app_main_setup ********************************************
 **********************************************************************************************************/  

void wifimon_app_main_setup( uint32_t tile_num ) {
    log_i("----------------- CURT -------- wifimon_app_main_setup__START $$$ TENSORFLOW $$$");

//********************************************* SET I2C SPEED ***********************************************************************************************************
    //    i2c.frequency(1000*freq);
    Wire.setClock((long)1000 * 1000);
    delay(5);

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



/********************************************************************************************************/
/************************** VL53L0X INITIALIZATION AND DIAGNOSTICS **************************************/
/************************** VL53L0X INITIALIZATION AND DIAGNOSTICS **************************************/
/************************** VL53L0X INITIALIZATION AND DIAGNOSTICS **************************************/  
/********************************************************************************************************/

 // ========== INIT VL53L0X DISTANCE SENSOR ========== 
   // Serial.println("== INIT VL53L0X MICRO LIDAR DISTANCE SENSOR");
    io_timeout = 0;
    VL53L0X_init();
    //VL53L0X_init();
    //VL53L0X_init();
    //vl53l0x.setTimeout(500);

    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    VL53L0X_startContinuous(100);
  //  VL53L0X_startContinuous(100);
  //  VL53L0X_startContinuous(100);



  /********************************************************************************************************/
/*************************** MLX90641 INITIALIZATION AND DIAGNOSTICS **************************************/
/*************************** MLX90641 INITIALIZATION AND DIAGNOSTICS **************************************/
/*************************** MLX90641 INITIALIZATION AND DIAGNOSTICS **************************************/  
/********************************************************************************************************/
    delay(1);
    //MLX90640_I2CFreqSet(1000) sets frequency to 1MHz
 //   MLX90641_I2CFreqSet(1000);

   // delay(3);
    if (isConnected() == false) {
        log_i("MLX90641 not detected at default I2C address. Please check wiring. Freezing.");
       // while (1);
    } else {
        log_i("MLX90641 detected");
    }


    //weird online thingy https://www.instructables.com/Infrared-Thermal-Imaging-Camera-With-MLX90640-and-/
    MLX90641_I2CWrite(0x33, 0x800D, 6401);

    //Get device parameters - We only have to do this once
    int status;

    //delay(3);
    status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
    errorno = status;//MLX90641_CheckEEPROMValid(eeMLX90641);//eeMLX90641[10] & 0x0040;//
    
    if (status != 0) {
        log_i("MLX90641 Failed to load system parameters");
        MLX41_SETUP_FAIL = true;
    //   while(1);
    } else {
        log_i("MLX90641 load system parameters sucess");
    }

    //delay(3);
    status = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);
    //errorno = status;
    if (status != 0) {
        log_i("MLX90641 Parameter extraction failed");
        MLX41_SETUP_FAIL = true;
   //     while(1);
    }

   // delay(3);

    //Once params are extracted, we can release eeMLX90641 array

    //MLX90641_SetRefreshRate(MLX90641_address, 0x02); //Set rate to 2Hz
    MLX90641_SetRefreshRate(MLX90641_address, 0x03); //Set rate to 4Hz
    //MLX90641_SetRefreshRate(MLX90641_address, 0x07); //Set rate to 64Hz

delay(3);



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



  //********************************************* CURT ADD VL53L0X ***********************************************************************************************************
    if (VL53L0X_timeoutOccurred()) { 
        log_i("VL53L0X TIMEOUT"); 
        //!! FLAG BAD DATA
    } else {
        data_VL53L0X_distance = (float)VL53L0X_readRangeContinuousMillimeters();
        if(data_VL53L0X_distance > 9999){
            data_VL53L0X_distance = (float)VL53L0X_readRangeContinuousMillimeters();
        }
        //data_VL53L0X_distance = (float)VL53L0X_readRangeSingleMillimeters();
        log_i("VL53L0X distance: %f", data_VL53L0X_distance);
    }

      


  //********************************************* CURT ADD MLX90641 ***********************************************************************************************************
//================================== READ MLX90641 INTERVAL ===================================
  /*  if(MLX41_SETUP_FAIL){
        int status_1;
        int status_2;
        status_1 = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
        status_2 = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);

        if( (status_1 == 0) && (status_2 == 0) ){
            MLX41_SETUP_FAIL = false;
        } else {
            log_i("MLX90641 FAIL");
        }
    } else { */
  //      readMLX90641();
      //  delay(10);
   // }


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
     * scan i2c devices   I2C TWO WIRE SCANNER !!
     */
    /*
    for( uint8_t address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        if ( Wire.endTransmission() == 0 )
            log_i("I2C device at: 0x%02x", address );

    }
    */
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



/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              MLX90641 FUNCTIONS                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

//------------------------------------------------------------------------------
  
int MLX90641_DumpEE(uint8_t slaveAddr, uint16_t *eeData)
{
     int error = 1;
     error = MLX90641_I2CRead(slaveAddr, 0x2400, 832, eeData);
     if (error == 0)
     {
        error = MLX90641_HammingDecode(eeData);  
     }
         
     return error;
}

//------------------------------------------------------------------------------

int MLX90641_HammingDecode(uint16_t *eeData)
{
    int error = 0;
    int16_t parity[5];
    int8_t D[16];
    int16_t check;
    uint16_t data;
    uint16_t mask;
    
    for (int addr=16; addr<832; addr++)
    {
        parity[0] = -1;
        parity[1] = -1;
        parity[2] = -1;
        parity[3] = -1;
        parity[4] = -1;
        
        data = eeData[addr];
        mask = 1;
        for( int i = 0; i < 16; i++)
        {          
          D[i] = (data & mask) >> i;
          mask = mask << 1;
        }
        
        parity[0] = D[0]^D[1]^D[3]^D[4]^D[6]^D[8]^D[10]^D[11];
        parity[1] = D[0]^D[2]^D[3]^D[5]^D[6]^D[9]^D[10]^D[12];
        parity[2] = D[1]^D[2]^D[3]^D[7]^D[8]^D[9]^D[10]^D[13];
        parity[3] = D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[14];
        parity[4] = D[0]^D[1]^D[2]^D[3]^D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[11]^D[12]^D[13]^D[14]^D[15];
       
        if ((parity[0]!=0) || (parity[1]!=0) || (parity[2]!=0) || (parity[3]!=0) || (parity[4]!=0))
        {        
            check = (parity[0]<<0) + (parity[1]<<1) + (parity[2]<<2) + (parity[3]<<3) + (parity[4]<<4);
    
            if ((check > 15)&&(check < 32))
            {
                switch (check)
                {    
                    case 16:
                        D[15] = 1 - D[15];
                        break;
                    
                    case 24:
                        D[14] = 1 - D[14];
                        break;
                        
                    case 20:
                        D[13] = 1 - D[13];
                        break;
                        
                    case 18:
                        D[12] = 1 - D[12];
                        break;                                
                        
                    case 17:
                        D[11] = 1 - D[11];
                        break;
                        
                    case 31:
                        D[10] = 1 - D[10];
                        break;
                        
                    case 30:
                        D[9] = 1 - D[9];
                        break;
                    
                    case 29:
                        D[8] = 1 - D[8];
                        break;                
                    
                    case 28:
                        D[7] = 1 - D[7];
                        break;
                        
                    case 27:
                        D[6] = 1 - D[6];
                        break;
                            
                    case 26:
                        D[5] = 1 - D[5];
                        break;    
                        
                    case 25:
                        D[4] = 1 - D[4];
                        break;     
                        
                    case 23:
                        D[3] = 1 - D[3];
                        break; 
                        
                    case 22:
                        D[2] = 1 - D[2];
                        break; 
                            
                    case 21:
                        D[1] = 1 - D[1];
                        break; 
                        
                    case 19:
                        D[0] = 1 - D[0];
                        break;     
                                     
                }
               
                if(error == 0)
                {
                    error = -9;
                   
                }
                
                data = 0;
                mask = 1;
                for( int i = 0; i < 16; i++)
                {                    
                    data = data + D[i]*mask;
                    mask = mask << 1;
                }
       
            }
            else
            {
                error = -10;                
            }   
         }
        
        eeData[addr] = data & 0x07FF;
    }
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90641_GetFrameData(uint8_t slaveAddr, uint16_t *frameData)
{
    uint16_t dataReady = 1;
    uint16_t controlRegister1;
    uint16_t statusRegister;
    int error = 1;
    uint8_t cnt = 0;
    uint8_t subPage = 0;
    
    dataReady = 0;
    while(dataReady == 0)
    {
        error = MLX90641_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
    }   
    subPage = statusRegister & 0x0001;
        
    while(dataReady != 0 && cnt < 5)
    { 
        error = MLX90641_I2CWrite(slaveAddr, 0x8000, 0x0030);
        if(error == -1)
        {
            return error;
        }
            
        if(subPage == 0)
        { 
            error = MLX90641_I2CRead(slaveAddr, 0x0400, 32, frameData); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x0440, 32, frameData+32); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x0480, 32, frameData+64); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x04C0, 32, frameData+96); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x0500, 32, frameData+128); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x0540, 32, frameData+160); 
            if(error != 0)
            {
                return error;
            }
        }    
        else
        {
             error = MLX90641_I2CRead(slaveAddr, 0x0420, 32, frameData); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x0460, 32, frameData+32); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x04A0, 32, frameData+64); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x04E0, 32, frameData+96); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x0520, 32, frameData+128); 
            if(error != 0)
            {
                return error;
            }
            error = MLX90641_I2CRead(slaveAddr, 0x0560, 32, frameData+160); 
            if(error != 0)
            {
                return error;
            }
        }   
        
        error = MLX90641_I2CRead(slaveAddr, 0x0580, 48, frameData+192); 
        if(error != 0)
        {
            return error;
        }            
                   
        error = MLX90641_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
        subPage = statusRegister & 0x0001;      
        cnt = cnt + 1;
    }
    
    if(cnt > 4)
    {
        return -8;
    }    
    
    error = MLX90641_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    
    frameData[240] = controlRegister1;
    frameData[241] = statusRegister & 0x0001;
    
    if(error != 0)
    {
        return error;
    }
    
    return frameData[241];    
}

//------------------------------------------------------------------------------

int MLX90641_ExtractParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    int error = MLX90641_CheckEEPROMValid(eeData);
    
    if(error == 0)
    {
        MLX90641_ExtractVDDParameters(eeData, mlx90641);
        MLX90641_ExtractPTATParameters(eeData, mlx90641);
        MLX90641_ExtractGainParameters(eeData, mlx90641);
        MLX90641_ExtractTgcParameters(eeData, mlx90641);
        MLX90641_ExtractEmissivityParameters(eeData, mlx90641);
        MLX90641_ExtractResolutionParameters(eeData, mlx90641);
        MLX90641_ExtractKsTaParameters(eeData, mlx90641);
        MLX90641_ExtractKsToParameters(eeData, mlx90641);
        MLX90641_ExtractAlphaParameters(eeData, mlx90641);
        MLX90641_ExtractOffsetParameters(eeData, mlx90641);
        MLX90641_ExtractKtaPixelParameters(eeData, mlx90641);
        MLX90641_ExtractKvPixelParameters(eeData, mlx90641);
        MLX90641_ExtractCPParameters(eeData, mlx90641);
        error = MLX90641_ExtractDeviatingPixels(eeData, mlx90641);  
    }
    
    return error;

}

//------------------------------------------------------------------------------

int MLX90641_SetResolution(uint8_t slaveAddr, uint8_t resolution)
{
    uint16_t controlRegister1;
    int value;
    int error;
    
    value = (resolution & 0x03) << 10;
    
    error = MLX90641_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    
    if(error == 0)
    {
        value = (controlRegister1 & 0xF3FF) | value;
        error = MLX90641_I2CWrite(slaveAddr, 0x800D, value);        
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90641_GetCurResolution(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int resolutionRAM;
    int error;
    
    error = MLX90641_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    resolutionRAM = (controlRegister1 & 0x0C00) >> 10;
    
    return resolutionRAM; 
}

//------------------------------------------------------------------------------

int MLX90641_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate)
{
    uint16_t controlRegister1;
    int value;
    int error;
    
    value = (refreshRate & 0x07)<<7;
    
    error = MLX90641_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error == 0)
    {
        value = (controlRegister1 & 0xFC7F) | value;
        error = MLX90641_I2CWrite(slaveAddr, 0x800D, value);
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90641_GetRefreshRate(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int refreshRate;
    int error;
    
    error = MLX90641_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    refreshRate = (controlRegister1 & 0x0380) >> 7;
    
    return refreshRate;
}

//------------------------------------------------------------------------------

void MLX90641_CalculateTo(uint16_t *frameData, const paramsMLX90641 *params, float emissivity, float tr, float *result)
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP;
    float irData;
    float alphaCompensated;
    float Sx;
    float To;
    float alphaCorrR[8];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;
    
    subPage = frameData[241];
    vdd = MLX90641_GetVdd(frameData, params);
    ta = MLX90641_GetTa(frameData, params);    
    ta4 = (ta + 273.15);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    
    taTr = tr4 - (tr4-ta4)/emissivity;
    
    ktaScale = pow(2,(double)params->ktaScale);
    kvScale = pow(2,(double)params->kvScale);
    alphaScale = pow(2,(double)params->alphaScale);
    
    alphaCorrR[1] = 1 / (1 + params->ksTo[1] * 20);
    alphaCorrR[0] = alphaCorrR[1] / (1 + params->ksTo[0] * 20);
    alphaCorrR[2] = 1 ;
    alphaCorrR[3] = (1 + params->ksTo[2] * params->ct[3]);
    alphaCorrR[4] = alphaCorrR[3] * (1 + params->ksTo[3] * (params->ct[4] - params->ct[3]));
    alphaCorrR[5] = alphaCorrR[4] * (1 + params->ksTo[4] * (params->ct[5] - params->ct[4]));
    alphaCorrR[6] = alphaCorrR[5] * (1 + params->ksTo[5] * (params->ct[6] - params->ct[5]));
    alphaCorrR[7] = alphaCorrR[6] * (1 + params->ksTo[6] * (params->ct[7] - params->ct[6]));
    
//------------------------- Gain calculation -----------------------------------    
    gain = frameData[202];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = params->gainEE / gain; 
  
//------------------------- To calculation -------------------------------------        
    irDataCP = frameData[200];  
    if(irDataCP > 32767)
    {
        irDataCP = irDataCP - 65536;
    }
    irDataCP = irDataCP * gain;

    irDataCP = irDataCP - params->cpOffset * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    
    for( int pixelNumber = 0; pixelNumber < 192; pixelNumber++)
    {      
        irData = frameData[pixelNumber];
        if(irData > 32767)
        {
            irData = irData - 65536;
        }
        irData = irData * gain;
        
        kta = (float)params->kta[pixelNumber]/ktaScale;
        kv = (float)params->kv[pixelNumber]/kvScale;
            
        irData = irData - params->offset[subPage][pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3));                
    
        irData = irData - params->tgc * irDataCP;
        
        irData = irData / emissivity;
        
        alphaCompensated = SCALEALPHA*alphaScale/params->alpha[pixelNumber];
        alphaCompensated = alphaCompensated*(1 + params->KsTa * (ta - 25));
        
        Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
        Sx = sqrt(sqrt(Sx)) * params->ksTo[2];
        
        To = sqrt(sqrt(irData/(alphaCompensated * (1 - params->ksTo[2] * 273.15) + Sx) + taTr)) - 273.15;
                
        if(To < params->ct[1])
        {
            range = 0;
        }
        else if(To < params->ct[2])   
        {
            range = 1;            
        }   
        else if(To < params->ct[3])
        {
            range = 2;            
        }
        else if(To < params->ct[4])
        {
            range = 3;            
        }
        else if(To < params->ct[5])
        {
            range = 4;            
        }
        else if(To < params->ct[6])
        {
            range = 5;            
        }
        else if(To < params->ct[7])
        {
            range = 6;            
        }
        else
        {
            range = 7;            
        }      
        
        To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + params->ksTo[range] * (To - params->ct[range]))) + taTr)) - 273.15;
        
        result[pixelNumber] = To;
    }
}

//------------------------------------------------------------------------------

void MLX90641_GetImage(uint16_t *frameData, const paramsMLX90641 *params, float *result)
{
    float vdd;
    float ta;
    float gain;
    float irDataCP;
    float irData;
    float alphaCompensated;
    float image;
    uint16_t subPage;
    
    subPage = frameData[241];
    
    vdd = MLX90641_GetVdd(frameData, params);
    ta = MLX90641_GetTa(frameData, params);
    
//------------------------- Gain calculation -----------------------------------    
    gain = frameData[202];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = params->gainEE / gain; 
  
//------------------------- Image calculation -------------------------------------    
    irDataCP = frameData[200];  
    if(irDataCP > 32767)
    {
        irDataCP = irDataCP - 65536;
    }
    irDataCP = irDataCP * gain;

    irDataCP = irDataCP - params->cpOffset * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    
    for( int pixelNumber = 0; pixelNumber < 192; pixelNumber++)
    {
        irData = frameData[pixelNumber];
        if(irData > 32767)
        {
            irData = irData - 65536;
        }
        irData = irData * gain;
        
        irData = irData - params->offset[subPage][pixelNumber]*(1 + params->kta[pixelNumber]*(ta - 25))*(1 + params->kv[pixelNumber]*(vdd - 3.3));
        
        irData = irData - params->tgc * irDataCP;
            
        alphaCompensated = (params->alpha[pixelNumber] - params->tgc * params->cpAlpha);
            
        image = irData*alphaCompensated;
            
        result[pixelNumber] = image;
    }
}

//------------------------------------------------------------------------------

float MLX90641_GetVdd(uint16_t *frameData, const paramsMLX90641 *params)
{
    float vdd;
    float resolutionCorrection;
    
    int resolutionRAM;    
    
    vdd = frameData[234];
    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }
    resolutionRAM = (frameData[240] & 0x0C00) >> 10;
    resolutionCorrection = pow(2, (double)params->resolutionEE) / pow(2, (double)resolutionRAM);
    vdd = (resolutionCorrection * vdd - params->vdd25) / params->kVdd + 3.3;
    
    return vdd;
}

//------------------------------------------------------------------------------

float MLX90641_GetTa(uint16_t *frameData, const paramsMLX90641 *params)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;
    
    vdd = MLX90641_GetVdd(frameData, params);
    
    ptat = frameData[224];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
    
    ptatArt = frameData[192];
    if(ptatArt > 32767)
    {
        ptatArt = ptatArt - 65536;
    }
    ptatArt = (ptat / (ptat * params->alphaPTAT + ptatArt)) * pow(2, (double)18);
    
    ta = (ptatArt / (1 + params->KvPTAT * (vdd - 3.3)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25;
    
    return ta;
}

//------------------------------------------------------------------------------

int MLX90641_GetSubPageNumber(uint16_t *frameData)
{
    return frameData[241];    

}    

//------------------------------------------------------------------------------
void MLX90641_BadPixelsCorrection(uint16_t *pixels, float *to, paramsMLX90641 *params)
{   
    float ap[2];
    uint8_t pix;
    uint8_t line;
    uint8_t column;
    
    pix = 0;
    while(pixels[pix]< 65535)
    {
        line = pixels[pix]>>5;
        column = pixels[pix] - (line<<5);
               
        if(column == 0)
        {
            to[pixels[pix]] = to[pixels[pix]+1];            
        }
        else if(column == 1 || column == 14)
        {
            to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0;                
        } 
        else if(column == 15)
        {
            to[pixels[pix]] = to[pixels[pix]-1];
        } 
        else
        {            
            ap[0] = to[pixels[pix]+1] - to[pixels[pix]+2];
            ap[1] = to[pixels[pix]-1] - to[pixels[pix]-2];
            if(fabs(ap[0]) > fabs(ap[1]))
            {
                to[pixels[pix]] = to[pixels[pix]-1] + ap[1];                        
            }
            else
            {
                to[pixels[pix]] = to[pixels[pix]+1] + ap[0];                        
            }
                    
        }                      
     
        pix = pix + 1;    
    }    
}

//------------------------------------------------------------------------------

void MLX90641_ExtractVDDParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    int16_t kVdd;
    int16_t vdd25;
    
    kVdd = eeData[39];
    if(kVdd > 1023)
    {
        kVdd = kVdd - 2048;
    }
    kVdd = 32 * kVdd;
    
    vdd25 = eeData[38];
    if(vdd25 > 1023)
    {
        vdd25 = vdd25 - 2048;
    }
    vdd25 = 32 * vdd25;
    
    mlx90641->kVdd = kVdd;
    mlx90641->vdd25 = vdd25; 
}

//------------------------------------------------------------------------------

void MLX90641_ExtractPTATParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;
    
    KvPTAT = eeData[43];
    if(KvPTAT > 1023)
    {
        KvPTAT = KvPTAT - 2048;
    }
    KvPTAT = KvPTAT/4096;
    
    KtPTAT = eeData[42];
    if(KtPTAT > 1023)
    {
        KtPTAT = KtPTAT - 2048;
    }
    KtPTAT = KtPTAT/8;
    
    vPTAT25 = 32 * eeData[40] + eeData[41];
    
    alphaPTAT = eeData[44] / 128.0f;
    
    mlx90641->KvPTAT = KvPTAT;
    mlx90641->KtPTAT = KtPTAT;    
    mlx90641->vPTAT25 = vPTAT25;
    mlx90641->alphaPTAT = alphaPTAT;   
}

//------------------------------------------------------------------------------

void MLX90641_ExtractGainParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    int16_t gainEE;
    
    gainEE = 32 * eeData[36] + eeData[37];

    mlx90641->gainEE = gainEE;    
}

//------------------------------------------------------------------------------

void MLX90641_ExtractTgcParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    float tgc;
    tgc = eeData[51] & 0x01FF;
    if(tgc > 255)
    {
        tgc = tgc - 512;
    }
    tgc = tgc / 64.0f;
    
    mlx90641->tgc = tgc;        
}

//------------------------------------------------------------------------------

void MLX90641_ExtractEmissivityParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    float emissivity;
    emissivity = eeData[35];
       
    if(emissivity > 1023)
    {
        emissivity = emissivity - 2048;
    }
    emissivity = emissivity/512;
    
    mlx90641->emissivityEE = emissivity;
}
    
//------------------------------------------------------------------------------

void MLX90641_ExtractResolutionParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    uint8_t resolutionEE;
    resolutionEE = (eeData[51] & 0x0600) >> 9;    
    
    mlx90641->resolutionEE = resolutionEE;
}

//------------------------------------------------------------------------------

void MLX90641_ExtractKsTaParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    float KsTa;
    KsTa = eeData[34];
    if(KsTa > 1023)
    {
        KsTa = KsTa - 2048;
    }
    KsTa = KsTa / 32768.0f;
    
    mlx90641->KsTa = KsTa;
}

//------------------------------------------------------------------------------

void MLX90641_ExtractKsToParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    int KsToScale;
    
    mlx90641->ct[0] = -40;
    mlx90641->ct[1] = -20;
    mlx90641->ct[2] = 0;
    mlx90641->ct[3] = 80;
    mlx90641->ct[4] = 120;
    mlx90641->ct[5] = eeData[58];
    mlx90641->ct[6] = eeData[60];
    mlx90641->ct[7] = eeData[62];
     
    KsToScale = eeData[52];
    KsToScale = 1 << KsToScale;
    
    mlx90641->ksTo[0] = eeData[53];
    mlx90641->ksTo[1] = eeData[54];
    mlx90641->ksTo[2] = eeData[55];
    mlx90641->ksTo[3] = eeData[56];
    mlx90641->ksTo[4] = eeData[57];
    mlx90641->ksTo[5] = eeData[59];
    mlx90641->ksTo[6] = eeData[61];
    mlx90641->ksTo[7] = eeData[63];
    
    
    for(int i = 0; i < 8; i++)
    {
        if(mlx90641->ksTo[i] > 1023)
        {
            mlx90641->ksTo[i] = mlx90641->ksTo[i] - 2048;
        }
        mlx90641->ksTo[i] = mlx90641->ksTo[i] / KsToScale;
    } 
}

//------------------------------------------------------------------------------

void MLX90641_ExtractAlphaParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    float rowMaxAlphaNorm[6];
    uint16_t scaleRowAlpha[6];
    uint8_t alphaScale;
    float alphaTemp[192];
    float temp;
    int p = 0;

    scaleRowAlpha[0] = (eeData[25] >> 5) + 20;
    scaleRowAlpha[1] = (eeData[25] & 0x001F) + 20;
    scaleRowAlpha[2] = (eeData[26] >> 5) + 20;
    scaleRowAlpha[3] = (eeData[26] & 0x001F) + 20;
    scaleRowAlpha[4] = (eeData[27] >> 5) + 20;
    scaleRowAlpha[5] = (eeData[27] & 0x001F) + 20;

    
    for(int i = 0; i < 6; i++)
    {
        rowMaxAlphaNorm[i] = eeData[28 + i] / pow(2,(double)scaleRowAlpha[i]);
        rowMaxAlphaNorm[i] = rowMaxAlphaNorm[i] / 2047.0f;
    }

    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            alphaTemp[p] = eeData[256 + p] * rowMaxAlphaNorm[i]; 
            alphaTemp[p] = alphaTemp[p] - mlx90641->tgc * mlx90641->cpAlpha;
            alphaTemp[p] = SCALEALPHA/alphaTemp[p];
        }
    }
    
    temp = alphaTemp[0];
    for(int i = 1; i < 192; i++)
    {
        if (alphaTemp[i] > temp)
        {
            temp = alphaTemp[i];
        }
    }
    
    alphaScale = 0;
    while(temp < 32768)
    {
        temp = temp*2;
        alphaScale = alphaScale + 1;
    } 
    
    for(int i = 0; i < 192; i++)
    {
        temp = alphaTemp[i] * pow(2,(double)alphaScale);        
        mlx90641->alpha[i] = (temp + 0.5);        
        
    } 
    
    mlx90641->alphaScale = alphaScale;      
}

//------------------------------------------------------------------------------

void MLX90641_ExtractOffsetParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    int scaleOffset;
    int16_t offsetRef;
    int16_t tempOffset; 
    
    scaleOffset = eeData[16] >> 5;
    scaleOffset = 1 << scaleOffset;

    offsetRef = 32 * eeData[17] + eeData[18];
    if (offsetRef > 32767)
    {
        offsetRef = offsetRef - 65536;
    }

    for(int i = 0; i < 192; i++)
    {
        tempOffset = eeData[64 + i];
        if(tempOffset > 1023)
        {
           tempOffset = eeData[64 + i] - 2048; 
        }
        mlx90641->offset[0][i] = tempOffset * scaleOffset + offsetRef;
        
        tempOffset = eeData[640 + i];
        if(tempOffset > 1023)
        {
           tempOffset = eeData[640 + i] - 2048; 
        }
        mlx90641->offset[1][i] = tempOffset * scaleOffset + offsetRef;
    }
}

//------------------------------------------------------------------------------

void MLX90641_ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    int16_t ktaAvg;
    int16_t tempKta;
    float ktaTemp[192];
    float temp;

    ktaAvg = eeData[21];
    if (ktaAvg > 1023)
    {
        ktaAvg = ktaAvg - 2048;
    }
  
    ktaScale1 = eeData[22] >> 5;
    ktaScale2 = eeData[22] & 0x001F;

    for(int i = 0; i < 192; i++)
    {
        tempKta = (eeData[448 + i] >> 5);
        if (tempKta > 31)
        {
            tempKta = tempKta - 64;
        }

        ktaTemp[i] = tempKta * pow(2,(double)ktaScale2);
        ktaTemp[i] = ktaTemp[i] + ktaAvg;
        ktaTemp[i] = ktaTemp[i] / pow(2,(double)ktaScale1);
    }
    
    temp = fabs(ktaTemp[0]);
    for(int i = 1; i < 192; i++)
    {
        if (fabs(ktaTemp[i]) > temp)
        {
            temp = fabs(ktaTemp[i]);
        }
    }
    
    ktaScale1 = 0;
    while(temp < 64)
    {
        temp = temp*2;
        ktaScale1 = ktaScale1 + 1;
    }    
     
    for(int i = 0; i < 192; i++)
    {
        temp = ktaTemp[i] * pow(2,(double)ktaScale1);
        if (temp < 0)
        {
            mlx90641->kta[i] = (temp - 0.5);
        }
        else
        {
            mlx90641->kta[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90641->ktaScale = ktaScale1;
}

//------------------------------------------------------------------------------

void MLX90641_ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    uint8_t kvScale1;
    uint8_t kvScale2;
    int16_t kvAvg;
    int16_t tempKv;
    float kvTemp[192];
    float temp;

    kvAvg = eeData[23];
    if (kvAvg > 1023)
    {
        kvAvg = kvAvg - 2048;
    }
  
    kvScale1 = eeData[24] >> 5;
    kvScale2 = eeData[24] & 0x001F;

    for(int i = 0; i < 192; i++)
    {
        tempKv = (eeData[448 + i] & 0x001F);
        if (tempKv > 15)
        {
            tempKv = tempKv - 32;
        }

        kvTemp[i] = tempKv * pow(2,(double)kvScale2);
        kvTemp[i] = kvTemp[i] + kvAvg;
        kvTemp[i] = kvTemp[i] / pow(2,(double)kvScale1);
    }
    
    temp = fabs(kvTemp[0]);
    for(int i = 1; i < 192; i++)
    {
        if (fabs(kvTemp[i]) > temp)
        {
            temp = fabs(kvTemp[i]);
        }
    }
    
    kvScale1 = 0;
    while(temp < 64)
    {
        temp = temp*2;
        kvScale1 = kvScale1 + 1;
    }    
     
    for(int i = 0; i < 192; i++)
    {
        temp = kvTemp[i] * pow(2,(double)kvScale1);
        if (temp < 0)
        {
            mlx90641->kv[i] = (temp - 0.5);
        }
        else
        {
            mlx90641->kv[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90641->kvScale = kvScale1;        
}

//------------------------------------------------------------------------------

void MLX90641_ExtractCPParameters(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    float alphaCP;
    int16_t offsetCP;
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = eeData[46];
    
    offsetCP = 32 * eeData[47] + eeData[48];
    if (offsetCP > 32767)
    {
        offsetCP = offsetCP - 65536;
    }
       
    alphaCP = eeData[45];
    if (alphaCP > 1023)
    {
        alphaCP = alphaCP - 2048;
    }
    
    alphaCP = alphaCP /  pow(2,(double)alphaScale);
    
    
    cpKta = eeData[49] & 0x001F;
    if (cpKta > 31)
    {
        cpKta = cpKta - 64;
    }
    ktaScale1 = eeData[49] >> 6;    
    mlx90641->cpKta = cpKta / pow(2,(double)ktaScale1);
    
    cpKv = eeData[50] & 0x001F;
    if (cpKv > 31)
    {
        cpKv = cpKv - 64;
    }
    kvScale = eeData[50] >> 6;
    mlx90641->cpKv = cpKv / pow(2,(double)kvScale);
       
    mlx90641->cpAlpha = alphaCP;
    mlx90641->cpOffset = offsetCP;
}

//------------------------------------------------------------------------------

float MLX90641_GetEmissivity(const paramsMLX90641 *mlx90641)
{
    return  mlx90641->emissivityEE;
}

//------------------------------------------------------------------------------

int MLX90641_ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90641 *mlx90641)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;

    int warn = 0;
    
    for(pixCnt = 0; pixCnt<3; pixCnt++)
    {
        mlx90641->brokenPixels[pixCnt] = 0xFFFF;
    }
        
    pixCnt = 0;    
    while (pixCnt < 192 && brokenPixCnt < 3)
    {
        if((eeData[pixCnt+64] == 0) && (eeData[pixCnt+256] == 0) && (eeData[pixCnt+448] == 0) && (eeData[pixCnt+640] == 0))
        {
            mlx90641->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }    
        
        pixCnt = pixCnt + 1;
        
    } 
    
    if(brokenPixCnt > 1)  
    {
        warn = -3;
    }         
    
    return warn;
       
}
 
 //------------------------------------------------------------------------------
 
 int MLX90641_CheckEEPROMValid(uint16_t *eeData)  
 {
     int deviceSelect;
     deviceSelect = ((uint16_t)eeData[10]) & 0x0040;
     
     if(deviceSelect != 0)
     {
         return 0;
     }
     
     return -7;    
 }        






/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              MLX90615 FUNCTIONS                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

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




/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              APDS-9500 FUNCTIONS                    */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

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





/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/

/*              VL53L0X FUNCTIONS                     */

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/



// Public Methods //////////////////////////////////////////////////////////////

void VL53L0X_setAddress(uint8_t new_addr)
{
  VL53L0X_writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
  address = new_addr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool VL53L0X_init(bool io_2v8)
{
  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    VL53L0X_writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      VL53L0X_readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  VL53L0X_writeReg(0x88, 0x00);

  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  stop_variable = VL53L0X_readReg(0x91);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  VL53L0X_writeReg(MSRC_CONFIG_CONTROL, VL53L0X_readReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  VL53L0X_setSignalRateLimit(0.25);

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!VL53L0X_getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  VL53L0X_readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  VL53L0X_writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  VL53L0X_writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x09, 0x00);
  VL53L0X_writeReg(0x10, 0x00);
  VL53L0X_writeReg(0x11, 0x00);

  VL53L0X_writeReg(0x24, 0x01);
  VL53L0X_writeReg(0x25, 0xFF);
  VL53L0X_writeReg(0x75, 0x00);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x4E, 0x2C);
  VL53L0X_writeReg(0x48, 0x00);
  VL53L0X_writeReg(0x30, 0x20);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x30, 0x09);
  VL53L0X_writeReg(0x54, 0x00);
  VL53L0X_writeReg(0x31, 0x04);
  VL53L0X_writeReg(0x32, 0x03);
  VL53L0X_writeReg(0x40, 0x83);
  VL53L0X_writeReg(0x46, 0x25);
  VL53L0X_writeReg(0x60, 0x00);
  VL53L0X_writeReg(0x27, 0x00);
  VL53L0X_writeReg(0x50, 0x06);
  VL53L0X_writeReg(0x51, 0x00);
  VL53L0X_writeReg(0x52, 0x96);
  VL53L0X_writeReg(0x56, 0x08);
  VL53L0X_writeReg(0x57, 0x30);
  VL53L0X_writeReg(0x61, 0x00);
  VL53L0X_writeReg(0x62, 0x00);
  VL53L0X_writeReg(0x64, 0x00);
  VL53L0X_writeReg(0x65, 0x00);
  VL53L0X_writeReg(0x66, 0xA0);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x22, 0x32);
  VL53L0X_writeReg(0x47, 0x14);
  VL53L0X_writeReg(0x49, 0xFF);
  VL53L0X_writeReg(0x4A, 0x00);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x7A, 0x0A);
  VL53L0X_writeReg(0x7B, 0x00);
  VL53L0X_writeReg(0x78, 0x21);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x23, 0x34);
  VL53L0X_writeReg(0x42, 0x00);
  VL53L0X_writeReg(0x44, 0xFF);
  VL53L0X_writeReg(0x45, 0x26);
  VL53L0X_writeReg(0x46, 0x05);
  VL53L0X_writeReg(0x40, 0x40);
  VL53L0X_writeReg(0x0E, 0x06);
  VL53L0X_writeReg(0x20, 0x1A);
  VL53L0X_writeReg(0x43, 0x40);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x34, 0x03);
  VL53L0X_writeReg(0x35, 0x44);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x31, 0x04);
  VL53L0X_writeReg(0x4B, 0x09);
  VL53L0X_writeReg(0x4C, 0x05);
  VL53L0X_writeReg(0x4D, 0x04);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x44, 0x00);
  VL53L0X_writeReg(0x45, 0x20);
  VL53L0X_writeReg(0x47, 0x08);
  VL53L0X_writeReg(0x48, 0x28);
  VL53L0X_writeReg(0x67, 0x00);
  VL53L0X_writeReg(0x70, 0x04);
  VL53L0X_writeReg(0x71, 0x01);
  VL53L0X_writeReg(0x72, 0xFE);
  VL53L0X_writeReg(0x76, 0x00);
  VL53L0X_writeReg(0x77, 0x00);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x0D, 0x01);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0x01, 0xF8);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x8E, 0x01);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  VL53L0X_writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  VL53L0X_writeReg(GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  VL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  VL53L0X_setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!VL53L0X_performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!VL53L0X_performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Write an 8-bit register
void VL53L0X_writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Write a 16-bit register
void VL53L0X_writeReg16Bit(uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF); // value high byte
  Wire.write( value       & 0xFF); // value low byte
  last_status = Wire.endTransmission();
}

// Write a 32-bit register
void VL53L0X_writeReg32Bit(uint8_t reg, uint32_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write((value >> 24) & 0xFF); // value highest byte
  Wire.write((value >> 16) & 0xFF);
  Wire.write((value >>  8) & 0xFF);
  Wire.write( value        & 0xFF); // value lowest byte
  last_status = Wire.endTransmission();
}

// Read an 8-bit register
uint8_t VL53L0X_readReg(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  value = Wire.read();

  return value;
}

// Read a 16-bit register
uint16_t VL53L0X_readReg16Bit(uint8_t reg)
{
  uint16_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2);
  value  = (uint16_t)Wire.read() << 8; // value high byte
  value |=           Wire.read();      // value low byte

  return value;
}

// Read a 32-bit register
uint32_t VL53L0X_readReg32Bit(uint8_t reg)
{
  uint32_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)4);
  value  = (uint32_t)Wire.read() << 24; // value highest byte
  value |= (uint32_t)Wire.read() << 16;
  value |= (uint16_t)Wire.read() <<  8;
  value |=           Wire.read();       // value lowest byte

  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void VL53L0X_writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
  Wire.beginTransmission(address);
  Wire.write(reg);

  while (count-- > 0)
  {
    Wire.write(*(src++));
  }

  last_status = Wire.endTransmission();
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void VL53L0X_readMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, count);

  while (count-- > 0)
  {
    *(dst++) = Wire.read();
  }
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool VL53L0X_setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  VL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
float VL53L0X_getSignalRateLimit(void)
{
  return (float)VL53L0X_readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool VL53L0X_setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53L0X_getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool VL53L0X_setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    VL53L0X_writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    VL53L0X_writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    VL53L0X_writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x30);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      case 10:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      case 12:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      case 14:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    VL53L0X_writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  VL53L0X_setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = VL53L0X_readReg(SYSTEM_SEQUENCE_CONFIG);
  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  VL53L0X_performSingleRefCalibration(0x0);
  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X_getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(VL53L0X_readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(VL53L0X_readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X_startContinuous(uint32_t period_ms)
{
  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  VL53L0X_writeReg(0x91, stop_variable);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = VL53L0X_readReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    VL53L0X_writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    VL53L0X_writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    VL53L0X_writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void VL53L0X_stopContinuous(void)
{
  VL53L0X_writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  VL53L0X_writeReg(0x91, 0x00);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X_readRangeContinuousMillimeters(void)
{
  startTimeout();
  while ((VL53L0X_readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = VL53L0X_readReg16Bit(RESULT_RANGE_STATUS + 10);

  VL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t VL53L0X_readRangeSingleMillimeters(void)
{
  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  VL53L0X_writeReg(0x91, stop_variable);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  VL53L0X_writeReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (VL53L0X_readReg(SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  return VL53L0X_readRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53L0X_timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53L0X_getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);

  VL53L0X_writeReg(0xFF, 0x06);
  VL53L0X_writeReg(0x83, VL53L0X_readReg(0x83) | 0x04);
  VL53L0X_writeReg(0xFF, 0x07);
  VL53L0X_writeReg(0x81, 0x01);

  VL53L0X_writeReg(0x80, 0x01);

  VL53L0X_writeReg(0x94, 0x6b);
  VL53L0X_writeReg(0x83, 0x00);
  startTimeout();
  while (VL53L0X_readReg(0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
  VL53L0X_writeReg(0x83, 0x01);
  tmp = VL53L0X_readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  VL53L0X_writeReg(0x81, 0x00);
  VL53L0X_writeReg(0xFF, 0x06);
  VL53L0X_writeReg(0x83, VL53L0X_readReg(0x83)  & ~0x04);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x01);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void VL53L0X_getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = VL53L0X_readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void VL53L0X_getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = VL53L0X_readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(VL53L0X_readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(VL53L0X_readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte)
{
  VL53L0X_writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((VL53L0X_readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return false; }
  }

  VL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  VL53L0X_writeReg(SYSRANGE_START, 0x00);

  return true;
}
