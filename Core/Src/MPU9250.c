/*
 * MPU9250.c
 *
 *  Created on: Feb 28, 2019
 *      Author: Desert
 */

#include "MPU9250.h"
#include "math.h"
const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

const uint16_t _dev_add = 208;
// 400 kHz
const uint32_t _i2cRate = 400000;

// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t DLPF_184 = 0x01;
const uint8_t DLPF_92 = 0x02;
const uint8_t DLPF_41 = 0x03;
const uint8_t DLPF_20 = 0x04;
const uint8_t DLPF_10 = 0x05;
const uint8_t DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;

const uint8_t XA_OFFSET_H = 0x77;
const uint8_t XA_OFFSET_L = 0x78;
const uint8_t YA_OFFSET_H = 0x7A;
const uint8_t YA_OFFSET_L = 0x7B;
const uint8_t ZA_OFFSET_H = 0x7D;
const uint8_t ZA_OFFSET_L = 0x7E;
// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;


static uint8_t _buffer[21];
static uint8_t _mag_adjust[3];

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float_t magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float_t gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float_t ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float_t temperature;
float_t SelfTest[6];

int delt_t = 0; // used to control display output rate
int count = 0;  // used to control display output rate

// parameters for 6 DoF sensor fusion calculations
const float_t PI = 3.14159265358979323846;
const float_t GyroMeasError = PI * (60.0 / 180.0);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
const float_t beta = sqrt(3.0 / 4.0) * GyroMeasError;  // compute beta
const float_t GyroMeasDrift = PI * (1.0 / 180.0);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float_t zeta = sqrt(3.0 / 4.0) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float_t Kp = 2.0 * 5.0; // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
float_t Ki = 0.0;

float_t pitch, yaw, roll;
float_t deltat = 0.0f;                             // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
float_t q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float_t eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method

__weak void MPU9250_OnActivate()
{
}
#ifndef USE_SPI
bool  MPU9250_IsConnected()
{
  if(HAL_I2C_IsDeviceReady(&_MPU9250_I2C,_dev_add,1,HAL_MAX_DELAY)==HAL_OK)
    return true;
  else
    return false;
}

void MPU_I2C_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  HAL_I2C_Mem_Write(&_MPU9250_I2C,_dev_add,WriteAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumByteToWrite,HAL_MAX_DELAY);
}

void MPU_I2C_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  uint8_t data = ReadAddr | READWRITE_CMD;
  HAL_I2C_Master_Transmit(&_MPU9250_I2C,_dev_add,&data,1,HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&_MPU9250_I2C,_dev_add,pBuffer,NumByteToRead,HAL_MAX_DELAY);
}
#endif
#ifdef USE_SPI
static inline void MPU9250_Activate()
{
  MPU9250_OnActivate();
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;
  if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
  {
    return -1;
  }
  else
  {
  }
  return receivedbyte;
}

void MPU_SPI_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  MPU9250_Activate();
  SPIx_WriteRead(WriteAddr);
  while(NumByteToWrite>=0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  MPU9250_Deactivate();
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  MPU9250_Activate();
  uint8_t data = ReadAddr | READWRITE_CMD;
  HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
  MPU9250_Deactivate();
}
#endif
/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data)
{
  #ifdef USE_SPI
  MPU_SPI_Write(&data, subAddress, 1);
  #else
  MPU_I2C_Write(&data, subAddress, 1);
  #endif
  HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  #ifdef USE_SPI
  MPU_SPI_Read(dest, subAddress, count);
  #else
  MPU_I2C_Read(dest, subAddress, count);
  #endif
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
  // set slave 0 to the AK8963 and set for write
  writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);

  // set the register to the desired AK8963 sub address
  writeRegister(I2C_SLV0_REG,subAddress);

  // store the data for write
  writeRegister(I2C_SLV0_DO,data);

  // enable I2C and send 1 byte
  writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
  // set slave 0 to the AK8963 and set for read
  writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

  // set the register to the desired AK8963 sub address
  writeRegister(I2C_SLV0_REG,subAddress);

  // enable I2C and request the bytes
  writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

  // takes some time for these registers to fill
  HAL_Delay(1);

  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  readRegisters(EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI(){
  // read the WHO AM I register
  readRegisters(WHO_AM_I,1,_buffer);

  // return the register value
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963(){
  // read the WHO AM I register
  readAK8963Registers(AK8963_WHO_AM_I,1,_buffer);
  // return the register value
  return _buffer[0];
}

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init()
{
  #ifndef USE_SPI
  while(MPU9250_IsConnected() == false)
  {
    HAL_Delay(100);
  }
  #endif
  // select clock source to gyro
  writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
  // enable I2C master mode
  writeRegister(USER_CTRL, I2C_MST_EN);
  // set the I2C bus speed to 400 kHz
  writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  HAL_Delay(10);
  // reset the AK8963
  writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
  // select clock source to gyro
  writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  uint8_t who = whoAmI();
  if((who != 0x71) && ( who != 0x73))
  {
    return 1;
  }

  // enable accelerometer and gyro
  writeRegister(PWR_MGMNT_2,SEN_ENABLE);

  // setting accel range to 16G as default
  writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G);

  // setting the gyro range to 2000DPS as default
  writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS);

  // setting bandwidth to 184Hz as default
  writeRegister(ACCEL_CONFIG2,DLPF_184);

  // setting gyro bandwidth to 184Hz
  writeRegister(CONFIG,DLPF_184);

  // setting the sample rate divider to 0 as default
  writeRegister(SMPDIV,0x00);

  // enable I2C master mode
  writeRegister(USER_CTRL,I2C_MST_EN);

  // set the I2C bus speed to 400 kHz
  writeRegister(I2C_MST_CTRL,I2C_MST_CLK);

  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
  if( whoAmIAK8963() != 0x48 )
  {
    return 1;
  }

  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

  HAL_Delay(100); // long wait between AK8963 mode changes

  // set AK8963 to FUSE ROM access
  writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);

  // long wait between AK8963 mode changes
  HAL_Delay(100);

  // read the AK8963 ASA registers and compute magnetometer scale factors
  readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

  // long wait between AK8963 mode changes
  HAL_Delay(100);

  // set AK8963 to 16 bit resolution, 100 Hz update rate
  writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

  // long wait between AK8963 mode changes
  HAL_Delay(100);

  // select clock source to gyro
  writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(AK8963_HXL,7,_buffer);

  // successful init, return 0
  return 0;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
  writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
  writeRegister(GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
  writeRegister(ACCEL_CONFIG2,bandwidth);
  writeRegister(CONFIG,bandwidth);
}

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
  /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
  writeRegister(SMPDIV,19);

  if(srd > 9)
  {
    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

    // long wait between AK8963 mode changes
    HAL_Delay(100);

    // set AK8963 to 16 bit resolution, 8 Hz update rate
    writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1);

    // long wait between AK8963 mode changes
    HAL_Delay(100);

    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);

  }
  else
  {
    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    // long wait between AK8963 mode changes
    HAL_Delay(100);
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

    // long wait between AK8963 mode changes
    HAL_Delay(100);

    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);
  }

  writeRegister(SMPDIV, srd);
}

/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetData(int16_t* AccData, int16_t* MagData, int16_t* GyroData)
{
  // grab the data from the MPU9250
  readRegisters(ACCEL_OUT, 21, _buffer);

  // combine into 16 bit values
  AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  GyroData[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];

  int16_t magx = (((int16_t)_buffer[15]) << 8) | _buffer[14];
  int16_t magy = (((int16_t)_buffer[17]) << 8) | _buffer[16];
  int16_t magz = (((int16_t)_buffer[19]) << 8) | _buffer[18];

  MagData[0] = (int16_t)((float_t)magx * ((float_t)(_mag_adjust[0] - 128) / 256.0f + 1.0f));
  MagData[1] = (int16_t)((float_t)magy * ((float_t)(_mag_adjust[1] - 128) / 256.0f + 1.0f));
  MagData[2] = (int16_t)((float_t)magz * ((float_t)(_mag_adjust[2] - 128) / 256.0f + 1.0f));
}

void calibrateMPU9250(float_t * dest1, float_t * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeRegister(PWR_MGMNT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeRegister(PWR_MGMNT_1, 0x01);
  writeRegister(PWR_MGMNT_2, 0x00);
  HAL_Delay(200);

// Configure device for bias calculation
  writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
  writeRegister(FIFO_EN, 0x00);      // Disable FIFO
  writeRegister(PWR_MGMNT_1, 0x00);   // Turn on internal clock source
  writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
  writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeRegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay(15);

// Configure MPU9250 gyro and accelerometer for bias calculation
  writeRegister(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeRegister(SMPDIV, 0x00);  // Set sample rate to 1 kHz
  writeRegister(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeRegister(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeRegister(USER_CTRL, 0x40);   // Enable FIFO
  writeRegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeRegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readRegisters(FIFO_COUNT, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readRegisters(FIFO_READ, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeRegister(XG_OFFSET_H, data[0]);
  writeRegister(XG_OFFSET_L, data[1]);
  writeRegister(YG_OFFSET_H, data[2]);
  writeRegister(YG_OFFSET_L, data[3]);
  writeRegister(ZG_OFFSET_H, data[4]);
  writeRegister(ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float_t) gyro_bias[0]/(float_t) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float_t) gyro_bias[1]/(float_t) gyrosensitivity;
  dest1[2] = (float_t) gyro_bias[2]/(float_t) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readRegisters(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readRegisters(YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readRegisters(ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeRegister(XA_OFFSET_H, data[0]);
  writeRegister(XA_OFFSET_L, data[1]);
  writeRegister(YA_OFFSET_H, data[2]);
  writeRegister(YA_OFFSET_L, data[3]);
  writeRegister(ZA_OFFSET_H, data[4]);
  writeRegister(ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float_t)accel_bias[0]/(float_t)accelsensitivity;
   dest2[1] = (float_t)accel_bias[1]/(float_t)accelsensitivity;
   dest2[2] = (float_t)accel_bias[2]/(float_t)accelsensitivity;
}




// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float_t ax, float_t ay, float_t az, float_t gx, float_t gy, float_t gz, float_t mx, float_t my, float_t mz)
{
    float_t q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float_t norm;
    float_t hx, hy, _2bx, _2bz;
    float_t s1, s2, s3, s4;
    float_t qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float_t _2q1mx;
    float_t _2q1my;
    float_t _2q1mz;
    float_t _2q2mx;
    float_t _4bx;
    float_t _4bz;
    float_t _2q1 = 2.0f * q1;
    float_t _2q2 = 2.0f * q2;
    float_t _2q3 = 2.0f * q3;
    float_t _2q4 = 2.0f * q4;
    float_t _2q1q3 = 2.0f * q1 * q3;
    float_t _2q3q4 = 2.0f * q3 * q4;
    float_t q1q1 = q1 * q1;
    float_t q1q2 = q1 * q2;
    float_t q1q3 = q1 * q3;
    float_t q1q4 = q1 * q4;
    float_t q2q2 = q2 * q2;
    float_t q2q3 = q2 * q3;
    float_t q2q4 = q2 * q4;
    float_t q3q3 = q3 * q3;
    float_t q3q4 = q3 * q4;
    float_t q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}



 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones.
void MahonyQuaternionUpdate(float_t ax, float_t ay, float_t az, float_t gx, float_t gy, float_t gz, float_t mx, float_t my, float_t mz)
{
    float_t q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float_t norm;
    float_t hx, hy, bx, bz;
    float_t vx, vy, vz, wx, wy, wz;
    float_t ex, ey, ez;
    float_t pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float_t q1q1 = q1 * q1;
    float_t q1q2 = q1 * q2;
    float_t q1q3 = q1 * q3;
    float_t q1q4 = q1 * q4;
    float_t q2q2 = q2 * q2;
    float_t q2q3 = q2 * q3;
    float_t q2q4 = q2 * q4;
    float_t q3q3 = q3 * q3;
    float_t q3q4 = q3 * q4;
    float_t q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

