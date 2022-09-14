#include "mbed.h"
#include "arm_book_lib.h"

#include "MPU_9250.h"

//=====[Declaration of private defines]========================================



// MPU-9250 Registers

#define MPU9250_SELF_TEST_X_GYRO 0x00
#define MPU9250_SELF_TEST_Y_GYRO 0x01
#define MPU9250_SELF_TEST_Z_GYRO 0x02

#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F

#define MPU9250_XG_OFFSET_H      0x13
#define MPU9250_XG_OFFSET_L      0x14
#define MPU9250_YG_OFFSET_H      0x15
#define MPU9250_YG_OFFSET_L      0x16
#define MPU9250_ZG_OFFSET_H      0x17
#define MPU9250_ZG_OFFSET_L      0x18
#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_LP_ACCEL_ODR     0x1E
#define MPU9250_WOM_THR          0x1F

#define MPU9250_FIFO_EN          0x23
#define MPU9250_I2C_MST_CTRL     0x24
#define MPU9250_I2C_SLV0_ADDR    0x25
#define MPU9250_I2C_SLV0_REG     0x26
#define MPU9250_I2C_SLV0_CTRL    0x27
#define MPU9250_I2C_SLV1_ADDR    0x28
#define MPU9250_I2C_SLV1_REG     0x29
#define MPU9250_I2C_SLV1_CTRL    0x2A
#define MPU9250_I2C_SLV2_ADDR    0x2B
#define MPU9250_I2C_SLV2_REG     0x2C
#define MPU9250_I2C_SLV2_CTRL    0x2D
#define MPU9250_I2C_SLV3_ADDR    0x2E
#define MPU9250_I2C_SLV3_REG     0x2F
#define MPU9250_I2C_SLV3_CTRL    0x30
#define MPU9250_I2C_SLV4_ADDR    0x31
#define MPU9250_I2C_SLV4_REG     0x32
#define MPU9250_I2C_SLV4_DO      0x33
#define MPU9250_I2C_SLV4_CTRL    0x34
#define MPU9250_I2C_SLV4_DI      0x35
#define MPU9250_I2C_SLV4_DONE	 0x40
#define MPU9250_I2C_MST_STATUS   0x36
#define MPU9250_INT_PIN_CFG      0x37
#define MPU9250_INT_ENABLE       0x38
#define MPU9250_INT_STATUS       0x3A

#define MPU9250_ACCEL_XOUT_H     0x3B
#define MPU9250_ACCEL_XOUT_L     0x3C
#define MPU9250_ACCEL_YOUT_H     0x3D
#define MPU9250_ACCEL_YOUT_L     0x3E
#define MPU9250_ACCEL_ZOUT_H     0x3F
#define MPU9250_ACCEL_ZOUT_L     0x40
#define MPU9250_TEMP_OUT_H       0x41
#define MPU9250_TEMP_OUT_L       0x42
#define MPU9250_GYRO_XOUT_H      0x43
#define MPU9250_GYRO_XOUT_L      0x44
#define MPU9250_GYRO_YOUT_H      0x45
#define MPU9250_GYRO_YOUT_L      0x46
#define MPU9250_GYRO_ZOUT_H      0x47
#define MPU9250_GYRO_ZOUT_L      0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60

#define MPU9250_I2C_SLV0_DO      0x63
#define MPU9250_I2C_SLV1_DO      0x64
#define MPU9250_I2C_SLV2_DO      0x65
#define MPU9250_I2C_SLV3_DO      0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET  0x68
#define MPU9250_MOT_DETECT_CTRL  0x69
#define MPU9250_USER_CTRL        0x6A
#define MPU9250_PWR_MGMT_1       0x6B  //The reset value is  0x01
#define MPU9250_PWR_MGMT_2       0x6C

#define MPU9250_FIFO_COUNTH      0x72
#define MPU9250_FIFO_COUNTL      0x73
#define MPU9250_FIFO_R_W         0x74
#define MPU9250_WHO_AM_I         0x75  //The reset value is  0x71
#define MPU9250_XA_OFFSET_H      0x77
#define MPU9250_XA_OFFSET_L      0x78
#define MPU9250_YA_OFFSET_H      0x7A
#define MPU9250_YA_OFFSET_L      0x7B
#define MPU9250_ZA_OFFSET_H      0x7D
#define MPU9250_ZA_OFFSET_L      0x7E

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C//<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_HXL       0x03  // data
#define AK8963_HXH       0x04
#define AK8963_HYL       0x05
#define AK8963_HYH       0x06
#define AK8963_HZL       0x07
#define AK8963_HZH       0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2     0x0B  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0

#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

//I2C_SLV0_CTRL register masks
#define MPU9250_I2C_SLV0_EN_MASK        0x80
#define MPU9250_I2C_SLV0_BYTE_SW_MASK   0x40
#define MPU9250_I2C_SLV0_REG_DIS_MASK   0x20
#define MPU9250_I2C_SLV0_GRP_MASK       0x10
#define MPU9250_I2C_SLV0_LENG_MASK      0x0F


#define MPU9250_READREG 0x80
#define MPU9250_WRITEREG 0x00

//I2C

#define MPU_I2C_ADDR	0x68
#define I2C_READ_MSK	0x01
#define I2C_WRITE_MSK	0x00



// MPU Config
#define RoomTemp_Offset 0.00
#define Temp_Sensitivity 333.87


//SPI ---- revisar!!!!
#define SPI4_MOSI PE_6
#define SPI4_SCK  PE_2
#define SPI4_CS   PE_4
#define SPI4_MISO PE_5




//=====[Declaration of private data types]=====================================

//=====[Declaration and initialization of public global objects]===============
// chip select
DigitalOut SPI_MPU9250_Chip_Select(SPI4_CS);
// spi
SPI SPI_MPU9250(SPI4_MOSI, SPI4_MISO, SPI4_SCK);

//=====[Declaration of external public global variables]=======================



//=====[Declaration and initialization of public global variables]=============
MPU9250_Status_TypeDef status = MPU9250_IDLE;

float Accelerometer_Bias[3] = {0,0,0};
float Gyroscope_Bias[3] = {0,0,0};
float Magnetometer_Bias[3] = {0,0,0};

uint8_t Accelerometer_Scale= 2;   //2g , 4g , 8g , 16g
uint16_t Gyroscope_Scale= 250;    //250dps , 500dps , 1000dps , 2000dps
uint16_t Magnetometer_Scale=4800; //4800uT
/*	The values asigned correspond to the default values	*/



//=====[Declaration and initialization of private global variables]============


//=====[Declarations (prototypes) of private functions]========================

//=====[Implementations of public functions]===================================
void Enable_CS_MPU9250(void)
{
	status = MPU9250_BUSY;
	SPI_MPU9250_Chip_Select=0;
	//CS_Status = CS_MPU;
}


/**
   * @brief Disables the MPU9250s chip Select and sets the IC state to IDLE after a transaction
   *
   * @param None
   * @retval None
   */
void Disable_CS_MPU9250(void)
{
	SPI_MPU9250_Chip_Select=1;
	status = MPU9250_IDLE;
	//CS_Status = CS_IDLE;
}


/**
  * @brief  Reads one byte from the MPU9250s registers in blocking mode.
  *
  * @param  reg: register address.
  * @param  pvalue: pointer to variable where data is saved.
  * @retval None.
  */
void Read_Byte_MPU9250(uint8_t reg , uint8_t *pvalue)
{
	Read_Bytes_MPU9250(reg,pvalue,1);
}
/**
  * @brief  Reads multiples numbers of bytes from the MPU9250s registers given by
  * 		the parameter "size" in a sequential manner in blocking mode.
  *
  *	@note	Reads registers starting from the starting address "reg" towards bigger addresses
  * 		sequentially (in increments of 1 byte)
  *
  * @param  reg: address of the first register to be read.
  * @param  pvalue: pointer to buffer where data is stored.
  * @param  size: number of bytes to be read.
  * @retval None.
  */
void Read_Bytes_MPU9250(uint8_t reg , uint8_t *pvalue, uint8_t size)
{
	uint8_t r,i;

	r = MPU9250_READREG | reg;


	SPI_MPU9250.lock();
	Enable_CS_MPU9250();

	//HAL_SPI_Transmit(hspi, &r,1,HAL_MAX_DELAY);				//Transmit register addr
	//while (HAL_SPI_GetState(hspi)!=HAL_SPI_STATE_READY);	//Wait for SPI bus to become available again
	//HAL_SPI_Receive(hspi,pvalue,size,HAL_MAX_DELAY);		//Receive bytes from registers
	//while (HAL_SPI_GetState(hspi)!=HAL_SPI_STATE_READY);	//Wait for SPI bus to become available again

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	//SPI_MPU9250.write(tx_buffer   , tx_length, rx_buffer, rx_length)
	//                 (const char* , int      , char *   , int      )

	char tx_buffer[32]={0};
	char rx_buffer[32]={0};
	int tx_length=1;
	int rx_length;

	rx_length=tx_length;

	//transfiero la orden a la IMU de lo que se va a leer
	tx_buffer[0]=(char)r ;
	SPI_MPU9250.write(tx_buffer, tx_length,rx_buffer,rx_length);
	//ahora pido los "size" valores requeridos
	tx_buffer[0]=0; //dummy data
	tx_length=(int) size;
	rx_length=tx_length;
	SPI_MPU9250.write(tx_buffer, tx_length,rx_buffer,rx_length);

	for (i=0;i<size;i++)
	{
		*(pvalue+i)=(uint8_t) rx_buffer[i];
	}
	//SPI_MPU9250.write(pr, 1, prx_buffer, rx_length);

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	Disable_CS_MPU9250();	//Disable CS to free SPI bus
	SPI_MPU9250.unlock();
}

/**
  * @brief  Writes one byte to the MPU9250s registers in blocking mode.
  *
  * @param  reg: register address.
  * @param  value: data to be written.
  * @retval None.
  */
void Write_Byte_MPU9250(uint8_t reg , uint8_t value)
{
	uint8_t r;

	char tx_buffer[1]={0};
	char rx_buffer[1]={0};
	int tx_length=1;
	int rx_length=1;


	r = MPU9250_WRITEREG | reg;

	SPI_MPU9250.lock();
	Enable_CS_MPU9250();

	//HAL_SPI_Transmit(hspi, &r,1,HAL_MAX_DELAY);				//Transmit register addr
	//while (HAL_SPI_GetState(hspi)!=HAL_SPI_STATE_READY);	//Wait for SPI bus to become available again
	//HAL_SPI_Transmit(hspi,&value,1,HAL_MAX_DELAY);			//Transmit byte to MPUs register
	//while (HAL_SPI_GetState(hspi)!=HAL_SPI_STATE_READY);	//Wait for SPI bus to become available again


	// primero se le indica qué registro se va a escribir
	tx_buffer[0]=(char)r ;
	SPI_MPU9250.write(tx_buffer, tx_length, rx_buffer, rx_length);
	// luego se carga en el registro el valor deseado
	tx_buffer[0]=(char) value;
	SPI_MPU9250.write(tx_buffer, tx_length, rx_buffer, rx_length);

	Disable_CS_MPU9250();									//Disable CS to free SPI bus
	SPI_MPU9250.unlock();
}

/**
  * @brief  Initializes MPU9250 general registers.
  *
  * @note   This function sets the clock source, enables the Accelerometer and Gyroscope, enable I2C master.
  * 		and set I2C clock to 400kHz
  *
  * @param  handle_spi: SPI handler to use for transfers
  * @retval None
  */
//void Init_MPU9250(SPI_HandleTypeDef *handle_spi)
void Init_MPU9250(void)

{
	SPI_MPU9250.format(8, 3);
	SPI_MPU9250.frequency(1000000);




	//hspi = handle_spi;
	delay(1);
	Write_Byte_MPU9250(MPU9250_PWR_MGMT_1,0x80);	//Reset
	delay(5);									//Wait for internal registers to reset
	Write_Byte_MPU9250(MPU9250_PWR_MGMT_1,0x01);	//Autoselect best available clock source
	delay(1);
	Write_Byte_MPU9250(MPU9250_PWR_MGMT_2,0x00);	//Enable Accel and Gyro
	delay(1);
	Write_Byte_MPU9250(MPU9250_INT_PIN_CFG,0x30);	//0x30 Bypass disable and interrupt config (no se por que se toca el interrupt, abría que ver si se puede usar o no hace falta)
	delay(1);
	Write_Byte_MPU9250(MPU9250_USER_CTRL,0x20|0x10);//Enable I2C master module and disabel I2C Slave modules
	delay(1);
	Write_Byte_MPU9250(MPU9250_I2C_MST_CTRL,0x1D); 	//400kHz I2C Master frequency, STOP between transfers
	delay(1);
	Write_Byte_MPU9250(MPU9250_FIFO_EN,0x00); 		//Disable Fifo
	delay(1);
}

/**
  * @brief  Configures the internal sample rate divider for the Accelerometer.
  *
  * @note   Fsample/(1+div)
  *
  * @param  div: divider value (see equation above to calculate Fs)
  * @retval None
  */
void Set_Sample_Rate_Divider(uint8_t div)
{
	Write_Byte_MPU9250(MPU9250_SMPLRT_DIV,div);		//Only used for 1kHz internal sample rate
	delay(1);									//SMPL_RATE / (1+SMPL_RATE_DIVIDER)
}
/**
  * @brief  Initializes MPU9250 Accelerometer specific registers.
  *
  * @note   Filter can be bypassed (ACCEL_BYPASS_DLPF). The configuration of the other
  * 		settings are not yet implemented
  *
  * @param  scale: Sets the scale of the sensor (2g,4g,8g and 16g)
  * @param  sample_mode: Sets the base Fs and Bandwith of the filter
  * @param  operation_mode: Sets between continuous operation and low power mode
  * @retval None
  */
void Init_Accel_MPU9250(uint8_t scale, uint8_t sample_mode,uint8_t operation_mode)
{
	Set_Sample_Rate_Divider(0);
	Write_Byte_MPU9250(MPU9250_ACCEL_CONFIG,scale);				//Sets the Accelelerometer full scale
	delay(1);

	switch(scale)
	{
		case ACCEL_SCALE_2:	Accelerometer_Scale=2;				//Set the correct scale for data processing
							break;
		case ACCEL_SCALE_4:	Accelerometer_Scale=4;
							break;
		case ACCEL_SCALE_8:	Accelerometer_Scale=8;
							break;
		case ACCEL_SCALE_16:Accelerometer_Scale=16;
							break;
	}
	Write_Byte_MPU9250(MPU9250_ACCEL_CONFIG2,sample_mode);		//Sets the sampling frequency and LPF BW
	delay(1);
	if (operation_mode == NORMAL_OPERATION)						//Normal mode is selected
	{
		Write_Byte_MPU9250(MPU9250_PWR_MGMT_1,0x01);
		delay(1);
	}
	else														//Low power mode is selected
	{
		Write_Byte_MPU9250(MPU9250_LP_ACCEL_ODR,operation_mode);
		delay(1);
		Write_Byte_MPU9250(MPU9250_PWR_MGMT_1,0x21);			//Enable Cycle bit to use low power ODR
		delay(1);
	}
}

/**
  * @brief  Initializes MPU9250 Gyroscope specific registers.
  *
  * @note   Filter can be bypassed (GYRO_BYPASS_MODE_1 & GYRO_BYPASS_MODE_2). The configuration of the other
  * 		settings are not yet implemented
  *
  * @param  scale: Sets the scale of the sensor (250dps,500dps,1000dps and 2000dps)
  * @param  sample_mode: Sets the base Fs and Bandwith of the filter
  * @retval None
  */
void Init_Gyro_MPU9250(uint8_t scale, uint8_t sample_mode)
{
	if(sample_mode == GYRO_BYPASS_MODE_1)
	{
		Write_Byte_MPU9250(MPU9250_GYRO_CONFIG,scale | 0x01);	//bypass DLPF FS=32kHz BW=8800Hz
		delay(1);
		return;
	}
	else if(sample_mode == GYRO_BYPASS_MODE_2)
	{
		Write_Byte_MPU9250(MPU9250_GYRO_CONFIG,scale | 0x10);	//bypass DLPF FS=32kHz BW=3600Hz
		delay(1);
		return;
	}
	else
	{
		Write_Byte_MPU9250(MPU9250_GYRO_CONFIG,scale);			//[1:0]=00, DLPF enabled
		delay(1);
		Write_Byte_MPU9250(MPU9250_CONFIG,sample_mode);			//Sets the sampling frequency and LPF BW
		delay(1);
	}
	switch(scale)												//Set the correct scale for data processing
	{
		case GYRO_SCALE_250:	Gyroscope_Scale=250;
								break;
		case GYRO_SCALE_500:	Gyroscope_Scale=500;
								break;
		case GYRO_SCALE_1000:	Gyroscope_Scale=1000;
								break;
		case GYRO_SCALE_2000:	Gyroscope_Scale=2000;
								break;
	}
}

/**
  * @brief  Initializes the AK8963´s Magnetometer  registers.
  *
  * @note   The AK8963 is accessed via 2 ways. In one hand the configuration is done through SLV4 and the internal I2C bus,
  * 		while on the other hand, data read can be done through either SLV0 (on the background) or SLV4.
  *			SLV0 samples all of the Magnetometer data registers at the frequency defined by the SAMPLE_RATE_DIVIDER and
  *			saves it on the EXT_SENS_DATA 0 to 7 while the SLV4 reads at one byte at a time.
  *
  * @param  res: Sets the bit resolution of the data output.
  * @param  sample_mode: Sets between Fs=8Hz and Fs=100Hz.
  * @retval None
  */
void Init_Mag_AK8963(uint8_t bit_res, uint8_t sample_rate)
{

	Write_Byte_MPU9250(MPU9250_I2C_SLV4_ADDR,AK8963_ADDRESS|MPU9250_WRITEREG); 	//Load AK8963 addr
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_REG,AK8963_CNTL2); 						//Register to be written
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_DO,0x01);								//Soft reset
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_CTRL,0x80);								//Enable transfer
	delay(1);

	Write_Byte_MPU9250(MPU9250_I2C_SLV4_ADDR,AK8963_ADDRESS|MPU9250_WRITEREG);	//Load AK8963 addr
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_REG,AK8963_CNTL1);						//Register to be written
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_DO,bit_res|sample_rate);				//sets up desired resolution and sample rate
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_CTRL,0x80);								//Enable transfer
	delay(1);

	Write_Byte_MPU9250(MPU9250_I2C_SLV4_ADDR,AK8963_ADDRESS|MPU9250_WRITEREG);	//Load AK8963 addr
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_REG,AK8963_ASTC); 						//Selftest register
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_DO,0x00);								//Disable Selftest for normal operation
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_CTRL,0x80);								//Begin I2C transmission
	delay(1);

	Write_Byte_MPU9250(MPU9250_I2C_SLV4_ADDR,AK8963_ADDRESS|MPU9250_READREG); 	//Load AK8963 addr to SLV4 ADDR to read
	delay(1);


	Write_Byte_MPU9250(MPU9250_I2C_SLV0_ADDR,AK8963_ADDRESS|MPU9250_READREG); 	//Load AK8963 addr to SLV0 ADDR
	Write_Byte_MPU9250(MPU9250_I2C_SLV0_REG,AK8963_ST1); 						//Load AK8963 the ST1 addr to sample all the status and data registers
 	Write_Byte_MPU9250(MPU9250_I2C_SLV0_CTRL,0x88);								//Enable data read at 1kHz (8 bytes)
	delay(1);

	Write_Byte_MPU9250(MPU9250_I2C_SLV1_ADDR,AK8963_ADDRESS|MPU9250_READREG); 	//Load AK8963 addr to SLV1 ADDR
	Write_Byte_MPU9250(MPU9250_I2C_SLV1_REG,AK8963_WHO_AM_I); 					//Load AK8963 WAI for debugging purposes
	Write_Byte_MPU9250(MPU9250_I2C_SLV1_CTRL,0x81);								//Enable data read at 1kHz (1 bytes,WAI)
	delay(1);

}

/**
  * @brief  Puts the MPU9250 in sleep mode.
  *
  * @param  None
  * @retval None
  */
void Sleep_MPU9250(void)
{

	Write_Byte_MPU9250(MPU9250_PWR_MGMT_1,SLEEP_MODE);
	delay(10); //delay por las dudas, hay que ver si es necesario
}

/**
  * @brief  Wakes up the MPU9250 from sleep mode.
  *
  * @param  None
  * @retval None
  */
void Wake_Up_MPU9250(void)
{
	uint8_t Power_Register_Status;
	Read_Byte_MPU9250(MPU9250_PWR_MGMT_1,&Power_Register_Status);
	Write_Byte_MPU9250(MPU9250_PWR_MGMT_1,Power_Register_Status & 0x01);	// Wake up and Auoselect clk
	delay(10);															// Wait for device to wake up
}


/**
  * @brief  Reads the all of the IMUs data registers
  *
  *	@note	Communication with AK8963 is established through the internal
  *			I2C via SLV0 reading the EXT_SENS registers (read only).
  *
  *			rawData structure
  *			Array Element N°	Description
  *			0					ACCEL X AXIS High
  *			1					ACCEL X AXIS Low
  *			2					ACCEL Y AXIS High
  *			3					ACCEL Y AXIS Low
  *			4					ACCEL Z AXIS High
  *			5					ACCEL Z AXIS Low
  *			6					TEMP High
  *			7					TEMP Low
  *			8					GYRO X AXIS High
  *			9					GYRO X AXIS Low
  *			10					GYRO Y AXIS High
  *			11					GYRO Y AXIS Low
  *			12					GYRO Z AXIS High
  *			13					GYRO Z AXIS Low
  *			14					AK8963 ST1
  *			15					AK8963 X AXIS LOW
  *			16					AK8963 X AXIS HIGH
  *			17					AK8963 Y AXIS LOW
  *			18					AK8963 Y AXIS HIGH
  *			19					AK8963 Z AXIS LOW
  *			20					AK8963 Z AXIS HIGH
  *			21					AK8963 ST2
  *			22					WAI (debugging)


  *			25-37				Dot Care
  *
  *			IMU_Data & rawData structure
  *			Array Element N°	Description
  *			0					ACCEL X AXIS
  *			1					ACCEL Y AXIS
  *			2					ACCEL Z AXIS
  *			3					TEMP
  *			4					GYRO X AXIS
  *			5					GYRO Y AXIS
  *			6					GYRO Z AXIS
  *			7					MAG X AXIS
  *			8					MAG Y AXIS
  *			9					MAG Z AXIS
  *
  *
  * @param  buffer: pointer to buffer where data is saved.
  * @retval None
  */

void Get_IMU_Data(float *buffer)
{

	uint8_t rawData[23]={0};
	int16_t erawData[10]={0};

	Read_Bytes_MPU9250(MPU9250_ACCEL_XOUT_H, rawData, 23); // Accel 6 regs, temp 2 regs, Gyro 6 regs,
														   // 24 Ext Sens Data Regs (9 Used)
	/*	Accel X Axis	*/
	erawData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	*(buffer) = ((float) erawData[0]) * ((float)Accelerometer_Scale/32768.0)  - Accelerometer_Bias[0];
	/*	Accel Y Axis	*/
	erawData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	*(buffer+1) = ((float) erawData[1]) * ((float)Accelerometer_Scale/32768.0)  - Accelerometer_Bias[1];
	/*	Accel Z Axis	*/
	erawData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	*(buffer+2) = ((float) erawData[2]) * ((float)Accelerometer_Scale/32768.0)  - Accelerometer_Bias[2];
	/*	Temp Sens	*/
	erawData[3] = (uint16_t) (((int16_t)rawData[6] << 8) | rawData[7]);
	*(buffer+3) = (( ((float) erawData[3]) - RoomTemp_Offset )/Temp_Sensitivity) +  21;  //from register datasheet
	/*	Gyro X Axis	*/
	erawData[4] = (uint16_t) (((int16_t)rawData[8] << 8) | rawData[9]);
	*(buffer+4) = ((float) erawData[4]) * ((float)Gyroscope_Scale/32768.0)  - Gyroscope_Bias[0];
	/*	Gyro Y Axis	*/
	erawData[5] = (uint16_t) (((int16_t)rawData[10] << 8) | rawData[11]);
	*(buffer+5) = ((float) erawData[5]) * ((float)Gyroscope_Scale/32768.0)  - Gyroscope_Bias[1];
	/*	Gyro Z Axis	*/
	erawData[6] = (uint16_t) (((int16_t)rawData[12] << 8) | rawData[13]);
	*(buffer+6) = ((float) erawData[6]) * ((float)Gyroscope_Scale/32768.0)  - Gyroscope_Bias[2];

	if(!(rawData[21] & 0x08))	//Check for sensor overflow

	{
		/*	Mag X Axis	*/
		erawData[7] = (uint16_t) (((int16_t)rawData[16] << 8) | rawData[15]);
		*(buffer+7) = ((float) erawData[7]) * ((float)Magnetometer_Scale/32760.0)  - Magnetometer_Bias[0];
		/*	Mag Y Axis	*/
		erawData[8] = (uint16_t) (((int16_t)rawData[18] << 8) | rawData[17]);
		*(buffer+8) = ((float) erawData[8]) * ((float)Magnetometer_Scale/32760.0)  - Magnetometer_Bias[1];
		/*	Mag Z Axis	*/
		erawData[9] = (uint16_t) (((int16_t)rawData[20] << 8) | rawData[19]);
		*(buffer+9) = ((float) erawData[9]) * ((float)Magnetometer_Scale/32760.0)  - Magnetometer_Bias[2];
	}

}



/**
  * @brief  Reads the temperature raw data from the MPU registers and
  * 		saves after processing it.
  *
  * @param  Temperature_Value: Temperature value in float format.
  * @retval None.
  */

void Get_Temperature_MPU9250(float *Temperature_Value)
{
	uint8_t Temp_Count[2];
	int16_t Temperature_Count;


	Read_Byte_MPU9250(MPU9250_TEMP_OUT_H,&Temp_Count[0]);
	Read_Byte_MPU9250(MPU9250_TEMP_OUT_L,&Temp_Count[1]);

	Temperature_Count = (int16_t)(((int16_t)Temp_Count[0] << 8) | Temp_Count[1]);
	*Temperature_Value=(( ((float) Temperature_Count) - RoomTemp_Offset )/Temp_Sensitivity) +  21;  //from register datasheet

}

/**
  * @brief  Reads the Accelerometer raw data from the MPU registers and
  * 		saves it on a buffer.
  *
  * @param  Accelerometer_Data: pointer to buffer where data is saved.
  * @retval None
  */
void MPU9250_Get_Accelerometer_Data(float *Accelerometer_Data)
{
	uint8_t Accelerometer_Counts[6];
	int16_t Accelerometer_Raw_Data[3];

	Read_Bytes_MPU9250(MPU9250_ACCEL_XOUT_H, Accelerometer_Counts, 6); //sequential register read
	/*Array structure detail: Acelerometer_Counts ->	X_H
	 * 													X_L
	 * 													Y_H
	 * 													Y_L
	 * 													Z_H
	 * 													Z_L
	 */
	for (uint8_t i = 0; i < 3; i++)
	{
		Accelerometer_Raw_Data[i] = ((int16_t)Accelerometer_Counts[i*2] << 8) | Accelerometer_Counts[i*2+1];
		*Accelerometer_Data = ((float) Accelerometer_Raw_Data[i]) * ((float)Accelerometer_Scale/32768.0)  - Accelerometer_Bias[i];
		Accelerometer_Data++;
	}
}

/**
  * @brief  Reads the Gyroscope raw data from the MPU registers and
  * 		saves it on a buffer.
  *
  * @param  Gyroscope_Data: pointer to buffer where data is saved.
  * @retval None
  */
void MPU9250_Get_Gyroscope_Data(float *Gyroscope_Data)
{
	uint8_t Gyroscope_Counts[6];
	int16_t Gyroscope_Raw_Data[3];

	Read_Bytes_MPU9250(MPU9250_GYRO_XOUT_H, Gyroscope_Counts, 6); //sequential register read
	/*Array structure detail: Gyroscope_Counts ->		X_H
	 * 													X_L
	 * 													Y_H
	 * 													Y_L
	 * 													Z_H
	 * 													Z_L
	 */
	for (uint8_t i = 0; i < 3; i++)
	{
		Gyroscope_Raw_Data[i] = ((int16_t)Gyroscope_Counts[i*2] << 8) | Gyroscope_Counts[i*2+1];
		*Gyroscope_Data = ((float) Gyroscope_Raw_Data[i]) * ((float)Gyroscope_Scale/32768.0)  - Gyroscope_Bias[i];
		Gyroscope_Data++;
	}
}



/**
  * @brief  Reads the Magnetometer raw data from the AK8963 registers and
  * 		saves it on a buffer.
  *
  *	@note	Communication with AK8963 is established through the internal
  *			I2C via SLV4 registers (read only)
  *
  * @param  Magnetometer_Data: pointer to buffer where data is saved.
  * @retval None
  */
void AK8963_SLV4_Get_Magnetometer_Data(float *Magnetometer_Data)
{
	uint8_t Magnetometer_Counts[6];
	int16_t Magnetometer_Raw_Data[3];
	uint8_t ST2_Register;

	for(uint8_t i=0;i<3;i++)
	{

			Write_Byte_MPU9250(MPU9250_I2C_SLV4_REG,AK8963_HXL+(i*2));			//Reading LSB
			Write_Byte_MPU9250(MPU9250_I2C_SLV4_CTRL,0x80);						//Format L:H
			delay(1);
			Read_Byte_MPU9250(MPU9250_I2C_SLV4_DI,&Magnetometer_Counts[2*i]);	//Save H_AXIS_LOW in buffer

			Write_Byte_MPU9250(MPU9250_I2C_SLV4_REG,AK8963_HXH+(i*2));			//Reading MSB
			Write_Byte_MPU9250(MPU9250_I2C_SLV4_CTRL,0x80);						//Format L:H
			delay(1);
			Read_Byte_MPU9250(MPU9250_I2C_SLV4_DI,&Magnetometer_Counts[2*i+1]);	//Save H_AXIS_HIGH in buffer

	}
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_REG,AK8963_ST2);
	Write_Byte_MPU9250(MPU9250_I2C_SLV4_CTRL,0x80);
	delay(1);
	Read_Byte_MPU9250(MPU9250_I2C_SLV4_DI,&ST2_Register);					//Reading ST2 Register

	if(!(ST2_Register&0x80))
	{
		for(uint8_t i=0;i<3;i++)
		{
			Magnetometer_Raw_Data[i] = ((int16_t)Magnetometer_Counts[i*2+1] << 8) | Magnetometer_Counts[i*2];
			*(Magnetometer_Data+i+7) = ((float) Magnetometer_Raw_Data[i]) * ((float)Magnetometer_Scale/32760.0)  - Magnetometer_Bias[i];
		}
	}
}



/**
  * @brief  Reads the Accelerometer, Gyroscope and temperature sensor raw data from the MPU9250 registers and
  * 		saves it on a buffer.
  *
  * @param  buffer: pointer to buffer where data is saved.
  * @retval None
  */

void getDataMPU(float *buffer)
{

	uint8_t buffer_Counts[14];
	int16_t buffer_Raw_Data[7];

	Read_Bytes_MPU9250(MPU9250_ACCEL_XOUT_H, buffer_Counts, 14); //sequential register read


	for (uint8_t i = 0; i < 3; i++)
		{
			buffer_Raw_Data[i] = ((int16_t)buffer_Counts[i*2] << 8) | buffer_Counts[i*2+1];
			*(buffer+i) = ((float) buffer_Raw_Data[i]) * ((float)Accelerometer_Scale/32768.0)  - Accelerometer_Bias[i];
		}


	buffer_Raw_Data[3] = (uint16_t) (((int16_t)buffer_Counts[6] << 8) | buffer_Counts[7]);
	*(buffer+3) = (( ((float) buffer_Raw_Data[3]) - RoomTemp_Offset )/Temp_Sensitivity) +  21;  //from register datasheet


	for (uint8_t i = 0; i < 3; i++)
		{
			buffer_Raw_Data[i+4] = ((int16_t)buffer_Counts[i*2+8] << 8) | buffer_Counts[i*2+1+8];
			*(buffer+4+i) = ((float) buffer_Raw_Data[i+4]) * ((float)Gyroscope_Scale/32768.0)  - Gyroscope_Bias[i];
		}

}



//=====[Implementations of private functions]==================================





// anuladas hasta que requieran uso:
/*--- extern I2C_HandleTypeDef hi2c1;            -------*/
/*--- extern CS_Status_TypeDef CS_Status;            ---*/
