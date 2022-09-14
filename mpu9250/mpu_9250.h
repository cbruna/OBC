//=====[#include guards - begin]===============================================
#ifndef MPU9250_H_
#define MPU9250_H_

//=====[Declaration of public defines]=========================================
#define SET_H_RESET				0x80
#define SLEEP_MODE				0x40
#define SRST					0x01
#define CONFIG_ERROR			0x01
#define AK8963_SAMPLING_8HZ		0x02
#define	AK8963_SAMPLING_100HZ	0x06
#define AK8963_MAG_14_BITS		0x00
#define AK8963_MAG_16_BITS		0x10
#define GYRO_SCALE_250			0x00
#define	GYRO_SCALE_500			0x08
#define	GYRO_SCALE_1000			0x10
#define	GYRO_SCALE_2000			0x18
#define ACCEL_SCALE_2			0x00
#define	ACCEL_SCALE_4			0x08
#define	ACCEL_SCALE_8			0x10
#define	ACCEL_SCALE_16			0x18
#define GYRO_BYPASS_MODE_1		0xFF	//FS=32kHz, BW=8800Hz
#define GYRO_BYPASS_MODE_2		0xFE	//FS=32kHz,	BW=3600Hz
#define GYRO_DLPF_MODE_1		0		//FS=8kHz, BW=250Hz
#define GYRO_DLPF_MODE_2		1		//FS=1kHz, BW=184Hz
#define GYRO_DLPF_MODE_3		2		//FS=1kHz, BW=92Hz
#define GYRO_DLPF_MODE_4		3		//FS=1kHz, BW=41Hz
#define GYRO_DLPF_MODE_5		4		//FS=1kHz, BW=20Hz
#define GYRO_DLPF_MODE_6		5		//FS=1kHz, BW=10Hz
#define GYRO_DLPF_MODE_7		6		//FS=1kHz, BW=5Hz
#define GYRO_DLPF_MODE_8		7		//FS=8kHz, BW=3600Hz
#define ACCEL_BYPASS_DLPF		0x08	//FS=4kHz, BW=1046Hz
#define ACCEL_DLPF_MODE_1		0x00	//FS=1kHz, BW=218.1Hz
#define ACCEL_DLPF_MODE_2		0x01	//FS=1kHz, BW=218.1Hz
#define ACCEL_DLPF_MODE_3		0x02	//FS=1kHz, BW=99Hz
#define ACCEL_DLPF_MODE_4		0x03	//FS=1kHz, BW=44.8Hz
#define ACCEL_DLPF_MODE_5		0x04	//FS=1kHz, BW=21.2Hz
#define ACCEL_DLPF_MODE_6		0x05	//FS=1kHz, BW=10.2Hz
#define ACCEL_DLPF_MODE_7		0x06	//FS=1kHz, BW=5.05Hz
#define ACCEL_DEC2				0x07	//FS=1kHz, BW=420Hz
#define	NORMAL_OPERATION		0x10	//Low power disabled
#define	ODR_MODE_1				0		//Wake up Freq 0.24Hz
#define	ODR_MODE_2				1		//Wake up Freq 0.49Hz
#define	ODR_MODE_3				2		//Wake up Freq 0.98Hz
#define	ODR_MODE_4				3		//Wake up Freq 1.95Hz
#define	ODR_MODE_5				4		//Wake up Freq 3.91Hz
#define	ODR_MODE_6				5		//Wake up Freq 7.81Hz
#define	ODR_MODE_7				6		//Wake up Freq 15.63Hz
#define	ODR_MODE_8				7		//Wake up Freq 31.25Hz
#define	ODR_MODE_9				8		//Wake up Freq 65.5Hz
#define	ODR_MODE_10				9		//Wake up Freq 125Hz
#define	ODR_MODE_11				10		//Wake up Freq 250Hz
#define	ODR_MODE_12				11		//Wake up Freq 500Hz
#define	OVERFLOW_FILTER_OFF		0

//=====[Declaration of public data types]======================================
typedef enum
{
	MPU9250_BUSY = 0x01,
	MPU9250_IDLE = 0x02
} MPU9250_Status_TypeDef;

//=====[Declarations (prototypes) of public functions]=========================

void Reset_CS_MPU9250(void);
void Set_CS_MPU9250(void);

void Read_Byte_MPU9250(uint8_t reg , uint8_t *pvalue);
void Read_Bytes_MPU9250(uint8_t reg , uint8_t *pvalue, uint8_t size);
void Write_Byte_MPU9250(uint8_t reg , uint8_t value);

//void Init_MPU9250(SPI_HandleTypeDef *handle_spi);
void Init_MPU9250(void); // new definition
void Init_Mag_AK8963(uint8_t bit_res, uint8_t sample_rate);
void Init_Accel_MPU9250(uint8_t scale, uint8_t sample_mode,uint8_t operation_mode);
void Init_Gyro_MPU9250(uint8_t scale, uint8_t sample_mode);
void Set_Sample_Rate_Divider(uint8_t div);
void Sleep_MPU9250(void);
void Wake_Up_MPU9250(void);

/* Individual data read	*/
void Get_Temperature_MPU9250(float *Temperature_Value);
void MPU9250_Get_Accelerometer_Data(float *Accelerometer_Data);
void MPU9250_Get_Gyroscope_Data(float *Gyroscope_Data);
void AK8963_SLV4_Get_Magnetometer_Data(float *Magnetometer_Data);

/*	Accel, Gyro , Temp & Magnetometer data batch read*/
void Get_IMU_Data(float *buffer);

/* ccel, Gyro & Temp data batch read	*/
void getDataMPU(float *buffer);

//=====[#include guards - end]=================================================

#endif

