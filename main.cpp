//=====[Libraries]=============================================================
#include "arm_book_lib.h"
#include "mbed.h"

#include "temperature_sensor.h"
#include "pc_serial_com.h"
#include "ble_com.h"
#include "date_and_time.h"

#include "mpu_9250.h"
#include "MadgwickAHRS.h"

#include <math.h>


#define CONTEO_PARA_TRAMA 5 // cantidad de unidades de tiempo dt para manejo de TRAMA

DigitalOut 	led(LED3);
Ticker		interrupcionTemporal;

bool leerSensores=false,enviarTrama=false;

char temperaturaString[16]="";

char aceleracionesString[32]="";
//                         "ax= xx.xx ay= yy.yy az= zz.zz \r\n"
char girosString[32]="";
//                         "gx= xx.xx gy= yy.yy gz= zz.zz \r\n"
char magnetosString[32]="";
//                         "mx= xx.xx my= yy.yy mz= zz.zz \r\n"

char rollString[16]="";
char pitchString[16]="";
char yawString[16]="";
char angulosString[32]="";

char tramaString[64]="";
char timeStamp[32]="";
const char mensajeInicio[]="Inicializando RTC previo al vuelo\r\n";

float rolido,cabeceo,guiniada;

float PI= 3.14;
float dt=0.1;

int i;
int conteo = CONTEO_PARA_TRAMA;



void actualizarDatos()
{
	//entra cada dt segs
	leerSensores=true;

	//conteo para manejo de trama
	if(conteo>0)
	{
		conteo--;

		if(conteo<=0)
		{
			// indicador luminoso de conteo completado
			led=!led;

			conteo=0;
			enviarTrama=true;
		}
	}
}

//=====[Main function, the program entry point after power on or reset]========


int main()
{

	static float imuData[10]={0};


	//--- asigno la interrupción a la función
	interrupcionTemporal.attach(&actualizarDatos,dt);

	temperatureSensorInit();


	//--- inicializo IMU
	Init_MPU9250();
	Init_Accel_MPU9250(ACCEL_SCALE_2,ACCEL_BYPASS_DLPF,NORMAL_OPERATION);
	Init_Gyro_MPU9250(GYRO_SCALE_2000,GYRO_BYPASS_MODE_1);
	Init_Mag_AK8963(AK8963_MAG_16_BITS,AK8963_SAMPLING_100HZ);


	//--- inicializo RTC
	pcSerialComStringWrite(mensajeInicio);
	commandSetDateAndTime();

	while (true)
    {

    	if(leerSensores)
    	{
    		leerSensores=false; // limpio el flag

    		//---temperatura
    		temperatureSensorUpdate();

    		//enviar a pc por puerto serie
    		//sprintf(temperaturaString,"temp = %.1f °C\r\n",temperatureSensorReadCelsius());
    		//pcSerialComStringWrite(temperaturaString);


    		//-- leer dato IMU
    		Get_IMU_Data(imuData);

    		//-- debug datos inerciales y magnetométricos crudos
    		/*
    		sprintf(aceleracionesString,"ax= %.2f ay= %.2f az= %.2f \r\n",imuData[0],imuData[1],imuData[2]);
    		pcSerialComStringWrite(aceleracionesString);
    		sprintf(girosString,"gx= %.2f gy= %.2f gz= %.2f \r\n",imuData[4],imuData[5],imuData[6]);
    		pcSerialComStringWrite(girosString);
    		sprintf(magnetosString,"mx= %.2f my= %.2f mz= %.2f \r\n",imuData[7],imuData[8],imuData[9]);
    		pcSerialComStringWrite(magnetosString);
			*/

    		//--- cálculo de actitud
    		//estimador Madgwick

    		/*for(i=4;i<7;i++)
    			if(abs(imuData[i])<5)
    				imuData[i]=0;
			*/

    		MadgwickAHRSupdate(imuData[4]*PI/180, imuData[5]*PI/180, imuData[6]*PI/180, imuData[0], imuData[1], imuData[2], imuData[8], imuData[7], -imuData[9]);

    		//enviar a pc por puerto serie
    		/*
    		sprintf(rollString,"roll = %.1f °\r\n",GetRoll());
    		pcSerialComStringWrite(rollString);

    		sprintf(pitchString,"pitch = %.1f °\r\n",GetPitch());
    		pcSerialComStringWrite(pitchString);

    		sprintf(yawString,"yaw = %.1f °\r\n",GetYaw());
    		pcSerialComStringWrite(yawString);
    		*/

    		//--- ángulos de actitud y rumbo
    		//sprintf(angulosString,"roll= %.1f \t pitch= %.1f \t yaw= %.1f \r\n",GetRoll(),GetPitch(),GetYaw());


    		rolido=GetRoll();
    		cabeceo=GetPitch();
    		guiniada=GetYaw();

    		//sprintf(angulosString,"roll= %.1f \t pitch= %.1f \t yaw= %.1f \r\n",rolido,cabeceo,guiniada);

    		//enviar a PC por puerto serie
    		//pcSerialComStringWrite(angulosString);

    		//enviar a "Estación Terrena" por BLE (Bluetooth)
    		//bleComStringWrite(angulosString);

    	}

    	if(enviarTrama)
    	{
    		enviarTrama=false; //limpio el flag

    		conteo=CONTEO_PARA_TRAMA; //restablezco contador

    		//armado de trama
    		//sprintf(angulosString,"roll= %.1f \t pitch= %.1f \t yaw= %.1f \r\n",rolido,cabeceo,guiniada);
    		//enviar a PC por puerto serie
    		//pcSerialComStringWrite(angulosString);

    		//armado de trama
    		sprintf(tramaString,"OBC-3: %s \t:temp=%.1f \t roll= %.1f \t pitch= %.1f \t yaw= %.1f \r\n",dateAndTimeRead(),temperatureSensorReadCelsius(),rolido,cabeceo,guiniada);
    		//enviar a PC por puerto serie
    		pcSerialComStringWrite(tramaString);


    		//enviar a "Estación Terrena" por BLE (Bluetooth)
    		bleComStringWrite(tramaString);

    	}

    }
}
