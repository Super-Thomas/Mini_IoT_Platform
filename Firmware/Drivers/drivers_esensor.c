/*
	Drivers for BME680
	Thanks to BME680_driver(https://github.com/BoschSensortec/BME680_driver)
*/
#include <stdio.h>
#include <string.h>
#include <NuMicro.h>
#include "main.h"
#include "drivers_esensor.h"

#define BME680_SLAVE_ADDRESS			0x76
#define BME680_I2C_TIMEOUT				0xFF

struct	bme680_calib_data {
	/*! Variable to store calibrated humidity data */
	uint16_t par_h1;
	uint16_t par_h2;
	int8_t par_h3;
	int8_t par_h4;
	int8_t par_h5;
	uint8_t par_h6;
	int8_t par_h7;
	
	/*! Variable to store calibrated gas data */
	int8_t par_gh1;
	int16_t par_gh2;
	int8_t par_gh3;
	
	/*! Variable to store calibrated temperature data */
	uint16_t par_t1;
	int16_t par_t2;
	int8_t par_t3;
	
	/*! Variable to store calibrated pressure data */
	uint16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int16_t par_p4;
	int16_t par_p5;
	int8_t par_p6;
	int8_t par_p7;
	int16_t par_p8;
	int16_t par_p9;
	uint8_t par_p10;

	/*! Variable to store t_fine size */
	int32_t t_fine;

	/*! Variable to store heater resistance range */
	uint8_t res_heat_range;
	int8_t res_heat_val;
	
	/*! Variable to store error range */
	int8_t range_sw_err;
};

/*!
 * @brief BME680 sensor settings structure which comprises of ODR,
 * over-sampling and filter settings.
 */
struct	bme680_tph_sett {
	/*! Humidity oversampling */
	uint8_t os_hum;
	/*! Temperature oversampling */
	uint8_t os_temp;
	/*! Pressure oversampling */
	uint8_t os_pres;
	/*! Filter coefficient */
	uint8_t filter;
};

/*!
 * @brief BME680 gas sensor which comprises of gas settings
 *  and status parameters
 */
struct	bme680_gas_sett {
	/*! Variable to store nb conversion */
	uint8_t nb_conv;
	/*! Variable to store heater control */
	uint8_t heatr_ctrl;
	/*! Run gas enable value */
	uint8_t run_gas;
	/*! Heater temperature value */
	uint16_t heatr_temp;
	/*! Duration profile value */
	uint16_t heatr_dur;
};

struct	bme680_dev {
	/*! Chip Id */
	//uint8_t chip_id;
	/*! Device Id */
	//uint8_t dev_id;
	/*! SPI/I2C interface */
	//enum bme680_intf intf;
	/*! Memory page used */
	//uint8_t mem_page;
	/*! Ambient temperature in Degree C */
	int8_t amb_temp;
	/*! Sensor calibration data */
	struct bme680_calib_data calib;
	/*! Sensor settings */
	struct bme680_tph_sett tph_sett;
	/*! Gas Sensor settings */
	struct bme680_gas_sett gas_sett;
	/*! Sensor power modes */
	uint8_t power_mode;
	/*! New sensor fields */
	uint8_t new_fields;
	/*! Store the info messages */
	//uint8_t info_msg;
	/*! Bus read function pointer */
	//bme680_com_fptr_t read;
	/*! Bus write function pointer */
	//bme680_com_fptr_t write;
	/*! delay function pointer */
	//bme680_delay_fptr_t delay_ms;
	/*! Communication function result */
	//int8_t com_rslt;
};

struct	bme680_field_data {
	/*! Contains new_data, gasm_valid & heat_stab */
	uint8_t status;
	/*! The index of the heater profile used */
	uint8_t gas_index;
	/*! Measurement index to track order */
	uint8_t meas_index;

	/*! Temperature in degree celsius */
	int16_t temperature;
	/*! Pressure in Pascal */
	uint32_t pressure;
	/*! Humidity in % relative humidity x1000 */
	uint32_t humidity;
	/*! Gas resistance in Ohms */
	uint32_t gas_resistance;
	
	int16_t air_quality_score;
};

struct bme680_dev g_bme680;
struct bme680_field_data g_bme680_data;

void ESENS_Init(void)
{
	// Unlock protected registers
  SYS_UnlockReg();
	
	// Enable I2C1 peripheral clock
  CLK_EnableModuleClock(I2C1_MODULE);
	
	// Set PD multi-function pins for I2C1
	SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk)) | (SYS_GPD_MFPL_PD4MFP_I2C1_SDA | SYS_GPD_MFPL_PD5MFP_I2C1_SCL);
	
	// Lock protected registers
  SYS_LockReg();
	
	// Open I2C module and set bus clock
  I2C_Open(I2C1, 100000);
	
	// Enable interrupt
	//I2C_EnableInt(I2C1);
  //NVIC_EnableIRQ(I2C1_IRQn);
}

uint8_t ESENS_Write(uint8_t u8RegAddr, uint8_t u8Data)
{
  uint8_t u8Err = 0;
	uint32_t u32TimeoutCnt = 0;
	
	do
	{
		u8Err = 0;

		// Send start
		I2C_START(I2C1);
		I2C_WAIT_READY(I2C1);
		
		if (I2C_GET_STATUS(I2C1) == 0x08)
		{
			// Send slave address
			I2C_SET_DATA(I2C1, (BME680_SLAVE_ADDRESS << 1));
			I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
			I2C_WAIT_READY(I2C1);
			
			if (I2C_GET_STATUS(I2C1) == 0x18) // ACK from Slave
			{
				// Send register address
				I2C_SET_DATA(I2C1, u8RegAddr);
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
				I2C_WAIT_READY(I2C1);
				
				if (I2C_GET_STATUS(I2C1) == 0x28) // ACK from Slave
				{
					// Send register data
					I2C_SET_DATA(I2C1, u8Data);
					I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
					I2C_WAIT_READY(I2C1);
					
					if (I2C_GET_STATUS(I2C1) == 0x28) // ACK from Slave
					{
						// Send stop
						I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO | I2C_CTL_SI);
						TIMER_Delay(TIMER0, 100);
					}
					else
					{
						u8Err = 4;
					}
				}
				else
				{
					u8Err = 3;
				}
			}
			else
			{
				u8Err = 2;
			}
		}
		else
		{
			u8Err = 1;
		}
			
		if (u8Err)
		{
			// Send stop
			I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO | I2C_CTL_SI);
			TIMER_Delay(TIMER0, 100);
			u32TimeoutCnt++;
		}
			
		if (u32TimeoutCnt >= BME680_I2C_TIMEOUT)
		{
			break;
		}
	}
	while (u8Err);
	
	return u8Err;
}

uint8_t ESENS_Read(uint8_t u8RegAddr, uint8_t* pu8Data)
{
	uint8_t u8Err;
	uint32_t u32TimeoutCnt = 0;
	uint8_t u8Data = 0;
  
	do 
  {
		u8Err = 0;
       
		// Send start
		I2C_START(I2C1);
		I2C_WAIT_READY(I2C1);
		
		if (I2C_GET_STATUS(I2C1) == 0x08)
		{
			// Send slave address for write
			I2C_SET_DATA(I2C1, (BME680_SLAVE_ADDRESS << 1));
			I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
			I2C_WAIT_READY(I2C1);
			
			if (I2C_GET_STATUS(I2C1) == 0x18) // ACK from Slave
			{
				// Send register address for read
				I2C_SET_DATA(I2C1, u8RegAddr);
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
				I2C_WAIT_READY(I2C1);
				
				if (I2C_GET_STATUS(I2C1) == 0x28) // ACK from Slave
				{
					// Send stop and start
					I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA | I2C_CTL_STO | I2C_CTL_SI);
					I2C_WAIT_READY(I2C1);
					
					if (I2C_GET_STATUS(I2C1) == 0x08) // ACK from Slave
					{
						// Send slave address for read
						I2C_SET_DATA(I2C1, (BME680_SLAVE_ADDRESS << 1) | 0x01);
						I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
						I2C_WAIT_READY(I2C1);
						
						if (I2C_GET_STATUS(I2C1) == 0x40) // ACK from Slave
						{
							I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA); // Send ACK
							I2C_WAIT_READY(I2C1);
									
							// Read data
							u8Data = (uint8_t)I2C_GET_DATA(I2C1);
							
							if (I2C_GET_STATUS(I2C1) == 0x50) // ACK from Master
							{
								I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI); // Send NACK
								I2C_WAIT_READY(I2C1);
								
								if (I2C_GET_STATUS(I2C1) == 0x58) // NACK from Master
								{
									// Send stop
									I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO | I2C_CTL_SI);
									TIMER_Delay(TIMER0, 100);
								}
								else
								{
									u8Err = 7;
								}
							}
							else
							{
								u8Err = 6;
							}
						}
						else
						{
							u8Err = 5;
						}
					}
					else
					{
						u8Err = 4;
					}
				}
				else
				{
					u8Err = 3;
				}
			}
			else
			{
				u8Err = 2;
			}
		}
		else
		{
			u8Err = 1;
		}

    if (u8Err)
    {
			// Send stop
      I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO | I2C_CTL_SI);
			TIMER_Delay(TIMER0, 100);
			u32TimeoutCnt++;
    }
		
		if (u32TimeoutCnt >= BME680_I2C_TIMEOUT)
		{
			break;
		}
	} while (u8Err);
	
	*pu8Data = u8Data;
	
	return u8Err;
}

uint8_t ESENS_ReadChipID(void)
{
	uint8_t u8ChipID = 0;
	
	ESENS_Read(0xD0, &u8ChipID);
	
	return u8ChipID;
}

void ESENS_Get_Calib_Data(void)
{
	uint8_t u8Data[2];
	
	/* Temperature related coefficients */
	ESENS_Read(0xE9, &u8Data[0]);
	ESENS_Read(0xEA, &u8Data[1]);
	g_bme680.calib.par_t1 = (uint16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0x8A, &u8Data[0]);
	ESENS_Read(0x8B, &u8Data[1]);
	g_bme680.calib.par_t2 = (int16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0x8C, &u8Data[0]);
	g_bme680.calib.par_t3 = (int8_t)(u8Data[0]);
	
	/* Pressure related coefficients */
	ESENS_Read(0x8E, &u8Data[0]);
	ESENS_Read(0x8F, &u8Data[1]);
	g_bme680.calib.par_p1 = (uint16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0x90, &u8Data[0]);
	ESENS_Read(0x91, &u8Data[1]);
	g_bme680.calib.par_p2 = (int16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0x92, &u8Data[0]);
	g_bme680.calib.par_p3 = (int8_t)(u8Data[0]);
	
	ESENS_Read(0x94, &u8Data[0]);
	ESENS_Read(0x95, &u8Data[1]);
	g_bme680.calib.par_p4 = (int16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0x96, &u8Data[0]);
	ESENS_Read(0x97, &u8Data[1]);
	g_bme680.calib.par_p5 = (int16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0x99, &u8Data[0]);
	g_bme680.calib.par_p6 = (int8_t)(u8Data[0]);
	
	ESENS_Read(0x98, &u8Data[0]);
	g_bme680.calib.par_p7 = (int8_t)(u8Data[0]);
	
	ESENS_Read(0x9C, &u8Data[0]);
	ESENS_Read(0x9D, &u8Data[1]);
	g_bme680.calib.par_p8 = (int16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0x9E, &u8Data[0]);
	ESENS_Read(0x9F, &u8Data[1]);
	g_bme680.calib.par_p9 = (int16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0xA0, &u8Data[0]);
	g_bme680.calib.par_p10 = (uint8_t)(u8Data[0]);
	
	/* Humidity related coefficients */
	ESENS_Read(0xE2, &u8Data[0]);
	ESENS_Read(0xE3, &u8Data[1]);
	g_bme680.calib.par_h1 = (uint16_t)((u8Data[1] << 4) | (u8Data[0] & 0x0F));
	
	ESENS_Read(0xE2, &u8Data[0]);
	ESENS_Read(0xE1, &u8Data[1]);
	g_bme680.calib.par_h2 = (uint16_t)((u8Data[1] << 4) | (u8Data[0] >> 4));
	
	ESENS_Read(0xE4, &u8Data[0]);
	g_bme680.calib.par_h3 = (int8_t)(u8Data[0]);
		
	ESENS_Read(0xE5, &u8Data[0]);
	g_bme680.calib.par_h4 = (int8_t)(u8Data[0]);
		
	ESENS_Read(0xE6, &u8Data[0]);
	g_bme680.calib.par_h5 = (int8_t)(u8Data[0]);
		
	ESENS_Read(0xE7, &u8Data[0]);	
	g_bme680.calib.par_h6 = (uint8_t)(u8Data[0]);
		
	ESENS_Read(0xE8, &u8Data[0]);
	g_bme680.calib.par_h7 = (int8_t)(u8Data[0]);
	
	/* Gas heater related coefficients */
	ESENS_Read(0xED, &u8Data[0]);
	g_bme680.calib.par_gh1 = (int8_t)(u8Data[0]);
	
	ESENS_Read(0xEB, &u8Data[0]);
	ESENS_Read(0xEC, &u8Data[1]);
	g_bme680.calib.par_gh2 = (int16_t)((u8Data[1] << 8) | u8Data[0]);
	
	ESENS_Read(0xEE, &u8Data[0]);
	g_bme680.calib.par_gh3 = (int8_t)(u8Data[0]);

	/* Other coefficients */
	ESENS_Read(0x02, &u8Data[0]);
	g_bme680.calib.res_heat_range = ((u8Data[0] & 0x30) / 16);
	
	ESENS_Read(0x00, &u8Data[0]);
	g_bme680.calib.res_heat_val = (int8_t)(u8Data[0]);

	ESENS_Read(0x04, &u8Data[0]);
	g_bme680.calib.range_sw_err = ((int8_t)u8Data[0] & (int8_t)0xf0) / 16;
}

float ESENS_Calc_Heater_Res(uint16_t temp)
{
	float var1 = 0;
	float var2 = 0;
	float var3 = 0;
	float var4 = 0;
	float var5 = 0;
	float res_heat = 0;

	if (temp > 400) /* Cap temperature */
		temp = 400;

	var1 = (((float)g_bme680.calib.par_gh1 / (16.0f)) + 49.0f);
	var2 = ((((float)g_bme680.calib.par_gh2 / (32768.0f)) * (0.0005f)) + 0.00235f);
	var3 = ((float)g_bme680.calib.par_gh3 / (1024.0f));
	var4 = (var1 * (1.0f + (var2 * (float)temp)));
	var5 = (var4 + (var3 * (float)g_bme680.amb_temp));
	res_heat = (uint8_t)(3.4f * ((var5 * (4 / (4 + (float)g_bme680.calib.res_heat_range)) *
		(1/(1 + ((float)g_bme680.calib.res_heat_val * 0.002f)))) - 25));

	return res_heat;
}

uint8_t ESENS_Calc_Heater_Dur(uint16_t dur)
{
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		durval = (uint8_t) (dur + (factor * 64));
	}

	return durval;
}

void ESENS_Set_Gas_Config(void)
{
	uint8_t u8Data;
	
	u8Data = (uint8_t)ESENS_Calc_Heater_Res(g_bme680.gas_sett.heatr_temp);
	ESENS_Write(0x5a, u8Data);
	
	u8Data = ESENS_Calc_Heater_Dur(g_bme680.gas_sett.heatr_dur);
	ESENS_Write(0x64, u8Data);

	g_bme680.gas_sett.nb_conv = 0;
}

void ESENS_Set_Sensor_Mode(void)
{
	uint8_t u8Data = 0;
	
	ESENS_Read(0x74, &u8Data);
	
	u8Data &= ~(0x3);
	u8Data |= g_bme680.power_mode;
	
	ESENS_Write(0x74, u8Data);
}

void ESENS_Set_Oversampling(void)
{
	uint8_t u8Data = 0;
	
	// humidty oversampling
	ESENS_Read(0x72, &u8Data);
	u8Data &= ~(0x07);
	u8Data |= g_bme680.tph_sett.os_hum;
	ESENS_Write(0x72, u8Data);
	
	// temperature oversampling
	ESENS_Read(0x74, &u8Data);
	u8Data &= ~(0xE0);
	u8Data |= (g_bme680.tph_sett.os_temp << 5);
	ESENS_Write(0x74, u8Data);
	
	// pressure oversampling
	ESENS_Read(0x74, &u8Data);
	u8Data &= ~(0x1C);
	u8Data |= (g_bme680.tph_sett.os_pres << 2);
	ESENS_Write(0x74, u8Data);
}

void ESENS_Select_Filter(void)
{
	uint8_t u8Data = 0;
	
	ESENS_Read(0x75, &u8Data);
	u8Data &= ~(0x1C);
	u8Data |= g_bme680.tph_sett.filter << 2;
	ESENS_Write(0x75, u8Data);
}

void ESENS_Select_RunGas(void)
{
	uint8_t u8Data = 0;
	
	ESENS_Read(0x71, &u8Data);
	u8Data &= ~(0x10);
	u8Data |= g_bme680.gas_sett.run_gas << 4;
	ESENS_Write(0x71, u8Data);
}

void ESENS_Select_NBConv(void)
{
	uint8_t u8Data = 0;
	
	ESENS_Read(0x71, &u8Data);
	u8Data &= ~(0x0F);
	u8Data |= g_bme680.gas_sett.nb_conv;
	ESENS_Write(0x71, u8Data);
}

int16_t ESENS_Calc_Temperature(uint32_t temp_adc)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int16_t calc_temp;

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t)g_bme680.calib.par_t1 << 1);
	var2 = (var1 * (int32_t)g_bme680.calib.par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t)g_bme680.calib.par_t3 << 4)) >> 14;
	g_bme680.calib.t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((g_bme680.calib.t_fine * 5) + 128) >> 8);

	return calc_temp;
}

/** BME680 pressure calculation macros */
/*! This max value is used to provide precedence to multiplication or division
 * in pressure compensation equation to achieve least loss of precision and
 * avoiding overflows.
 * i.e Comparing value, BME680_MAX_OVERFLOW_VAL = INT32_C(1 << 30)
 */
#define BME680_MAX_OVERFLOW_VAL      (0x40000000)

uint32_t ESENS_Calc_Pressure(uint32_t pres_adc)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t pressure_comp;

	var1 = (((int32_t)g_bme680.calib.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)g_bme680.calib.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)g_bme680.calib.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)g_bme680.calib.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		((int32_t)g_bme680.calib.par_p3 << 5)) >> 3) +
		(((int32_t)g_bme680.calib.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)g_bme680.calib.par_p1) >> 15;
	pressure_comp = 1048576 - pres_adc;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
	if (pressure_comp >= BME680_MAX_OVERFLOW_VAL)
		pressure_comp = ((pressure_comp / var1) << 1);
	else
		pressure_comp = ((pressure_comp << 1) / var1);
	var1 = ((int32_t)g_bme680.calib.par_p9 * (int32_t)(((pressure_comp >> 3) *
		(pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) *
		(int32_t)g_bme680.calib.par_p8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
		(int32_t)(pressure_comp >> 8) *
		(int32_t)g_bme680.calib.par_p10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
		((int32_t)g_bme680.calib.par_p7 << 7)) >> 4);

	return (uint32_t)pressure_comp;
}

uint32_t ESENS_Calc_Humidity(uint16_t hum_adc)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	temp_scaled = (((int32_t)g_bme680.calib.t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t)g_bme680.calib.par_h1 * 16)))
		- (((temp_scaled * (int32_t)g_bme680.calib.par_h3) / ((int32_t) 100)) >> 1);
	var2 = ((int32_t)g_bme680.calib.par_h2
		* (((temp_scaled * (int32_t)g_bme680.calib.par_h4) / ((int32_t) 100))
			+ (((temp_scaled * ((temp_scaled * (int32_t)g_bme680.calib.par_h5) / ((int32_t) 100))) >> 6)
				/ ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t)g_bme680.calib.par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t)g_bme680.calib.par_h7) / ((int32_t) 100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 100000) /* Cap at 100%rH */
		calc_hum = 100000;
	else if (calc_hum < 0)
		calc_hum = 0;

	return (uint32_t) calc_hum;
}

uint32_t ESENS_Calc_Gas_Resistance(uint16_t gas_res_adc, uint8_t gas_range)
{
	int64_t var1;
	uint64_t var2;
	int64_t var3;
	uint32_t calc_gas_res;
	/**Look up table 1 for the possible gas range values */
	uint32_t lookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
		UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };
	/**Look up table 2 for the possible gas range values */
	uint32_t lookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
		UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016),
		UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000),
		UINT32_C(250000), UINT32_C(125000) };

	var1 = (int64_t) ((1340 + (5 * (int64_t)g_bme680.calib.range_sw_err)) *
		((int64_t) lookupTable1[gas_range])) >> 16;
	var2 = (((int64_t) ((int64_t) gas_res_adc << 15) - (int64_t) (16777216)) + var1);
	var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) >> 9);
	calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

	return calc_gas_res;
}

void ESENS_Read_Field_Data(void)
{
	uint8_t u8Data[3] = { 0, };
	uint8_t gas_range;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res;
	
	while (!(u8Data[0] & 0x80))
	{
		ESENS_Read(0x1D, &u8Data[0]);
	}
	g_bme680_data.status = u8Data[0] & 0x80;
	g_bme680_data.gas_index = u8Data[0] & 0x0F;
	
	ESENS_Read(0x1E, &u8Data[0]);
	g_bme680_data.meas_index = u8Data[0];
	
	/* read the raw data from the sensor */
	ESENS_Read(0x1F, &u8Data[2]);
	ESENS_Read(0x20, &u8Data[1]);
	ESENS_Read(0x21, &u8Data[0]);
	adc_pres = (uint32_t)(((uint32_t)u8Data[2] * 4096) | ((uint32_t)u8Data[1] * 16) | ((uint32_t)u8Data[0] / 16));
	
	ESENS_Read(0x22, &u8Data[2]);
	ESENS_Read(0x23, &u8Data[1]);
	ESENS_Read(0x24, &u8Data[0]);
	adc_temp = (uint32_t)(((uint32_t)u8Data[2] * 4096) | ((uint32_t)u8Data[1] * 16) | ((uint32_t)u8Data[0] / 16));

	ESENS_Read(0x25, &u8Data[1]);
	ESENS_Read(0x26, &u8Data[0]);
	adc_hum = (uint16_t)(((uint32_t)u8Data[1] * 256) | (uint32_t)u8Data[0]);
	
	ESENS_Read(0x2A, &u8Data[1]);
	ESENS_Read(0x2B, &u8Data[0]);
	adc_gas_res = (uint16_t)((uint32_t)u8Data[1] * 4 | (((uint32_t)u8Data[0]) / 64));
	
	gas_range = u8Data[0] & 0x0F;

	g_bme680_data.status |= u8Data[0] & 0x20;
	g_bme680_data.status |= u8Data[0] & 0x10;
	
	if (g_bme680_data.status & 0x80)
	{
		g_bme680_data.temperature = ESENS_Calc_Temperature(adc_temp);
		g_bme680_data.pressure = ESENS_Calc_Pressure(adc_pres);
		g_bme680_data.humidity = ESENS_Calc_Humidity(adc_hum);
		g_bme680_data.gas_resistance = ESENS_Calc_Gas_Resistance(adc_gas_res, gas_range);
	}
}

int16_t ESENS_Get_Humidity_Score(void)
{
	int16_t humidity_score;
	float hum_reference = 40.0f;
	
	//Calculate humidity contribution to IAQ index
  float current_humidity = g_bme680_data.humidity / 1000.0f;
  if (current_humidity >= 38 && current_humidity <= 42) // Humidity +/-5% around optimum
    humidity_score = 0.25 * 100;
  else
  { // Humidity is sub-optimal
    if (current_humidity < 38)
      humidity_score = 0.25 / hum_reference * current_humidity * 100;
    else
    {
      humidity_score = ((-0.25 / (100 - hum_reference) * current_humidity) + 0.416666) * 100;
    }
  }
  return humidity_score;
}

int16_t ESENS_Get_Gas_Score(void)
{
	int16_t gas_score;
	float gas_reference = g_bme680_data.gas_resistance;
	float gas_lower_limit = 5000;   // Bad air quality limit
  float gas_upper_limit = 50000;  // Good air quality limit 
  if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit; 
  if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
  gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;
	return gas_score;
}

uint8_t* ESENS_CalculateIAQ(int score)
{
  score = (100 - score) * 5;
  if      (score >= 301) return (uint8_t *)"Hazardous";
  else if (score >= 201 && score <= 300 ) return (uint8_t *)"Very Unhealthy";
  else if (score >= 176 && score <= 200 ) return (uint8_t *)"Unhealthy";
  else if (score >= 151 && score <= 175 ) return (uint8_t *)"Unhealthy for Sensitive Groups";
  else if (score >=  51 && score <= 150 ) return (uint8_t *)"Moderate";
  else if (score >=  00 && score <=  50 ) return (uint8_t *)"Good";
	
	return NULL;
}

void ESENS_Measure_Init(void)
{
	/* amb_temp can be set to 25 prior to configuring the gas sensor 
		or by performing a few temperature readings without operating the gas sensor. */
	g_bme680.amb_temp = 25;
	
	g_bme680.tph_sett.os_hum = 2;
	g_bme680.tph_sett.os_temp = 4;
	g_bme680.tph_sett.os_pres = 3;
	ESENS_Set_Oversampling();
	
	g_bme680.tph_sett.filter = 2;
	ESENS_Select_Filter();
}

void ESENS_Measure(void)
{
	int16_t humidity_score, gas_score;
	
	g_bme680.gas_sett.run_gas = 1;
	ESENS_Select_RunGas();
	
	g_bme680.gas_sett.heatr_temp = 350;
	g_bme680.gas_sett.heatr_dur = 150;
	ESENS_Set_Gas_Config();
	
	g_bme680.power_mode = 1;
	ESENS_Set_Sensor_Mode();
	
	ESENS_Read_Field_Data();
	
	//printf("Temperature = %.2f C\n", g_bme680_data.temperature / 100.0f);
	g_bme680.amb_temp = (int8_t)(g_bme680_data.temperature / 100.0f); // update amb temp for gas measure
  //printf("Pressure = %.2f hPa\n", g_bme680_data.pressure / 100.0f);
  //printf("Humidity = %.2f %%\n", g_bme680_data.humidity / 1000.0f);
	/* Avoid using measurements from an unstable heating setup */
  //if(g_bme680_data.status & 0x20)
	//	printf("Gas = %d ohms\n", g_bme680_data.gas_resistance);
	
	humidity_score = ESENS_Get_Humidity_Score();
  gas_score      = ESENS_Get_Gas_Score();
	
	// Combine results for the final IAQ index value (0-100% where 100% is good quality air)
  g_bme680_data.air_quality_score = humidity_score + gas_score;
	//printf("Score = %d\n", g_bme680_data.air_quality_score);
	//printf("Score = %s\n", ESENS_CalculateIAQ(g_bme680_data.air_quality_score));
}

int16_t ESENS_Get_Temperature(void)
{
	return g_bme680_data.temperature;
}

uint32_t ESENS_Get_Pressure(void)
{
	return g_bme680_data.pressure;
}

uint32_t ESENS_Get_Humidity(void)
{
	return g_bme680_data.humidity;
}

uint32_t ESENS_Get_Gas_Resistance(void)
{
	return g_bme680_data.gas_resistance;
}

int16_t ESENS_Get_Air_Quality_Score(void)
{
	return g_bme680_data.air_quality_score;
}
