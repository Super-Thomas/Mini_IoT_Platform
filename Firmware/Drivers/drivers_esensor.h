#ifndef __DRIVERS_ESENSOR_H_
#define __DRIVERS_ESENSOR_H_

void ESENS_Init(void);
uint8_t ESENS_Write(uint8_t u8RegAddr, uint8_t u8Data);
uint8_t ESENS_Read(uint8_t u8RegAddr, uint8_t* pu8Data);
uint8_t ESENS_ReadChipID(void);
void ESENS_Get_Calib_Data(void);
float ESENS_Calc_Heater_Res(uint16_t temp);
uint8_t ESENS_Calc_Heater_Dur(uint16_t dur);
void ESENS_Set_Gas_Config(void);
void ESENS_Set_Sensor_Mode(void);
void ESENS_Set_Oversampling(void);
void ESENS_Select_Filter(void);
void ESENS_Select_RunGas(void);
void ESENS_Select_NBConv(void);
int16_t ESENS_Calc_Temperature(uint32_t temp_adc);
uint32_t ESENS_Calc_Pressure(uint32_t pres_adc);
uint32_t ESENS_Calc_Humidity(uint16_t hum_adc);
uint32_t ESENS_Calc_Gas_Resistance(uint16_t gas_res_adc, uint8_t gas_range);
void ESENS_Read_Field_Data(void);
void ESENS_Measure_Init(void);
void ESENS_Measure(void);
int16_t ESENS_Get_Temperature(void);
uint32_t ESENS_Get_Pressure(void);
uint32_t ESENS_Get_Humidity(void);
uint32_t ESENS_Get_Gas_Resistance(void);
int16_t ESENS_Get_Air_Quality_Score(void);

#endif //__DRIVERS_ESENSOR_H_
