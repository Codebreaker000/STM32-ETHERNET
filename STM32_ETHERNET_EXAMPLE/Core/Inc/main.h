/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	c5 = 0x01,
	ls17 = 0x02,
	ls2 = 0x03,
	ls16 = 0x04,
	ls21 = 0x05,
	ls22 = 0x06,
	lp23 = 0x07,
	hs1a = 0x08,
	es15 = 0x09,
	sqb1 = 0x0A,
	sqb2 = 0x0B,
	lp26 = 0x0C,
	sqb3 = 0x0D,
	ws1 = 0x0E
}connector_type;

typedef enum {
	a = 0x01,
	b = 0x02,
	c = 0x03,
	d = 0x04,
	e = 0x05,
	f = 0x06,
	g = 0x07,
	h = 0x08,
	j = 0x09,
	k = 0x0A,
	s = 0x0B,
	t = 0x0C,
	w = 0x0D,
	x = 0x0E
}pin_num;

typedef enum power_check_status{
	IDLE = 0,
	UNDER_VOLTAGE = 1,
	NORMAL = 2,
	OVER_VOLTAGE = 3
}POWER_CHECK_STATUS;

typedef enum power_supply_identifier {
	T_PS = 0x05,
	AUX_PS = 0x10,
	HMG_PS = 0x15,
	PROP_BAT1_PS = 0x20,
	PROP_BAT2_PS = 0x25,
	C_PS = 0x30,
	DS_PS = 0x35
}power_supply_identifier;

typedef enum {
	PS_OFF = 0x01,
	PS_ON = 0x02,
	DS_ON = 0x03,
	DS_OFF = 0x04
}power_supply_cmd;

typedef enum{
	POWER_SUPPLY_ERROR = 0x01,
	POWER_SUPPLY_ON = 0x02,
	POWER_SUPPLY_OFF = 0x03
}POWER_SUPPLY_STATUS;

typedef enum{
	MDAC_MODE = 0x40,
	IIRS_MODE = 0x41,
	HMG_MODE = 0x42
}MS_UNIT_IDENTIFIER;

typedef enum{
	MDAC_REAL = 0x01,
	MDAC_MONITOR = 0x02,
	IIRS_APP = 0x04,
	IIRS_MONITOR = 0x08,
	IIRS_RECORD = 0x0C,
	IIRS_LOAD = 0x10,
	IIRS_ERASE = 0x14,
	HMG_APP = 0x20,
	HMG_MONITOR = 0x40
}MS_WORD;


typedef union {
   float f;
   uint32_t h;
} hexfloat;



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
POWER_CHECK_STATUS power_check (connector_type connector,pin_num pin_num);

POWER_SUPPLY_STATUS power_supply_operation (power_supply_identifier power_supply_type,power_supply_cmd power_supply_command);
void ps_relay_off(power_supply_identifier power_supply_type);
void ps_relay_on(power_supply_identifier power_supply_type);
uint8_t ps_feedback(power_supply_identifier power_supply_type);

uint8_t mode_selection_operation(MS_UNIT_IDENTIFIER ms_unit, MS_WORD mode_word);
void ms_relay_off(MS_UNIT_IDENTIFIER ms_unit, uint8_t line);
void ms_relay_on(MS_UNIT_IDENTIFIER ms_unit, uint8_t line);

void ds_sim(uint16_t cmd_meter, uint8_t channel);

/* ADC Functions Addition */


void SADC_REG_CONFIG(uint8_t addr, uint8_t data, uint8_t wr_nrd);
void SADC_CMD_CONFIG(uint16_t cmd, uint8_t* readback);
void SADC_COMM(uint8_t* readbuf);
void SADC_CONVERT(uint8_t* readbuf, hexfloat* outval);

void mdac_slave(uint8_t* mdac_data );
void iirs_slave(uint8_t* iirs_data);
void hmg_slave(uint8_t* hmg_data);


void delaymicroseconds(uint32_t us);
void delaymilliseconds(uint32_t ms);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */


#define DATA_MODE_HEX (uint8_t)0x00
#define DATA_MODE_TEXT (uint8_t)0x01







#define V_15P_UPPER (uint32_t)0x4179999A
#define V_15P_LOWER (uint32_t)0x41666666
#define V_15N_UPPER (uint32_t)0xC179999A
#define V_15N_LOWER (uint32_t)0xC1666666
#define V_5P_UPPER  (uint32_t)0x40A66666
#define V_5P_LOWER  (uint32_t)0x4099999A


/* Feedback Messages of TM*/
#define ITP_TM_HEADER (uint16_t)0xFAFA
#define ITP_TM_FOOTER (uint16_t)0xAFAF
#define ITP_TM_PAYLOAD(uint16_t)0x001D
/* Feedback Messages of CCU*/
#define ITP_CCU_HEADER (uint16_t)0xAB0B
#define ITP_CCU_FOOTER (uint16_t)0xBA0A
#define ITP_CCU_PAYLOAD (uint16_t)0x0018
/* Feedback Messages of HCP*/
#define ITP_HCP_HEADER (uint16_t)0xEB0B
#define ITP_HCP_FOOTER (uint16_t)0xBE0E
/* PPS INCOMING Message*/
#define ITP_PPS_HEADER (uint16_t)0x5A5A
#define ITP_PPS_FOOTER (uint16_t)0xA9A9

#define slave_addr 0x80
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
