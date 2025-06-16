/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/

/* Private Function Prototypes *********************************************/

/* Main Function ***********************************************************/
int main(void){

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for(;;){
	}
}

/***************************************************************************/
/* User Task */
void UserTask(void *argument){


//	 ADCSelectPort(P2);
//	 ADCSelectPort(P3);

	/* put your code here, to run repeatedly. */
	while(1){

//	ReadADCChannel(P2,"top",&adcalue1);
//	ReadADCChannel(P2,"bottom",&adcalue2);
//	ReadADCChannel(P3,"top",&adcalue3);
//	ReadADCChannel(P3,"bottom",&adcalue4);

	}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
