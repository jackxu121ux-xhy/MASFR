#include "robotMotion.h"

void robotContinuousForward(int8_t firstSide, float division, uint8_t* stopFlag){  //firstSide = 1 for left, firstSide = -1 for right, division < 1
	
	while(*stopFlag == 0){
		//STAGE 1
		if(firstSide == -1){
			HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
		}else if(firstSide == 1){
			HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
		}
		
		delay_ms(500);
		
		int16_t control1 = 2048. - 1024. * division * firstSide;
		WritePosEx(1, control1, 1200, 50);
		WritePosEx(2, control1, 1200, 50);
		WritePosEx(3, control1, 1200, 50);
		WritePosEx(4, control1, 1200, 50);
		
		delay_ms((uint16_t)(2000. * division));
		
		//STAGE 2
		if(firstSide == -1){
			HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
		}else if(firstSide == 1){
			HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
		}
		
		delay_ms(500);
		
		int16_t control2 = 2048. + 1024. * division * firstSide;
		WritePosEx(1, control2, 1200, 50);
		WritePosEx(2, control2, 1200, 50);
		WritePosEx(3, control2, 1200, 50);
		WritePosEx(4, control2, 1200, 50);
		
		delay_ms((uint16_t)(2000. * division));
	}
	
	//STAGE 3
	if(firstSide == -1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	}else if(firstSide == 1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	}
	
	delay_ms(500);
	
	WritePosEx(1, 2048, 1200, 50);
	WritePosEx(2, 2048, 1200, 50);
	WritePosEx(3, 2048, 1200, 50);
	WritePosEx(4, 2048, 1200, 50);
	
	delay_ms((uint16_t)(1000. * division));
	
}

void robotOneStepForward(int8_t firstSide, float division){  //firstSide = 1 for left, firstSide = -1 for right, division < 1
	
	//STAGE 1
	if(firstSide == -1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	}else if(firstSide == 1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	}
	
	delay_ms(500);
	
	int16_t control1 = 2048. - 1024. * division * firstSide;
	WritePosEx(1, control1, 1200, 50);
	WritePosEx(2, control1, 1200, 50);
	WritePosEx(3, control1, 1200, 50);
	WritePosEx(4, control1, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
	
	//STAGE 2
	if(firstSide == -1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	}else if(firstSide == 1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	}
	
	delay_ms(500);
	
	int16_t control2 = 2048. + 1024. * division * firstSide;
	WritePosEx(1, control2, 1200, 50);
	WritePosEx(2, control2, 1200, 50);
	WritePosEx(3, control2, 1200, 50);
	WritePosEx(4, control2, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
	
	//STAGE 3
	if(firstSide == -1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	}else if(firstSide == 1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	}
	
	delay_ms(500);
	
	WritePosEx(1, 2048, 1200, 50);
	WritePosEx(2, 2048, 1200, 50);
	WritePosEx(3, 2048, 1200, 50);
	WritePosEx(4, 2048, 1200, 50);
	
	delay_ms((uint16_t)(1000. * division));
}

void robotOneStepBackward(int8_t firstSide, float division){  //firstSide = 1 for left, firstSide = -1 for right, division < 1
	
	//STAGE 1
	if(firstSide == 1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	}else if(firstSide == -1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	}
	
	delay_ms(500);
	
	int16_t control1 = 2048. + 1024. * division * firstSide;
	WritePosEx(1, control1, 1200, 50);
	WritePosEx(2, control1, 1200, 50);
	WritePosEx(3, control1, 1200, 50);
	WritePosEx(4, control1, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
	
	//STAGE 2
	if(firstSide == 1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	}else if(firstSide == -1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	}
	
	delay_ms(500);
	
	int16_t control2 = 2048. - 1024. * division * firstSide;
	WritePosEx(1, control2, 1200, 50);
	WritePosEx(2, control2, 1200, 50);
	WritePosEx(3, control2, 1200, 50);
	WritePosEx(4, control2, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
	
	//STAGE 3
	if(firstSide == 1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	}else if(firstSide == -1){
		HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	}
	
	delay_ms(500);
	
	WritePosEx(1, 2048, 1200, 50);
	WritePosEx(2, 2048, 1200, 50);
	WritePosEx(3, 2048, 1200, 50);
	WritePosEx(4, 2048, 1200, 50);
	
	delay_ms((uint16_t)(1000. * division));
}

void robotOneStepLeft(float division){  
	
	//STAGE 1
	HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	
	delay_ms(500);
	
	int16_t control1 = 2048. + 1024. * division;
	WritePosEx(1, control1, 1200, 50);
	WritePosEx(2, control1, 1200, 50);
	WritePosEx(3, control1, 1200, 50);
	WritePosEx(4, control1, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
	
	//STAGE 2

	HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	
	delay_ms(500);
	
	int16_t control2 = 2048. - 1024. * division;
	WritePosEx(1, control2, 1200, 50);
	WritePosEx(2, control2, 1200, 50);
	WritePosEx(3, control2, 1200, 50);
	WritePosEx(4, control2, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
}

void robotOneStepRight(float division){  
	
	//STAGE 1
	HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_SET);
	
	delay_ms(500);
	
	int16_t control1 = 2048. - 1024. * division;
	WritePosEx(1, control1, 1200, 50);
	WritePosEx(2, control1, 1200, 50);
	WritePosEx(3, control1, 1200, 50);
	WritePosEx(4, control1, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
	
	//STAGE 2

	HAL_GPIO_WritePin(GPIOB, Solenoid1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Solenoid2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Solenoid3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Solenoid4_Pin, GPIO_PIN_RESET);
	
	delay_ms(500);
	
	int16_t control2 = 2048. + 1024. * division;
	WritePosEx(1, control2, 1200, 50);
	WritePosEx(2, control2, 1200, 50);
	WritePosEx(3, control2, 1200, 50);
	WritePosEx(4, control2, 1200, 50);
	
	delay_ms((uint16_t)(2000. * division));
}