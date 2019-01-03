#ifndef IS_PID_H_
#define IS_PID_H_

#include "stdbool.h"
#include "stdint.h"




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
	/* Private */
			float PID_KP;
			float PID_KI;
			float PID_KD;
			uint16_t PID_AntiWindup;
			float PID_outLimit;
	
			float PID_output;
	
			float PID_error;
			float PID_error_prior;
			float PID_integral;
			float PID_derivative;
			uint8_t PID_itegralClamp;
			bool PID_saturation;
			bool PID_signCompare;
			
} IS_PID_struct;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IS_PID_initialize(IS_PID_struct* DataStruct);
float IS_PID_calculate(IS_PID_struct* DataStruct, float input, float dt);





#endif
