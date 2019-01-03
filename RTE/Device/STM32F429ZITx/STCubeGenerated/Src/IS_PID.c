#include "IS_PID.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IS_PID_initialize(IS_PID_struct* DataStruct)
{
			DataStruct->PID_KP=100;
			DataStruct->PID_KI=0;
			DataStruct->PID_KD=0;
			DataStruct->PID_AntiWindup=100;
			DataStruct->PID_outLimit=100;
	
			DataStruct->PID_output=0;
	
			DataStruct->PID_error = 0;
			DataStruct->PID_error_prior = 0;
			DataStruct->PID_integral = 0;
			DataStruct->PID_derivative=0;
			DataStruct->PID_itegralClamp=1;
			DataStruct->PID_saturation=false;
			DataStruct->PID_signCompare=false;

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float IS_PID_calculate(IS_PID_struct* DataStruct, float input, float dt)
{
	
			DataStruct->PID_error = (input); 
	
			DataStruct->PID_integral = DataStruct->PID_integral + (DataStruct->PID_itegralClamp*DataStruct->PID_error*dt);	
			if(DataStruct->PID_integral> DataStruct->PID_AntiWindup) DataStruct->PID_integral= DataStruct->PID_AntiWindup;
			else if(DataStruct->PID_integral< -DataStruct->PID_AntiWindup) DataStruct->PID_integral= -DataStruct->PID_AntiWindup;
		
			DataStruct->PID_derivative = (DataStruct->PID_error-DataStruct->PID_error_prior)/dt;
		
			DataStruct->PID_output = DataStruct->PID_KP*DataStruct->PID_error + DataStruct->PID_KI*DataStruct->PID_integral + DataStruct->PID_KD*DataStruct->PID_derivative;
			DataStruct->PID_error_prior = DataStruct->PID_error;

			//check saturation and limit regulator output
			if(DataStruct->PID_output> DataStruct->PID_outLimit)
			{
				DataStruct->PID_output= DataStruct->PID_outLimit;      
				DataStruct->PID_saturation=true;
			}
      else if(DataStruct->PID_output< -DataStruct->PID_outLimit) 
			{
				DataStruct->PID_output= -DataStruct->PID_outLimit;
				DataStruct->PID_saturation=true;
			}
			else
			{
				DataStruct->PID_saturation=false;
			}
			
		  //compare signs of error and output
			if (DataStruct->PID_error*DataStruct->PID_output>=0)
			{
				DataStruct->PID_signCompare=true;
			}
			else
			{
			 DataStruct->PID_signCompare=false;
			}
			 
	  	//compare saturation and sign of error and output to do camp or no clamp
			if (DataStruct->PID_signCompare && DataStruct->PID_saturation)
			{
			 DataStruct->PID_itegralClamp=0;
			}
		 	else
			{
			 DataStruct->PID_itegralClamp=1;
			}
	
	return DataStruct->PID_output;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
