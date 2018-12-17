#include "IS_complementaryFilter.h"
#include <math.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IS_CompFilter_initialize(IS_CompFilter_struct* DataStruct)
{
  	DataStruct->rollAcc=0;
		DataStruct->rollGyro=0;
		DataStruct->roll=0;
		DataStruct->A=0.001;
		DataStruct->OffsetAngleDynamic = 0;
		DataStruct->OffsetAngleStatic = -90.00f + 2.55f;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IS_CompFilter_calculate(IS_CompFilter_struct* DataStruct, float acc1, float acc2, float gyro, float dt)
{
	DataStruct->rollGyro = DataStruct->roll + gyro*dt; 				
  DataStruct->rollAcc = atan2f(acc1,acc2) * 180 / 3.1415f + DataStruct->OffsetAngleStatic + DataStruct->OffsetAngleDynamic;			
  DataStruct->roll = DataStruct->rollGyro * (1-DataStruct->A) + DataStruct->rollAcc * DataStruct->A ; 
}