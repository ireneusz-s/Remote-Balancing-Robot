#ifndef IS_COMPLEMENTARYFILTER_H_
#define IS_COMPLEMENTARYFILTER_H_


typedef struct {
	/* Private */
		 float rollAcc;
		 float rollGyro;
		 float roll;
		 float A;
		 float OffsetAngleDynamic;
		 float OffsetAngleStatic;
} IS_CompFilter_struct;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IS_CompFilter_initialize(IS_CompFilter_struct* DataStruct);
void IS_CompFilter_calculate(IS_CompFilter_struct* DataStruct, float acc1, float acc2, float gyro, float dt);





#endif
