#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_


int64_t LowPassFilter(int32_t a[], bool ready, uint8_t idx, uint16_t numberOfSamples);

void SM_Move(int32_t speed, int8_t turn, bool enable);

uint8_t sign(float x);

float FIRfilter (float value);
float FIRfilter2 (float value);
float FIRfilter3 (float value);










#endif
