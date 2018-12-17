#ifndef COMMANDS_H_
#define COMMANDS_H_


void commandLED(char *args);

void commandSM_Move(char *args);
void commandSM_Turn(char *args);

void commandPID_Kp(char *args);
void commandPID_Ki(char *args);
void commandPID_Kd(char *args);
void commandPID_AntiWindup(char *args);

void commandPID2_Kp(char *args);
void commandPID2_Ki(char *args);
void commandPID2_Kd(char *args);
void commandPID2_AntiWindup(char *args);

void commandOffset_Ang(char *args);
void commandCF_A(char *args);
void commandSampleRate(char *args);
void commandDeviceOn(char *args);


#endif
