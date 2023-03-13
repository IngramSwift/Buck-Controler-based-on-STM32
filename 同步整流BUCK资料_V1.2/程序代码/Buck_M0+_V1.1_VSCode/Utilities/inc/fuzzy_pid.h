#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#include "stm32f0xx.h"

void FuzzySubjection(float f_e, float f_ec);
float FuzzyPID_KpCalc(float f_e, float f_ec);
float FuzzyPID_KiCalc(float f_e, float f_ec);
void FuzzyPID_Calc(float f_e, float f_ec, float* kp, float* ki);

#endif















