#pragma once 

#include <math.h>
#include "dcc_Mat.h"

typedef struct {
	double p;
	double dp;
	double ddp;
}IntegValLimit;

double fndLimit(double dInputData, double dLimit[2]);
void fnvInitInteg(IntegValLimit *strInputData);
IntegValLimit fnstrIntegLimit(IntegValLimit strOldData, double dAcc, double dLimits[6], double dControlT);
void fnvIntegLimit(double *dPosIn, double *dVelIn, double dAcc, double dLimits[6], double dControlT);
void fnvVeloLimit(double *dPosIn, double dVel, double dLimits[4], double dControlT);
double fndFilterTimeLag(double filtered_in, double data_in, double control_t, double Lag_T);
