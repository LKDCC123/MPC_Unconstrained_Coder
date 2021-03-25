#include "dcc_con_base.h"
#include "math.h"

double fndLimit(double dInputData, double dLimit[2]) {
	double dOutputData;
	
	dOutputData = fmax(dLimit[0], dInputData);
	dOutputData = fmin(dLimit[1], dOutputData);

	return dOutputData;
}

void fnvInitInteg(IntegValLimit *strInputData) {
	strInputData->p = 0.0;
	strInputData->dp = 0.0;
	strInputData->ddp = 0.0;
}


/**
Inputs: olddata[p, dp, ddp], Acc, Limits[PosMin, PosMax, VelMin, VelMax, AccMin, AccMax], CONTROL_T
Output: updateddata[p, dp, ddp]
*/
IntegValLimit fnstrIntegLimit(IntegValLimit strOldData, double dAcc, double dLimits[6],  double dControlT) {
	IntegValLimit strUpdatedData;
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}

	dAcc = fndLimit(dAcc, dAccelerateLimit);
	strUpdatedData.dp = strOldData.dp + dAcc * dControlT;
	strUpdatedData.dp = fndLimit(strUpdatedData.dp, dVelocityLimit);
	strUpdatedData.p = strOldData.p + strUpdatedData.dp * dControlT;
	strUpdatedData.p = fndLimit(strUpdatedData.p, dPositionLimit);
	strUpdatedData.dp = (strUpdatedData.p - strOldData.p) / dControlT;
	strUpdatedData.ddp = dAcc;

	return strUpdatedData;
}

void fnvIntegLimit(double *dPosIn, double *dVelIn, double dAcc, double dLimits[6], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	double dVelOld = *dVelIn;
	double dPosOld = *dPosIn;
	double dAccIn = dAcc;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}

	dAccIn = fndLimit(dAcc, dAccelerateLimit);
	*dVelIn = dVelOld + dAccIn * dControlT;
	*dVelIn = fndLimit(*dVelIn, dVelocityLimit);
	*dPosIn = dPosOld + *dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);
	*dVelIn = (*dPosIn - dPosOld) / dControlT;

}

void fnvVeloLimit(double *dPosIn, double dVel, double dLimits[4], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dPosOld = *dPosIn;
	double dVelIn = dVel;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
	}

	dVelIn = fndLimit(dVel, dVelocityLimit);
	*dPosIn = dPosOld + dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);

}

double fndFilterTimeLag(double filtered_in, double data_in, double control_t, double Lag_T)
{
	double filtered_out;

	filtered_out = (control_t * data_in + Lag_T * filtered_in) / (control_t + Lag_T);

	return filtered_out;
}

