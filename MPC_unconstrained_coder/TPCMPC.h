#pragma once 

#define nNumPre 250
#define nStateNum 3

extern double dTPCMPCConval[2];

void fnvTPCMPCCalConval(double dVeZmpRefx_250x1[nNumPre][1], double dVeZmpRefy_250x1[nNumPre][1], double dVeStatex_3x1[nStateNum][1], double dVeStatey_3x1[nStateNum][1]);
