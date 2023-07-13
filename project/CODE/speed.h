#ifndef __SPEED_H
#define __SPEED_H

#include "headfile.h"
#include "math.h"
#include "data.h"
#include "pid.h"
#include "element.h"
#include "other.h"
#include "ringjudge.h"


void setMotorL(int16 motorSetL);
void setMotorR(int16 motorSetL);
void renew(void);
void diffSpeedSet1(void);

#endif