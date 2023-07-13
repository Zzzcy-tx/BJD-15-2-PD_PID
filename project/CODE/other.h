#ifndef __OTHER_H
#define __OTHER_H
#include "headfile.h"
#include "data.h"
#include "stdlib.h"
#include "ringjudge.h"
#include "filter.h"
void allInit(void);

void any_change(int16 id, char cp[]);
void rx_analysis(void);
float angle_change(float ang);
int8 get_key(uint8 mode);
void my_iap_init();
#endif