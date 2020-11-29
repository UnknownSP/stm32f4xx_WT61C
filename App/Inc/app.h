#ifndef __APP_H
#define __APP_H

/*NO Device mode*/
#define _NO_DEVICE 1

int appTask(void);
int appInit(void);

#define DD_NUM_OF_MD 0
#define DD_NUM_OF_AB 0

#define DD_NUM_OF_LD 0
#define DD_NUM_OF_SS 0
#define DD_USE_ENCODER1 0
#define DD_USE_ENCODER2 0
#define DD_USE_ENCODER3 0
#define DD_USE_ENCODER4 0
#define DD_NUM_OF_SV 0

#define USE_RASPI_CONTROL 0
#define RASPI_SWITCH_NUM 64
#define DD_USE_RC 0

#include "DD_RC.h"
#include "DD_LD.h"
#include "DD_MD.h"
#include "DD_SV.h"
#include "DD_SS.h"

#define I2C_ODMETRY 0

#define CENTRAL_THRESHOLD 4

#define MD_GAIN ( DD_MD_MAX_DUTY / DD_RC_ANALOG_MAX )

#endif