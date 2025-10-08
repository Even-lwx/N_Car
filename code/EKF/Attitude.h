/*
 * Attitude.h
 *
 *  Created on: 2024��3��9��
 *      Author: huawei
 */

#ifndef CODE_ATTITUDE_H_
#define CODE_ATTITUDE_H_
#include "zf_common_headfile.h"
void Attitude_Init(void); //attitude init
void Attitude_Calculate(void);  //attitude calculate

extern float gyroscope[3];
extern float accelerometer[3];
extern float gyroscopeOffset[3];
#endif /* CODE_ATTITUDE_H_ */
