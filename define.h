/*
 * define.h
 *
 *  Created on: 14.06.2016
 *      Author: Shcheblykin
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#include <stdio.h>

/// ������.
#define TRUE 1
/// ����.
#define FALSE 0

extern volatile uint8_t debug;
extern void setTP1(uint8_t value);
extern void switchTP1();

#endif /* DEFINE_H_ */
