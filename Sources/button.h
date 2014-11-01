/*
 * button.h
 *
 *  Created on: Sep 4, 2014
 *      Author: B51761
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include<stdint.h>
#include"MKL16Z4.h"
void button_init();
void button_deinit();
uint8_t button_get_value();
#endif /* BUTTON_H_ */
