/*
 * vofa.h
 *
 *  Created on: Nov 28, 2024
 *      Author: WML
 */

#ifndef INC_VOFA_H_
#define INC_VOFA_H_

 /*----------example-------------
 //init
     Upper_Computer_Init(&pitch_filter);
     Upper_Computer_Init(&car_V);
     Upper_Computer_Init(&rtU.L_ia);
     Upper_Computer_Init(&rtU.L_ib);
     Upper_Computer_Init(&rtU.L_ic);
     Upper_Computer_Init(&Roll_delta);
 //loop transmit
     Upper_Computer_Show_Wave();
 -----------------end-------------*/
extern void Upper_Computer_Show_Wave(void);
extern void Upper_Computer_Init(float*);//maximum 10 channels;can modify

#endif /* INC_VOFA_H_ */

