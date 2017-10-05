/*
 * workshop.h
 *
 *  Created on: 2016/02/19
 *      Author: Alex
 */

#ifndef WORKSHOP_H_
#define WORKSHOP_H_

void daikei(float _dist, float _Vmax, float _Vmin, float _acc,int _wall);
void cho_sinchi(float _ang);
extern volatile float accel,speed,total_dist;
extern volatile int wall_control;
#endif /* WORKSHOP_H_ */
