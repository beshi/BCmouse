/*
 * daikei.h
 *
 *  Created on: 2017/10/09
 *      Author: Alex
 */

#ifndef DAIKEI_H_
#define DAIKEI_H_

void daikei_for_pass_kai(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kabekire);
void reverse_daikei(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall);

#endif /* DAIKEI_H_ */
