/*
 * adachihou.h
 *
 *  Created on: 2016/02/20
 *      Author: Alex
 */

#ifndef ADACHIHOU_H_
#define ADACHIHOU_H_

void adachihou_q2_kichikukan(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search, float hikisuu_angacc,
		float hikisuu_angle1_r, float hikisuu_angle2_r, float hikisuu_angle1_l,
		float hikisuu_angle2_l, float offset_r1, float offset_r2,
		float offset_l1, float offset_l2);
void adachihou_q2(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search);
void adachihou(int hikisuu_goal_x, int hikisuu_goal_y);

#endif /* ADACHIHOU_H_ */
