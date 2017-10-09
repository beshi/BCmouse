/*
 * BC_subroutine.h
 *
 *  Created on: 2017/05/05
 *      Author: Alex
 */

#ifndef BC_SUBROUTINE_H_
#define BC_SUBROUTINE_H_

void battery_ADconvert(void);
void sensor_ADconvert(void);
void motor_direction(char direction_right, char direction_left);
void wait(int cnt);
void interrupt_GPT0_GTCCRA(void);
void inrerrupt_GPT0_GTCCRB(void);
int mode_select(void);
int number_select(void);
void watched_wall_front(int hikisuu_x, int hikisuu_y,
		int hikisuu_direction_count);
void watched_wall_right(int hikisuu_x, int hikisuu_y,
		int hikisuu_direction_count);
void watched_wall_left(int hikisuu_x, int hikisuu_y,
		int hikisuu_direction_count);
char is_Exist_Wall(char hikisuu_x, char hikisuu_y, char hikisuu_direction_count);
char is_saved_wall_exist(char hikisuu_x, char hikisuu_y,
		char hikisuu_direction_count);
void test_hidaritehou_slalom(int hikisuu_goal_x, int hikisuu_goal_y);
void add_wall_front(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count);
void add_wall_right(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count);
void add_wall_left(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count);
void add_saved_wall_front(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count);
void add_saved_wall_right(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count);
void add_saved_wall_left(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count);
char is_the_Wall_watched(char hikisuu_x, char hikisuu_y,
		char hikisuu_direction_count);
char is_saved_wall_watched(char hikisuu_x, char hikisuu_y,
		char hikisuu_direction_count);
void slalom_for_tuning(float hikisuu_angle, float angle1, float angle2, float omega_max,
		float hikisuu_angacc, float unclock_wise,
		float hikisuu_balance_velocity, float dist1, float dist2);
void temp_test_mazedata();

#endif /* BC_SUBROUTINE_H_ */
