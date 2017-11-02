/*
 * BC_pass_old.h
 *
 *  Created on: 2017/10/21
 *      Author: Alex
 */

#ifndef BC_PASS_OLD_H_
#define BC_PASS_OLD_H_
void Add_UnknownWall_Front(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count);
char is_Exist_Unknown_Wall(char hikisuu_x, char hikisuu_y, char hikisuu_direction_count);
void Reset_Unknown_Wall();
void Set_Temp_Goal();
void Reset_Temp_Goal();
void make_pass(int hikisuu_goal_x, int hikisuu_goal_y);
void convert_pass();
void exe_kamawari_pass_test(float hikisuu_vmax, float hikisuu_accel, char para_mode);
void temp_exe_pass_EX(float hikisuu_vmax, float hikisuu_accel, char hikisuu_mode);
void temp_exe_pass_EX(float hikisuu_vmax, float hikisuu_accel, char hikisuu_mode);

#endif /* BC_PASS_OLD_H_ */
