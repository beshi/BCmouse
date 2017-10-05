/***************************************************************/
/*                                                             */
/*      PROJECT NAME :  BCmouse                                */

/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#include "iodefine.h"
#include "stdarg.h"
#include "serial.h"
#include "BC_define.h"
#include "BC_init.h"
#include "BC_subroutine.h"
#include "math.h"

int SPIRead(int add);
void SPIWrite(int add, int data);
void WALL_INFORMATION_save(void);
void WATCHED_WALL_INFORMATION_save(void);
void maze_display(void);
void saved_maze_display(void);
void walkmap_display(void);
void skew_walkmap_display(void);
float kabekire_dist_right(float hikisuu_vel);
float kabekire_dist_left(float hikisuu_vel);
void test_daikei(float hikisuu_dist, float vmax, float hikisuu_accel, float v_0,
		float vterm, char hikisuu_wall);
void daikei_for_pass(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall);
void daikei_for_pass_kai(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kabekire);
void daikei_for_pass_kai2(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kabekire);
void reverse_daikei(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall);
void test_turn(float hikisuu_angle, float omega_max, float hikisuu_angacc,
		float unclock_wise, float hikisuu_balance_velocity);
void test_slalom(float hikisuu_angle, float omega_max, float hikisuu_angacc,
		float unclock_wise, float hikisuu_balance_velocity, float dist1,
		float dist2);
void slalom_2(float hikisuu_angle, float angle1, float angle2, float omega_max,
		float hikisuu_angacc, float unclock_wise,
		float hikisuu_balance_velocity, float dist1, float dist2);
void direction_xy();
void adachihou_q(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search);
void adachihou2_q(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search);
void make_pass(int hikisuu_goal_x, int hikisuu_goal_y);
void convert_pass_skew(void);
void exe_pass_test(float hikisuu_vmax, float hikisuu_accel, char para_mode);
void exe_pass_kai(float hikisuu_vmax, float hikisuu_v, float hikisuu_accel,
		char hikisuu_mode);
void q_walk_map_maker(int hikisuu_goal_x, int hikisuu_goal_y);
void q_saved_walk_map_maker( hikisuu_goal_x, hikisuu_goal_y);
void skew_queue_walkmap_maker(char hikisuu_goal_x, char hikisuu_goal_y);
void sensor_average(char number1, char number2);
void interrpt_CMT0(void);
void task_exe(int first_number, int second_number, int therd_number);
void unknown_WALL_add();
void unknown_WALL_remove();
void test_wall_control();
/*volatile typedef struct { // 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する
 float right;
 float left;
 } float_rightleft_t;
 volatile typedef struct { // 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する
 int right;
 int left;
 } int_rightleft_t;
 volatile typedef struct { // 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する
 int High;
 int Low;
 int SUM;
 } float_highlow_t;*/

volatile unsigned int queue[700];
volatile unsigned char sample_flag = 0, refer_flag = 0, gyro_enable = 0,
		reference_fin = 0, wall_control = 0, ei_flag_center = 0,
		ei_flag_rot = 0, direction_count, x, y, sensor_enable = 0, q_dist_flag =
				0, kabekire_right = 0, kabekire_left = 0, kabeiri_right = 0,
		kabeiri_left = 0, kabekire_enable = 0, kabekire_enable_2 = 0,
		kushi_judge = 0, pass[200] = { 0 }, motion[100] = { 0 }, last_p_i,
		reverse_flag = 0, pass_kabekire_straight = 0, wait_kabekire = 0,
		kabekire_read_flag = 0, fail_flag = 0, sen_fail_flag = 0,
		fail_count = 0, ei_flag_wall = 0, goal_x, goal_y, temp_goal_x,
		temp_goal_y, daikei_mode = 0;
volatile int sen_bat, sen_l_f_ON, sen_l_s_ON, sen_r_f_ON, sen_r_s_ON,
		sen_l_f_OFF, sen_l_s_OFF, sen_r_f_OFF, sen_r_s_OFF, gptcount_l = 125,
		gptcount_r = 125, sample_count = 0, refer_count = 0,
		buff_sen_right[10] = { 0.0 }, buff_sen_left[10] = { 0.0 },
		sen_count = 0, jkl = 0;
volatile unsigned short map[16][16] = { 0 }, row_temp[15] = { 0 },
		column_temp[15] = { 0 }, row_fix[15] = { 0 }, column_fix[15] = { 0 },
		row_watched_fix[15] = { 0 }, column_watched_fix[15] = { 0 },
		row_watched_temp[15] = { 0 }, column_watched_temp[15] = { 0 },
		level[16][15] = { 0 }, vertical[15][16] = { 0 };
volatile long cmt_count = 0;
volatile float Battery, dutty_r = 0.0, dutty_l = 0.0, omega = 0.0, angle = 0.0,
		reference_omega = 0.0, diff_omega[3] = { 0.0 }, diff_kabe[3] = { 0.0 },
		ideal_omega = 0.0, ideal_omega2 = 0.0, ideal_angle, ideal_angacc = 0.0,
		balance_velocity, balance_distance = 0.0, ideal_balance_velocity = 0.0,
		ideal_balance_accel = 0.0, ideal_balance_distance = 0.0,
		diff_balance_velocity[3], Error, Kp_wall_r, Kp_wall_l, EI_keisuu = 1.0,
		EP_keisuu = 1.0, pre_buff_sen_right[10] = { 0.0 },
		pre_buff_sen_left[10] = { 0.0 }, pre_kabe_ave_right = 0.0,
		pre_kabe_ave_left = 0.0, pre_ave_ave_right = 0.0,
		pre_ave_ave_left = 0.0, d_i = 0.0, adjust_before_dist = 0.0,
		adjust_distance = 0.0;
volatile float_rightleft_t velocity, total_distance, accel, old_velocity = {
		0.0, 0.0 }, ideal_velocity, ideal_accel = { 0.0 }, ideal_distance,
		diff_velocity[3] = { 0.0, 0.0 }, dutty = { 0.0, 0.0 }, dutty_foward = {
				0.0, 0.0 }, average_sensor, diff_average_sensor,
		average_sensor_old;
volatile int_rightleft_t old_tcnt, tcnt = { 32767, 32767 }, diff_tcnt,
		motor_direc = { 0, 0 };
volatile float_highlow_t gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
volatile float_pid_t K_center, K_rot, K_wall, Erorr_center, Erorr_rot,
		Error_wall;
volatile int_sensor_t sen;
volatile unsigned_short_fix_t a = { 0 };

volatile const turn_velocities_t turn[7] = {//[0]:重心速度500  [1]:重心速度650  [2]:速度750  [3]:速度800 [4]:速度900  [5]:速度  [6]:速度
		{
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
		{ 90.0, 22.5, 67.5, 5500.0, -1.0, 13.0, 12.0, 500.0, 0, 1 },	//右小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//右大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//斜め→右V90°ターン
				{ 90.0, 22.5, 67.5, 5500.0, 1.0, 5.0, 22.0, 500.0, 0, 1 },//左小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//左大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		}, {
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
		{ 90.0, 30.0, 60.0, 7500.0, -1.0, 4.0, 8.0, 650.0, 0, 1 }, //右小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右V90°ターン
				{ 90.0, 33.0, 57.0, 8320.0, 1.0, 5.0, 25.0, 650.0, 0, 1 }, //左小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		}, {
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
		{ 90.0, 41.0, 49.0, 9240.0, -1.0, 0.0, 18.0, 750.0, 0, 0 }, //右小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右V90°ターン
				{ 90.0, 40.0, 50.0, 10050.0, 1.0, 3.8, 22.0, 750.0, 0, 0 }, //左小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		}, {
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
		{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右小回り
				{ 90.0, 40.0, 50.0, 4300.0, -1.0, 43.0, 47.0, 800.0, 0, 1 }, //右大回り
				{ 180.0, 27.5, 152.5, 4450.0, -1.0, 25.0, 32.0, 800.0, 0, 1 }, //右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右V90°ターン
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左小回り
				{ 90.0, 40.0, 50.0, 4300.0, 1.0, 45.0, 62.0, 800.0, 0, 1 }, //左大回り
				{ 180.0, 28.5, 151.5, 4300.0, 1.0, 25.0, 32.0, 800.0, 0, 1 }, //左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		}, {
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
		{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 900.0, 0, 0 }, //右小回り
				{ 90.0, 32.0, 58.0, 8360.0, -1.0, 58.0, 89.0, 900.0, 0, 0 }, //右大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右V90°ターン
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 900.0, 0, 0 }, //左小回り
				{ 90.0, 32.0, 58.0, 8360.0, 1.0, 58.0, 93.0, 900.0, 0, 0 }, //左大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		}, {
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
		{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右V90°ターン
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		}, {
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
		{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→右V90°ターン
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 }, //斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		} };

//volatile float_parameters_t angle1, angle2, dit_bef, dist_aft;

//volatile signed short int sample[2500] = { 0 };		//データログのデータ型によって適宜変更する！
volatile float sample1[800] = { 0.0 }, sample2[800] = { 0.0 };

void assign_parameters() {
	turn[0].P_1_0.theta=90.0;
}

int SPIRead(int add) {
	int dumy, read_add;
	volatile unsigned int i;
//	PORT1.DDR.BIT.B0=1;	//Gyro_intのぽーと（意味なし？）
//	PORT1.DR.BIT.B0=0;

//	PORTA.DR.BIT.B3 = 0;  //バーストモードが使えないとき

//	RSPI0.SPDCR.BIT.SPLW = 1;		//バッファにはLONGでアクセス
	RSPI0.SPSR.BYTE = 0xa0;	//エラー無しの書き込み
	RSPI0.SPDCR.BIT.SPRDTD = 0;	//読み出すのは受信バッファ
//	RSPI0.SPDCR.BIT.SPRDTD = 1;	//読み出すのは送信バッファ

	RSPI0.SPCR.BIT.SPE = 1;		//RSPI機能は有効
	for (i = 0; i < 10; i++)
		;		//念のための待ち時間

	dumy = RSPI0.SPDR.LONG;		//32bitどうしで代入
	read_add = (add | (1 << 7));	//8bit長のデータで、最上位ビットを1としたので、MPUは読み取り状態
	RSPI0.SPDR.LONG = read_add;
	RSPI0.SPSR.BYTE;
//	while (RSPI0.SPSR.BIT.SPRF == 0) ;

	while (RSPI0.SPSR.BIT.IDLNF == 1) {

	}	//送受信完了待ち（のはず…）

	dumy = RSPI0.SPDR.LONG;
	RSPI0.SPDR.LONG = 0;
	RSPI0.SPSR.BYTE;

	while (RSPI0.SPSR.BIT.IDLNF == 1)
//	while (RSPI0.SPSR.BIT.SPRF == 0)
		;//送受信完了待ち（のはず…）
	for (i = 0; i < 10; i++)
		;		//念のための待ち時間
	RSPI0.SPCR.BIT.SPE = 0;		//RSPI機能は無効

	//	PORTA.DR.BIT.B3 = 1;	//バースト転送が使えないとき

	return (RSPI0.SPDR.LONG);	//ジャイロから帰ってきた値（受信バッファ）
}

void SPIWrite(int add, int data) {
	int dumy, i;
	RSPI0.SPSR.BYTE = 0xa0;	//エラー無しの書き込み
	RSPI0.SPDCR.BIT.SPLW = 1;		//バッファにはLONGでアクセス

	RSPI0.SPCR.BIT.SPE = 1;		//RSPI機能は有効
	for (i = 0; i < 10; i++)
		;		//念のための待ち時間

	dumy = RSPI0.SPDR.LONG;		//32bitどうしでダミー代入
	RSPI0.SPDR.LONG = add;	//指定アドレスへのデータ書き込み時には、ビット長の最上位ビットは0である
	RSPI0.SPSR.BYTE;
	while (RSPI0.SPSR.BIT.IDLNF == 1)
//	while (RSPI0.SPSR.BIT.SPRF == 0)
		;//送受信完了待ち（のはず…）
	dumy = RSPI0.SPDR.LONG;
	RSPI0.SPDR.LONG = data;		//データをバッファに代入
	RSPI0.SPSR.BYTE;
	while (RSPI0.SPSR.BIT.IDLNF == 1)
//	while (RSPI0.SPSR.BIT.SPRF == 0)
		;//送受信完了待ち（のはず…）
	dumy = RSPI0.SPDR.LONG;		//32bitどうしでダミー代入
	RSPI0.SPCR.BIT.SPE = 0;		//RSPI機能は無効
}
void COPPY_SAVEDMAZE_TO_TEMP() {		//保存した壁情報をtemp移し替える
	int qx;
	for (qx = 0; qx < x_size; qx++) {//全てリセットする（あえてわかりやすく書いている。）
		column_temp[qx] = 0;
		row_temp[qx] = 0;
		column_watched_temp[qx] = 0;
		row_watched_temp[qx] = 0;
	}
	for (qx = 0; qx < x_size; qx++) {//情報をコピーする
		column_temp[qx] = column_fix[qx];
		row_temp[qx] = row_fix[qx];
		column_watched_temp[qx] = column_watched_fix[qx];
		row_watched_temp[qx] = row_watched_fix[qx];
	}
}

void WALL_INFORMATION_save() {		//壁情報の保存
	int qx;
	for (qx = 0; qx < x_size; qx++) {
		column_fix[qx] |= column_temp[qx];
		row_fix[qx] |= row_temp[qx];
	}
}

void WATCHED_WALL_INFORMATION_save() {		//壁を見たかどうかの情報の保存
	int qx;
	for (qx = 0; qx < x_size; qx++) {
		column_watched_fix[qx] |= column_watched_temp[qx];
		row_watched_fix[qx] |= row_watched_temp[qx];
	}
}

void maze_display() {
	volatile int j, yj, xj;
	myprintf("+");
	for (j = 0; j <= 14; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");

	for (yj = 15; yj > 0; yj--) {
		myprintf("|   ");
		for (xj = 1; xj <= 15; xj++) {
			if (is_Exist_Wall(xj, yj, 3) == 1) {
				myprintf("|   ");
			} else {
				myprintf("    ");
			}
		}
		myprintf("|\n\r");

		for (xj = 0; xj <= 15; xj++) {
			if (is_Exist_Wall(xj, yj, 2) == 1) {
				myprintf("+---");
			} else {
				myprintf("+   ");
			}
		}
		myprintf("+\n\r");
	}
	myprintf("|   ");
	for (xj = 1; xj <= 15; xj++) {
		if (is_Exist_Wall(xj, 0, 3) == 1) {
			myprintf("|   ");
		} else {
			myprintf("    ");
		}
	}
	myprintf("|\n\r");

	myprintf("+");
	for (j = 0; j <= 14; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");
}

void saved_maze_display() {
	volatile int j, yj, xj;
	myprintf("+");
	for (j = 0; j <= 14; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");

	for (yj = 15; yj > 0; yj--) {
		myprintf("|   ");
		for (xj = 1; xj <= 15; xj++) {
			if (is_saved_wall_exist(xj, yj, 3) == 1) {
				myprintf("|   ");
			} else {
				myprintf("    ");
			}
		}
		myprintf("|\n\r");

		for (xj = 0; xj <= 15; xj++) {
			if (is_saved_wall_exist(xj, yj, 2) == 1) {
				myprintf("+---");
			} else {
				myprintf("+   ");
			}
		}
		myprintf("+\n\r");
	}
	myprintf("|   ");
	for (xj = 1; xj <= 15; xj++) {
		if (is_saved_wall_exist(xj, 0, 3) == 1) {
			myprintf("|   ");
		} else {
			myprintf("    ");
		}
	}
	myprintf("|\n\r");

	myprintf("+");
	for (j = 0; j <= 14; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");
}

void walkmap_display() {
	volatile int j, yj, xj;
	myprintf("+");
	for (j = 0; j <= x_size - 1; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");

	for (yj = y_size; yj > 0; yj--) {
		myprintf("|");
		myprintf("%3d", map[0][yj]);
		for (xj = 1; xj <= x_size; xj++) {

			if (is_Exist_Wall(xj, yj, 3) == 1) {	//西壁を読む
				myprintf("|");
			} else {
				myprintf(" ");
			}
			myprintf("%3d", map[xj][yj]);
		}
		myprintf("|\n\r");

		for (xj = 0; xj <= x_size; xj++) {
			if (is_Exist_Wall(xj, yj, 2) == 1) {
				myprintf("+---");
			} else {
				myprintf("+   ");
			}
		}
		myprintf("+\n\r");
	}
	myprintf("|");
	myprintf("%3d", map[0][0]);
	for (xj = 1; xj <= x_size; xj++) {
		if (is_Exist_Wall(xj, 0, 3) == 1) {
			myprintf("|");
		} else {
			myprintf(" ");
		}
		myprintf("%3d", map[xj][0]);
	}
	myprintf("|\n\r");

	myprintf("+");
	for (j = 0; j <= x_size - 1; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");

}

void skew_walkmap_display() {	//動作確認済み8/20
	volatile int j, yj, xj;

	myprintf("+");
	for (j = 0; j <= x_size - 1; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");

	myprintf("|  ");
	for (xj = 0; xj <= x_size - 1; xj++) {
		if (is_saved_wall_exist(xj, 15, 1) == 1) {	//東を読む
			myprintf(" |  ");
		} else {
			myprintf("%3d", vertical[xj][15]);
			myprintf(" ");
		}
	}
	myprintf(" |\n\r");

	for (yj = y_size - 1; yj >= 0; yj--) {
		for (xj = 0; xj <= x_size; xj++) {
			if (is_saved_wall_exist(xj, yj, 0) == 1) {	//北を読む
				myprintf("+---");
			} else {
				myprintf("+");
				myprintf("%3d", level[xj][yj]);
			}
		}
		myprintf("+\n\r");

		myprintf("|  ");
		for (xj = 0; xj <= x_size - 1; xj++) {

			if (is_saved_wall_exist(xj, yj, 1) == 1) {	//東壁を読む
				myprintf(" |  ");
			} else {
				myprintf("%3d", vertical[xj][yj]);
				myprintf(" ");
			}
		}
		myprintf(" |\n\r");
	}

	myprintf("+");
	for (j = 0; j <= x_size - 1; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");
}

float kabekire_dist_right(float hikisuu_vel) {
	float adjust_right;
	if (daikei_mode == 1) {	//探索用
		if (hikisuu_vel <= 500.0) {
			adjust_right = 83.5;	//84
		} else if (hikisuu_vel <= 700.0) {
			adjust_right = 84.0;	//。
		} else if (hikisuu_vel <= 800.0) {
			adjust_right = 85.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_right = 91.0;	//。
		} else if (hikisuu_vel > 1000.0) {
			adjust_right = 96.0;	//これはテキトー。
		}
	} else if (daikei_mode == 2) {
		if (hikisuu_vel <= 500.0) {
			adjust_right = 6.5;
		} else if (hikisuu_vel <= 700.0) {
			adjust_right = 6.0;	//これはテキトー。
		} else if (hikisuu_vel <= 800.0) {
			adjust_right = 5.0;	//これはテキトー。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_right = -1.0;	//。
		} else if (hikisuu_vel > 1000.0) {
			adjust_right = -5.0;	//これはテキトー。
		}
	}

	return adjust_right;
}
float kabekire_dist_left(float hikisuu_vel) {
	float adjust_left;
	if (daikei_mode == 1) {	//探索用
		if (hikisuu_vel <= 500.0) {
			adjust_left = 85.0;	//87.0
		} else if (hikisuu_vel <= 700.0) {
			adjust_left = 84.0;	//。
		} else if (hikisuu_vel <= 800.0) {
			adjust_left = 87.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_left = 94.0;	//。
		} else if (hikisuu_vel > 1000.0) {
			adjust_left = 95.0;	//これはテキトー。
		}
	} else if (daikei_mode == 2) {
		if (hikisuu_vel <= 500.0) {
			adjust_left = 5.0;
		} else if (hikisuu_vel <= 700.0) {
			adjust_left = 6.0;	//これはテキトー。
		} else if (hikisuu_vel <= 800.0) {
			adjust_left = 3.0;	//これはテキトー。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_left = -4.0;	//。
		} else if (hikisuu_vel > 1000.0) {
			adjust_left = -5.0;	//これはテキトー。
		}
	}

	return adjust_left;
}

void test_daikei(float hikisuu_dist, float vmax, float hikisuu_accel, float v_0,
		float vterm, char hikisuu_wall) {
	volatile float error, d_i = 0.0, adjust_distance;
	daikei_mode = 1;	//探索用モード

	if (q_dist_flag == 0) {	//直前にqeueを読んでいない場合は距離をリセット。
		ideal_balance_distance = 0.0;
		balance_distance = 0.0;
	} else {
		ideal_balance_distance = balance_distance;
	}

	q_dist_flag = 0;
	ideal_angacc = 0.0;
	ideal_omega = 0.0;	//←不要なのでは?
	ideal_omega2 = 0.0;	//必要。
	ideal_angle = 0.0;
	ei_flag_center = 1;
	ei_flag_rot = 1;
	ei_flag_wall = 0;	//added in 20170922　使わないので。
	kabekire_enable_2 = 1;	//1回の台形に対し、1回だけ壁切れを読めるようにする。
	wall_control = hikisuu_wall;
	Erorr_rot.i = 0.0;

//	error = 30.0;
	if (reverse_flag == 1) {
		error = 50.0;	//袋小路からの壁切れを読むため
	} else {
		error = 30.0;	//壁切れの判定範囲調整用の変数
	}

	if (hikisuu_dist
			<= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //三角加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}

			if (ideal_balance_distance
					<= hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0
									/ hikisuu_accel) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					> hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0 / hikisuu_accel
							/*&& ideal_balance_distance < hikisuu_dist*/) {
				ideal_balance_accel = -1.0 * hikisuu_accel;
				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_balance_accel = 0.0;
//					ideal_balance_velocity = 0.0;
					break;
				}
				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
			}
			/*else if (ideal_balance_distance > hikisuu_dist) {	//理想速度が0以下となる瞬間に速度を0とする
			 ideal_balance_accel = 0.0;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 break;
			 }*/
		}
	} else if (hikisuu_dist
			> ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //台形加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}

			d_i = 0.0;
			if (kabekire_right == 1) {	//壁切れによる補正
//				adjust_distance = kabekire_dist_right(balance_velocity);	//右壁切れの調整距離を判定する
				adjust_distance = kabekire_dist_right(vterm);	//右壁切れの調整距離を判定する
				while (1) {
//					if (84.0 + 90.0 * d_i - error <= balance_distance
//							&& balance_distance <= 87.0 + 90.0 * d_i + error) {	//±errorの範囲で壁切れを読む
//						break;	//breakするので、d_iは増加しない
//					} else if (84.0 + 90.0 * d_i + error > balance_distance) {
//						break;	//breakするので、d_iは増加しない
//					}
					if (adjust_distance + 90.0 * d_i - error <= balance_distance
							&& balance_distance
									<= adjust_distance + 90.0 * d_i + error) {//±errorの範囲で壁切れを読む
						break;	//breakするので、d_iは増加しない
					} else if (adjust_distance + 90.0 * d_i + error
							> balance_distance) {
						break;	//breakするので、d_iは増加しない
					}
					d_i = d_i + 1.0;
				}
				balance_distance = adjust_distance + 90.0 * (d_i);//壁切れ補正(89.0は決め打ち)
				ideal_balance_distance = adjust_distance + 90.0 * (d_i);//壁切れ補正
				LED_V3 = 1;
				kabekire_right = 0;
				kabekire_enable_2 = 0;
			} else if (kabekire_right == 2) {	//櫛切れによる補正(現在不使用-20170912)
//				adjust_distance = kabekire_dist_right(balance_velocity);	//右壁切れの調整距離を判定する
				adjust_distance = kabekire_dist_right(vterm);	//右壁切れの調整距離を判定する
				while (1) {
					if (adjust_distance + 90.0 * d_i - error <= balance_distance
							&& balance_distance
									<= adjust_distance + 90.0 * d_i + error) {//±errorの範囲で壁切れを読む
						break;	//breakするので、d_iは増加しない
					} else if (adjust_distance + 90.0 * d_i + error
							> balance_distance) {
						break;	//breakするので、d_iは増加しない
					}
					d_i = d_i + 1.0;
				}
				balance_distance = adjust_distance + 90.0 * (d_i);//壁切れ補正(97.0は決め打ち)
				ideal_balance_distance = adjust_distance + 90.0 * (d_i);//壁切れ補正
				LED_V1 = 1;
				kabekire_right = 0;
				kabekire_enable_2 = 0;
			} else if (kabekire_left == 1) {	//壁切れによる補正
//				adjust_distance = kabekire_dist_left(balance_velocity);	//右壁切れの調整距離を判定する
				adjust_distance = kabekire_dist_left(vterm);	//右壁切れの調整距離を判定する
				while (1) {
					if (adjust_distance + 90.0 * d_i - error <= balance_distance
							&& balance_distance
									<= adjust_distance + 90.0 * d_i + error) {//±errorの範囲で壁切れを読む
						break;	//breakするので、d_iは増加しない
					} else if (adjust_distance + 90.0 * d_i + error
							> balance_distance) {
						break;	//breakするので、d_iは増加しない
					}
					d_i = d_i + 1.0;
				}
				balance_distance = adjust_distance + 90.0 * (d_i);//壁切れ補正(87.0は決め打ち)
				ideal_balance_distance = adjust_distance + 90.0 * (d_i);//壁切れ補正
				LED2 = 1;
				kabekire_left = 0;
				kabekire_enable_2 = 0;
			} else if (kabekire_left == 2) {	//櫛切れによる補正(現在不使用-20170912)
//				adjust_distance = kabekire_dist_left(balance_velocity);	//右壁切れの調整距離を判定する
				adjust_distance = kabekire_dist_left(vterm);	//右壁切れの調整距離を判定する
				while (1) {
					if (adjust_distance + 90.0 * d_i - error <= balance_distance
							&& balance_distance
									<= adjust_distance + 90.0 * d_i + error) {//±errorの範囲で壁切れを読む
						break;	//breakするので、d_iは増加しない
					} else if (adjust_distance + 90.0 * d_i + error
							> balance_distance) {
						break;	//breakするので、d_iは増加しない
					}
					d_i = d_i + 1.0;
				}
				balance_distance = adjust_distance + 90.0 * (d_i);	//壁切れ補正
				ideal_balance_distance = adjust_distance + 90.0 * (d_i);//壁切れ補正
				LED_V4 = 1;
				kabekire_left = 0;
				kabekire_enable_2 = 0;
			}

			if (fabs(ideal_balance_distance)
					< ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)) {
				ideal_balance_accel = hikisuu_accel;
			} else if (fabs(ideal_balance_distance)
					>= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					&& fabs(ideal_balance_distance)
							<= (hikisuu_dist
									- (vmax * vmax - vterm * vterm) / 2.0
											/ hikisuu_accel)) {
				ideal_balance_accel = 0.0;
				ideal_balance_velocity = vmax;
				if (reverse_flag == 1 && ideal_balance_distance > 90.0 + 40.0) {//決め打ちの値以上となっても壁切れを読んでいない状態でbreak
					reverse_flag = 0;
					break;
				}
			} else if (fabs(ideal_balance_distance)	//実測値のbalance_distanceをもとにして制御するプログラム。
					> (hikisuu_dist
							- (vmax * vmax - vterm * vterm) / 2.0
									/ hikisuu_accel)
					/*&& ideal_balance_distance < hikisuu_dist*/) {
				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_balance_accel = 0.0;
					ideal_balance_velocity = 0.0;
					break;
				}
				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
				ideal_balance_accel = -1.0 * hikisuu_accel;
			}
			/* else if (ideal_balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
			 ideal_balance_accel = 0.0;
			 break;
			 LED_V2=1;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 }*/
		}
	}
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void daikei_for_pass(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall) {
	volatile float error;
	volatile char k_enable_enable = 1;	//クソ頭悪いフラグなので北信越語に直そうな…

	if (q_dist_flag == 0) {	//直前にqeueを読んでいない場合は距離をリセット。
		ideal_balance_distance = 0.0;
		balance_distance = 0.0;
	} else {
		ideal_balance_distance = balance_distance;
	}

	q_dist_flag = 0;
	ideal_angacc = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	ideal_angle = 0.0;
	ei_flag_center = 1;
	ei_flag_rot = 1;
	wall_control = hikisuu_wall;
	error = 40.0;	//カットした後距離より大きめにとる
//	if(reverse_flag == 1){
//		error =80.0;	//袋小路からの壁切れを読むため
//		reverse_flag=0;
//	}else{
//		error =20.0;	//壁切れの判定範囲調整用の変数
//	}

	if (hikisuu_dist
			<= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //三角加速
		while (1) {
			if (ideal_balance_distance
					<= hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0
									/ hikisuu_accel) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					> hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0 / hikisuu_accel
							/*&& ideal_balance_distance < hikisuu_dist*/) {
				ideal_balance_accel = -1.0 * hikisuu_accel;
				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_balance_accel = 0.0;
//					ideal_balance_velocity = 0.0;
					break;
				}
				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
			}
			/*else if (ideal_balance_distance > hikisuu_dist) {	//理想速度が0以下となる瞬間に速度を0とする
			 ideal_balance_accel = 0.0;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 break;
			 }*/
		}
	} else if (hikisuu_dist
			> ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //台形加速
		while (1) {
			d_i = 0.0;
			if (balance_distance > hikisuu_dist - 140.0
					&& balance_distance <= hikisuu_dist) {
				if (k_enable_enable == 1) {
					kabekire_enable_2 = 1;	//1回の台形に対し、1回だけ壁切れを読めるようにする。
				} else {
				}

				if (kabekire_right == 1) {	//壁切れによる補正
					while (1) {
						if (0.0 + 90.0 * d_i <= balance_distance
								&& balance_distance <= error + 90.0 * d_i) {//±errorの範囲で壁切れを読む
							break;	//breakするので、d_iは増加しない
						} else if (error + 90.0 * d_i > balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 89.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正(89.0は決め打ち)
					ideal_balance_distance = 89.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正
//				balance_distance = 89.0;
//				ideal_balance_distance = 89.0;
					LED_V3 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
				} else if (kabekire_right == 2) {	//櫛切れによる補正(現在不使用-20170912)
					while (1) {
						if (0.0 + 90.0 * d_i <= balance_distance
								&& balance_distance <= error + 90.0 * d_i) {//±errorの範囲で壁切れを読む
							break;	//breakするので、d_iは増加しない
						} else if (error + 90.0 * d_i > balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 97.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正(97.0は決め打ち)
					ideal_balance_distance = 97.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正
//				balance_distance = 97.0;
//				ideal_balance_distance = 97.0;
					LED_V1 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
				} else if (kabekire_left == 1) {	//壁切れによる補正
					while (1) {
						if (0.0 + 90.0 * d_i <= balance_distance
								&& balance_distance <= error + 90.0 * d_i) {//±errorの範囲で壁切れを読む
							break;	//breakするので、d_iは増加しない
						} else if (error + 90.0 * d_i > balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 87.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正(87.0は決め打ち)
					ideal_balance_distance = 87.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正
//				balance_distance = 87.0;
//				ideal_balance_distance = 87.0;
					LED2 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
				} else if (kabekire_left == 2) {	//櫛切れによる補正(現在不使用-20170912)
					while (1) {
						if (0.0 + 90.0 * d_i <= balance_distance
								&& balance_distance <= error + 90.0 * d_i) {//±errorの範囲で壁切れを読む
							break;	//breakするので、d_iは増加しない
						} else if (error + 90.0 * d_i > balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 97.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正
					ideal_balance_distance = 97.0
							+ 90.0 * (d_i - (float) pass_kabekire_straight);//壁切れ補正
//				balance_distance = 97.0;
//				ideal_balance_distance = 97.0;
					LED_V4 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
				}
			}

			if (ideal_balance_distance
					< ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					>= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					&& ideal_balance_distance
							<= (hikisuu_dist
									- (vmax * vmax - vterm * vterm) / 2.0
											/ hikisuu_accel)) {
				ideal_balance_accel = 0.0;
				ideal_balance_velocity = vmax;
			} else if (ideal_balance_distance//実測値のbalance_distanceをもとにして制御するプログラム。
					> (hikisuu_dist
							- (vmax * vmax - vterm * vterm) / 2.0
									/ hikisuu_accel)
					/*&& ideal_balance_distance < hikisuu_dist*/) {
				ideal_balance_accel = -1.0 * hikisuu_accel;
				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_balance_accel = 0.0;
					ideal_balance_velocity = 0.0;
					break;
				}
				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
			}
			/* else if (ideal_balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
			 ideal_balance_accel = 0.0;
			 break;
			 LED_V2=1;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 }*/
		}
	}
	kabekire_enable_2 = 0;	//壁切れを読まなかった場合にも次の壁切れを正常に読むため
	pass_kabekire_straight = 0;		//次の壁切れで使えるようにするため(頭悪いからやめような…)
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void daikei_for_pass_kai(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kabekire) {
	volatile float error;
	volatile char k_enable_enable = 0;	//クソ頭悪いフラグなので北信越語に直そうな…
	daikei_mode = 3;	//探索用モード

	k_enable_enable = hikisuu_kabekire;
	if (q_dist_flag == 0) {	//直前にqeueを読んでいない場合は距離をリセット。
		ideal_balance_distance = 0.0;
		balance_distance = 0.0;
	} else {
		ideal_balance_distance = balance_distance;
	}

	q_dist_flag = 0;
	ideal_angacc = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	ideal_angle = 0.0;
	ei_flag_center = 1;
	ei_flag_rot = 1;
	wall_control = hikisuu_wall;
	error = 30.0;	//カットした後距離より大きめにとる
//	if(reverse_flag == 1){
//		error =80.0;	//袋小路からの壁切れを読むため
//		reverse_flag=0;
//	}else{
//		error =20.0;	//壁切れの判定範囲調整用の変数
//	}

	if (hikisuu_dist
			<= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //三角加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}
			if (ideal_balance_distance
					<= hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0
									/ hikisuu_accel) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					> hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0 / hikisuu_accel
							/*&& ideal_balance_distance < hikisuu_dist*/) {
				ideal_balance_accel = -1.0 * hikisuu_accel;
				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_balance_accel = 0.0;
//					ideal_balance_velocity = 0.0;
					break;
				}
				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
			}
			/*else if (ideal_balance_distance > hikisuu_dist) {	//理想速度が0以下となる瞬間に速度を0とする
			 ideal_balance_accel = 0.0;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 break;
			 }*/
		}
	} else if (hikisuu_dist
			> ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //台形加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}
			d_i = 0.0;
			if (balance_distance > hikisuu_dist - 120.0
			/*&& balance_distance <= hikisuu_dist*/) {//大前提として終了距離の直前でしか壁切れは読まない。
				if (k_enable_enable == 1) {
					kabekire_enable_2 = 1;	//CMT内でkabekire_rightなどを立てられるようにする
				}

				if (kabekire_right == 1) {	//壁切れによる補正
					while (1) {
						if (89.0 + 90.0 * d_i - error <= balance_distance
								&& balance_distance
										<= 89.0 + 90.0 * d_i + error) {	//±errorの範囲で壁切れを読む
							break;	//breakするので、d_iは増加しない
						} else if (89.0 + 90.0 * d_i + error
								> balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 89.0 + 90.0 * (d_i);	//壁切れ補正(89.0は決め打ち)
					ideal_balance_distance = 89.0 + 90.0 * (d_i);	//壁切れ補正

//				balance_distance = 89.0;
//				ideal_balance_distance = 89.0;
					LED_V3 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_right == 2) {	//櫛切れによる補正(現在不使用-20170912)
					while (1) {
						if (97.0 + 90.0 * d_i - error <= balance_distance
								&& balance_distance
										<= 97.0 + 90.0 * d_i + error) {	//±errorの範囲で壁切れを読む
							break;
						} else if (97.0 + 90.0 * d_i + error
								> balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 97.0 + 90.0 * (d_i);	//壁切れ補正(97.0は決め打ち)
					ideal_balance_distance = 97.0 + 90.0 * (d_i);	//壁切れ補正
//				balance_distance = 97.0;
//				ideal_balance_distance = 97.0;
					LED_V1 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_left == 1) {	//壁切れによる補正
					while (1) {
						if (84.0 + 90.0 * d_i - error <= balance_distance
								&& balance_distance
										<= 87.0 + 90.0 * d_i + error) {	//±errorの範囲で壁切れを読む
							break;
						} else if (84.0 + 90.0 * d_i + error
								> balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 84.0 + 90.0 * (d_i);	//壁切れ補正(87.0は決め打ち)
					ideal_balance_distance = 84.0 + 90.0 * (d_i);	//壁切れ補正
//				balance_distance = 87.0;
//				ideal_balance_distance = 87.0;
					LED2 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_left == 2) {	//櫛切れによる補正(現在不使用-20170912)
					while (1) {
						if (97.0 + 90.0 * d_i - error <= balance_distance
								&& balance_distance
										<= 97.0 + 90.0 * d_i + error) {	//±errorの範囲で壁切れを読む
							break;
						} else if (97.0 + 90.0 * d_i + error
								> balance_distance) {
							break;	//breakするので、d_iは増加しない
						}
						d_i = d_i + 1.0;
					}
					balance_distance = 97.0 + 90.0 * (d_i);	//壁切れ補正
					ideal_balance_distance = 97.0 + 90.0 * (d_i);	//壁切れ補正
//				balance_distance = 97.0;
//				ideal_balance_distance = 97.0;
					LED_V4 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				}
			}

			if (ideal_balance_distance
					< ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					>= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					&& ideal_balance_distance
							<= (hikisuu_dist
									- (vmax * vmax - vterm * vterm) / 2.0
											/ hikisuu_accel)) {
				ideal_balance_accel = 0.0;
				ideal_balance_velocity = vmax;
			} else if (ideal_balance_distance//実測値のbalance_distanceをもとにして制御するプログラム。
					> (hikisuu_dist
							- (vmax * vmax - vterm * vterm) / 2.0
									/ hikisuu_accel)
					/*&& ideal_balance_distance < hikisuu_dist*/) {

				if (wait_kabekire == 1 && kabekire_read_flag == 0) {//壁切れを読むまで待つ!!!
					if (balance_velocity >= vterm + 50.0) {
						ideal_balance_accel = -1.0 * hikisuu_accel;
					} else if (balance_velocity < vterm - 50.0) {
						ideal_balance_accel = 1.0 * hikisuu_accel;
					} else {
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = vterm;
					}
				} else {
					ideal_balance_accel = -1.0 * hikisuu_accel;
					if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = 0.0;
						break;
					}
					if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
						ideal_balance_accel = 0.0;
						break;
					}
				}
			}
			/* else if (ideal_balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
			 ideal_balance_accel = 0.0;
			 break;
			 LED_V2=1;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 }*/
		}
	}
	kabekire_enable_2 = 0;	//壁切れを読まなかった場合にも次の壁切れを正常に読むため
	wait_kabekire = 0;
	ei_flag_center = 0;
	ei_flag_rot = 0;
	kabekire_read_flag = 0;
}

void daikei_for_pass_kai2(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kabekire) {	//20170928(台形加速の距離の考え方を変える)
//	volatile char k_enable_enable = 0;	//クソ頭悪いフラグなので北信越語に直そうな…
	daikei_mode = 2;	//探索用モード

	kabekire_enable_2 = hikisuu_kabekire;
//	k_enable_enable = hikisuu_kabekire;

	ideal_balance_distance = 0.0;
	balance_distance = 0.0;

	q_dist_flag = 0;
	ideal_angacc = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	ideal_angle = 0.0;
	ei_flag_center = 1;
	ei_flag_rot = 1;
	wall_control = hikisuu_wall;

	if (hikisuu_dist
			<= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //三角加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}
			if (ideal_balance_distance
					<= hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0
									/ hikisuu_accel) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					> hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0 / hikisuu_accel
							/*&& ideal_balance_distance < hikisuu_dist*/) {
				ideal_balance_accel = -1.0 * hikisuu_accel;
				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_balance_accel = 0.0;
//					ideal_balance_velocity = 0.0;
					break;
				}
				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
			}
			/*else if (ideal_balance_distance > hikisuu_dist) {	//理想速度が0以下となる瞬間に速度を0とする
			 ideal_balance_accel = 0.0;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 break;
			 }*/
		}
	} else if (hikisuu_dist
			> ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					+ (vmax * vmax - vterm * vterm) / 2.0 / hikisuu_accel) { //台形加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}
			if (balance_distance > hikisuu_dist - 50.0
			/*&& balance_distance <= hikisuu_dist*/) {//大前提として終了距離の直前でしか壁切れは読まない。
//				if (k_enable_enable == 1) {
					kabekire_enable_2 = 1;	//CMT内でkabekire_rightなどを立てられるようにする
//				}

				if (kabekire_right == 1) {	//壁切れによる補正
					adjust_distance = kabekire_dist_right(vterm);
					balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正(1.0は決め打ち)
					ideal_balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正

					LED_V3 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
//					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_right == 2) {	//櫛切れによる補正(現在不使用-20170928)
					adjust_distance = kabekire_dist_right(vterm);
					balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正(1.0は決め打ち)
					ideal_balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正

					LED_V1 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
//					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_left == 1) {	//壁切れによる補正
					adjust_distance = kabekire_dist_left(vterm);
					balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正(6.0は決め打ち)
					ideal_balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正

					LED2 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
//					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_left == 2) {	//櫛切れによる補正(現在不使用-20170912)
					adjust_distance = kabekire_dist_left(vterm);
					balance_distance = hikisuu_dist - adjust_distance;	//壁切れ補正
					ideal_balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正

					LED_V4 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
//					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				}

				if (adjust_distance < 0.0) {
					adjust_before_dist = adjust_distance;
//					LED_V1 = 1;//debug
				} else {
					adjust_before_dist = 0.0;
//					LED_V4 = 1;//debug

				}
			}

			if (ideal_balance_distance
					< ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					>= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					&& ideal_balance_distance
							<= (hikisuu_dist
									- (vmax * vmax - vterm * vterm) / 2.0
											/ hikisuu_accel)) {
				ideal_balance_accel = 0.0;
				ideal_balance_velocity = vmax;
			} else if (ideal_balance_distance//実測値のbalance_distanceをもとにして制御するプログラム。
					> (hikisuu_dist
							- (vmax * vmax - vterm * vterm) / 2.0
									/ hikisuu_accel)
					/*&& ideal_balance_distance < hikisuu_dist*/) {

				if (wait_kabekire == 1 && kabekire_read_flag == 0) {//壁切れを読むまで待つ!!!
					if (balance_velocity >= vterm + 50.0) {
						ideal_balance_accel = -1.0 * hikisuu_accel;
					} else if (balance_velocity < vterm - 50.0) {
						ideal_balance_accel = 1.0 * hikisuu_accel;
					} else {
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = vterm;
					}
				} else {
					ideal_balance_accel = -1.0 * hikisuu_accel;
					if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = 0.0;
						break;
					}
					if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
						ideal_balance_accel = 0.0;
//						LED_V4 = 1;//debug1001
//						if (kabekire_read_flag == 0) {//debug1001
//							LED_V2 = 1;
//						}
//						if (wait_kabekire == 1) {//debug1001
//							LED_V1 = 1;
//						}
						break;
					}
				}
			}
		}
	}
	kabekire_enable_2 = 0;	//壁切れを読まなかった場合にも次の壁切れを正常に読むため
	wait_kabekire = 0;
	ei_flag_center = 0;
	ei_flag_rot = 0;
	kabekire_read_flag = 0;
}

void reverse_daikei(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall) {//accelとvelocityのみ負とするとバックする
	ideal_balance_distance = 0.0;
	balance_distance = 0.0;
	ideal_angacc = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	ideal_angle = 0.0;
	ei_flag_center = 1;
	ei_flag_rot = 1;
	wall_control = hikisuu_wall;
	reverse_flag = 1;	//壁切れの判定範囲を広くするためのフラグ。壁切れを検知するとフラグは下がる

	if (hikisuu_dist
			<= ((vmax * vmax - v_0 * v_0) / 2.0 / fabs(hikisuu_accel))
					+ (vmax * vmax - vterm * vterm) / 2.0
							/ fabs(hikisuu_accel)) { //三角加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}

			if (fabs(ideal_balance_distance)
					<= hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0
									/ fabs(hikisuu_accel)) {
				ideal_balance_accel = hikisuu_accel;
			} else if (fabs(ideal_balance_distance)
					> hikisuu_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0
									/ fabs(
											hikisuu_accel)
											/*&& ideal_balance_distance < hikisuu_dist*/) {
				ideal_balance_accel = -1.0 * hikisuu_accel;
				if (balance_velocity >= 0.0) {	//実際の速度が正となったら台形加速終了
					ideal_balance_accel = 0.0;
//					ideal_balance_velocity = 0.0;
					break;
				}
				if (fabs(balance_distance) > hikisuu_dist) {//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
			}
		}
	} else if (hikisuu_dist
			> ((vmax * vmax - v_0 * v_0) / 2.0 / fabs(hikisuu_accel))
					+ (vmax * vmax - vterm * vterm) / 2.0
							/ fabs(hikisuu_accel)) { //台形加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}

			if (fabs(ideal_balance_distance)
					< ((vmax * vmax - v_0 * v_0) / 2.0 / fabs(hikisuu_accel))) {
				ideal_balance_accel = hikisuu_accel;
			} else if (fabs(ideal_balance_distance)
					>= ((vmax * vmax - v_0 * v_0) / 2.0 / fabs(hikisuu_accel))
					&& fabs(ideal_balance_distance)
							<= (hikisuu_dist
									- (vmax * vmax - vterm * vterm) / 2.0
											/ fabs(hikisuu_accel))) {
				ideal_balance_accel = 0.0;
				ideal_balance_velocity = vmax;
			} else if (fabs(ideal_balance_distance) //実測値のbalance_distanceをもとにして制御するプログラム。
					> (hikisuu_dist
							- (vmax * vmax - vterm * vterm) / 2.0
									/ fabs(hikisuu_accel))
					/*&& ideal_balance_distance < hikisuu_dist*/) {
				ideal_balance_accel = -1.0 * hikisuu_accel;
				if (balance_velocity >= 0.0) {	//実際の速度が正となったら台形加速終了
					ideal_balance_accel = 0.0;
					ideal_balance_velocity = 0.0;
					break;
				}
				if (fabs(balance_distance) > hikisuu_dist) {//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					break;
				}
			}
		}
	}
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void test_turn(float hikisuu_angle, float omega_max, float hikisuu_angacc,
		float unclock_wise, float hikisuu_balance_velocity) {
	wall_control = 0;
	angle = 0.0;
	ideal_angle = 0.0;
	ideal_omega2 = 0.0;	//必要。
	ideal_balance_velocity = hikisuu_balance_velocity;
	Erorr_rot.i = 0.0;
	ei_flag_center = 1;
	ei_flag_rot = 1;

	if (hikisuu_angle / 2.0
			<= ((omega_max * omega_max) / 2.0 / hikisuu_angacc)) { //三角加速
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//daikeiの関数を抜けられる
			}

			if (fabs(ideal_angle) <= hikisuu_angle / 2.0) {
				ideal_angacc = unclock_wise * hikisuu_angacc;
			} else if (fabs(ideal_angle) > hikisuu_angle / 2.0
					&& fabs(ideal_angle) < hikisuu_angle) {
				ideal_angacc = -1.0 * unclock_wise * hikisuu_angacc;
				if ((unclock_wise * ideal_omega) < 0.0) {
					ideal_angle = unclock_wise * hikisuu_angle;
					ideal_angacc = 0.0;
					ideal_omega = 0.0;	//理想角度の絶対値が90度を越えた瞬間に角速度を0とする
					break;
				}
			} else if (fabs(ideal_angle) > hikisuu_angle) {
				ideal_angle = unclock_wise * hikisuu_angle;
				ideal_angacc = 0.0;
				ideal_omega = 0.0;	//理想角度の絶対値が90度を越えた瞬間に角速度を0とする
				LED1 = 1;
				break;
			}
		}
	}
	LED1 = 0;
	ei_flag_center = 0;
	ei_flag_rot = 0;
}
void test_slalom(float hikisuu_angle, float omega_max, float hikisuu_angacc,
		float unclock_wise, float hikisuu_balance_velocity, float dist1,
		float dist2) {
//	angle = 0.0;
	ideal_angle = 0.0;
	ideal_omega = 0.0;
	Erorr_rot.i = 0.0;
	ei_flag_center = 1;
	test_daikei(dist1, hikisuu_balance_velocity, 10.0, hikisuu_balance_velocity,
			hikisuu_balance_velocity, 0);
	ei_flag_rot = 1;
	ideal_balance_velocity = hikisuu_balance_velocity;
	if (hikisuu_angle / 2.0
			<= ((omega_max * omega_max) / 2.0 / hikisuu_angacc)) { //三角加速
		while (1) {
			if (fabs(ideal_angle) <= hikisuu_angle / 2.0) {
				ideal_angacc = unclock_wise * hikisuu_angacc;
			} else if (fabs(ideal_angle) > hikisuu_angle / 2.0
					&& fabs(ideal_angle) < hikisuu_angle) {
				ideal_angacc = -1.0 * unclock_wise * hikisuu_angacc;
				if ((unclock_wise * ideal_omega) < 0.0) {
					ideal_angle = unclock_wise * hikisuu_angle;
					ideal_angacc = 0.0;
					ideal_omega = 0.0;	//理想角度の絶対値が90度を越えた瞬間に角速度を0とする
					break;
				}
			} else if (fabs(ideal_angle) > hikisuu_angle) {
				ideal_angle = unclock_wise * hikisuu_angle;
				ideal_angacc = 0.0;
				ideal_omega = 0.0;	//理想角度の絶対値が90度を越えた瞬間に角速度を0とする
				break;
			}
		}
	} else if (hikisuu_angle
			> ((omega_max * omega_max) / 2.0 / hikisuu_angacc)
					+ (omega_max * omega_max) / 2.0 / hikisuu_angacc) { //台形加速
		while (1) {
			if (fabs(ideal_angle)
					< ((omega_max * omega_max) / 2.0 / hikisuu_angacc)) {
				ideal_angacc = unclock_wise * hikisuu_angacc;
			} else if (fabs(ideal_angle)
					>= ((omega_max * omega_max) / 2.0 / hikisuu_angacc)
					&& fabs(ideal_angle)
							<= (hikisuu_angle
									- (omega_max * omega_max) / 2.0
											/ hikisuu_angle)) {
				ideal_angacc = 0.0;
				ideal_omega = omega_max;
			} else if (fabs(ideal_angle)
					> (hikisuu_angle
							- (omega_max * omega_max) / 2.0 / hikisuu_angacc)
					&& fabs(ideal_angle) < hikisuu_angle) {
				ideal_angacc = -1.0 * unclock_wise * hikisuu_angacc;
				if (unclock_wise * ideal_omega <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_angacc = 0.0;
					ideal_omega = 0.0;
					break;
				}
			} else if (fabs(ideal_angle) > hikisuu_angle) {	//理想速度が0以下となる瞬間に速度を0とする
				ideal_angacc = 0.0;
				ideal_omega = 0.0;
				break;
//				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			}
		}
	}
	test_daikei(dist2, hikisuu_balance_velocity, 10.0, hikisuu_balance_velocity,
			hikisuu_balance_velocity, 0);
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void slalom_2(float hikisuu_angle, float angle1, float angle2, float omega_max,
		float hikisuu_angacc, float unclock_wise,
		float hikisuu_balance_velocity, float dist1, float dist2) {	//壁切れ等を見ない小回り等に使用
	ideal_angle = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	Erorr_rot.i = 0.0;
	ei_flag_center = 1;
	test_daikei(dist1, hikisuu_balance_velocity, 10.0, hikisuu_balance_velocity,
			hikisuu_balance_velocity, 1);
	ei_flag_rot = 1;
	ideal_balance_velocity = hikisuu_balance_velocity;
	wall_control = 0;
	ideal_omega = 0.0;
	if (angle1 < ((omega_max * omega_max) / 2.0 / hikisuu_angacc)) { //普通にスラローム
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//slalomの関数を抜けられる
			}

			if (fabs(ideal_angle) < angle1) {
				ideal_angacc = unclock_wise * hikisuu_angacc;
			} else if (fabs(ideal_angle) >= angle1
					&& fabs(ideal_angle) <= angle2) {
				ideal_angacc = 0.0;
//				ideal_omega = omega_max;
			} else if (fabs(ideal_angle) > angle2
					&& fabs(ideal_angle) < hikisuu_angle) {
				ideal_angacc = -1.0 * unclock_wise * hikisuu_angacc;
				if (unclock_wise * ideal_omega <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_angacc = 0.0;
					ideal_omega = 0.0;
					break;
				}
			} else if (fabs(ideal_angle) > hikisuu_angle) {	//理想速度が0以下となる瞬間に速度を0とする
				ideal_angacc = 0.0;
				ideal_omega = 0.0;
				break;
//				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			}
		}
	} else {

	}

	Erorr_rot.i = 0.0;
	test_daikei(dist2, hikisuu_balance_velocity, 10.0, hikisuu_balance_velocity,
			hikisuu_balance_velocity, 1);
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void slalom_for_tuning(float hikisuu_angle, float angle1, float angle2, float omega_max,
		float hikisuu_angacc, float unclock_wise,
		float hikisuu_balance_velocity, float dist1, float dist2) {	//壁制御無し
	ideal_angle = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	Erorr_rot.i = 0.0;
	q_dist_flag = 0;
	ei_flag_center = 1;
	test_daikei(dist1, hikisuu_balance_velocity, 10.0, hikisuu_balance_velocity,
			hikisuu_balance_velocity, 0);
	ei_flag_rot = 1;
	ideal_balance_velocity = hikisuu_balance_velocity;
	wall_control = 0;
	ideal_omega = 0.0;
	if (angle1 < ((omega_max * omega_max) / 2.0 / hikisuu_angacc)) { //普通にスラローム
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//slalomの関数を抜けられる
			}

			if (fabs(ideal_angle) < angle1) {
				ideal_angacc = unclock_wise * hikisuu_angacc;
			} else if (fabs(ideal_angle) >= angle1
					&& fabs(ideal_angle) <= angle2) {
				ideal_angacc = 0.0;
			} else if (fabs(ideal_angle) > angle2
					&& fabs(ideal_angle) < hikisuu_angle) {
				ideal_angacc = -1.0 * unclock_wise * hikisuu_angacc;
				if (unclock_wise * ideal_omega <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_angacc = 0.0;
					ideal_omega = 0.0;
					break;
				}
			} else if (fabs(ideal_angle) > hikisuu_angle) {	//理想速度が0以下となる瞬間に速度を0とする
				ideal_angacc = 0.0;
				ideal_omega = 0.0;
				break;
			}
		}
	} else {

	}
	Erorr_rot.i = 0.0;
	test_daikei(dist2, hikisuu_balance_velocity, 10.0, hikisuu_balance_velocity,
			hikisuu_balance_velocity, 0);
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void turn_for_pass(float hikisuu_angle, float angle1, float angle2,
		float omega_max, float hikisuu_angacc, float unclock_wise,
		float hikisuu_balance_velocity, float dist1, float dist2) {
	ideal_angle = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	Erorr_rot.i = 0.0;
	ei_flag_center = 1;
	q_dist_flag = 0;
	daikei_for_pass_kai2(dist1, hikisuu_balance_velocity, 10.0,
			hikisuu_balance_velocity, hikisuu_balance_velocity, 1, 0);//壁切れ読まない
	ei_flag_rot = 1;
	ideal_balance_velocity = hikisuu_balance_velocity;
	wall_control = 0;	//ターン中は切る
	ideal_omega = 0.0;	//Error文を相殺するため
	if (angle1 < ((omega_max * omega_max) / 2.0 / hikisuu_angacc)) { //普通にスラローム
		while (1) {
			if (fail_flag == 1) {	//failセーフ
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
				break;	//turnの関数を抜けられる
			}

			if (fabs(ideal_angle) < angle1) {
				ideal_angacc = unclock_wise * hikisuu_angacc;
			} else if (fabs(ideal_angle) >= angle1
					&& fabs(ideal_angle) <= angle2) {
				ideal_angacc = 0.0;
//				ideal_omega = omega_max;
			} else if (fabs(ideal_angle) > angle2
					&& fabs(ideal_angle) < hikisuu_angle) {
				ideal_angacc = -1.0 * unclock_wise * hikisuu_angacc;
				if (unclock_wise * ideal_omega <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_angacc = 0.0;
					ideal_omega = 0.0;
					break;
				}
			} else if (fabs(ideal_angle) > hikisuu_angle) {	//理想速度が0以下となる瞬間に速度を0とする
				ideal_angacc = 0.0;
				ideal_omega = 0.0;
				break;
//				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			}
		}
	} else {

	}
	daikei_for_pass_kai2(dist2, hikisuu_balance_velocity, 10.0,
			hikisuu_balance_velocity, hikisuu_balance_velocity, 1, 1);	//壁切れ読む
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void direction_xy() {	//
	switch (direction_count) {
	case 0:		//North
		y++;
		break;
	case 1:		//East
		x++;
		break;
	case 2:		//South
		y--;
		break;
	case 3:		//West
		x--;
		break;
	}
}

void adachihou_q(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search) {		//正常な足立法。
	volatile float r_before, l_before, r_after, l_after;
	//	r_before = 12.0;
	//	r_after = 8.0;
	r_before = 10.5;
	r_after = 12.0;
	l_before = 7.8;
	l_after = 23.0;

	column_temp[0] |= 1;

	x = start_x;
	y = start_y;
	wait(300); 			//励磁直後は少し待つ！
	sample_flag = 1;

	test_daikei(90.0, 500.0, 3000.0, 0.0, 500.0, 0);
	direction_xy();
	//	setReached(x, y);
	watched_wall_front(0, 0, 0);		//初期に読めない壁だが、読んだことにする
	watched_wall_right(0, 0, 0);		//初期に読めない壁だが、読んだことにする
	while (1) {
		if (fail_flag == 1) {	//failセーフ
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			LED_V1 = 1;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 1;
			break;	//daikeiの関数を抜けられる
		}

		balance_distance = 0.0;		//ここから走行距離の測定開始
		if (map[x][y] == 255) {		//閉じ込められたら自動で抜け出す。
			if (is_Exist_Wall(temp_goal_x, temp_goal_y, 0) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 1) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 2) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 3) == 1) {
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			fail_flag = 1;
		}
//		if (sen.right_front > 850 && sen.left_front > 850
//				&& clash_count >= 1) {		//ぶつかり続けるとカウント追加
//			clash_count++;
//			if (clash_count >= 3) {
//				accident_flag = 1;
//			}
//		} else {
//			clash_count = 0;
//		}
//		if (accident_flag == 1) {
//			break;
//		}
		if (sen.right_front >= r_front_wall_judge) {	//前センサーの壁判断
			add_wall_front(x, y, direction_count);
		}
		if (sen.right_side >= r_wall_judge) {	//右センサーの壁判断
			add_wall_right(x, y, direction_count);
		}
		if (sen.left_side >= l_wall_judge) {	//左センサーの壁判断
			add_wall_left(x, y, direction_count);
		}
		watched_wall_front(x, y, direction_count);
		watched_wall_right(x, y, direction_count);
		watched_wall_left(x, y, direction_count);

		/*		if (x == goal_x && y == goal_y) {		//以下、クシつぶし

		 } else if (x == goal_x + 1 && y == goal_y) {

		 } else if (x == goal_x + 1 && y == goal_y + 1) {

		 } else if (x == goal_x && y == goal_y + 1) {
		 } else {

		 if (x > 0 && y < 15 && is_the_Wall_watched(x, y, 3) == 1	//左上横壁追加
		 && is_the_Wall_watched(x, y + 1, 3) == 1
		 && is_the_Wall_watched(x, y, 0) == 1
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_Exist_Wall(x, y + 1, 3) == 0
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_the_Wall_watched(x - 1, y, 0) == 0) {
		 add_wall_front(x - 1, y, 0);
		 watched_wall_front(x - 1, y, 0);
		 }
		 if (x < 15 && y < 15 && is_the_Wall_watched(x, y, 1) == 1	//右上横壁追加
		 && is_the_Wall_watched(x, y + 1, 1) == 1
		 && is_the_Wall_watched(x, y, 0) == 1
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_Exist_Wall(x, y + 1, 1) == 0
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_the_Wall_watched(x + 1, y, 0) == 0) {
		 add_wall_front(x + 1, y, 0);
		 watched_wall_front(x + 1, y, 0);
		 }
		 if (x < 15 && y > 0 && is_the_Wall_watched(x, y, 1) == 1	//右下横壁追加
		 && is_the_Wall_watched(x, y - 1, 1) == 1
		 && is_the_Wall_watched(x, y, 2) == 1
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_Exist_Wall(x, y - 1, 1) == 0
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_the_Wall_watched(x + 1, y - 1, 0) == 0) {
		 add_wall_front(x + 1, y - 1, 0);
		 watched_wall_front(x + 1, y - 1, 0);
		 }
		 if (x > 0 && y > 0 && is_the_Wall_watched(x, y, 3) == 1	//左下横壁追加
		 && is_the_Wall_watched(x, y - 1, 3) == 1
		 && is_the_Wall_watched(x, y, 2) == 1
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_Exist_Wall(x, y - 1, 3) == 0
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_the_Wall_watched(x - 1, y - 1, 0) == 0) {
		 add_wall_front(x - 1, y - 1, 0);
		 watched_wall_front(x - 1, y - 1, 0);
		 }
		 if (x > 0 && y < 15 && is_the_Wall_watched(x - 1, y, 0) == 1//左上縦壁追加
		 && is_the_Wall_watched(x, y, 0) == 1
		 && is_the_Wall_watched(x, y, 3) == 1
		 && is_Exist_Wall(x - 1, y, 0) == 0
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_the_Wall_watched(x, y + 1, 3) == 0) {
		 add_wall_left(x, y + 1, 0);
		 watched_wall_left(x, y + 1, 0);
		 }
		 if (x < 15 && y < 15 && is_the_Wall_watched(x, y, 0) == 1	//右上縦壁追加
		 && is_the_Wall_watched(x + 1, y, 0) == 1
		 && is_the_Wall_watched(x, y, 1) == 1
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_Exist_Wall(x + 1, y, 0) == 0
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_the_Wall_watched(x, y + 1, 1) == 0) {
		 add_wall_right(x, y + 1, 0);
		 watched_wall_right(x, y + 1, 0);
		 }
		 if (x < 15 && y > 0 && is_the_Wall_watched(x, y, 2) == 1	//右下縦壁追加
		 && is_the_Wall_watched(x + 1, y, 2) == 1
		 && is_the_Wall_watched(x, y, 1) == 1
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_Exist_Wall(x + 1, y, 2) == 0
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_the_Wall_watched(x, y - 1, 1) == 0) {
		 add_wall_right(x, y - 1, 0);
		 watched_wall_right(x, y - 1, 0);
		 }
		 if (x > 0 && y > 0 && is_the_Wall_watched(x - 1, y, 2) == 1//左下縦壁追加
		 && is_the_Wall_watched(x, y, 2) == 1
		 && is_the_Wall_watched(x, y, 3) == 1
		 && is_Exist_Wall(x - 1, y, 2) == 0
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_the_Wall_watched(x, y - 1, 3) == 0) {
		 add_wall_left(x, y - 1, 0);
		 watched_wall_left(x, y - 1, 0);
		 }
		 }
		 */
//		total_dist = 0.0;	DCでは不要
		LED1 = 1;
		q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
		LED1 = 0;

		switch (direction_count) {
		case 0:
			if (y < 15&& map[x][y + 1]
			< map[x][y]&& sen.right_front < r_front_wall_judge) {	//North & 直進

				test_daikei(180.0, 500.0, 3000.0, 500.0, 500.0, 1);
				direction_xy();
			} else if (x < 15&& map[x + 1][y]
			< map[x][y] && sen.right_side < r_wall_judge) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, r_before,
						r_after);	//右小回り
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (x > 0&& map[x - 1][y]
			< map[x][y]&& sen.left_side < l_wall_judge) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, 1.0, 500.0, l_before,
						l_after);	//左小回り
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 500.0, 3000.0, 0.0, 500.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 1:
			if (x
					< 15&& map[x+1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//East & 直進
				test_daikei(180.0, 500.0, 3000.0, 500.0, 500.0, 1);
				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_side < r_wall_judge) {//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, r_before,
						r_after);	//右小回り
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.left_side < l_wall_judge) {//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, 1.0, 500.0, l_before,
						l_after);	//左小回り
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 500.0, 3000.0, 0.0, 500.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 2:
			if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_front < r_front_wall_judge) {//South & 直進

				test_daikei(180.0, 500.0, 3000.0, 500.0, 500.0, 1);
				direction_xy();
			} else if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_side < r_wall_judge) {//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, r_before,
						r_after);	//右小回り
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (x
					< 15&& map[x+1][y] < map[x][y] && sen.left_side < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, 1.0, 500.0, l_before,
						l_after);	//左小回り
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				sample_flag = 1;

				test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 500.0, 3000.0, 0.0, 500.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 3:
			if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//South & 直進
				direction_xy();
				test_daikei(180.0, 500.0, 3000.0, 500.0, 500.0, 1);

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.right_side < r_wall_judge) {//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, r_before,
						r_after);	//右小回り
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.left_side < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, 1.0, 500.0, l_before,
						l_after);	//左小回り
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 500.0, 3000.0, 0.0, 500.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		}
		LED_V1 = 0;
		LED_V2 = 0;
		LED_V3 = 0;
		LED_V4 = 0;

		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}
	}
	test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 0);
//	if (accident_flag == 0) {
//		distance3(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
//		wait(300);
//	}
}

void adachihou2_q(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search) {		//正常な足立法。
	volatile float r_before, l_before, r_after, l_after;
	//	r_before = 12.0;
	//	r_after = 8.0;
	r_before = 10.5;
	r_after = 12.0;
	l_before = 7.8;
	l_after = 23.0;

	column_temp[0] |= 1;

	x = start_x;
	y = start_y;
	wait(300); 			//励磁直後は少し待つ！
	sample_flag = 1;

	test_daikei(90.0, 650.0, 5000.0, 0.0, 650.0, 0);
	direction_xy();
	//	setReached(x, y);
	watched_wall_front(0, 0, 0);		//初期に読めない壁だが、読んだことにする
	watched_wall_right(0, 0, 0);		//初期に読めない壁だが、読んだことにする
	while (1) {
		if (fail_flag == 1) {	//failセーフ
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			LED_V1 = 1;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 1;
			break;	//daikeiの関数を抜けられる
		}

		balance_distance = 0.0;		//ここから走行距離の測定開始
		if (map[x][y] == 255) {		//閉じ込められたら自動で抜け出す。
			if (is_Exist_Wall(temp_goal_x, temp_goal_y, 0) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 1) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 2) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 3) == 1) {
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			fail_flag = 1;
		}
//		if (sen.right_front > 850 && sen.left_front > 850
//				&& clash_count >= 1) {		//ぶつかり続けるとカウント追加
//			clash_count++;
//			if (clash_count >= 3) {
//				accident_flag = 1;
//			}
//		} else {
//			clash_count = 0;
//		}
//		if (accident_flag == 1) {
//			break;
//		}
		if (sen.right_front >= r_front_wall_judge) {	//前センサーの壁判断
			add_wall_front(x, y, direction_count);
		}
		if (sen.right_side >= r_wall_judge) {	//右センサーの壁判断
			add_wall_right(x, y, direction_count);
		}
		if (sen.left_side >= l_wall_judge) {	//左センサーの壁判断
			add_wall_left(x, y, direction_count);
		}
		watched_wall_front(x, y, direction_count);
		watched_wall_right(x, y, direction_count);
		watched_wall_left(x, y, direction_count);

		/*		if (x == goal_x && y == goal_y) {		//以下、クシつぶし

		 } else if (x == goal_x + 1 && y == goal_y) {

		 } else if (x == goal_x + 1 && y == goal_y + 1) {

		 } else if (x == goal_x && y == goal_y + 1) {
		 } else {

		 if (x > 0 && y < 15 && is_the_Wall_watched(x, y, 3) == 1	//左上横壁追加
		 && is_the_Wall_watched(x, y + 1, 3) == 1
		 && is_the_Wall_watched(x, y, 0) == 1
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_Exist_Wall(x, y + 1, 3) == 0
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_the_Wall_watched(x - 1, y, 0) == 0) {
		 add_wall_front(x - 1, y, 0);
		 watched_wall_front(x - 1, y, 0);
		 }
		 if (x < 15 && y < 15 && is_the_Wall_watched(x, y, 1) == 1	//右上横壁追加
		 && is_the_Wall_watched(x, y + 1, 1) == 1
		 && is_the_Wall_watched(x, y, 0) == 1
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_Exist_Wall(x, y + 1, 1) == 0
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_the_Wall_watched(x + 1, y, 0) == 0) {
		 add_wall_front(x + 1, y, 0);
		 watched_wall_front(x + 1, y, 0);
		 }
		 if (x < 15 && y > 0 && is_the_Wall_watched(x, y, 1) == 1	//右下横壁追加
		 && is_the_Wall_watched(x, y - 1, 1) == 1
		 && is_the_Wall_watched(x, y, 2) == 1
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_Exist_Wall(x, y - 1, 1) == 0
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_the_Wall_watched(x + 1, y - 1, 0) == 0) {
		 add_wall_front(x + 1, y - 1, 0);
		 watched_wall_front(x + 1, y - 1, 0);
		 }
		 if (x > 0 && y > 0 && is_the_Wall_watched(x, y, 3) == 1	//左下横壁追加
		 && is_the_Wall_watched(x, y - 1, 3) == 1
		 && is_the_Wall_watched(x, y, 2) == 1
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_Exist_Wall(x, y - 1, 3) == 0
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_the_Wall_watched(x - 1, y - 1, 0) == 0) {
		 add_wall_front(x - 1, y - 1, 0);
		 watched_wall_front(x - 1, y - 1, 0);
		 }
		 if (x > 0 && y < 15 && is_the_Wall_watched(x - 1, y, 0) == 1//左上縦壁追加
		 && is_the_Wall_watched(x, y, 0) == 1
		 && is_the_Wall_watched(x, y, 3) == 1
		 && is_Exist_Wall(x - 1, y, 0) == 0
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_the_Wall_watched(x, y + 1, 3) == 0) {
		 add_wall_left(x, y + 1, 0);
		 watched_wall_left(x, y + 1, 0);
		 }
		 if (x < 15 && y < 15 && is_the_Wall_watched(x, y, 0) == 1	//右上縦壁追加
		 && is_the_Wall_watched(x + 1, y, 0) == 1
		 && is_the_Wall_watched(x, y, 1) == 1
		 && is_Exist_Wall(x, y, 0) == 0
		 && is_Exist_Wall(x + 1, y, 0) == 0
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_the_Wall_watched(x, y + 1, 1) == 0) {
		 add_wall_right(x, y + 1, 0);
		 watched_wall_right(x, y + 1, 0);
		 }
		 if (x < 15 && y > 0 && is_the_Wall_watched(x, y, 2) == 1	//右下縦壁追加
		 && is_the_Wall_watched(x + 1, y, 2) == 1
		 && is_the_Wall_watched(x, y, 1) == 1
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_Exist_Wall(x + 1, y, 2) == 0
		 && is_Exist_Wall(x, y, 1) == 0
		 && is_the_Wall_watched(x, y - 1, 1) == 0) {
		 add_wall_right(x, y - 1, 0);
		 watched_wall_right(x, y - 1, 0);
		 }
		 if (x > 0 && y > 0 && is_the_Wall_watched(x - 1, y, 2) == 1//左下縦壁追加
		 && is_the_Wall_watched(x, y, 2) == 1
		 && is_the_Wall_watched(x, y, 3) == 1
		 && is_Exist_Wall(x - 1, y, 2) == 0
		 && is_Exist_Wall(x, y, 2) == 0
		 && is_Exist_Wall(x, y, 3) == 0
		 && is_the_Wall_watched(x, y - 1, 3) == 0) {
		 add_wall_left(x, y - 1, 0);
		 watched_wall_left(x, y - 1, 0);
		 }
		 }
		 */
//		total_dist = 0.0;	DCでは不要
		LED1 = 1;
		q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
		LED1 = 0;

		switch (direction_count) {
		case 0:
			if (y < 15&& map[x][y + 1]
			< map[x][y]&& sen.right_front < r_front_wall_judge) {	//North & 直進

				test_daikei(180.0, 650.0, 3000.0, 650.0, 650.0, 1);
				direction_xy();
			} else if (x < 15&& map[x + 1][y]
			< map[x][y] && sen.right_side < r_wall_judge) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0,
						8.0);		//右小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (x > 0&& map[x - 1][y]
			< map[x][y]&& sen.left_side < l_wall_judge) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0,
						25.0);		//左小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				test_daikei(90.0, 650.0, 5000.0, 650.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 650.0, 5000.0, 0.0, 650.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 1:
			if (x
					< 15&& map[x+1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//East & 直進
				test_daikei(180.0, 650.0, 3000.0, 650.0, 650.0, 1);
				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_side < r_wall_judge) {//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0,
						8.0);		//右小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.left_side < l_wall_judge) {//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0,
						25.0);		//左小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				test_daikei(90.0, 650.0, 5000.0, 650.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 650.0, 3500.0, 0.0, 650.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 2:
			if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_front < r_front_wall_judge) {//South & 直進

				test_daikei(180.0, 650.0, 3000.0, 650.0, 650.0, 1);
				direction_xy();
			} else if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_side < r_wall_judge) {//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0,
						8.0);		//右小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (x
					< 15&& map[x+1][y] < map[x][y] && sen.left_side < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0,
						25.0);		//左小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				sample_flag = 1;

				test_daikei(90.0, 650.0, 5000.0, 650.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 650.0, 5000.0, 0.0, 650.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 3:
			if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//South & 直進
				direction_xy();
				test_daikei(180.0, 650.0, 3000.0, 650.0, 650.0, 1);

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.right_side < r_wall_judge) {//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0,
						8.0);		//右小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, r_before, r_after);	//右折　保存
				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.left_side < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0,
						25.0);		//左小回り(速め)
//				test_slalom(90.0, 650.0, 4500.0, 1.0, 500.0, l_before, l_after);//左折
				direction_xy();

			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				test_daikei(90.0, 650.0, 5000.0, 650.0, 0.0, 0);
				wait(300);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(300);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(300);
				test_daikei(180.0, 650.0, 5000.0, 0.0, 650.0, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		}
		LED_V1 = 0;
		LED_V2 = 0;
		LED_V3 = 0;
		LED_V4 = 0;

		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}
	}
	test_daikei(90.0, 650.0, 5000.0, 650.0, 0.0, 0);
//	if (accident_flag == 0) {
//		distance3(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
//		wait(300);
//	}
}

void make_pass(int hikisuu_goal_x, int hikisuu_goal_y) {		//パスの作成（簡易版）
	volatile int p_i;
	column_temp[0] |= 1;
	direction_count = 0;		//スタート地点を考えている。
	x = 0;
	y = 1;
	p_i = 0;
	pass[0] = 255;
	q_saved_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
	q_dist_flag = 0;	//saved_walk_map_makerでは不要だが、一応入れておく

	while (1) {

		switch (direction_count) {
		case 0:
			if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 0) == 0) {	//North & 直進
				direction_xy();
				if (pass[p_i] <= 30) {
					pass[p_i] = pass[p_i] + 2;
				} else {
					p_i++;
					pass[p_i] = 2;
				}
			} else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 1) == 0) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 40;
			} else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 3) == 0) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 50;
			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				direction_xy();
			}
			break;

		case 1:
			if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 1) == 0) {	//East & 直進
				direction_xy();
				if (pass[p_i] <= 30) {
					pass[p_i] = pass[p_i] + 2;
				} else {
					p_i++;
					pass[p_i] = 2;
				}
			} else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 2) == 0) {	//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 40;

			} else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 0) == 0) {	//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 50;
			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				direction_xy();
			}
			break;

		case 2:
			if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 2) == 0) {	//South & 直進
				direction_xy();
				if (pass[p_i] <= 30) {
					pass[p_i] = pass[p_i] + 2;
				} else {
					p_i++;
					pass[p_i] = 2;
				}

			} else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 3) == 0) {	//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 40;
			} else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 1) == 0) {	//South & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 50;
			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				direction_xy();
			}
			break;

		case 3:
			if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 3) == 0) {	//West & 直進
				direction_xy();
				if (pass[p_i] <= 30) {
					pass[p_i] = pass[p_i] + 2;
				} else {
					p_i++;
					pass[p_i] = 2;
				}

			} else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 0) == 0) {	//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 40;
			} else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_saved_wall_exist(x, y, 2) == 0) {	//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				p_i++;
				pass[p_i] = 50;
			} else {
				if (direction_count == 0) {
					direction_count = 2;
				} else if (direction_count == 1) {
					direction_count = 3;
				} else if (direction_count == 2) {
					direction_count = 0;
				} else {
					direction_count = 1;
				}
				direction_xy();
			}
			break;
		}
		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			pass[p_i + 1] = 100;
			last_p_i = p_i + 1;
			break;
		}

	}
}

void convert_pass_skew() {
	volatile int read_p_i, motion_count, adjust_straight, thin = 0, s_count, j=0;
	read_p_i = 0;
	motion_count = 0;
	adjust_straight = 0;
	while (1) {
		if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] <= 30
				&& pass[read_p_i + 2] >= 1) {	//右の大廻ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 114;
			adjust_straight = -1;	//ターンの後ろ側の直進分を記憶し、次のmotionの直進部に反映させる
			read_p_i = read_p_i + 2;	//次のpassの読み込み位置
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] <= 30
				&& pass[read_p_i + 2] >= 1) {	//左の大廻ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 115;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;	//
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] == 40
				&& pass[read_p_i + 3] <= 30 && pass[read_p_i + 3] >= 1) {//右の180度ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 124;
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] == 50
				&& pass[read_p_i + 3] <= 30 && pass[read_p_i + 3] >= 1) {//左の180度ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 125;
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] == 50) {//右の45degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 134;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] == 40) {//左の45degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 135;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] == 40
				&& pass[read_p_i + 3] == 50) {	//右の135degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 144;
			adjust_straight = 0;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] == 50
				&& pass[read_p_i + 3] == 40) {	//左の135degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 145;
			adjust_straight = 0;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] == 50 && pass[read_p_i + 1] == 40
				&& pass[read_p_i + 2] >= 1 && pass[read_p_i + 2] <= 30) {//斜め→右45°ターン
			motion[motion_count] = 154;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] == 40 && pass[read_p_i + 1] == 50
				&& pass[read_p_i + 2] >= 1 && pass[read_p_i + 2] <= 30) {//斜め→左45°ターン
			motion[motion_count] = 155;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] == 50 && pass[read_p_i + 1] == 40
				&& pass[read_p_i + 2] == 40 && pass[read_p_i + 3] >= 1
				&& pass[read_p_i + 3] <= 30) {	//斜め→右135°ターン
			motion[motion_count] = 164;
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] == 40 && pass[read_p_i + 1] == 50
				&& pass[read_p_i + 2] == 50 && pass[read_p_i + 3] >= 1
				&& pass[read_p_i + 3] <= 30) {	//斜め→左135°ターン
			motion[motion_count] = 165;
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] == 50 && pass[read_p_i + 1] == 40
				&& pass[read_p_i + 2] == 40 && pass[read_p_i + 3] == 50) {//斜め→右V90°ターン
			motion[motion_count] = 174;
			adjust_straight = 0;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] == 40 && pass[read_p_i + 1] == 50
				&& pass[read_p_i + 2] == 50 && pass[read_p_i + 3] == 40) {//斜め→左V90°ターン
			motion[motion_count] = 175;
			adjust_straight = 0;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] == 40 && pass[read_p_i + 1] == 50
				&& pass[read_p_i + 2] == 40) {	//斜め→斜め直進
			motion[motion_count] = 201;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;
		} else if (pass[read_p_i] == 50 && pass[read_p_i + 1] == 40
				&& pass[read_p_i + 2] == 50) {	//斜め→斜め直進
			motion[motion_count] = 201;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;
		} else if (pass[read_p_i] == 255 && pass[read_p_i + 1] == 40) {	//開幕ターンがある場合
			if (pass[read_p_i + 2] == 50) {	//開幕左45°ターン→斜め
				motion[motion_count] = 84;
				adjust_straight = 0;
				read_p_i = read_p_i + 1;
			} else if (pass[read_p_i + 2] == 40 && pass[read_p_i + 3] == 50) {//開幕右135°ターン→斜め
				motion[motion_count] = 94;
				adjust_straight = 0;
				read_p_i = read_p_i + 2;
			} else if (pass[read_p_i + 2] >= 1 && pass[read_p_i + 2] <= 30) {//開幕右90°小回り→直進
				motion[motion_count] = 74;
				adjust_straight = -1;
				read_p_i = read_p_i + 2;
			} else {
			}
		}/* else if (pass[read_p_i] == 255 && pass[read_p_i + 1] >= 1	//これはうまく機能しないので除外20171002
				&& pass[read_p_i + 1] <= 30) {	//開幕直進の場合、直進区間を1区間だけ上乗せする。
			pass[read_p_i + 1] = pass[read_p_i + 1] + 1;
			read_p_i++;
		}*/ else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] == 100) {	//右の大廻ターン+停止
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 232;//204
			adjust_straight = -1;
			read_p_i = read_p_i + 2;	//次のpassの読み込み位置
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] == 100) {	//左の大廻ターンの+停止
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 233;//205
			adjust_straight = -1;
			read_p_i = read_p_i + 2;	//
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] == 40
				&& pass[read_p_i + 3] == 100) {	//右の180度ターン+停止
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 234;//214
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] == 50
				&& pass[read_p_i + 3] == 100) {	//左の180度ターン+停止
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 235;//215
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] == 50 && pass[read_p_i + 1] == 40
				&& pass[read_p_i + 2] == 100) {	//斜め→右45°ターン+停止
			motion[motion_count] = 236;//224
			adjust_straight = -1;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] == 40 && pass[read_p_i + 1] == 50
				&& pass[read_p_i + 2] == 100) {	//斜め→左45°ターン+停止
			motion[motion_count] = 237;//225
			adjust_straight = -1;
			read_p_i = read_p_i + 2;
		} else if (pass[read_p_i] == 50 && pass[read_p_i + 1] == 40
				&& pass[read_p_i + 2] == 40 && pass[read_p_i + 3] == 100) {	//斜め→右135°ターン+停止
			motion[motion_count] = 238;//234
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] == 40 && pass[read_p_i + 1] == 50
				&& pass[read_p_i + 2] == 50 && pass[read_p_i + 3] == 100) {	//斜め→左135°ターン+停止
			motion[motion_count] = 239;//235
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 100) {	//直進+停止
			motion[motion_count] = pass[read_p_i] + adjust_straight;
			motion_count++;
			motion[motion_count] = 240;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;	//
		} else if (pass[read_p_i] <= 30 && pass[read_p_i] >= 1) { //上記のいずれにも当てはまらず、ただの直進の場合(これはおそらく無いはず)
			motion[motion_count] = pass[read_p_i] + adjust_straight;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;
		} else {
			read_p_i++;
		}
		if (read_p_i - 1 >= last_p_i) {
			break;
		}
		motion_count++;	//動作のカウント（斜め直線ではカウントアップしない）
	}
	last_p_i = read_p_i - 1;	//変換後のpass[]を最後の値とする

	thin=0;
	read_p_i = 0;
	while (read_p_i != last_p_i) {
		s_count = 0;
		if (motion[read_p_i] == 201) {
			while (1) {
				if (motion[read_p_i + s_count] == 201) {
					s_count++;	//斜めストレートの個数を数えてくれる
				} else {
					break;
				}
			}
			motion[thin] = 200 + s_count;
			read_p_i = read_p_i + s_count - 1;
			thin++;
		} else if (motion[read_p_i] != 0) {
			motion[thin] = motion[read_p_i];
			thin++;
		} else if (motion[read_p_i] == 0) {

		}
		read_p_i++;
	}
	while(1){
		if(thin + j <= last_p_i){
			motion[thin+j]=0;
			j++;
		}else{
			break;
		}
	}

}

void convert_pass() {
	volatile signed short read_p_i = 0, motion_count = 0, adjust_straight = 0, thin = 0, j=0;/*motion_count = 1, //20171002に変更*/

	while (1) {

		if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] <= 30
				&& pass[read_p_i + 2] >= 1) {	//右の大廻ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 114;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;	//次のpassの読み込み位置
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] <= 30
				&& pass[read_p_i + 2] >= 1) {	//左の大廻ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 115;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;	//次のpassの読み込み位置
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40 && pass[read_p_i + 2] == 40
				&& pass[read_p_i + 3] <= 30 && pass[read_p_i + 3] >= 1) {//右の180度ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 124;
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50 && pass[read_p_i + 2] == 50
				&& pass[read_p_i + 3] <= 30 && pass[read_p_i + 3] >= 1) {//左の180度ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 125;
			adjust_straight = -1;
			read_p_i = read_p_i + 3;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 40) {	//直進からの減速右ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 184;
			adjust_straight = 0;
			read_p_i = read_p_i + 2;	//次のpassの読み込み位置

		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 50) {	//直進からの減速左ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 185;
			adjust_straight = 0;
			read_p_i = read_p_i + 2;	//次のpassの読み込み位置

		} else if (pass[read_p_i] == 40 && pass[read_p_i + 1] >= 1
				&& pass[read_p_i + 1] <= 30) {	//直進からの加速右ターン
			motion[motion_count] = 194;
			adjust_straight = -1;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else if (pass[read_p_i] == 50 && pass[read_p_i + 1] >= 1
				&& pass[read_p_i + 1] <= 30) {	//直進からの加速左ターン
			motion[motion_count] = 195;
			adjust_straight = -1;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else if (pass[read_p_i] <= 30 && pass[read_p_i] >= 1) { //上記のいずれにも当てはまらず、ただの直進の場合
			motion[motion_count] = pass[read_p_i] + adjust_straight;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;
//			myprintf("test");
		} else if (pass[read_p_i] == 40) {
			motion[motion_count] = 40;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else if (pass[read_p_i] == 50) {
			motion[motion_count] = 50;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else if (pass[read_p_i] == 255 && pass[read_p_i + 1] <= 30) {	//開幕直進がある場合
			motion[motion_count] = 255;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else if (pass[read_p_i] == 255) {	//開幕直進が無い場合
			motion[motion_count] = 253;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else if (pass[read_p_i] == 100 && pass[read_p_i - 1] <= 30) {	//終幕で直進がある場合
			motion[motion_count] = 100;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else if (pass[read_p_i] == 100) {	//終幕で直進が無い場合
			motion[motion_count] = 103;
			adjust_straight = 0;
			read_p_i = read_p_i + 1;	//次のpassの読み込み位置
		} else {
			read_p_i++;
			myprintf("test2\r\n");
			myprintf("read_p_i=%d\r\n", read_p_i);
			myprintf("notion_count=%d\r\n", motion_count);
		}
		if (read_p_i - 2 >= last_p_i) {
			break;
		}
		motion_count++;
	}
	last_p_i = read_p_i - 1;	//変換後のpass[]を最後の値とする

	thin=0;
	read_p_i = 0;
	while (read_p_i != last_p_i) {
		if (motion[read_p_i] != 0) {
			motion[thin] = motion[read_p_i];
			thin++;
		} else if (motion[read_p_i] == 0) {

		}
		read_p_i++;
	}
	while (1) {
		if (thin + j <= last_p_i) {
			motion[thin + j] = 0;
			j++;
		} else {
			break;
		}
	}

}

void exe_pass_test(float hikisuu_vmax, float hikisuu_accel, char para_mode) {
	volatile char aa, bb, cc;	//パラメータ選択  aa:小回りの速度  bb:大回り、Uターンの速度  cc:
	volatile int read_p_i;
	volatile float vel_low, vel_high, accel_normal;
	read_p_i = 0;

	if (para_mode == 1) {
		vel_low = 500.0;
		vel_high = 800.0;
		accel_normal = 5000.0;
		aa = 0;	//速度500
		bb = 3;	//速度800
	} else if (para_mode == 2) {

	} else if (para_mode == 3) {

	} else if (para_mode == 4) {

	}
	//以下、最初の動作のみ別枠で行う。
	if (motion[read_p_i] >= 1 && motion[read_p_i] <= 30) {
		daikei_for_pass_kai(90.0 * motion[1], vel_high, hikisuu_accel, 0.0,
				vel_high, 1, 0);
	} else if (motion[1] == 74) {	//開幕右小回り→1区間加速
		daikei_for_pass_kai(90.0, vel_low, accel_normal, 0.0, vel_low, 1, 0);
		slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, 13.0, 0.0);//右小回り(後距離0)  ????
		daikei_for_pass_kai(90.0, vel_high, accel_normal, vel_low, vel_high, 1,
				0);	//加速
	} else if (motion[read_p_i] == 84) {	//開幕右45°ターン→斜め

	} else if (motion[read_p_i] == 94) {	//開幕右135°ターン→斜め

	} else if (motion[read_p_i] == 253) {	//斜め無し・開幕ターンあり  学生大会で直した
		daikei_for_pass_kai(90.0, vel_low, accel_normal, 0.0, vel_low, 1, 0);//緩やかに加速する

	} else if (motion[read_p_i] == 255) {	//斜め無し・開幕ターン無し  学生大会で直した
		daikei_for_pass_kai(90.0, vel_high, accel_normal, 0.0, vel_high, 1, 0);

	} else {
	}
	//以下、最初以外の動作
	while (1) {
		if (fail_flag == 1) {	//failセーフ
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			break;	//daikeiの関数を抜けられる
		}

		read_p_i++;
		if ((motion[read_p_i + 1] == 114) || (motion[read_p_i + 1] == 115)
				|| (motion[read_p_i + 1] == 124)
				|| (motion[read_p_i + 1] == 125)
				/*|| motion[read_p_i + 1] == 100*/) {	//次が大回りとUターンの場合
			wait_kabekire = 1;	//壁切れを読むまで待つ(なぜか90区間直進ではフラグが立たない20171001)
		} else {
			wait_kabekire = 0;	//壁切れyomanai
		}
		if (motion[read_p_i] <= 30 && motion[read_p_i] >= 1) {	//数値の区間の半分直進
			daikei_for_pass_kai2(90.0 * motion[read_p_i], hikisuu_vmax,
					hikisuu_accel, vel_high, vel_high, 1, 1);
		} else {

			switch ((char) motion[read_p_i]) {
			/*			case 255:	//最初の90直進(斜め無しpass用で、開幕直進の場合)
			 daikei_for_pass_kai(90.0, vel_high, hikisuu_accel, 0.0,
			 vel_high, 1, 0);	//後のパスに繋げるために区間距離は短くした
			 break;
			 case 253:	//最初の90直進(斜め無しpass用で、開幕何らかのターンの場合)

			 break;*/
			case 0:
				break;
			case 40:	//右小回り
				slalom_2(turn[aa].P_1_0.theta, turn[aa].P_1_0.th1,
						turn[aa].P_1_0.th2, 1000.0, turn[aa].P_1_0.a_cc,
						turn[aa].P_1_0.wise, turn[aa].P_1_0.vel,
						turn[aa].P_1_0.d_f, turn[aa].P_1_0.d_r);	//小回り右スラローム
				break;
			case 50:	//左小回り
				slalom_2(turn[aa].P_1_8.theta, turn[aa].P_1_8.th1,
						turn[aa].P_1_8.th2, 1000.0, turn[aa].P_1_8.a_cc,
						turn[aa].P_1_8.wise, turn[aa].P_1_8.vel,
						turn[aa].P_1_8.d_f, turn[aa].P_1_8.d_r);	//小回り左スラローム
				break;
			case 114:	//右大廻ターン
				turn_for_pass(turn[bb].P_1_1.theta, turn[bb].P_1_1.th1,
						turn[bb].P_1_1.th2, 1000.0, turn[bb].P_1_1.a_cc,
						turn[bb].P_1_1.wise, turn[bb].P_1_1.vel,
						turn[bb].P_1_1.d_f + adjust_before_dist,
						turn[bb].P_1_1.d_r);	//右大廻ターン
				break;
			case 115:	//左大廻ターン
				turn_for_pass(turn[bb].P_1_9.theta, turn[bb].P_1_9.th1,
						turn[bb].P_1_9.th2, 1000.0, turn[bb].P_1_9.a_cc,
						turn[bb].P_1_9.wise, turn[bb].P_1_9.vel,
						turn[bb].P_1_9.d_f + adjust_before_dist,
						turn[bb].P_1_9.d_r);	//左大廻ターン
				break;
			case 124:	//右Uターン
				turn_for_pass(turn[bb].P_1_2.theta, turn[bb].P_1_2.th1,
						turn[bb].P_1_2.th2, 1000.0, turn[bb].P_1_2.a_cc,
						turn[bb].P_1_2.wise, turn[bb].P_1_2.vel,
						turn[bb].P_1_2.d_f + adjust_before_dist,
						turn[bb].P_1_2.d_r);	//右Uターン
				break;
			case 125:	//左Uターン
				turn_for_pass(turn[bb].P_1_10.theta, turn[bb].P_1_10.th1,
						turn[bb].P_1_10.th2, 1000.0, turn[bb].P_1_10.a_cc,
						turn[bb].P_1_10.wise, turn[bb].P_1_10.vel,
						turn[bb].P_1_10.d_f + adjust_before_dist,
						turn[bb].P_1_10.d_r);	//右Uターン
				break;
			case 134:	//右45°ターン→斜め
				break;
			case 135:	//左45°ターン→斜め
				break;
			case 144:	//右135°ターン→斜め
				break;
			case 145:	//左135°ターン→斜め
				break;
			case 154:	//斜め→右45°ターン
				break;
			case 155:	//斜め→左45°ターン
				break;
			case 164:	//斜め→右135°ターン
				break;
			case 165:	//斜め→左135°ターン
				break;
			case 174:	//斜め→右V90°ターン
				break;
			case 175:	//斜め→左V90°ターン
				break;
			case 184:	//減速+右ターン
				daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
						vel_high, vel_low, 1, 1);	//(壁切れyomu)
				slalom_2(turn[aa].P_1_0.theta, turn[aa].P_1_0.th1,
						turn[aa].P_1_0.th2, 1000.0, turn[aa].P_1_0.a_cc,
						turn[aa].P_1_0.wise, turn[aa].P_1_0.vel,
						turn[aa].P_1_0.d_f, turn[aa].P_1_0.d_r);	//右小回り
				break;
			case 185:	//減速+左ターン
				daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
						vel_high, vel_low, 1, 1);	//(壁切れをyomu)
				slalom_2(turn[aa].P_1_8.theta, turn[aa].P_1_8.th1,
						turn[aa].P_1_8.th2, 1000.0, turn[aa].P_1_8.a_cc,
						turn[aa].P_1_8.wise, turn[aa].P_1_8.vel,
						turn[aa].P_1_8.d_f, turn[aa].P_1_8.d_r);	//左小回り
				break;
			case 194:	//右ターン+加速
				slalom_2(turn[aa].P_1_0.theta, turn[aa].P_1_0.th1,
						turn[aa].P_1_0.th2, 1000.0, turn[aa].P_1_0.a_cc,
						turn[aa].P_1_0.wise, turn[aa].P_1_0.vel,
						turn[aa].P_1_0.d_f, turn[aa].P_1_0.d_r);	//右小回り
				daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, vel_low,
						vel_high, 1, 1);
				break;
			case 195:	//左ターン+加速
				slalom_2(turn[aa].P_1_8.theta, turn[aa].P_1_8.th1,
						turn[aa].P_1_8.th2, 1000.0, turn[aa].P_1_8.a_cc,
						turn[aa].P_1_8.wise, turn[aa].P_1_8.vel,
						turn[aa].P_1_8.d_f, turn[aa].P_1_8.d_r);	//左小回り
				daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, vel_low,
						vel_high, 1, 1);
				break;
			case 232:	//右大廻ターン+停止
				break;
			case 233:	//左大廻ターン+停止
				break;
			case 234:	//右Uターン+停止
				break;
			case 235:	//左Uターン+停止
				break;
			case 236:	//斜め→右45°ターン+停止
				break;
			case 237:	//斜め→左45°ターン+停止
				break;
			case 238:	//斜め→右135°ターン+停止
				break;
			case 239:	//斜め→左135°ターン+停止
				break;
			case 240:   //停止(90区間で最大限減速する)
				break;
			case 100:   //停止(90区間で)(斜め無しpass用)
				daikei_for_pass_kai2(90.0, vel_high, accel_normal, vel_high,
						0.0, 1, 0);
//				myprintf("test100\r\n");
				break;
			case 103:   //停止(90区間で)(斜め無しpass用)
				daikei_for_pass_kai2(90.0, vel_low, accel_normal, vel_low, 0.0,
						1, 0);
				break;
			}
		}
		if (read_p_i == last_p_i) {

			break;
		}
	}
//	if (motion[last_p_i] == 154 || motion[last_p_i] == 155
//			|| motion[last_p_i] == 164 || motion[last_p_i] == 165) {
//		daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 500.0, 0.0, 1, 0);//低速からの直進90
//	} else if (motion[last_p_i] == 184 || motion[last_p_i] == 185) {
//		daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 500.0, 0.0, 1, 0);//低速からの直進90
//	} else {
//		daikei_for_pass_kai(90.0, vel_high, hikisuu_accel, vel_high, 0.0, 1,
//				0);	//高速からの90直進
//	}
	wall_control = 0;	//ゴールで誤動作しないようにするため
	ideal_omega = 0.0;	//ゴールで誤動作しないようにするため
	ideal_balance_velocity = 0.0;
	wait(500);
	GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
	sensor_enable = 0;			//センサ切り
}

void exe_pass_kai(float hikisuu_vmax, float hikisuu_v, float hikisuu_accel,
		char hikisuu_mode) {
	volatile int read_p_j;
//	volatile float h_angle1_r, h_angle2_r, h_angacc_r, h_before_r, h_after_r,
//			h_before_l, h_after_l, h_vel;
//	if (hikisuu_mode == 1) {
//		h_angle1_r = 30.0;
//		h_angle2_r = 90.0 - 30.0;
//		h_angacc_r = 5500.0;
//		h_before_r = 13.0;
//		h_after_r = 12.0;
//		h_before_l = 5.0;
//		h_after_l = 8.0;
//		h_vel = 650.0;
//	} else if (hikisuu_mode = 2) {
//		h_angle1 = 30.0;
//		h_angle2 = 90.0 - 30.0;
//		h_angacc = 7500.0;
//		h_before_r = 5.0;
//		h_after_r = 8.0;
//		h_before_l = 5.0;
//		h_after_l = 8.0;
//		h_vel = 650.0;
//	}
	read_p_j = 0;
	//以下、最初の動作のみ別枠で行う。
	if (motion[read_p_j] >= 1 && motion[read_p_j] <= 30) {
		daikei_for_pass_kai(90.0 * motion[1], hikisuu_v, hikisuu_accel, 0.0,
				hikisuu_v, 1, 0);
	} else if (motion[read_p_j] == 74) {	//開幕右小回り→1区間加速	(???)
		daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 0.0, 500.0, 1, 0);
		slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, 13.0, 0.0);//右小回り(後距離0)  ???
//		test_slalom(90.0, 650.0, 4500.0, -1.0, 500.0, 8.5, 10.5);	//右折　保存
		daikei_for_pass_kai(90.0, hikisuu_v, hikisuu_accel, 500.0, hikisuu_v, 1,
				0);	//加速
	} else if (motion[read_p_j] == 84) {	//開幕右45°ターン→斜め

	} else if (motion[read_p_j] == 94) {	//開幕右135°ターン→斜め

	} else if (motion[read_p_j] == 253) {	//学生大会で追加
		if (hikisuu_mode == 1) {
			daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 0.0, 500.0, 1, 0);
		} else if (hikisuu_mode == 2) {
			daikei_for_pass_kai(90.0, 650.0, hikisuu_accel, 0.0, 650.0, 1, 0);
		} else if (hikisuu_mode == 3) {
			daikei_for_pass_kai(90.0, 750.0, hikisuu_accel, 0.0, 750.0, 1, 0);
		}
	} else if (motion[read_p_j] == 255) {
		daikei_for_pass_kai(90.0, hikisuu_v, hikisuu_accel, 0.0, hikisuu_v, 1,
				0);	//後のパスに繋げるために区間距離は短くした
	}
	//以下、最初以外の動作
	while (1) {
		if (fail_flag == 1) {	//failセーフ
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			break;	//daikeiの関数を抜けられる
		}

		read_p_j++;
		if ((motion[read_p_j + 1] == 114) || (motion[read_p_j + 1] == 115)
				|| (motion[read_p_j + 1] == 124)
				|| (motion[read_p_j + 1] == 125)
				/*|| motion[read_p_j + 1] == 100*/) {	//次が大回りとUターンの場合
			wait_kabekire = 1;	//壁切れを読むまで待つ
		} else {
			wait_kabekire = 0;	//壁切れyomanai
		}
		if (motion[read_p_j] <= 30 && motion[read_p_j] >= 1) {	//数値の区間の半分直進
			daikei_for_pass_kai2(90.0 * motion[read_p_j], hikisuu_vmax,
					hikisuu_accel, hikisuu_v, hikisuu_v, 1, 1);
//			myprintf("test0\r\n");
		} else {

			switch ((char) motion[read_p_j]) {
			case 255:	//最初の90直進(斜め無しpass用で、開幕直進の場合)
//				daikei_for_pass_kai(90.0, 800.0, 4000.0, 0.0, 800.0, 0);
				daikei_for_pass_kai(90.0, hikisuu_v, hikisuu_accel, 0.0,
						hikisuu_v, 1, 0);	//後のパスに繋げるために区間距離は短くした
//				myprintf("test255\r\n");
				break;
			case 253:	//最初の90直進(斜め無しpass用で、開幕何らかのターンの場合)
				if (hikisuu_mode == 1) {
					daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 0.0, 500.0,
							1, 0);
				} else if (hikisuu_mode == 2) {
					daikei_for_pass_kai(90.0, 650.0, hikisuu_accel, 0.0, 650.0,
							1, 0);
				} else if (hikisuu_mode == 3) {
					daikei_for_pass_kai(90.0, 750.0, hikisuu_accel, 0.0, 750.0,
							1, 0);
				}
				break;
			case 0:
				break;
			case 40:	//右折
				if (hikisuu_mode == 1) {
					slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, 13.0,
							12.0);	//右小回り
				} else if (hikisuu_mode == 2) {
					slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0,
							8.0);		//右小回り(速め)
				} else if (hikisuu_mode == 3) {
					slalom_2(90.0, 41.0, 49.0, 2000.0, 9240.0, -1.0, 750.0, 0.0,
							18.0);	//右小回り(3)
				}
				break;
			case 50:	//左折
				if (hikisuu_mode == 1) {
					slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, 1.0, 500.0, 5.0,
							22.0);	//左小回り
				} else if (hikisuu_mode == 2) {
					slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0,
							25.0);		//左小回り(速め)
				} else if (hikisuu_mode == 3) {
					slalom_2(90.0, 40.0, 50.0, 2000.0, 10050.0, 1.0, 750.0, 3.8,
							22.0);	//左小回り(3)
				}

				break;
			case 114:	//右大廻ターン
				turn_for_pass(90.0, 40.0, 50.0, 1000.0, 4300.0, -1.0, 800.0,
						43.0 + adjust_before_dist, 47.0);	//右大回り
//				slalom_2(90.0, 40.0, 50.0, 1000.0, 4300.0, -1.0, 800.0, 35.0,
//						45.0);	//右大回り
				break;
			case 115:	//左大廻ターン
				turn_for_pass(90.0, 40.0, 50.0, 1000.0, 4300.0, 1.0, 800.0,
						45.0 + adjust_before_dist, 62.0);	//左大回り
//				slalom_2(90.0, 40.0, 50.0, 1000.0, 4300.0, 1.0, 800.0, 30.0,
//						50.0);	//左大回り
				break;
			case 124:	//右Uターン
				turn_for_pass(180.0, 27.5, 152.5, 1000.0, 4450.0, -1.0, 800.0,
						15.0 + adjust_before_dist + 10.0, 22.0 + 10.0);	//右Uターン
//				slalom_2(180.0, 27.5, 152.5, 1000.0, 4450.0, -1.0, 800.0, 0.0,
//						14.0);	//右Uターン
				break;
			case 125:	//左Uターン
				turn_for_pass(180.0, 28.5, 151.5, 1000.0, 4300.0, 1.0, 800.0,
						15.0 + adjust_before_dist + 10.0, 22.0 + 10.0);	//左Uターン
//				slalom_2(180.0, 28.5, 151.5, 1000.0, 4300.0, 1.0, 800.0, 0.0,
//						15.0);	//左Uターン
				break;
			case 134:	//右45°ターン→斜め
				break;
			case 135:	//左45°ターン→斜め
				break;
			case 144:	//右135°ターン→斜め
				break;
			case 145:	//左135°ターン→斜め
				break;
			case 154:	//斜め→右45°ターン
				break;
			case 155:	//斜め→左45°ターン
				break;
			case 164:	//斜め→右135°ターン
				break;
			case 165:	//斜め→左135°ターン
				break;
			case 174:	//斜め→右V90°ターン
				break;
			case 175:	//斜め→左V90°ターン
				break;
			case 184:	//減速+右ターン
				if (hikisuu_mode == 1) {
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
							hikisuu_v, 500.0, 1, 1);	//(壁切れyomu)
					slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, 13.0,
							12.0);	//右小回り
				} else if (hikisuu_mode == 2) {
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
							hikisuu_v, 650.0, 1, 1);	//(壁切れをyomu)
					slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0,
							8.0);		//右小回り(速め)
				} else if (hikisuu_mode == 3) {
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
							hikisuu_v, 750.0, 1, 1);	//(壁切れをyomu)
					slalom_2(90.0, 41.0, 49.0, 2000.0, 9240.0, -1.0, 750.0, 0.0,
							18.0);	//右小回り(3)
				}

				break;
			case 185:	//減速+左ターン
				if (hikisuu_mode == 1) {
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
							hikisuu_v, 500.0, 1, 1);	//(壁切れをyomanai)
					slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, 1.0, 500.0, 5.0,
							22.0);	//左小回り
				} else if (hikisuu_mode == 2) {
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
							hikisuu_v, 650.0, 1, 1);	//(壁切れをyomanai)
					slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0,
							25.0);		//左小回り(速め)
				} else if (hikisuu_mode == 3) {
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel,
							hikisuu_v, 750.0, 1, 1);	//(壁切れをyomanai)
					slalom_2(90.0, 40.0, 50.0, 2000.0, 10050.0, 1.0, 750.0, 3.8,
							22.0);	//左小回り(3)
				}
				break;
			case 194:	//右ターン+加速
				if (hikisuu_mode == 1) {
					slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, 13.0,
							12.0);	//右小回り
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, 500,
							hikisuu_v, 1, 1);
				} else if (hikisuu_mode == 2) {
					slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0,
							8.0);		//右小回り(速め)
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, 650,
							hikisuu_v, 1, 1);
				} else if (hikisuu_mode == 3) {
					slalom_2(90.0, 41.0, 49.0, 2000.0, 9240.0, -1.0, 750.0, 0.0,
							18.0);	//右小回り(3)
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, 750,
							hikisuu_v, 1, 1);
				}
				break;
			case 195:	//左ターン+加速
				if (hikisuu_mode == 1) {
					slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, 1.0, 500.0, 5.0,
							22.0);	//左小回り
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, 500,
							hikisuu_v, 1, 1);
				} else if (hikisuu_mode == 2) {
					slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0,
							25.0);		//左小回り(速め)
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, 650,
							hikisuu_v, 1, 1);
				} else if (hikisuu_mode == 3) {
					slalom_2(90.0, 40.0, 50.0, 2000.0, 10050.0, 1.0, 750.0, 3.8,
							22.0);	//左小回り(3)
					daikei_for_pass_kai2(90.0, hikisuu_vmax, hikisuu_accel, 750,
							hikisuu_v, 1, 1);
				}
				break;
			case 232:	//右大廻ターン+停止
				break;
			case 233:	//左大廻ターン+停止
				break;
			case 234:	//右Uターン+停止
				break;
			case 235:	//左Uターン+停止
				break;
			case 236:	//斜め→右45°ターン+停止
				break;
			case 237:	//斜め→左45°ターン+停止
				break;
			case 238:	//斜め→右135°ターン+停止
				break;
			case 239:	//斜め→左135°ターン+停止
				break;
			case 240:   //停止(90区間で最大限減速する)
				break;
			case 100:   //停止(90区間で)(斜め無しpass用)
				daikei_for_pass_kai(90.0, hikisuu_v, hikisuu_accel, hikisuu_v,
						0.0, 1, 0);
//				myprintf("test100\r\n");
				break;
			case 103:   //停止(90区間で)(斜め無しpass用)
				if (hikisuu_mode == 1) {
					daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 500.0, 0.0,
							0, 0);
				} else if (hikisuu_mode == 2) {
					daikei_for_pass_kai(90.0, 650.0, hikisuu_accel, 650.0, 0.0,
							0, 0);
				} else if (hikisuu_mode == 3) {
					daikei_for_pass_kai(90.0, 750.0, hikisuu_accel, 750.0, 0.0,
							0, 0);
				}
				break;
			}
		}
		if (read_p_j == last_p_i) {

			break;
		}
	}
//	if (motion[last_p_i] == 154 || motion[last_p_i] == 155
//			|| motion[last_p_i] == 164 || motion[last_p_i] == 165) {
//		daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 500.0, 0.0, 1, 0);//低速からの直進90
//	} else if (motion[last_p_i] == 184 || motion[last_p_i] == 185) {
//		daikei_for_pass_kai(90.0, 500.0, hikisuu_accel, 500.0, 0.0, 1, 0);//低速からの直進90
//	} else {
//		daikei_for_pass_kai(90.0, hikisuu_v, hikisuu_accel, hikisuu_v, 0.0, 1,
//				0);	//高速からの90直進
//	}
	wall_control = 0;	//ゴールで誤動作しないようにするため
	ideal_omega = 0.0;	//ゴールで誤動作しないようにするため
	ideal_balance_velocity = 0.0;
	wait(500);
	GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
	sensor_enable = 0;			//センサ切り
}

void q_walk_map_maker(int hikisuu_goal_x, int hikisuu_goal_y) {
	volatile unsigned short qx, qy, head, tail, q[257];
	q_dist_flag = 1;
	for (qx = 0; qx <= x_size; qx++)		// マップの初期化,255の代入
			{
		for (qy = 0; qy <= y_size; qy++) {
			map[qx][qy] = 255;
		}
	}
	map[hikisuu_goal_x][hikisuu_goal_y] = 0;			// 目標地点に距離０を書き込む
	q[0] = (hikisuu_goal_x * 16 + hikisuu_goal_y);		// 目標地点の座標を記憶
	head = 0;							// 先頭位置を初期化
	tail = 1;							// 末尾位置は、最後の情報位置＋１

	while (head != tail)				// 配列の中身が空ならループを抜ける（更新できないとループを抜ける）
	{
		qy = q[head] & 0x0f;       		// 配列から区画の座標を取り出す
		qx = q[head] >> 4;
		head++;							// 情報を取り出したので先頭位置をずらす

		if (qy < y_size)							// 北側
		{
			if (is_Exist_Wall(qx, qy, 0) == 0)	//北に壁がない場合
					{
				if (map[qx][qy + 1] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx][qy + 1] = map[qx][qy] + 1;
					q[tail] = (qx * 16 + qy + 1);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす

				}
			}
		}
		if (qx < x_size)     // 東側
		{
			if (is_Exist_Wall(qx, qy, 1) == 0)	//東に壁がない場合
					{
				if (map[qx + 1][qy] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx + 1][qy] = map[qx][qy] + 1;
					q[tail] = ((qx + 1) * 16 + qy);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす

				}
			}
		}
		if (qy > 0)     // 南側
				{
			if (is_Exist_Wall(qx, qy, 2) == 0)	//南に壁がない場合
					{
				if (map[qx][qy - 1] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx][qy - 1] = map[qx][qy] + 1;
					q[tail] = (qx * 16 + qy - 1);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす

				}
			}
		}
		if (qx > 0)     // 西側
				{
			if (is_Exist_Wall(qx, qy, 3) == 0)	//西に壁がない場合
					{
				if (map[qx - 1][qy] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx - 1][qy] = map[qx][qy] + 1;
					q[tail] = ((qx - 1) * 16 + qy);     // 次の区画の座標を記憶
					tail++;     // 情=報を入れたので末尾位置をずらす

				}
			}
		}
	}

}

void q_saved_walk_map_maker(int hikisuu_goal_x, int hikisuu_goal_y) {
	volatile unsigned short qx, qy, head, tail, q[257];
	q_dist_flag = 0;
	for (qx = 0; qx <= x_size; qx++)		// マップの初期化,255の代入
			{
		for (qy = 0; qy <= y_size; qy++) {
			map[qx][qy] = 255;
		}
	}
	map[hikisuu_goal_x][hikisuu_goal_y] = 0;			// 目標地点に距離０を書き込む
	q[0] = (hikisuu_goal_x * 16 + hikisuu_goal_y);		// 目標地点の座標を記憶
	head = 0;							// 先頭位置を初期化
	tail = 1;							// 末尾位置は、最後の情報位置＋１

	while (head != tail)				// 配列の中身が空ならループを抜ける（更新できないとループを抜ける）
	{
		qy = q[head] & 0x0f;       		// 配列から区画の座標を取り出す
		qx = q[head] >> 4;
		head++;							// 情報を取り出したので先頭位置をずらす

		if (qy < y_size)							// 北側
		{
			if (is_saved_wall_exist(qx, qy, 0) == 0)	//北に壁がない場合
					{
				if (map[qx][qy + 1] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx][qy + 1] = map[qx][qy] + 1;
					q[tail] = (qx * 16 + qy + 1);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす
				}
			}
		}
		if (qx < x_size)     // 東側
		{
			if (is_saved_wall_exist(qx, qy, 1) == 0)	//東に壁がない場合
					{
				if (map[qx + 1][qy] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx + 1][qy] = map[qx][qy] + 1;
					q[tail] = ((qx + 1) * 16 + qy);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす
				}
			}
		}
		if (qy > 0)     // 南側
				{
			if (is_saved_wall_exist(qx, qy, 2) == 0)	//南に壁がない場合
					{
				if (map[qx][qy - 1] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx][qy - 1] = map[qx][qy] + 1;
					q[tail] = (qx * 16 + qy - 1);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす
				}
			}
		}
		if (qx > 0)     // 西側
				{
			if (is_saved_wall_exist(qx, qy, 3) == 0)	//西に壁がない場合
					{
				if (map[qx - 1][qy] == 255)	//さらにマップが過去に更新されていない場合
						{
					map[qx - 1][qy] = map[qx][qy] + 1;
					q[tail] = ((qx - 1) * 16 + qy);     // 次の区画の座標を記憶
					tail++;     // 情=報を入れたので末尾位置をずらす
				}
			}
		}
	}

}

void skew_queue_walkmap_maker(char hikisuu_goal_x, char hikisuu_goal_y) {
	volatile unsigned short qx, qy, head = 0, tail = 0, max = 65535;
	volatile unsigned char jj, kk, level_or_vertical = 0;
	volatile unsigned int count;

	for (jj = 0; jj <= 15; jj++)		// マップの初期化,maxの代入
			{
		for (kk = 0; kk <= 14; kk++) {		// 壁が存在するところにはmaxを代入
			level[jj][kk] = max;
			vertical[kk][jj] = max;
			if (is_saved_wall_exist(kk, jj, 1) == 0) {	//東を読む
				vertical[kk][jj] = max - 1;
			}
			if (is_saved_wall_exist(jj, kk, 0) == 0) {
				level[jj][kk] = max - 1;
			}
		}
	}

	if (is_saved_wall_exist(hikisuu_goal_x, hikisuu_goal_y, 0) == 0) {	//北
		level[hikisuu_goal_x][hikisuu_goal_y] = 0;
		queue[head] = 0 * 256 + hikisuu_goal_x * 16 + hikisuu_goal_y;//levelに代入するので先頭は0
		head++;
	}
	if (is_saved_wall_exist(hikisuu_goal_x, hikisuu_goal_y, 1) == 0) {	//東
		vertical[hikisuu_goal_x][hikisuu_goal_y] = 0;
		queue[head] = 1 * 256 + hikisuu_goal_x * 16 + hikisuu_goal_y;//verticalに代入するので1
		head++;
	}
	if (is_saved_wall_exist(hikisuu_goal_x, hikisuu_goal_y, 2) == 0) {	//南
		level[hikisuu_goal_x][hikisuu_goal_y - 1] = 0;
		queue[head] = 0 * 256 + hikisuu_goal_x * 16 + (hikisuu_goal_y - 1);	//levelに代入するので0
		head++;
	}
	if (is_saved_wall_exist(hikisuu_goal_x, hikisuu_goal_y, 3) == 0) {	//西
		vertical[hikisuu_goal_x - 1][hikisuu_goal_y] = 0;
		queue[head] = 1 * 256 + (hikisuu_goal_x - 1) * 16 + hikisuu_goal_y;	//verticalに代入するので1
		head++;
	}
	myprintf("queue[0]=%d\n\r", queue[0]);
	myprintf("queue[1]=%d\n\r", queue[1]);
	myprintf("queue[2]=%d\n\r", queue[2]);
	myprintf("queue[3]=%d\n\r", queue[3]);

	tail = head + 1;		//末尾位置は、最後の情報位置＋１
	head = 0;				//先頭は0から読み込み開始
	count = 0;
	while (head != tail)				// 配列の中身が空ならループを抜ける（更新できないとループを抜ける）
	{
		qy = queue[head] & 0x0f;       		// 配列から区画の座標を取り出す
		qx = (queue[head] >> 4) & 0x0f;
		level_or_vertical = queue[head] >> 8;
		head++;							// 情報を取り出したので先頭位置をずらす
		myprintf("qx=%d\n\r", qx);
		myprintf("qy=%d\n\r", qy);
		myprintf("level_or_vertical=%d\n\r", level_or_vertical);
		myprintf("count=%d\n\r", count);
		count++;

		if (level_or_vertical == 0) {							// levelを読み込む場合
//以下、壁ではないかつ代入しようとした点の歩数が代入値よりも大きい場合
			if (qy < 14 && level[qx][qy + 1] > level[qx][qy] + 7
					&& level[qx][qy + 1] < max) {				//北側のlevelを判定
				level[qx][qy + 1] = level[qx][qy] + 7;
				queue[tail] = 0 * 256 + qx * 16 + (qy + 1);
				tail++;
			}
			if (qy > 0 && level[qx][qy - 1] > level[qx][qy] + 7
					&& level[qx][qy - 1] < max) {				//南側のlevelを判定
				level[qx][qy - 1] = level[qx][qy] + 7;
				queue[tail] = 0 * 256 + qx * 16 + (qy - 1);
				tail++;
			}
			if (qx < 15 && qy < 15 && vertical[qx][qy + 1] > level[qx][qy] + 5
					&& vertical[qx][qy + 1] < max) {		//北東側のverticalを判定
				vertical[qx][qy + 1] = level[qx][qy] + 5;
				queue[tail] = 1 * 256 + qx * 16 + (qy + 1);
				tail++;
			}
			if (qx > 0 && qy < 15
					&& vertical[qx - 1][qy + 1] > level[qx][qy] + 5
					&& vertical[qx - 1][qy + 1] < max) {	//北西側のverticalを判定
				vertical[qx - 1][qy + 1] = level[qx][qy] + 5;
				queue[tail] = 1 * 256 + (qx - 1) * 16 + (qy + 1);
				tail++;
			}
			if (qx < 15 && vertical[qx][qy] > level[qx][qy] + 5
					&& vertical[qx][qy] < max) {		//南東側のverticalを判定
				vertical[qx][qy] = level[qx][qy] + 5;
				queue[tail] = 1 * 256 + qx * 16 + (qy);
				tail++;
			}
			if (qx > 0 && vertical[qx - 1][qy] > level[qx][qy] + 5
					&& vertical[qx - 1][qy] < max) {	//南西側のverticalを判定
				vertical[qx - 1][qy] = level[qx][qy] + 5;
				queue[tail] = 1 * 256 + (qx - 1) * 16 + (qy);
				tail++;
			}

		} else if (level_or_vertical == 1) {				// verticalを読み込む場合
//以下、壁ではないかつ代入しようとした点の歩数が代入値よりも大きい場合
			if (qx < 14 && vertical[qx + 1][qy] > vertical[qx][qy] + 7
					&& vertical[qx + 1][qy] < max) {			//東側のverticalを判定
				vertical[qx + 1][qy] = vertical[qx][qy] + 7;
				queue[tail] = 1 * 256 + (qx + 1) * 16 + (qy);
				tail++;
			}
			if (qx > 0 && vertical[qx - 1][qy] > vertical[qx][qy] + 7
					&& vertical[qx - 1][qy] < max) {			//西側のverticalを判定
				vertical[qx - 1][qy] = vertical[qx][qy] + 7;
				queue[tail] = 1 * 256 + (qx - 1) * 16 + (qy);
				tail++;
			}
			if (qx < 15 && qy < 15 && level[qx + 1][qy] > vertical[qx][qy] + 5
					&& level[qx + 1][qy] < max) {		//北東側のlevelを判定
				level[qx + 1][qy] = vertical[qx][qy] + 5;
				queue[tail] = 0 * 256 + (qx + 1) * 16 + (qy);
				tail++;
			}
			if (qx < 15 && qy < 15 && level[qx][qy] > vertical[qx][qy] + 5
					&& level[qx][qy] < max) {		//北西側のlevelを判定
				level[qx][qy] = vertical[qx][qy] + 5;
				queue[tail] = 0 * 256 + (qx) * 16 + (qy);
				tail++;
			}
			if (qx < 15 && qy > 0
					&& level[qx + 1][qy - 1] > vertical[qx][qy] + 5
					&& level[qx + 1][qy - 1] < max) {		//南東側のlevelを判定
				level[qx + 1][qy - 1] = vertical[qx][qy] + 5;
				queue[tail] = 0 * 256 + (qx + 1) * 16 + (qy - 1);
				tail++;
			}
			if (qx < 15 && qy > 0 && level[qx][qy - 1] > vertical[qx][qy] + 5
					&& level[qx][qy - 1] < max) {		//南西側のlevelを判定
				level[qx][qy - 1] = vertical[qx][qy] + 5;
				queue[tail] = 0 * 256 + (qx) * 16 + (qy - 1);
				tail++;
			}

		}
	}
}

void unknown_WALL_add() {		//パス用に,saved mazeに未探索壁を入れた
	volatile int qy, qx;
	for (qy = 0; qy < y_size; qy++) {
		for (qx = 0; qx < x_size; qx++) {
			if (is_saved_wall_watched(qx, qy, 0) == 0) {
				add_saved_wall_front(qx, qy, 0);
			}
			if (is_saved_wall_watched(qx, qy, 1) == 0) {
				add_saved_wall_right(qx, qy, 0);
			}
//			if (is_the_Wall_watched(qx, qy, 0) == 0) {
//				add_wall_front(qx, qy, 0);
//			}
//			if (is_the_Wall_watched(qx, qy, 1) == 0) {
//				add_wall_right(qx, qy, 0);
//			}
		}
	}
	for(qy = 0; qy < y_size ;qy++){	//右端の一列の横壁を判定する
		qx = x_size;
		if (is_saved_wall_watched(qx, qy, 0) == 0) {
			add_saved_wall_front(qx, qy, 0);
		}
	}
	for(qx = 0; qx < x_size ;qx++){	//上端の一行の縦壁を判定する
		qy = y_size;
		if (is_saved_wall_watched(qx, qy, 1) == 0) {
			add_saved_wall_right(qx, qy, 0);
		}
	}

}
void unknown_WALL_remove() {		//パス用に入れたsaved mazeの未探索壁を再び取り除いて探索へ！
//	watched_wall_front(0, 0, 0);		//足立法内で行っている
//	watched_wall_front(0, 0, 1);		//足立法内で行っている
	volatile int qy, qx, remove_row, remove_column;
	for (qy = 0; qy < y_size; qy++) {
		for (qx = 0; qx < x_size; qx++) {
			remove_row = 32768;//先頭のビットが1
			remove_column = 1;//末尾のビットが1
//			l = ~32768 >> qx;
			if (is_the_Wall_watched(qx, qy, 0) == 0) {	//北壁判断
//				row_temp[qy] = row_temp[qy] & ~remove_row >> qx;
				row_fix[qy] = row_fix[qy] & ~remove_row >> qx;

			}
			if (is_the_Wall_watched(qx, qy, 1) == 0) {	//東壁判断
				remove_column = remove_column << qy;
				remove_column = ~remove_column;
//				column_temp[qx] = column_temp[qx] & remove_column;
				column_fix[qx] = column_fix[qx] & remove_column;
			}
		}
	}
	for(qy = 0; qy < y_size ;qy++){	//右端の一列の横壁を判定する
		qx = x_size;
		if (is_the_Wall_watched(qx, qy, 0) == 0) {
//			row_temp[qy] = row_temp[qy] & ~remove_row >> qx;
			row_fix[qy] = row_fix[qy] & ~remove_row >> qx;
		}
	}
	for(qx = 0; qx < x_size ;qx++){	//上端の一行の縦壁を判定する
		qy = y_size;
		if (is_the_Wall_watched(qx, qy, 1) == 0) {
			remove_column = remove_column << qy;
			remove_column = ~remove_column;
//			column_temp[qx] = column_temp[qx] & remove_column;
			column_fix[qx] = column_fix[qx] & remove_column;
		}
	}


//		for (qx = 0; qx <= x_size; qx++) {
//			colum_watched[qx] = ~colum_watched[qx];
//			row_watched[qx] = ~row_watched[qx];
//			colum[qx] = colum[qx] & colum_watched[qx];
//			row[qx] = row[qx] & row_watched[qx];
//		}
}

void make_parameter() {

}

void test_wall_control() {
//	LED_V1 = 0;	//debug
//	LED_V2 = 0;	//debug
//	LED_V3 = 0;	//debug

	if ((sen.right_side > r_threshold) && (sen.left_side > l_threshold)) {//両壁ありの制御
//		Error = (float) ((sen.left_side - sen_left_refer)		//左に寄るとErrorは正
//		- (sen.right_side - sen_right_refer));
		if (fabs(diff_average_sensor.right) >= DIFF_THRESHOLD
				&& fabs(diff_average_sensor.left) < DIFF_THRESHOLD) {//吸い込まれ対策,右壁の変化大のとき右制御を切る
			Error = (float) (2.0 * (sen.left_side - sen_left_refer));
//			LED_V1 = 1;	//debug
		} else if (fabs(diff_average_sensor.left) >= DIFF_THRESHOLD
				&& fabs(diff_average_sensor.right) < DIFF_THRESHOLD) {//吸い込まれ対策,左壁の変化大のとき左制御を切る
			Error = (float) (-2.0 * (sen.right_side - sen_right_refer));
//			LED_V2 = 1;	//debug
		} else if (fabs(diff_average_sensor.right) < DIFF_THRESHOLD
				&& fabs(diff_average_sensor.left) < DIFF_THRESHOLD) {//両壁ともに安定の場合
			Error = (float) ((sen.left_side - sen_left_refer)
					- (sen.right_side - sen_right_refer));
		} else if (fabs(diff_average_sensor.right) >= -1.0 * DIFF_THRESHOLD
				&& fabs(diff_average_sensor.left) >= -1.0 * DIFF_THRESHOLD) {//両壁切れるor入る場合
			Error = 0.0;
		} else {
			Error = 0.0;	//必要ないはずだが一応
		}

	} else if ((sen.right_side > r_threshold)
			&& (sen.left_side <= l_threshold)) {	//右壁ありの制御

		if (fabs(diff_average_sensor.right) >= DIFF_THRESHOLD) {//吸い込まれ対策,右壁の変化大のとき右制御を切る
			Error = 0.0;
//			LED_V2 = 1;	//debug
		} else if (fabs(diff_average_sensor.right) < DIFF_THRESHOLD) {//右の変化量が小さいので右を使う
			Error = (float) (-2.0 * (sen.right_side - sen_right_refer));
//			LED_V3 = 1;	//debug
		} else {
			Error = 0.0;	//必要ないはずだが一応
		}

	} else if ((sen.right_side <= r_threshold)
			&& (sen.left_side > l_threshold)) {		//左壁ありの制御

		if (fabs(diff_average_sensor.left) >= DIFF_THRESHOLD) {	//吸い込まれ対策,左壁の変化大のとき左制御を切る
			Error = 0.0;
//			LED_V2 = 1;
		} else if (fabs(diff_average_sensor.left) < DIFF_THRESHOLD) {//左の変化量が小さいので、左を使う
			Error = (float) (2 * (sen.left_side - sen_left_refer));
//			LED_V3 = 1;	//debug
		} else {
			Error = 0.0;	//必要ないはずだが一応
		}
	} else {	//両壁無い場合の制御

		Error = 0.0;

	}

}

void sensor_average(char number1, char number2) {	//number1はバッファの個数. number2は
	char ii;
	buff_sen_right[0] = sen.right_side;
	buff_sen_left[0] = sen.left_side;

	sen_count++;
	if (sen_count == number2) {	//diffを計算するときの間隔を調整する
		//以下、壁切れの判断用の変数(確実に必要かは不明ー壁切れをdiffで検知するなら不要)
		pre_buff_sen_right[9] = pre_buff_sen_right[8];
		pre_buff_sen_right[8] = pre_buff_sen_right[7];
		pre_buff_sen_right[7] = pre_buff_sen_right[6];
		pre_buff_sen_right[6] = pre_buff_sen_right[5];
		pre_buff_sen_right[5] = pre_buff_sen_right[4];
		pre_buff_sen_right[4] = pre_buff_sen_right[3];
		pre_buff_sen_right[3] = pre_buff_sen_right[2];
		pre_buff_sen_right[2] = pre_buff_sen_right[1];
		pre_buff_sen_right[1] = pre_buff_sen_right[0];
		pre_buff_sen_right[0] = average_sensor.right;
		pre_buff_sen_left[9] = pre_buff_sen_left[8];
		pre_buff_sen_left[8] = pre_buff_sen_left[7];
		pre_buff_sen_left[7] = pre_buff_sen_left[6];
		pre_buff_sen_left[6] = pre_buff_sen_left[5];
		pre_buff_sen_left[5] = pre_buff_sen_left[4];
		pre_buff_sen_left[4] = pre_buff_sen_left[3];
		pre_buff_sen_left[3] = pre_buff_sen_left[2];
		pre_buff_sen_left[2] = pre_buff_sen_left[1];
		pre_buff_sen_left[1] = pre_buff_sen_left[0];
		pre_buff_sen_left[0] = average_sensor.left;

		//以下、前の時点での走行状態の判断用
		pre_ave_ave_right = (pre_buff_sen_right[6] + pre_buff_sen_right[7]//number2 x 5[ms]前の壁情報を取得する
				+ pre_buff_sen_right[8] + pre_buff_sen_right[9]) / 4.0;
		pre_ave_ave_left = (pre_buff_sen_left[6] + pre_buff_sen_left[7]
				+ pre_buff_sen_left[8] + pre_buff_sen_left[9]) / 4.0;
//		for (ii= 6; ii <= 9; ii++) {	//number2 x 5[ms]前の壁情報を取得する
//			pre_ave_ave_right += pre_buff_sen_right[ii];
//			pre_ave_ave_left += pre_buff_sen_left[ii];
//		}
//		pre_ave_ave_right = pre_ave_ave_right / (float) (4.0);	//前の走行状況を把握する
//		pre_ave_ave_left = pre_ave_ave_left / (float) (4.0);	//前の走行状況を把握する

		sen_count = 0;
	}
	//以下、更新後に現在の平均値を取得
	for (ii = 0; ii < number1 - 1; ii++) {
		buff_sen_right[ii + 1] = buff_sen_right[ii];
		buff_sen_left[ii + 1] = buff_sen_left[ii];
		average_sensor.right += buff_sen_right[ii];
		average_sensor.left += buff_sen_left[ii];
	}
	average_sensor.right = average_sensor.right / (float) number1;
	average_sensor.left = average_sensor.left / (float) number1;

	//以下、diffの算出
	diff_average_sensor.right = average_sensor.right - pre_buff_sen_right[0];
	diff_average_sensor.left = average_sensor.left - pre_buff_sen_left[0];

}

void interrupt_CMT0() {
	int i, kabekire_error;
//	float ed_rot, ed_center;
	unsigned char dir_r = 1, dir_l = 1;
	LED3 = 0;
	cmt_count++;
//	if (sen.right_front > 2000 && sen.left_front > 2000) {
//		sen_fail_flag = 1;
//		fail_count++;
//		if (fail_count > 500){
//			sen_fail_flag = 0;
//			fail_count = 0;
//		}
//	} else {
//		sen_fail_flag = 0;
//		fail_count = 0;
//	}
	if (Erorr_center.i > 3500.0 && sen.right_front > 2800
			&& sen.left_front > 2800) {
		fail_flag = 1;
	}
	if (sensor_enable == 1) {
		sensor_ADconvert();
	}
	if (sample_flag == 1 && sample_count < 800) {
		sample_count++;
		sample1[sample_count] = ideal_balance_velocity;
		sample2[sample_count] = ideal_balance_accel;
		LED3 = 1;
	}

	sensor_average(5, 7);	//added in 20170926 (5msの平均値、7ms間隔のdiff)

//理論値(重心の並進運動＆回転運動)
	ideal_balance_velocity += ideal_balance_accel * 0.001;
	ideal_balance_distance += ideal_balance_velocity * 0.001;
//	ideal_omega += ideal_angacc * 0.001;
	//	ideal_omega += ideal_angacc * 0.001 - 0.001 * Error;
	ideal_omega2 += ideal_angacc * 0.001;
	ideal_omega = ideal_omega2
			- (K_wall.p * Error_wall.p + K_wall.i * Error_wall.i
					+ K_wall.d * Error_wall.d);	//added in 20170921 (PID control method of wall control)
	ideal_angle += ideal_omega * 0.001;

//TCNTの差分をとる（場合分け不要）
	diff_tcnt.right = 32768 - MTU1.TCNT;	//TCNTの半分の値からの差分で考える
	diff_tcnt.left = 32768 - MTU2.TCNT;
	MTU1.TCNT = 32768;		//カウントを初期化
	MTU2.TCNT = 32768;

//ジャイロのリファレンス値を取得&実測値の計算
	if (gyro_enable == 1) {
		gyro_z.High = SPIRead(0x47);		//ｇyroセンサの値
		for (i = 0; i++; i < 10)
			;
		gyro_z.Low = SPIRead(0x48);
		for (i = 0; i++; i < 10)
			;
		gyro_z.SUM = (gyro_z.High << 8) + gyro_z.Low;
	}
	if (refer_flag == 1 && refer_count < 1000) {
		reference_omega = reference_omega + (float) gyro_z.SUM / 16.4;
		refer_count++;
		omega = (float) gyro_z.SUM / 16.4;
		LED1 = 1;
		if (refer_count == 1000) {
			reference_omega = reference_omega / 1000.0;	//average of omega for an 1s
			LED1 = 0;
			Erorr_center.i = 0.0;
			Erorr_rot.i = 0.0;
			reference_fin = 1;
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				LED_V1 = 1;
				LED_V2 = 1;
			}
		}
	} else {	//反時計回りが正方向
		omega = 90.0 / 89.5 * ((float) gyro_z.SUM / 16.4 - reference_omega);//角速度(degree/sec)、修正係数アリ
		angle = angle + omega * 0.001;
	}

//現在の測定値の取得
	velocity.right = 									//現在速度
			(diff_tcnt.right / 4.0 * 1000.0 / 4096.0 * 2 * pi) * d_tire / 2.0
					/ 4.0;
	velocity.left = -1.0
			* ((diff_tcnt.left / 4.0 * 1000.0 / 4096.0 * 2 * pi) * d_tire / 2.0
					/ 4.0);
	balance_velocity = (velocity.right + velocity.left) / 2.0;	//重心速度
	balance_distance += balance_velocity * 0.001;
	total_distance.right += velocity.right * 0.001;			//現在走行距離
	total_distance.left += velocity.left * 0.001;

	accel.right = fabs((float) (velocity.right - old_velocity.right) / 0.001);//現在加速度
	accel.left = fabs((float) (velocity.left - old_velocity.left) / 0.001);

	diff_balance_velocity[0] = ideal_balance_velocity - balance_velocity;//現在の、偏差
	diff_omega[0] = ideal_omega - omega;	//現在の、角速度偏差
	diff_kabe[0] = Error;

//フィードフォワード制御(重心の並進運動を考えるので、左右対称)
	dutty_foward.right = (((1.07 * 0.025 * m_body * ideal_balance_accel)
			/ (1.98 * 2.0 * 4.0))
			+ ((0.207 * (float) (diff_tcnt.right) * 60.0) / (4.0 * 4096.0)))
			/ 8.0;
	dutty_foward.left = (((1.07 * 0.025 * m_body * ideal_balance_accel)
			/ (1.98 * 2.0 * 4.0))
			+ ((0.207 * (float) (-diff_tcnt.left) * 60.0) / (4.0 * 4096.0)))
			/ 8.0;
//PIDの実体の計算(PID制御)
	Erorr_center.p = diff_balance_velocity[0];
	Erorr_rot.p = diff_omega[0];
	Error_wall.p = diff_kabe[0];
	Erorr_center.d = diff_balance_velocity[0] - diff_balance_velocity[1];
	Erorr_rot.d = diff_omega[0] - diff_omega[1];
	Error_wall.d = diff_kabe[0] - diff_kabe[1];

	if (ei_flag_center == 1) {
		Erorr_center.i = Erorr_center.i + diff_balance_velocity[0];
	} else {
		Erorr_center.i = 0.0;
	}
	if (ei_flag_rot == 1) {
		Erorr_rot.i = Erorr_rot.i + diff_omega[0] * 0.001;
	} else {
		Erorr_rot.i = 0.0;
	}
	if (ei_flag_wall == 1) {
		Error_wall.i = Error_wall.i + diff_kabe[0];
	} else {
		Error_wall.i = 0.0;
	}
	if (wall_control == 1) {	//壁制御ON
		test_wall_control();	//added in 20170923

		//以下、7x5=35ms前の状態を判定する
		if (reverse_flag == 1) {	//袋小路での壁切れは中央でなくても確実に読めるようにする
			kabekire_error = 500;
		} else {		//中央にいるかを判定する範囲
			kabekire_error = 200;
		}
		if ((pre_ave_ave_right > r_threshold)
				&& (pre_ave_ave_left > l_threshold)) {	//両方のセンサが使える時
			if (pre_ave_ave_right > sen_right_refer - kabekire_error
					&& pre_ave_ave_right < sen_right_refer + kabekire_error
					&& pre_ave_ave_left > sen_left_refer - kabekire_error
					&& pre_ave_ave_left < sen_left_refer + kabekire_error) {//中央にいると判定
				kabekire_enable = 1;
			}
		} else if (pre_ave_ave_left <= l_threshold
				&& pre_ave_ave_right > r_threshold) {	//右センサのみ使用可能の時
			if (pre_ave_ave_right > sen_right_refer - kabekire_error
					&& pre_ave_ave_right < sen_right_refer + kabekire_error) {//中央にいるかの判定
				kabekire_enable = 1;
			}
		} else if (pre_ave_ave_right <= r_threshold
				&& pre_ave_ave_left > l_threshold) {	//左センサのみ使用可能の時
			if (pre_ave_ave_left > sen_left_refer - kabekire_error
					&& pre_ave_ave_left < sen_left_refer + kabekire_error) {//中央にいるかの判定
				kabekire_enable = 1;
			}
		} else {	//中央にいない、もしくは両櫛の場合は壁切れは読まない
			if (wait_kabekire == 1) {
				kabekire_enable = 1;
			} else {
				kabekire_enable = 0;
			}
		}
		//以下、櫛の判定をする
		kushi_judge = 0;
		if ((pre_buff_sen_right[3] - pre_buff_sen_right[4]) > 100.0
				&& +(pre_buff_sen_right[4] - pre_buff_sen_right[5]) > 100.0
				&& +(pre_buff_sen_right[5] - pre_buff_sen_right[6]) > 100.0) {//右は櫛である
			kushi_judge = 1;	//右は櫛である
		}
		if ((pre_buff_sen_left[4] - pre_buff_sen_left[5]) > 70.0
				&& +(pre_buff_sen_left[5] - pre_buff_sen_left[6]) > 70.0
				&& +(pre_buff_sen_left[6] - pre_buff_sen_left[7]) > 70.0) {	//左は櫛である(判定値は緩めにした)
			if (kushi_judge == 1) {
				kushi_judge = 3;	//両櫛である（使わないが。）
			} else {
				kushi_judge = 2;	//左は櫛である
			}
		}
		//以下、壁切れの状態を判定する
		if (kabekire_enable_2 == 1 && kabekire_enable == 1) {//enable_2は台形加速中一回のみ壁切れを読むために必要
			if (diff_average_sensor.right <= -1.0 * DIFF_WALL_THRE
					&& fabs(diff_average_sensor.left) < DIFF_WALL_THRE
					&& average_sensor.right < 500.0) {	//右の壁切れのみを検知
				if (kushi_judge == 1) {	//右が櫛である場合
					//					kabekire_right = 2;
					kabekire_right = 1;
					kabekire_left = 0;
				} else {	//右がただの壁切れの場合
					kabekire_right = 1;
					kabekire_left = 0;
				}
				reverse_flag = 0;		//次の壁切れは中央値の判断が厳しくなる
			} else if (fabs(diff_average_sensor.right) < DIFF_WALL_THRE
					&& diff_average_sensor.left <= -1.0 * DIFF_WALL_THRE
					&& average_sensor.left < 500.0) {	//左の壁切れのみを検知
				if (kushi_judge == 2) {	//左が櫛である場合
					kabekire_right = 0;
					kabekire_left = 1;
					//					kabekire_left = 2;
				} else {	//左がただの櫛である場合
					kabekire_right = 0;
					kabekire_left = 1;
				}
				reverse_flag = 0;		//次の壁切れは中央値の判断が厳しくなる
			} else if (diff_average_sensor.right <= -1.0 * DIFF_WALL_THRE
					&& diff_average_sensor.left <= -1.0 * DIFF_WALL_THRE
					&& average_sensor.right < 500.0) {//両方の壁切れを検知(なるべく櫛切れでなく壁切れを読む)
				if (kushi_judge == 1) {	//両方の壁切れのうち、右が櫛なら左の壁切れを読む
					kabekire_right = 0;
					kabekire_left = 1;
				} else if (kushi_judge == 2) {	//両方の壁切れのうち、左が櫛なら右の壁切れを読む
					kabekire_right = 1;
					kabekire_left = 0;
				} else {	//どちらも櫛でない場合、右の壁切れを読む
					kabekire_right = 1;
					kabekire_left = 0;
				}
				reverse_flag = 0;		//次の壁切れは中央値の判断が厳しくなる
			}
		}
	} else {
		Error = 0.0;
	}

//以下、重心を用いらた並進運動と回転運動の重ね合わせ
	/*	dutty.right = dutty_foward.right + K_center.p * diff_balance_velocity[0]
	 + K_center.d * (Erorr_center.d) + K_center.i * (Erorr_center.i)
	 + EP_keisuu * K_rot.p * diff_omega[0] + K_rot.d * (Erorr_rot.d)
	 + EI_keisuu * K_rot.i * (Erorr_rot.i) - Error * Kp_wall_r;
	 dutty.left = dutty_foward.left + K_center.p * diff_balance_velocity[0]
	 + K_center.d * (Erorr_center.d) + K_center.i * (Erorr_center.i)
	 - (EP_keisuu * K_rot.p * diff_omega[0] + K_rot.d * (Erorr_rot.d)
	 + EI_keisuu * K_rot.i * (Erorr_rot.i)) + Error * Kp_wall_l;*///20170921現在まで使用
	dutty.right = dutty_foward.right + K_center.p * Erorr_center.p//壁制御値を角速度に代入したもの
	+ K_center.d * (Erorr_center.d) + K_center.i * (Erorr_center.i)
			+ K_rot.p * (Erorr_rot.p) + K_rot.d * (Erorr_rot.d)
			+ K_rot.i * (Erorr_rot.i);
	dutty.left = dutty_foward.left + K_center.p * Erorr_center.p
			+ K_center.d * (Erorr_center.d) + K_center.i * (Erorr_center.i)
			- (K_rot.p * (Erorr_rot.p) + K_rot.d * (Erorr_rot.d)
					+ K_rot.i * (Erorr_rot.i));

//制御において、減速時は回転方向を逆にする(正しいはず)
	if (dutty.right < 0) {
		dutty.right = fabs(dutty.right);
		dir_r = 0;
	}
	if (dutty.left < 0) {
		dutty.left = fabs(dutty.left);
		dir_l = 0;
	}
	motor_direction(dir_l, dir_r);	//正転…1、逆転…0

	/*	if (dutty.left > 0.9 || dutty.right > 0.9) {
	 GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
	 LED_V1 = 1;
	 LED_V2 = 1;
	 LED_V3 = 1;
	 }*/

	gptcount_r = (int) ((1.0 - dutty.right) * 125);		//duty→gptへの変換
	gptcount_l = (int) ((1.0 - dutty.left) * 125);

	old_tcnt.right = MTU1.TCNT;
	old_tcnt.left = MTU2.TCNT;
	old_velocity.right = velocity.right;	//現在加速度計算に使用
	old_velocity.left = velocity.left;
	diff_balance_velocity[2] = diff_balance_velocity[1];	//前回の偏差を前々回の偏差として代入
	diff_omega[2] = diff_omega[1];
	diff_kabe[2] = diff_kabe[1];
	diff_balance_velocity[1] = diff_balance_velocity[0];	//現在の偏差をひとつ前として代入
	diff_omega[1] = diff_omega[0];
	diff_kabe[1] = diff_kabe[0];
	average_sensor_old.right = average_sensor.right;
	average_sensor_old.left = average_sensor.left;

	GPT0.GTCCRA = gptcount_l;			//Duty比設定
	GPT0.GTCCRB = gptcount_r;

}

void task_exe(int first_number, int second_number, int therd_number) {//実行プログラムはこちらへ
	volatile int test, kk = 0, i;
	volatile float r_before, l_before, r_after, l_after, totalangle, angle1, angle2, accel,
			c_wise, vel, max_accel, max_vel;

	switch (first_number) {
	case 1:	//パラメータ調整用の第1層

		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 2500)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		sample_flag = 0;

		//以下、ターン調整-第2層
		switch (second_number) {
		case 1:
			sample_flag = 1;
			vel = 500.0;
			test_daikei(90.0 * 1.0, vel, 5000.0, 0.0, vel, 1);
			test_daikei(90.0 * 4.0, vel, 5000.0, vel, vel, 1);
			test_daikei(90.0 * 1.0, vel, 5000.0, vel, 0.0, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			wall_control = 0;
			sensor_enable = 0;

			break;
		case 2:
			sample_flag = 1;
			vel = 800.0;
			test_daikei(90.0 * 1.0, vel, 5000.0, 0.0, vel, 1);
			test_daikei(90.0 * 4.0, vel, 5000.0, vel, vel, 1);
			test_daikei(90.0 * 1.0, vel, 5000.0, vel, 0.0, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			wall_control = 0;
			sensor_enable = 0;

			break;
		case 3:
			sample_flag = 1;
			vel = 1000.0;
			test_daikei(90.0 * 1.0, vel, 5000.0, 0.0, vel, 1);
			test_daikei(90.0 * 4.0, vel, 5000.0, vel, vel, 1);
			test_daikei(90.0 * 1.0, vel, 5000.0, vel, 0.0, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			wall_control = 0;
			sensor_enable = 0;

			break;
		case 4:
			vel = 800.0;
			test_daikei(180.0, vel, 5000, 0, vel, 0);
			slalom_2(turn[0].P_1_1.theta, turn[0].P_1_1.th1, turn[0].P_1_1.th2,
					2000, turn[0].P_1_1.a_cc, turn[0].P_1_1.wise,
					turn[0].P_1_1.vel, turn[0].P_1_1.d_f, turn[0].P_1_1.d_r);//保存パラメタ
			test_daikei(90.0, vel, 4000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
			vel = 800.0;
			test_daikei(180.0, vel, 5000, 0, vel, 0);
			turn_for_pass(90.0, 40.0, 50.0, 1000.0, 4300.0, -1.0, 800.0, 43.0,
					47.0);	//右大回り
			test_daikei(90.0, vel, 4000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			totalangle =135.0;
			angle1 = 29.0;
			angle2 = totalangle - angle1;
			accel = 5840.0;
			r_before = 80.0;
			r_after = 75.0;
			c_wise = -1.0;
			vel = 650.0;
			Erorr_rot.i = 0.0;
			test_daikei(180.0, vel, 5000, 0, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 7:
			totalangle =135.0;
			angle1 = 28.0;
			angle2 = totalangle - angle1;
			accel = 5950.0;
			r_before = 80.0;
			r_after = 75.0;
			c_wise = -1.0;
			vel = 650.0;
			Erorr_rot.i = 0.0;
			test_daikei(180.0, vel, 5000, 0, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 8:
			totalangle =135.0;
			angle1 = 27.0;
			angle2 = totalangle - angle1;
			accel = 6050.0;
			r_before = 80.0;
			r_after = 75.0;
			c_wise = -1.0;
			vel = 650.0;
			Erorr_rot.i = 0.0;
			test_daikei(180.0, vel, 5000, 0, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, vel, 0);
			slalom_for_tuning(totalangle, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(63.6, vel, 6000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 9:

			break;
		case 10:

			break;
		}

		break;

	case 2:	//ターン調整ⅱの第1層

		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 2700)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		sample_flag = 0;

		//以下、ターン調整-第2層
		switch (second_number) {
		case 1:
			angle1 = 22.0;
			angle2 = 45.0 - 22.0;
			accel = 7360.0;
			r_before = 80.0;
			r_after = 35.0;
			c_wise = -1.0;
			vel = 650.0;
			test_daikei(127.3, vel, 6000, 0, vel, 0);
			slalom_for_tuning(90.0, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(90.0, vel, 6000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 2:
			angle1 = 22.0;
			angle2 = 45.0 - 22.0;
			accel = 7360.0;
			r_before = 82.0;
			r_after = 34.0;
			c_wise = -1.0;
			vel = 650.0;
			test_daikei(127.3, vel, 6000, 0, vel, 0);
			slalom_for_tuning(90.0, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(90.0, vel, 6000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:
			sample_flag=1;
			vel = 650.0;
			test_daikei(127.3, vel, 6000, 0, vel, 0);
			test_daikei(127.3, vel, 6000, vel, vel, 0);
			test_daikei(127.3, vel, 6000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			angle1 = 32.0;
			angle2 = 90.0 - 32.0;
			accel = 8360.0;
			r_before = 58.0;
			r_after = 93.0;
			c_wise = 1.0;
			vel = 900.0;
			test_daikei(180.0, vel, 4000, 0, vel, 0);
			slalom_2(90.0, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(180.0, vel, 4000, vel, 0, 0);

			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
			angle1 = 30.0;
			angle2 = 90.0 - 30.0;
			accel = 7500.0;
			r_before = 8.0;
			r_after = 14.0;
			c_wise = -1.0;
			vel = 650.0;
			test_daikei(180.0, vel, 4000, 0, vel, 0);
			slalom_2(90.0, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(90.0, vel, 4000, vel, 0, 0);

			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			angle1 = 33.0;
			angle2 = 90.0 - 33.0;
			accel = 8320.0;
			r_before = 5.0;
			r_after = 26.0;
			c_wise = 1.0;
			vel = 650;
			test_daikei(180.0, vel, 4000, 0, vel, 0);
			slalom_2(90.0, angle1, angle2, 2000.0, accel, c_wise, vel, r_before,
					r_after);		//保存パラメタ
			test_daikei(90.0, vel, 4000, vel, 0, 0);

			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 7:
			vel = 650.0;
			test_daikei(180.0, vel, 4000, 0, vel, 0);
			slalom_2(90.0, 33.0, 57.0, 2000.0, 8320.0, 1.0, 650.0, 5.0, 25.0);//左小回り(速め)
			test_daikei(90.0, vel, 4000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 8:
			vel = 650.0;
			test_daikei(180.0, vel, 4000, 0, vel, 0);
			slalom_2(90.0, 30.0, 60.0, 2000.0, 7500.0, -1.0, 650.0, 4.0, 8.0);//右小回り(速め)
			test_daikei(90.0, vel, 4000, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		}

		break;

	case 3:	//足立法調整用の第1層
		switch (second_number) {
		case 1:	//往復の足立法の修正版-第2層(revised in 10/01)

			sensor_enable = 1;
			wait(500);

			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			LED5 = 1;
			sample_flag = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.
			direction_count = 0;
			adachihou_q(goal_x, goal_y, 0, 0, 500, 2000);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			if (direction_count == 0) {
				direction_count = 2;
			} else if (direction_count == 1) {
				direction_count = 3;
			} else if (direction_count == 2) {
				direction_count = 0;
			} else {
				direction_count = 1;
			}
			test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
			wait(1000);
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.
			adachihou_q(0, 0, goal_x, goal_y, 500, 2000);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;
			LED4 = 0;
			if (fail_flag == 1) {
			} else {
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}

			break;
		case 2:	//片道の足立法-第2層

			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;
			direction_count = 0;
			adachihou_q(goal_x, goal_y, 0, 0, 500, 2000);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:	//座標変換後の足立法-第2層

			temp_goal_x = 15;
			temp_goal_y = 15;
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			direction_count = 0;
			adachihou_q(temp_goal_x, temp_goal_y, 0, 0, 500.0, 2000.0);	//仮の値
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(500);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			if (direction_count == 0) {
				direction_count = 2;
			} else if (direction_count == 1) {
				direction_count = 3;
			} else if (direction_count == 2) {
				direction_count = 0;
			} else {
				direction_count = 1;
			}
			test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
			wait(1000);
			adachihou_q(goal_x, goal_y, temp_goal_x, temp_goal_y, 500.0,
					2000.0);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			sensor_enable = 1;
			wait(500);

			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			LED5 = 1;
			sample_flag = 0;
			direction_count = 0;
			adachihou2_q(goal_x, goal_y, 0, 0, 650, 5000);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			if (direction_count == 0) {
				direction_count = 2;
			} else if (direction_count == 1) {
				direction_count = 3;
			} else if (direction_count == 2) {
				direction_count = 0;
			} else {
				direction_count = 1;
			}
			test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
			wait(1000);
			adachihou2_q(0, 0, goal_x, goal_y, 650, 5000);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;
			LED4 = 0;
			if (fail_flag == 1) {
			} else {
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}

			break;
		case 5:
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;
			direction_count = 0;
			adachihou2_q(goal_x, goal_y, 0, 0, 650, 5000);	//重心速度速いバージョン
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			temp_goal_x = 15;
			temp_goal_y = 15;
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			direction_count = 0;
			adachihou2_q(temp_goal_x, temp_goal_y, 0, 0, 650.0, 5000.0);//仮の値
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(500);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			if (direction_count == 0) {
				direction_count = 2;
			} else if (direction_count == 1) {
				direction_count = 3;
			} else if (direction_count == 2) {
				direction_count = 0;
			} else {
				direction_count = 1;
			}
			test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
			wait(1000);
			adachihou2_q(11, 7, temp_goal_x, temp_goal_y, 650.0, 5000.0);//仮の値
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(500);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			if (direction_count == 0) {
				direction_count = 2;
			} else if (direction_count == 1) {
				direction_count = 3;
			} else if (direction_count == 2) {
				direction_count = 0;
			} else {
				direction_count = 1;
			}
			test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
			wait(1000);
			adachihou2_q(goal_x, goal_y, 11, 7, 650.0, 5000.0);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			WALL_INFORMATION_save();
			WATCHED_WALL_INFORMATION_save();
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 7:
			break;
		}

		break;

	case 4:		//パス調整用―第1階層
		switch (second_number) {
		case 1:	//斜め無しパスの修正版(revised in 10/01)
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
//			 WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
//			 WALL_INFORMATION_save();	//正しい元情報に戻った

			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			exe_pass_kai(1200.0, 800.0, 4000.0, 1);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 2:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 1;

			exe_pass_kai(2000.0, 800.0, 7000.0, 1);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 1;

			exe_pass_kai(1200.0, 800.0, 4000.0, 2);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 1;

			exe_pass_kai(2000.0, 800.0, 7000.0, 2);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 1;

			exe_pass_kai(2200.0, 800.0, 8000.0, 2);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 1;

			exe_pass_test(1200.0, 4000.0, 1);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;
			break;
		case 7:

			break;
		case 8:

			break;
		case 9:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			break;
		case 10:
			LED5 = 1;	//斜めpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();	//make_passのために保存する
			make_pass(goal_x, goal_y);
			convert_pass_skew();	//斜め用のパスに変換
			skew_queue_walkmap_maker(goal_x, goal_y);
			skew_walkmap_display();
			LED5 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//元通り

			break;
		}
		break;

	case 5:		//直進、壁切れ調整用-第1層
		Kp_wall_l = 0.0001;
		Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 3000)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		switch (second_number) {
		case 1:
//			max_accel=7500.0;
			max_accel = 9500.0;
			max_vel = 3000.0;
			sample_flag = 1;
			daikei_for_pass_kai(90.0 * 7.0, max_vel, max_accel, 0.0, max_vel, 1,
					0);
			daikei_for_pass_kai(90.0 * 4.0, max_vel, max_accel, max_vel,
					max_vel, 1, 0);
			daikei_for_pass_kai(90.0 * 7.0, max_vel, max_accel, max_vel, 0.0, 1,
					0);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;
			break;

		case 2:
			max_accel = 7000.0;
			max_vel = 3000.0;
			sample_flag = 1;
			daikei_for_pass_kai(90.0 * 8.0, max_vel, max_accel, 0.0, max_vel, 1,
					0);
			daikei_for_pass_kai(90.0 * 4.0, max_vel, max_accel, max_vel,
					max_vel, 1, 0);
			daikei_for_pass_kai(90.0 * 8.0, max_vel, max_accel, max_vel, 0.0, 1,
					0);

//			daikei_for_pass_kai(90.0, 800.0, 4000.0, 0.0, 800.0, 1, 0);
//			wait_kabekire = 1;
//			daikei_for_pass_kai(90.0 * 4.0, 800.0, 4000.0, 800.0, 800.0, 1, 1);
//			turn_for_pass(90.0, 40.0, 50.0, 1000.0, 4300.0, -1.0, 800.0,
//					43.0, 47.0);	//右大回り
//			daikei_for_pass_kai(90.0, 800.0, 4000.0, 800.0, 0.0, 1, 0);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;
			break;

		case 3:
			sample_flag = 0;
			daikei_for_pass_kai(90.0 * 4.0, 2000.0, 7000.0, 0.0, 2000.0, 1, 0);
			daikei_for_pass_kai(90.0 * 4.0, 2000.0, 7000.0, 2000.0, 2000.0, 1,
					0);
			daikei_for_pass_kai(90.0 * 4.0, 2000.0, 7000.0, 2000.0, 0.0, 1, 0);

//			daikei_for_pass(90.0, 800.0, 4000.0, 0.0, 800.0, 0);
//			daikei_for_pass(70.0, 800.0, 4000.0, 800.0, 800.0, 0);
//			daikei_for_pass(90.0 * 2.0, 800.0, 4000.0, 800.0, 800.0, 1);	//passでは90.0進むためには180.0と入力する必要がある！
//			turn_for_pass(90.0, 40.0, 50.0, 1000.0, 4300.0, -1.0, 800.0,
//					43.0 + 90.0, 47.0);	//右大回り
//			daikei_for_pass(90.0, 800.0, 4000.0, 800.0, 0.0, 0);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			sample_flag = 1;
			test_daikei(90.0 * 1.0, 2500.0, 8000.0, 0.0, 2500.0, 1);
			test_daikei(90.0 * 4.0, 2500.0, 8000.0, 2500.0, 2500.0, 1);
			test_daikei(90.0 * 1.0, 2500.0, 8000.0, 2500.0, 0.0, 1);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
			daikei_for_pass(90.0, 800.0, 4000.0, 0.0, 800.0, 0);
			daikei_for_pass(133.0, 800.0, 4000.0, 800.0, 800.0, 1);
			daikei_for_pass(90.0, 800.0, 4000.0, 800.0, 0.0, 0);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			break;
		case 7:
			break;
		}
		break;

	case 6:	//北信越パス予備用-第1層

		switch (second_number) {		//以下、北信越pass-第2層

		case 1:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;

//			exe_pass_kai(2500.0, 800.0, 8000.0, 1);//走った（北信越調整）
			exe_pass_kai(2500.0, 800.0, 9000.0, 1);
			ideal_balance_velocity = 0.0;
			ideal_omega = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 2:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;

			exe_pass_kai(2500.0, 800.0, 10000.0, 1);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;

			exe_pass_kai(2500.0, 800.0, 9000.0, 2);
			ideal_balance_velocity = 0.0;
			ideal_omega = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;

			exe_pass_kai(2500.0, 800.0, 10000.0, 2);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;

			exe_pass_kai(2800.0, 800.0, 10000.0, 2);
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 0;

			exe_pass_kai(2500.0, 800.0, 9000.0, 3);	//モード3！
			ideal_balance_velocity = 0.0;
			ideal_omega = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 7:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			Kp_wall_l = 0.0001;
			Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 3000)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			EI_keisuu = 1.0;
			sample_flag = 1;

			exe_pass_kai(2800.0, 800.0, 10000.0, 3);
			ideal_balance_velocity = 0.0;
			ideal_omega = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 8:

			break;
		}

		break;

	case 7:
		//以下、表示系統調整-第2層
		switch (second_number) {
		case 1:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
			skew_queue_walkmap_maker(goal_x, goal_y);
			LED4 = 0;
			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った(はず)

			for (i = 0; i <= last_p_i; i++) {
				myprintf("pass[%d]=%d\r\n", i, pass[i]);
			}
			for (i = 0; i <= last_p_i; i++) {
				myprintf("motion[%d]=%d\r\n", i, motion[i]);
			}
			maze_display();
			saved_maze_display();
			walkmap_display();
			myprintf("map[0][1]=%3d\r\n", map[0][1]);
			myprintf("map[0][1]=%3d\r\n", 1);
			skew_walkmap_display();

			break;
		case 2:
			myprintf("sample1, sample2\n\r");
			for (kk = 1; kk < 800 - 1; kk++) {
				myprintf("%f, %f\n\r", sample1[kk], sample2[kk]);
			}
//			myprintf("%f\n\r", sample1[799]);
//			myprintf("sample2\n\r");
//			for (kk = 1; kk < 800; kk++) {
//				myprintf("%f\n\r,", sample2[kk]);
//			}

			break;
		case 3:
			sensor_enable = 1;
			while (1) {
				myprintf(
						"sen.left_front=%d sen.left_side=%d sen.right_side=%d sen.right_front=%d\n\r",
						sen.left_front, sen.left_side, sen.right_side,
						sen.right_front);
			}
//			while (1) {
//				myprintf(
//						"average_sensor.right=%f average_sensor.left=%f sen.right_side=%d sen.right_front=%d\n\r",
//						average_sensor.right, average_sensor.left,
//						sen.right_side, sen.right_front);
//				myprintf(
//						"pre_ave_ave_right=%f pre_ave_ave_left=%f diff_average_sensor.right=%f diff_average_sensor.left=%f\n\r",
//						pre_ave_ave_right, pre_ave_ave_left,
//						diff_average_sensor.right, diff_average_sensor.left);
//			}
			break;
		case 4:
			for (i = 0; i <= 15; i++) {
				myprintf("column_fix[%d]=%d\r\n", i, column_fix[i]);
			}
			for (i = 0; i <= 15; i++) {
				myprintf("row_fix[%d]=%d\r\n", i, row_fix[i]);
			}

			break;
		case 5:
			while (1) {
				test = SPIRead(117);
				for (i = 0; i++; i < 100000)
					;
				myprintf("who am I=%d\n\r", test);
				test = SPIRead(0x6b);	//Power Management1：Sleep解除、温度センサー使用不可
				myprintf("power_mgmt_1=%d\n\r", test);
			}

			break;
		case 6:
			LED4 = 1;	//斜め無しpassの作成
			maze_display();
			saved_maze_display();
			unknown_WALL_add();	//tempに代入
			WALL_INFORMATION_save();//fixに代入
			maze_display();
			saved_maze_display();
//			make_pass(goal_x, goal_y);
//			convert_pass();	//斜め無し用のパスに変換
//			skew_queue_walkmap_maker(goal_x, goal_y);
			LED4 = 0;
			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った(はずがない。)
			maze_display();
			saved_maze_display();

//			for (i = 0; i <= last_p_i; i++) {
//				myprintf("pass[%d]=%d\r\n", i, pass[i]);
//			}
//			for (i = 0; i <= last_p_i; i++) {
//				myprintf("motion[%d]=%d\r\n", i, motion[i]);
//			}
//			maze_display();
//			saved_maze_display();
//			walkmap_display();
//			myprintf("map[0][1]=%3d\r\n", map[0][1]);
//			myprintf("map[0][1]=%3d\r\n", 1);
//			skew_walkmap_display();

			break;
		case 7:
			maze_display();
			saved_maze_display();
			walkmap_display();

			break;
		case 8:

			break;
		}

		break;

	case 8:		//宴会芸
		/*		test = SPIRead(117);
		 myprintf("who am I=%d\n\r", test);
		 test = SPIRead(0x6b);
		 myprintf("power_mgmt_1=%d\n\r", test);
		 wait(10);
		 test = SPIRead(0x1b);
		 myprintf("1b=%d\n\r", test);
		 wait(10);
		 test = SPIRead(0x6a);
		 myprintf("6a=%d\n\r", test);
		 wait(10);
		 test = SPIRead(0x6c);
		 myprintf("6c=%d\n\r", test);

		 PORT1.DDR.BIT.B0 = 1;
		 PORT1.DR.BIT.B0 = 1;
		 wait(1000);
		 */
		Kp_wall_l = 0.0001;
		Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 3000)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		ideal_omega = 0.0;
		ideal_balance_velocity = 0.0;

		wall_control = 0;
		ei_flag_center = 1;	//宴会芸では必要
		ei_flag_rot = 1;	//宴会芸では必要
		EI_keisuu = 1.0;

		while (1) {
			test = SPIRead(117);
			myprintf("who am I=%d\n\r", test);
			myprintf("gyro_z.sum=%d omega=%f angle=%f reference_omega=%f\n\r",
					gyro_z.SUM, omega, angle, reference_omega);
			myprintf("diff_omega[0]=%f , duty.right=%f\n\r", diff_omega[0],
					dutty.right);
		}
		break;

	case 9:
		Kp_wall_l = 0.0001;
		Kp_wall_r = 0.0001;	//ジャイロアリならこれでも発散しない
		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 3000)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		EI_keisuu = 1.0;
		sample_flag = 0;
		adachihou_q(goal_x, goal_y, 0, 0, 500, 2000);
		wall_control = 0;
		ideal_balance_velocity = 0.0;
		wait(1000);
		GPT.GTSTR.BIT.CST0 = 0;		//
		WALL_INFORMATION_save();
		LED5 = 1;
		make_pass(goal_x, goal_y);
		LED5 = 0;
		convert_pass_skew();

		break;

	case 10:

		break;

	case 11:
		switch (second_number) {
		case 1:
			while (1) {		//LEDのデモンストレーション
				//	PORT2.DR.BIT.B4 = 1;
				wait(1000);
				LED1 = 1;
				wait(200);
				LED2 = 1;	//LED2
				wait(200);
				LED3 = 1;
				wait(200);
				LED4 = 1;	//LED4
				wait(200);
				LED5 = 1;	//LED5
				while (1) {
					wait(200);
					LED_V1 = 1;		//LED_V1
					wait(200);
					LED_V2 = 1;	//LED_V2
					wait(200);
					LED_V3 = 1;	//LED_V3
					wait(200);
					LED_V4 = 1;

					wait(400);
					LED_V1 = 0;
					LED_V2 = 0;
					LED_V3 = 0;
					LED_V4 = 0;

				}
			}
			break;
		case 2://パスの確認用
			temp_test_mazedata();
			make_pass(goal_x, goal_y);
			convert_pass();	//斜め無し用のパスに変換
		//	convert_pass_skew();	//斜めパスに変換
			skew_queue_walkmap_maker(goal_x, goal_y);
			for (i = 0; i <= last_p_i; i++) {
				myprintf("pass[%d]=%d\r\n", i, pass[i]);
			}
			for (i = 0; i <= last_p_i; i++) {
				myprintf("motion[%d]=%d\r\n", i, motion[i]);
			}
			maze_display();
			saved_maze_display();
			walkmap_display();
			skew_walkmap_display();

			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		case 9:
			break;
		case 10:
			break;
		case 11:	//座標変更モード!!
			goal_x = number_select();
			goal_y = number_select();
			myprintf("goal_x=%d  goal_y=%d\r\n", goal_x, goal_y);
			break;
		}
		break;
	}
}

int main(void) {
	volatile int i, test;
	volatile char first_number, second_number, third_number, main_count;
	volatile float battery_refer = 0.0;
	for (i = 0; i < 100000; i++)
		;	//ジャイロの電圧が安定するまでの待ち時間
//	PORT1.DDR.BIT.B0 = 1;	//Gyro_int(意味なし?)
//	PORT1.DR.BIT.B0 = 0;

	init_CPU();
	init_sci();
	init_CMT();
	init_MTU();
//	init_RSPI();
	init_GPT();
	init_INTERFACE();
	init_ADconvert();

	/*	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;	//MTUストップ解除

	 MTU.TSTRA.BYTE = 0;
	 MTU2.TCR.BIT.TPSC = 2;
	 MTU2.TCR.BIT.CCLR = 1;	//TGRAでカウンタクリア
	 MTU2.TMDR1.BIT.MD = 3;	//モード2
	 MTU2.TCR.BIT.CKEG = 0;	//立ち上がりエッジでカウント
	 MTU2.TIOR.BIT.IOA = 1;	//初期出力low,コンペアマッチでlow
	 MTU2.TIOR.BIT.IOB = 2;	//初期出力low,コンペアマッチでhigh

	 MTU2.TGRA = 6250-1;//1msでカウントのはず
	 MTU2.TGRB = 5000;

	 MTU.TSTRA.BIT.CST2 = 1;	//MTU2スタート

	 while (1) {
	 if (MTU2.TSR.BIT.TGFA ==1) {
	 LED_V3 = 1;
	 myprintf("TCNT_A=%d\n\r",MTU2.TCNT);
	 }
	 }
	 */
	CMT.CMSTR0.BIT.STR0 = 1;	//タイマースタート
	init_RSPI();	//waitを使うので、CMT起動後に設定する

//	SYSTEM.MSTPCRA.BIT.ACSE = 0;		//モジュール
//	SYSTEM.DPSBYCR.BIT.IOKEEP = 0;	//I/O保持解除
//	PORT2.ICR.BIT.B4 = 0;
	wait(200);
	for (i = 0; i < 100; i++) {
		battery_ADconvert();
		battery_refer = battery_refer + Battery;
	}
	battery_refer = battery_refer / 100.0;
	myprintf("battery_refer=%f\n\r", battery_refer);

	if (battery_refer <= 7.6) {	//大会だけ7.6
		LED1 = 1;
		LED5 = 1;
		while (1) {
			battery_ADconvert();
			myprintf("Battery=%f\n\r", Battery);
		}
	}

	K_center.p = Kp;
	K_center.i = Ki;
	K_center.d = Kd;
	K_rot.p = Kp_rot;
	K_rot.i = Ki_rot;
	K_rot.d = Kd_rot;
	K_wall.p = 0.15;	//0.1	//壁制御はP制御のみ
	K_wall.i = 0.0;
	K_wall.d = 0.0;
	goal_x = GOAL_X;	//ひとまずこれで代入。変更できるのは座標変更モードのみ。
	goal_y = GOAL_Y;	//ひとまずこれで代入。変更できるのは座標変更モードのみ。

	assign_parameters();
//	myprintf("test1=%f\r\n", turnvel[0].turnpara_1_0.theta);
//	myprintf("test1=%f\r\n", turnvel[0].turnpara_1_0.theta1);
//	myprintf("test1=%f\r\n", turnvel[1].turnpara_1_0.theta);

	/*	last_p_i = 11;
	 pass[0] = 255;
	 pass[1] = 4;
	 pass[2] = 40;
	 pass[3] = 40;
	 pass[4] = 50;
	 pass[5] = 50;
	 pass[6] = 2;
	 pass[7] = 50;
	 pass[8] = 40;
	 pass[9] = 40;
	 pass[10] = 50;
	 pass[11] = 4;

	 for (i = 0; i <= last_p_i; i++) {
	 myprintf("pass[%d]=%d\r\n", i, pass[i]);
	 }

	 convert_pass_skew();
	 for (i = 0; i <= last_p_i; i++) {
	 myprintf("motion[%d]=%d\r\n", i, motion[i]);
	 }
	 */
	/*	 column_fix[8] = 0x55b8;
	 column_fix[9] = 0x8a14;
	 column_fix[10] = 0x94d4;
	 column_fix[11] = 0x6814;
	 column_fix[12] = 0x5a14;
	 column_fix[13] = 0xa614;
	 row_fix[0] = 0x2;
	 row_fix[1] = 0x20;
	 row_fix[2] = 0x11c;
	 row_fix[3] = 0x2391;
	 row_fix[4] = 0x60;
	 row_fix[5] = 0x38;
	 row_fix[6] = 0x3c;
	 row_fix[7] = 0x2f8;
	 row_fix[8] = 0x1bc;
	 row_fix[9] = 0xc5;
	 row_fix[10] = 0x200;
	 */
	/*
	 make_pass(2, 1);
	 for (i = 0; i <= last_p_i; i++) {
	 myprintf("pass[%d]=%d\r\n", i, pass[i]);
	 }
	 convert_pass();
	 //	convert_pass_skew();
	 for (i = 0; i <= last_p_i; i++) {
	 myprintf("motion[%d]=%d\r\n", i, motion[i]);
	 }

	 skew_queue_walkmap_maker(2, 1);
	 skew_walkmap_display();
	 */
//	column_temp[0]=99;
//	column_temp[1]=92;
//	column_temp[2]=168;
//	column_temp[3]=71;
//	column_temp[4]=255;
//	column_temp[5]=0;
//	column_temp[6]=0;
//	column_temp[7]=0;
//	column_temp[8]=0;
//	column_temp[9]=0;
//	column_temp[10]=0;
//	column_temp[11]=0;
//	column_temp[12]=0;
//	column_temp[13]=0;
//	column_temp[14]=0;
//	row_temp[0]=0;
//	row_temp[1]=20480;
//	row_temp[2]=40960;
//	row_temp[3]=36864;
//	row_temp[4]=26624;
//	row_temp[5]=4096;
//	row_temp[6]=8192;
//	row_temp[7]=63488;
//	row_temp[8]=0;
//	row_temp[9]=0;
//	row_temp[10]=0;
//	row_temp[11]=0;
//	row_temp[12]=0;
//	row_temp[13]=0;
//	row_temp[14]=0;
//	column_watched_temp[0]=99;
//	column_watched_temp[1]=92;
//	column_watched_temp[2]=168;
//	column_watched_temp[3]=71;
//	column_watched_temp[4]=255;
//	column_watched_temp[5]=0;
//	column_watched_temp[6]=0;
//	column_watched_temp[7]=0;
//	column_watched_temp[8]=0;
//	column_watched_temp[9]=0;
//	column_watched_temp[10]=0;
//	column_watched_temp[11]=0;
//	column_watched_temp[12]=0;
//	column_watched_temp[13]=0;
//	column_watched_temp[14]=0;
//	row_watched_temp[0]=0;
//	row_watched_temp[1]=20480;
//	row_watched_temp[2]=40960;
//	row_watched_temp[3]=36864;
//	row_watched_temp[4]=26624;
//	row_watched_temp[5]=4096;
//	row_watched_temp[6]=8192;
//	row_watched_temp[7]=63488;
//	row_watched_temp[8]=0;
//	row_watched_temp[9]=0;
//	row_watched_temp[10]=0;
//	row_watched_temp[11]=0;
//	row_watched_temp[12]=0;
//	row_watched_temp[13]=0;
//	row_watched_temp[14]=0;
//	WALL_INFORMATION_save();
//
//	maze_display();
//	saved_maze_display();
//	unknown_WALL_add();
//	maze_display();
//	unknown_WALL_remove();
//	maze_display();
//
//	walkmap_display();


	while (1) {
		ideal_angacc = 0.0;	//fail_flagを抜けるために必要(と思われる)
		ideal_omega = 0.0;	//fail_flagを抜けるために必要(と思われる)
		ideal_balance_accel = 0.0;	//fail_flagを抜けるために必要
		ideal_balance_velocity = 0.0;	//fail_flagを抜けるために必要
		wait(100);
		fail_flag = 0;

		main_count = 0;
		if (main_count == 0) {
			first_number = mode_select();
			wait(100);
			main_count++;
		}
		if (main_count == 1) {
			second_number = mode_select();
			wait(100);
			main_count++;
		}
		if (main_count == 2) {
			//	third_number = mode_select();
			main_count++;
		}
		if (main_count == 3) {
			task_exe(first_number, second_number, third_number);//実行文はこちらへどうぞ
		}
	}

}

