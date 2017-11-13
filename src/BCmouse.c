/***************************************************************/
/*                                                             */
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
#include "BC_daikei.h"
#include "BC_pass_old.h"
#include "BC_search_algorithm.h"
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
void daikei_for_pass_EX(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kebekire);
void daikei_for_pass_kai2(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kabekire);
void test_turn(float hikisuu_angle, float omega_max, float hikisuu_angacc,
		float unclock_wise, float hikisuu_balance_velocity);
void turn_for_skew_pass(float hikisuu_angle, float angle1, float angle2,
		float omega_max, float hikisuu_angacc, float unclock_wise,
		float hikisuu_v_init, float hikisuu_v_skew, float hikisuu_v_term, float dist1, float dist2, char h_skew_kabekire);
void test_slalom(float hikisuu_angle, float omega_max, float hikisuu_angacc,
		float unclock_wise, float hikisuu_balance_velocity, float dist1,
		float dist2);
void slalom_2(float hikisuu_angle, float angle1, float angle2, float omega_max,
		float hikisuu_angacc, float unclock_wise,
		float hikisuu_balance_velocity, float dist1, float dist2);
void direction_xy();
void Search_UnknownWall_Pass(char hikisuu_goal_x, char hikisuu_goal_y);	//一時的にここ
void Search_UnknownWall_Pass_R(char hikisuu_goal_x, char hikisuu_goal_y);	//一時的にここ
void new_serch_algorithm(char hikisuu_goal_x, char hikisuu_goal_y, char start_x,
		char start_y, char para_mode);
void adachihou_q(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search);
void adachihou2_q(char hikisuu_goal_x, char hikisuu_goal_y, char start_x,
		char start_y, char para_mode);
//void make_pass(int hikisuu_goal_x, int hikisuu_goal_y);
void move_xy_skew(char hikisuu_v_l, char hikisuu_direction);
void convert_pass_skew(void);
void exe_pass_EX(float hikisuu_vmax, float hikisuu_skew_vmax, float hikisuu_accel, char hikisuu_mode);
void temp_exe_pass_EX(float hikisuu_vmax, float hikisuu_accel, char hikisuu_mode);
void q_walk_map_maker(int hikisuu_goal_x, int hikisuu_goal_y);
void q_saved_walk_map_maker( hikisuu_goal_x, hikisuu_goal_y);
void GoalPosition_assign(char hikisuu_x, char hikisuu_y);
void GoalPosition_remove(char hikisuu_x, char hikisuu_y);
short GoalPosition_check(char hikisuu_x, char hikisuu_y);
char Compete_Near_Position(char current_posi, char position1, char position2);
void q_new_walk_map_maker(char hikisuu_goal_x, char hikisuu_goal_y, char hikisuu_goal_size, char posi_x, char posi_y);
void skew_queue_walkmap_maker(char hikisuu_goal_x, char hikisuu_goal_y);
void sensor_average(char number1, char number2);
void interrpt_CMT0(void);
void task_exe(int first_number, int second_number, int therd_number);
void unknown_WALL_add();
void unknown_WALL_remove();
void turn_angle_tuning(float h_totalangle, float h_angle1, float h_accel,
		float h_front, float h_rear, float h_c_wise, float h_vel, char h_count);
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
		kushi_judge = 0, pass[200] = { 0 }, motion[100] = { 0 }, direc[200] = { 0 }, last_p_i,
		reverse_flag = 0, pass_kabekire_straight = 0, wait_kabekire = 0,
		kabekire_read_flag = 0, fail_flag = 0, sen_fail_flag = 0,
		fail_count = 0, ei_flag_wall = 0, goal_x, goal_y, temp_goal_x,
		temp_goal_y, daikei_mode = 0, aa = 0, bb = 0, cc = 0, time_flag = 0, possible_tamp_goal[50], kabekire_LED = 0, LED_count = 0,
		Pass_Goal_x, Pass_Goal_y;
volatile int sen_bat, sen_l_f_ON, sen_l_s_ON, sen_r_f_ON, sen_r_s_ON,
		sen_l_f_OFF, sen_l_s_OFF, sen_r_f_OFF, sen_r_s_OFF, gptcount_l = 125,
		gptcount_r = 125, sample_count = 0, refer_count = 0,
		buff_sen_right[10] = { 0.0 }, buff_sen_left[10] = { 0.0 },
		sen_count = 0 ;
volatile unsigned short map[16][16] = { 0 }, row_temp[15] = { 0 },
		column_temp[15] = { 0 }, row_fix[15] = { 0 }, column_fix[15] = { 0 },
		row_watched_fix[15] = { 0 }, column_watched_fix[15] = { 0 },
		row_watched_temp[15] = { 0 }, column_watched_temp[15] = { 0 }, unknown_wall_row[15] = { 0 }, unknown_wall_column[15] = { 0 },
		level[16][15] = { 0 }, vertical[15][16] = { 0 }, log_count = 0, read_P_I = 0 ,goal_position[16] = { 0 };
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
		adjust_distance = 0.0, vel_low, vel_high, accel_normal, buff_sen_right_front[10] = { 0.0 }, buff_sen_left_front[10] = { 0.0 };
volatile float_rightleft_t velocity, total_distance, accel, old_velocity = {
		0.0, 0.0 }, ideal_velocity, ideal_accel = { 0.0 }, ideal_distance,
		diff_velocity[3] = { 0.0, 0.0 }, dutty = { 0.0, 0.0 }, dutty_foward = {
				0.0, 0.0 }, average_sensor, average_front_sen, diff_average_sensor, diff_average_front_sen,
		average_sensor_old;
volatile int_rightleft_t old_tcnt, tcnt = { 32767, 32767 }, diff_tcnt,
		motor_direc = { 0, 0 };
volatile float_highlow_t gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
volatile float_pid_t K_center, K_rot, K_wall, K_skew_wall, Erorr_center, Erorr_rot,
		Error_wall;
volatile int_sensor_t sen;
volatile unsigned_short_fix_t a = { 0 };
volatile kabekire_t flags_kabekire = { 0 };
volatile bits_t skew_kabekire;

volatile const turn_velocities_t turn[10] = {//[0]:重心速度500  [1]:重心速度650  [2]:速度750  [3]:速度800 [4]:速度900  [5]:速度  [6]:速度
		{
		/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
				{ 90.0, 22.5, 67.5, 5500.0, -1.0, 13.0, 13.0, 500.0, 0, 1 },	//右小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//右大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//右Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//斜め→右V90°ターン
				{ 90.0, 22.5, 67.5, 5500.0, 1.0, 6.5, 22.0, 500.0, 0, 1 },//左小回り
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//左大回り
				{ 180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 },	//左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },	//斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 },//斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0 } //斜め→左V90°ターン
		}, {
			  /*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
				{  90.0,  30.0,  60.0, 7500.0,  -1.0,   2.5,  11.0,  650.0,    0,     1 }, //0.右小回り
				{  90.0,   0.0,   0.0,    0.0,   0.0,   0.0,   0.0,  650.0,    0,     0 }, //1.右大回り
				{ 180.0,   0.0,   0.0,    0.0,   0.0,   0.0,   0.0,  650.0,    0,     0 }, //2.右Uターン
				{  45.0,  22.0,  23.0, 7360.0,  -1.0,  35.2,  71.5,  650.0,    1,     0 }, //3.右45°ターン→斜め
				{ 135.0,  27.0, 108.0, 6050.0,  -1.0,  87.0,  70.0,  650.0,    1,     0 }, //4.右135°ターン→斜め
				{  45.0,  22.0,  23.0, 7360.0,  -1.0,  72.2,  39.0,  650.0,    1,     0 }, //5.斜め→右45°ターン
				{ 135.0,  27.0, 108.0, 6050.0,  -1.0,  76.0, 100.0,  650.0,    1,     0 }, //6.斜め→右135°ターン
				{  90.0,  30.0,  60.0, 7500.0,  -1.0,  40.0,  51.0,  650.0,    1,     0 }, //7.斜め→右V90°ターン
				{  90.0,  33.0,  57.0, 8320.0,   1.0,   7.0,  22.0,  650.0,    0,     1 }, //8.左小回り
				{  90.0,   0.0,   0.0,    0.0,   0.0,   0.0,   0.0,  650.0,    0,     0 }, //9.左大回り
				{ 180.0,   0.0,   0.0,    0.0,   0.0,   0.0,   0.0,  650.0,    0,     0 }, //10.左Uターン
				{  45.0,  22.0,  23.0, 7360.0,   1.0,  32.0,  86.5,  650.0,    1,     0 }, //11.左45°ターン→斜め
				{ 135.0,  27.0, 108.0, 6050.0,   1.0,  87.0,  87.0,  650.0,    1,     0 }, //12.左135°ターン→斜め
				{  45.0,  22.0,  23.0, 7360.0,   1.0,  67.0,  50.0,  650.0,    1,     0 }, //13.斜め→左45°ターン
				{ 135.0,  27.0, 108.0, 6050.0,   1.0,  74.0, 104.0,  650.0,    1,     0 }, //14.斜め→左135°ターン
				{  90.0,  33.0,  57.0, 8320.0,   1.0,  39.0,  60.0,  650.0,    1,     0 }  //15.斜め→左V90°ターン
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
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //右45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //右135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //斜め→右45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //斜め→右135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //斜め→右V90°ターン
				{ 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 0, 0 }, //左小回り
				{ 90.0, 40.0, 50.0, 4300.0, 1.0, 45.0, 62.0, 800.0, 0, 1 }, //左大回り
				{ 180.0, 28.5, 151.5, 4300.0, 1.0, 25.0, 32.0, 800.0, 0, 1 }, //左Uターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //左45°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //左135°ターン→斜め
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //斜め→左45°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 }, //斜め→左135°ターン
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 800.0, 1, 0 } //斜め→左V90°ターン
		}, {
				/*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
				{  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 900.0, 0, 0 }, //右小回り
				{ 90.0, 32.0, 58.0, 8360.0, -1.0, 58.0, 89.0, 900.0, 0, 0 }, //右大回り
				{  0.0,   0.0,   0.0,    0.0,   0.0,   0.0,   0.0,   0.0,     0,     0 }, //右Uターン
				{ 44.7,  22.0,  23.0, 8885.0,  -1.0,  16.0,  72.0,  900.0,    1,     0 }, //右45°ターン→斜め
				{134.0,  45.0,  89.0, 9350.0,  -1.0,  75.0,  78.0,  900.0,    1,     0 }, //右135°ターン→斜め
				{ 44.7,  22.0,  23.0, 8885.0,  -1.0,  56.0,  35.0,  900.0,    1,     0 }, //斜め→右45°ターン
				{134.0,  45.0,  89.0, 9350.0,  -1.0,  60.0,  92.0,  900.0,    1,     0 }, //斜め→右135°ターン
				{ 89.0,  40.0,  49.0, 9650.0,  -1.0,  18.0,  36.0,  900.0,    1,     0 }, //斜め→右V90°ターン	//xxx ←ここ
				{  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 900.0, 0, 0 }, //左小回り
				{ 90.0, 32.0, 58.0, 8360.0, 1.0, 58.0, 93.0, 900.0, 0, 0 }, //左大回り
				{  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 }, //左Uターン
				{ 44.7,  22.0,  23.0, 8885.0,   1.0,  19.0,  70.0,  900.0,    1,     0 }, //左45°ターン→斜め
				{134.0,  45.0,  89.0, 9350.0,   1.0,  78.0,  83.0,  900.0,    1,     0 }, //左135°ターン→斜め
				{ 44.7,  22.0,  23.0, 8885.0,   1.0,  55.0,  31.0,  900.0,    1,     0 }, //斜め→左45°ターン
				{134.0,  45.0,  89.0, 9350.0,   1.0,  65.0, 100.0,  900.0,    1,     0 }, //斜め→左135°ターン
				{ 89.0,  40.0,  49.0, 9650.0,   1.0,  18.0,  38.0,  900.0,    1,     0 } //斜め→左V90°ターン	//xxx ←ここ
		}, {
			  /*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
				{  90.0,   0.0,   0.0,    0.0,   0.0,   0.0,   0.0, 1000.0,    0,     0 }, //右小回り
				{  89.5,  35.0,  54.5, 8750.0,  -1.0,  52.0,  73.0, 1000.0,    0,     0 }, //右大回り	//xxx ←ここ
				{ 179.5,  41.5, 138.0, 5000.0,  -1.0,   3.0,  25.0, 1000.0,    0,     0 }, //右Uターン
				{  44.4,  22.0,  22.2, 9850.0,  -1.0,  13.0,  64.0, 1000.0,    1,     0 }, //右45°ターン→斜め
				{ 134.2,  45.0,  89.2, 9900.0,  -1.0,  59.0,  74.0, 1000.0,    1,     0 }, //右135°ターン→斜め
				{  44.4,  22.0,  22.2, 9850.0,  -1.0,  50.0,  32.0, 1000.0,    1,     0 }, //斜め→右45°ターン
				{ 134.2,  45.0,  89.2, 9900.0,  -1.0,  47.0,  79.0, 1000.0,    1,     0 }, //斜め→右135°ターン
				{  89.1,  40.0,  49.1,12150.0,  -1.0,  16.0,  37.0, 1000.0,    1,     0 }, //斜め→右V90°ターン
				{  90.0,   0.0,   0.0,    0.0,   0.0,   0.0,   0.0, 1000.0,    1,     0 }, //左小回り
				{  89.5,  35.0,  54.5, 8750.0,   1.0,  48.0,  78.0, 1000.0,    1,     0 }, //左大回り	//xxx ←ここ
				{ 179.5,  40.5, 139.0, 5000.0,   1.0,   2.0,  27.0, 1000.0,    1,     0 }, //左Uターン
				{  44.2,  22.0,  22.2, 9850.0,   1.0,   7.0,  83.0, 1000.0,    1,     0 }, //左45°ターン→斜め
				{ 134.2,  45.0,  89.2, 9900.0,   1.0,  62.0,  76.0, 1000.0,    1,     0 }, //左135°ターン→斜め
				{  44.2,  22.0,  22.2, 9850.0,   1.0,  49.0,  33.0, 1000.0,    1,     0 }, //斜め→左45°ターン
				{ 134.2,  45.0,  89.2, 9900.0,   1.0,  48.0,  92.0, 1000.0,    1,     0 }, //斜め→左135°ターン
				{  89.1,  40.0,  49.1,12150.0,   1.0,  20.0,  40.0, 1000.0,    1,     0 } //斜め→左V90°ターン
		}, {
			  /*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1100.0,     0,     0 }, //右小回り
				{  89.2,  40.0,  39.2, 9850.0,  -1.0,   0.0,   0.0, 1100.0,     0,     0 }, //右大回り	//xxx ←ここ
				{ 180.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1100.0,     0,     0 }, //右Uターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1100.0,     1,     0 }, //右45°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1100.0,     1,     0 }, //右135°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1100.0,     1,     0 }, //斜め→右45°ターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1100.0,     1,     0 }, //斜め→右135°ターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1100.0,     1,     0 }, //斜め→右V90°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1100.0,     0,     0 }, //左小回り
				{  89.2,  40.0,  39.2, 9850.0,   1.0,   0.0,   0.0, 1100.0,     0,     0 }, //左大回り	//xxx ←ここ
				{ 180.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1100.0,     0,     0 }, //左Uターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1100.0,     1,     0 }, //左45°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1100.0,     1,     0 }, //左135°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1100.0,     1,     0 }, //斜め→左45°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1100.0,     1,     0 }, //斜め→左135°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1100.0,     1,     0 } //斜め→左V90°ターン
		}, {			//turn_angle_tuning(179.4, 34.0, 8000.0, 5.0, 5.0, -1.0, 1200.0, 1);

			  /*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel,  skew, enable}*/
				{   0.0,   0.0,   0.0,     0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //右小回り
				{  89.2,  40.0,  49.2,  9850.0,  -1.0,  25.0,  73.0, 1200.0,     0,     0 }, //右大回り	//xxx ←ここ
				{ 179.4,  34.0, 145.4,  8000.0,  -1.0,   2.0,  54.0, 1200.0,     0,     0 }, //右Uターン
				{  44.3,  22.0,  22.3, 15650.0,  -1.0,   6.0,  80.0, 1200.0,     1,     0 }, //右45°ターン→斜め
				{ 134.0,  50.0,  84.0,  9950.0,  -1.0,  18.0,  47.0, 1200.0,     1,     0 }, //右135°ターン→斜め
				{  44.3,  22.0,  22.3, 15650.0,  -1.0,  49.0,  38.0, 1200.0,     1,     0 }, //斜め→右45°ターン
				{ 134.0,  50.0,  84.0,  9950.0,  -1.0,   6.0,  62.0, 1200.0,     1,     0 }, //斜め→右135°ターン
				{  89.5,  40.0,  49.5, 14050.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→右V90°ターン
				{   0.0,   0.0,   0.0,     0.0,   1.0,   0.0,   0.0, 1200.0,     0,     0 }, //左小回り
				{  89.2,  40.0,  49.2,  9850.0,   1.0,  30.0,  72.0, 1200.0,     0,     0 }, //左大回り	//xxx ←ここ
				{ 180.0,  32.0, 148.0,  8000.0,   1.0,   5.0,  45.0, 1200.0,     0,     0 }, //左Uターン
				{  44.3,  22.0,  22.3, 15650.0,   1.0,   4.0,  82.0, 1200.0,     1,     0 }, //左45°ターン→斜め
				{ 134.0,  50.0,  84.0,  9950.0,   1.0,  20.0,  47.0, 1200.0,     1,     0 }, //左135°ターン→斜め
				{  44.3,  22.0,  22.3, 15650.0,   1.0,  51.0,  38.0, 1200.0,     1,     0 }, //斜め→左45°ターン
				{ 134.0,  50.0,  84.0,  9950.0,   1.0,   8.0,  59.0, 1200.0,     1,     0 }, //斜め→左135°ターン
				{  89.5,  40.0,  49.5, 14050.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 } //斜め→左V90°ターン
		}, {
				  /*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //右小回り
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //右大回り	//xxx ←ここ
				{ 180.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //右Uターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //右45°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //右135°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→右45°ターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→右135°ターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→右V90°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     0,     0 }, //左小回り
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //左大回り	//xxx ←ここ
				{ 180.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     0,     0 }, //左Uターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //左45°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //左135°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→左45°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→左135°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 } //斜め→左V90°ターン
		}, {
				  /*{     θ,    θ1,    θ2, angacc,  wise, front,  rear,    vel, skew, enable}*/
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //右小回り
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //右大回り	//xxx ←ここ
				{ 180.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //右Uターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //右45°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //右135°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→右45°ターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→右135°ターン
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→右V90°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     0,     0 }, //左小回り
				{   0.0,   0.0,   0.0,    0.0,  -1.0,   0.0,   0.0, 1200.0,     0,     0 }, //左大回り	//xxx ←ここ
				{ 180.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     0,     0 }, //左Uターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //左45°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //左135°ターン→斜め
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→左45°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 }, //斜め→左135°ターン
				{   0.0,   0.0,   0.0,    0.0,   1.0,   0.0,   0.0, 1200.0,     1,     0 } //斜め→左V90°ターン
		} };

//volatile float_parameters_t angle1, angle2, dit_bef, dist_aft;

//volatile signed short int sample[2500] = { 0 };		//データログのデータ型によって適宜変更する！
//volatile float sample1[800] = { 0.0 }, sample2[800] = { 0.0 };
volatile float sample1[SAMPLE_NUMBER] = { 0.0 }, sample2[SAMPLE_NUMBER] = { 0.0 };
volatile float Log1[LOG_NUMBER] = {0.0}, Log2[LOG_NUMBER] = {0.0}, Log3[LOG_NUMBER] = {0.0},
		Log4[LOG_NUMBER] = {0.0}, Log5[LOG_NUMBER] = {0.0}/*, Log6[LOG_NUMBER] = {0.0}, Log7[LOG_NUMBER] = {0.0},
		Log8[LOG_NUMBER] = {0.0}, Log9[LOG_NUMBER] = {0.0}*//*, Log10[LOG_NUMBER] = {0.0}, Log11[LOG_NUMBER] = {0.0}*/ ;

void assign_parameters(char para_mode) {
	if (para_mode == 1) {
		vel_low = 500.0;
		vel_high = 800.0;
		accel_normal = 5000.0;
		aa = 0;	//小回り速度500
		bb = 3;	//大回り速度800
		cc = 1;	//斜め速度(nasi)
	} else if (para_mode == 2) {
		vel_low = 650.0;
		vel_high = 800.0;
		accel_normal = 6000.0;
		aa = 1;	//速度650
		bb = 3;	//速度800
		cc = 1;	//斜め速度650
	} else if (para_mode == 3) {
		vel_low = 750.0;
		vel_high = 800.0;
		accel_normal = 7000.0;
		aa = 2;	//速度750
		bb = 3;	//速度900
		cc = 1;	//斜め速度650
	} else if (para_mode == 4) {
		vel_low = 900.0;
		vel_high = 1000.0;
		accel_normal = 6000.0;
		aa = 4;	//速度900(小回り)
		bb = 5;	//大廻り速度1000
		cc = 4;	//斜め速度900
	} else if (para_mode == 5) {
		vel_low = 900.0;
		vel_high = 1200.0;
		accel_normal = 6000.0;
		aa = 4;	//速度900(小回り)
		bb = 7;	//大廻り速度1200
		cc = 4;	//斜め速度900
	} else if (para_mode == 6) {
		vel_low = 1000.0;
		vel_high = 1000.0;
		accel_normal = 7000.0;
		aa = 4;	//速度900(小回り)
		bb = 5;	//大廻り速度1000
		cc = 5;	//斜め速度1000
	} else if (para_mode == 7) {
		vel_low = 1000.0;
		vel_high = 1200.0;
		accel_normal = 7000.0;
		aa = 4;	//速度900(小回り)
		bb = 7;	//大廻り速度1200
		cc = 5;	//斜め速度1000
	}

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
	volatile char qx;
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

void pass_walkmap_display() {
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

			if (is_saved_wall_exist(xj, yj, 3) == 1) {	//西壁を読む
				myprintf("|");
			} else {
				myprintf(" ");
			}
			myprintf("%3d", map[xj][yj]);
		}
		myprintf("|\n\r");

		for (xj = 0; xj <= x_size; xj++) {
			if (is_saved_wall_exist(xj, yj, 2) == 1) {
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
		if (is_saved_wall_exist(xj, 0, 3) == 1) {
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

void Unknown_Wall_Pass_display() {
	volatile int j, yj, xj;
	myprintf("+");
	for (j = 0; j <= 14; j++) {
		myprintf("---+");
	}
	myprintf("---+\n\r");

	for (yj = 15; yj > 0; yj--) {
		myprintf("|   ");
		for (xj = 1; xj <= 15; xj++) {
			if (is_Exist_Unknown_Wall(xj, yj, 3) == 1) {
				myprintf("|   ");
			} else {
				myprintf("    ");
			}
		}
		myprintf("|\n\r");

		for (xj = 0; xj <= 15; xj++) {
			if (is_Exist_Unknown_Wall(xj, yj, 2) == 1) {
				myprintf("+---");
			} else {
				myprintf("+   ");
			}
		}
		myprintf("+\n\r");
	}
	myprintf("|   ");
	for (xj = 1; xj <= 15; xj++) {
		if (is_Exist_Unknown_Wall(xj, 0, 3) == 1) {
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


float kabekire_dist_right(float hikisuu_vel) {
	float adjust_right;
	if(skew_kabekire.bit1==1){	//値大⇒進む距離大
		if (hikisuu_vel <= 700.0) {
			adjust_right = 16.0;	//。
		} else if (hikisuu_vel <= 900.0) {
			adjust_right = 9.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_right = 12.0;	//。
		} else if (hikisuu_vel <= 1100.0) {
			adjust_right = 12.0;	//テキトー。
		} else if (hikisuu_vel <= 1200.0) {
			adjust_right = 12.0;	//テキトー。
		} else if (hikisuu_vel > 1200.0) {
			adjust_right = 5.0;	//これはテキトー。
		}

	}else if (daikei_mode == 1) {	//探索用
		if (hikisuu_vel <= 500.0) {
			adjust_right = 87.0;	//84
		} else if (hikisuu_vel <= 700.0) {
			adjust_right = 87.0;	//。
		} else if (hikisuu_vel <= 800.0) {
			adjust_right = 87.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_right = 94.0;	//。
		} else if (hikisuu_vel <= 1200.0) {
			adjust_right = 89.0;	//これはテキトー。
		} else if (hikisuu_vel > 1200.0){

		}
	} else if (daikei_mode == 2) {
		if (hikisuu_vel <= 500.0) {
			adjust_right = 3.0;
		} else if (hikisuu_vel <= 700.0) {
			adjust_right = 3.0;	//。
		} else if (hikisuu_vel <= 800.0) {
			adjust_right = 3.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_right = -4.0;	//。
		} else if (hikisuu_vel <= 1200.0) {
			adjust_right = 1.0;	//。
		} else if (hikisuu_vel <= 1400.0) {
			adjust_right = 1.0;	//テキトー。
		} else if (hikisuu_vel > 1400.0) {
			adjust_right = 0.0;	//これはテキトー。
		}
	}

	return adjust_right;
}
float kabekire_dist_left(float hikisuu_vel) {
	float adjust_left;
	if(skew_kabekire.bit1==1){	//値大⇒進む距離大
		if (hikisuu_vel <= 700.0) {
			adjust_left = 16.0;	//。
		} else if (hikisuu_vel <= 900.0) {
			adjust_left = 6.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_left = 4.0;	//。
		} else if (hikisuu_vel <= 1100.0) {
			adjust_left = 4.0;	//これはテキトー。
		} else if (hikisuu_vel <= 1200.0) {
			adjust_left = 4.0;	//これはテキトー。
		} else if (hikisuu_vel > 1200.0) {
			adjust_left = 2.0;	//これはテキトー。
		}

	}else if (daikei_mode == 1) {	//探索用
		if (hikisuu_vel <= 500.0) {
			adjust_left = 92.0;	//87.0
		} else if (hikisuu_vel <= 700.0) {
			adjust_left = 92.0;	//。
		} else if (hikisuu_vel <= 800.0) {
			adjust_left = 93.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_left = 94.0;	//。
		} else if (hikisuu_vel <= 1200.0) {
			adjust_left = 95.0;	//。
		} else if (hikisuu_vel > 1200.0) {
			adjust_left = 95.0;	//これはテキトー。
		}
	} else if (daikei_mode == 2) {
		if (hikisuu_vel <= 500.0) {
			adjust_left =  -2.0;
		} else if (hikisuu_vel <= 700.0) {
			adjust_left =  -2.0;	//。
		} else if (hikisuu_vel <= 800.0) {
			adjust_left =  -3.0;	//。
		} else if (hikisuu_vel <= 1000.0) {
			adjust_left =  -4.0;	//。
		} else if (hikisuu_vel <= 1200.0) {
			adjust_left =  -5.0;	//。
		} else if (hikisuu_vel <= 1400.0) {
			adjust_left =  -5.0;	//これはテキトー。
		} else if (hikisuu_vel > 1400.0) {
			adjust_left =  -5.0;	//これはテキトー。
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
		error = 60.0;	//袋小路からの壁切れを読むため
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
			if(balance_velocity < vmax - 200.0){	//xxx 追加したが要確認
				wall_control = 0;
				LED1=1;//debug
			}else{
				wall_control = hikisuu_wall;
				LED1=0;//debug
			}
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
//				LED_V1 = 1;
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
				LED_V4 = 1;
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
//				LED_V4 = 1;
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
	kabekire_enable_2=0;
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void detect_kabekire(float dist,float hikisuu_vel){	//ideal_b_dist, b_dist, adj_bfor_distを計算
	//dist:台形関数が進むべき距離
	float adj_dist;
	if (flags_kabekire.detect_r == 1) {	//壁切れによる補正
		adj_dist = kabekire_dist_right(hikisuu_vel);
		balance_distance =  dist - adj_dist;//壁切れ補正(1.0は決め打ち)//変更！
		ideal_balance_distance =  dist - adj_dist;//壁切れ補正//変更！

		adjust_distance = kabekire_dist_right(hikisuu_vel);
//					LED2 = 1;	//debug
		LED_V3 = 1;	//debug
//					LED_V3 = 1;
		flags_kabekire.detect_r=0;
		flags_kabekire.detect_l=0;
		flags_kabekire.enable1=0;
		flags_kabekire.enable2=0;
		flags_kabekire.detected = 1;	//壁切れは検出済み
		kabekire_LED = 1;
	} else if (flags_kabekire.detect_l == 1) {	//壁切れによる補正
		adj_dist = kabekire_dist_left(hikisuu_vel);
		balance_distance = dist - adj_dist;//壁切れ補正(6.0は決め打ち)変更！
		ideal_balance_distance = dist - adj_dist;//壁切れ補正変更！

		adjust_distance = kabekire_dist_right(hikisuu_vel);
		LED_V4 = 1;
		flags_kabekire.detect_r=0;
		flags_kabekire.detect_l=0;
		flags_kabekire.enable1=0;
		flags_kabekire.enable2=0;
		flags_kabekire.detected = 1;	//壁切れは検出済み
		kabekire_LED = 1;
	}
	if (adj_dist < 0.0) {
		adjust_before_dist = adj_dist;
//					LED_V1 = 1;//debug
	} else {
		adjust_before_dist = 0.0;
//					LED_V4 = 1;//debug
	}

}

void daikei_for_pass_EX(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kebekire) {
	float run_dist;
	daikei_mode = 2;	//壁切れ補正関数用変数
	if(flags_kabekire.wait==1 && hikisuu_kebekire==1){
		run_dist = hikisuu_dist - 20.0;	//20.0前には壁切れを読むために減速しきる。
	}else{
		run_dist = hikisuu_dist;
	}

	flags_kabekire.enable1 = 0;
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

	flags_kabekire.enable2 = hikisuu_kebekire;	//壁切れを読むかの判定

	if (run_dist
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
			if (balance_distance > hikisuu_dist - 50.0) {//終了距離の直前でしか壁切れは読まない。50.0はハードコーディング
				if (flags_kabekire.enable2 == 1) {
					flags_kabekire.enable1 = 1;	//CMT内でkabekire_rightなどを立てられるようにする
				}
				detect_kabekire(hikisuu_dist, vterm);	//ideal_b_dist, b_dist, adj_bfor_distを計算,壁切れ検出
			}

			if (ideal_balance_distance
					<= run_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0
									/ hikisuu_accel) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					> run_dist / 2.0
							+ (vterm * vterm - v_0 * v_0) / 4.0 / hikisuu_accel) {

				if (balance_distance <= run_dist) {
					ideal_balance_accel = -1.0 * hikisuu_accel;
				} else {
					ideal_balance_accel = 0.0;
					ideal_balance_velocity = vterm;
				}

				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了(0で終わる場合)
					ideal_balance_accel = 0.0;
					ideal_balance_velocity = 0.0;
					break;
				}

				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					if (flags_kabekire.wait == 1 && flags_kabekire.detected == 0
							&& hikisuu_kebekire == 1) {	//壁切れを読むまで待つ!!!hikisuu_kebekire==1は必要（前距離等での誤動作防止10/10）
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = vterm;
					} else {
						LED_V1=1;
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = vterm;
						break;
					}
				}
			}			/*else if (ideal_balance_distance > hikisuu_dist) {	//理想速度が0以下となる瞬間に速度を0とする
			 ideal_balance_accel = 0.0;
			 //				ideal_balance_velocity = 0.0;	//後にslalom関数などが続くことを考えて、速度ゼロにはしない
			 break;
			 }*/
		}
	} else if (run_dist
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
			if (balance_distance > hikisuu_dist - 50.0) {//終了距離の直前でしか壁切れは読まない。50.0はハードコーディング
				if (flags_kabekire.enable2 == 1) {
					flags_kabekire.enable1 = 1;	//CMT内でkabekire_rightなどを立てられるようにする
				}
				detect_kabekire(hikisuu_dist, vterm);	//ideal_b_dist, b_dist, adj_bfor_distを計算,壁切れ検出

			}

			if (ideal_balance_distance
					< ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)) {
				ideal_balance_accel = hikisuu_accel;
			} else if (ideal_balance_distance
					>= ((vmax * vmax - v_0 * v_0) / 2.0 / hikisuu_accel)
					&& ideal_balance_distance
							<= (run_dist
									- (vmax * vmax - vterm * vterm) / 2.0
											/ hikisuu_accel)) {
				ideal_balance_accel = 0.0;
				ideal_balance_velocity = vmax;
			} else if (ideal_balance_distance//実測値のbalance_distanceをもとにして制御するプログラム。
					> (run_dist
							- (vmax * vmax - vterm * vterm) / 2.0
									/ hikisuu_accel)) {
				if(vmax == vterm){	//斜めなど、等速直進では減速しないようにする
					ideal_balance_accel = 0.0;
					ideal_balance_velocity = vmax;
				}else{
					if(balance_distance <= run_dist){
						ideal_balance_accel = -1.0 * hikisuu_accel;
					}else{
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = vterm;
					}
				}
				if (balance_velocity <= 0.0) {	//実際の速度が負となったら台形加速終了
					ideal_balance_accel = 0.0;
					ideal_balance_velocity = 0.0;
					break;
				}
				if (balance_distance > hikisuu_dist) {	//走行距離が理想値を超えたらbreak
					ideal_balance_accel = 0.0;
					if (flags_kabekire.wait == 1 && flags_kabekire.detected == 0
							&& hikisuu_kebekire == 1) {	//壁切れを読むまで待つ!!!hikisuu_kebekire==1は必要（前距離等での誤動作防止10/10）
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = vterm;
					} else {
						ideal_balance_accel = 0.0;
						ideal_balance_velocity = vterm;
						break;
					}
//						LED_V4 = 1;//debug1001
//						if (kabekire_read_flag == 0) {//debug1001
//							LED_V2 = 1;
//						}
//						if (wait_kabekire == 1) {//debug1001
//							LED_V1 = 1;
//						}
				}
			}
		}
	}
	flags_kabekire.detected=0;	//壁切れを読まなかった場合にも次の壁切れを正常に読むため
//	flags_kabekire.wait=0;
	flags_kabekire.enable1 = 0;	//xx　パスのバグ要因となっていたので確認せよ→確認済み
	flags_kabekire.enable2 = 0;	//xx　パスのバグ要因となっていたので確認せよ→確認済み
	ei_flag_center = 0;
	ei_flag_rot = 0;
//	LED_V3=0;
//	LED_V4=0;
}


void daikei_for_pass_kai2(float hikisuu_dist, float vmax, float hikisuu_accel,
		float v_0, float vterm, char hikisuu_wall, char hikisuu_kabekire) {	//20170928(台形加速の距離の考え方を変える)
	volatile char k_enable_enable = 0;	//クソ頭悪いフラグなので北信越語に直そうな…
	daikei_mode = 2;	//探索用モード

	kabekire_enable_2 = 0;
	k_enable_enable = hikisuu_kabekire;

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
				if (k_enable_enable == 1) {
					kabekire_enable_2 = 1;	//CMT内でkabekire_rightなどを立てられるようにする
				}

				if (kabekire_right == 1) {	//壁切れによる補正
					adjust_distance = kabekire_dist_right(vterm);
					balance_distance =  hikisuu_dist - adjust_distance;//壁切れ補正(1.0は決め打ち)//変更！
					ideal_balance_distance =  hikisuu_dist - adjust_distance;//壁切れ補正//変更！

//					LED2 = 1;	//debug
//					LED3 = 1;	//debug

//					LED_V3 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_right == 2) {	//櫛切れによる補正(現在不使用-20170928)
					adjust_distance = kabekire_dist_right(vterm);
					balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正(1.0は決め打ち)//変更！
					ideal_balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正//変更！

//					LED_V1 = 1;
//					LED2 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_left == 1) {	//壁切れによる補正
					adjust_distance = kabekire_dist_left(vterm);
					balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正(6.0は決め打ち)変更！
					ideal_balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正変更！

					LED2 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
					kabekire_read_flag = 1;
				} else if (kabekire_left == 2) {	//櫛切れによる補正(現在不使用-20170912)
					adjust_distance = kabekire_dist_left(vterm);
					balance_distance = hikisuu_dist - adjust_distance;	//壁切れ補正変更！
					ideal_balance_distance = hikisuu_dist - adjust_distance;//壁切れ補正変更！

					LED_V4 = 1;
					kabekire_left = 0;
					kabekire_right = 0;
					kabekire_enable_2 = 0;
					k_enable_enable = 0;	//一度壁切れを読むと、この台形加速内では読めなくする。
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

void turn_for_pass(float hikisuu_angle, float angle1, float angle2,
		float omega_max, float hikisuu_angacc, float unclock_wise,
		float hikisuu_balance_velocity, float dist1, float dist2) {
	ideal_angle = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	Erorr_rot.i = 0.0;
	ei_flag_center = 1;
	q_dist_flag = 0;
	if(dist1 < 0){
		dist1 = 0.01;
	}
//	test_daikei(dist1, hikisuu_balance_velocity, 10.0, hikisuu_balance_velocity, hikisuu_balance_velocity, 0);
	daikei_for_pass_EX(dist1, hikisuu_balance_velocity, 6000.0,
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
	daikei_for_pass_EX(dist2, hikisuu_balance_velocity, 6000.0,
			hikisuu_balance_velocity, hikisuu_balance_velocity, 1, 1);	//壁切れ読む
	flags_kabekire.wait=0;	//xxx 追加20171031
	ei_flag_center = 0;
	ei_flag_rot = 0;
}

void turn_for_skew_pass(float hikisuu_angle, float angle1, float angle2,
		float omega_max, float hikisuu_angacc, float unclock_wise,
		float hikisuu_v_init, float hikisuu_v_skew, float hikisuu_v_term, float dist1, float dist2, char h_skew_kabekire) {
	volatile char local_wall_control = 0;
	ideal_angle = 0.0;
	ideal_omega = 0.0;
	ideal_omega2 = 0.0;	//必要。
	Erorr_rot.i = 0.0;
	ei_flag_center = 1;
	q_dist_flag = 0;
	if(h_skew_kabekire == 1){	//後が斜めの場合は斜め壁切れにする。
		skew_kabekire.bit1=1;
	}else{
		local_wall_control = 1;
	}
	daikei_for_pass_EX(dist1, hikisuu_v_init, 6000.0,
			hikisuu_v_init, hikisuu_v_skew, 0, 0);//壁切れ読まない	//普通はv_initのほうが大きい
	ei_flag_rot = 1;
	ideal_balance_velocity = hikisuu_v_skew;
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
	daikei_for_pass_EX(dist2, hikisuu_v_term, 6000.0,
			hikisuu_v_skew, hikisuu_v_term, local_wall_control, 1);	//壁切れ読む	//v_termの方が大きいと想定
	flags_kabekire.wait=0;	//xxx 追加20171031
	skew_kabekire.bit1=0;
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
void Search_UnknownWall_Pass(char hikisuu_goal_x, char hikisuu_goal_y) {	//仮想足立により、道壁を記録する
	volatile char ii,sx,sy, s_direction;
	sx = x;
	sy = y;
	s_direction = direction_count;

	direction_count = 0;		//スタート地点を考えている。
	x = 0;
	y = 1;
	q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
	Reset_Unknown_Wall();

	while (1) {
		switch (direction_count) {
		case 0:
			if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//North & 直進
				if(is_the_Wall_watched(x, y, 0) == 0){
					Add_UnknownWall_Front(x, y, 0);
				}
				direction_xy();
//				myprintf("direc=0,straight\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);

			} else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//North & 右折
				if(is_the_Wall_watched(x, y, 1) == 0){
					Add_UnknownWall_Front(x, y, 1);
				}
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
//				myprintf("direc=0,right\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);

			} else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//North & 左折
				if(is_the_Wall_watched(x, y, 3) == 0){
					Add_UnknownWall_Front(x, y, 3);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
//				myprintf("direc=0,left\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);

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
//				myprintf("direc=0,else\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);
				direction_xy();
			}
			break;

		case 1:
			if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//East & 直進
				if(is_the_Wall_watched(x, y, 1) == 0){
					Add_UnknownWall_Front(x, y, 1);
				}

				direction_xy();

			} else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//East & 右折
				if(is_the_Wall_watched(x, y, 2) == 0){
					Add_UnknownWall_Front(x, y, 2);
				}

				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

			} else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//East & 左折
				if(is_the_Wall_watched(x, y, 0) == 0){
					Add_UnknownWall_Front(x, y, 0);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
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
				direction_xy();
			}
			break;

		case 2:
			if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//South & 直進
				if(is_the_Wall_watched(x, y, 2) == 0){
					Add_UnknownWall_Front(x, y, 2);
				}

				direction_xy();

			} else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//South & 右折
				if(is_the_Wall_watched(x, y, 3) == 0){
					Add_UnknownWall_Front(x, y, 3);
				}

				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

			} else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//South & 左折
				if(is_the_Wall_watched(x, y, 1) == 0){
					Add_UnknownWall_Front(x, y, 1);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
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
				direction_xy();
			}
			break;

		case 3:
			if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//West & 直進
				if(is_the_Wall_watched(x, y, 3) == 0){
					Add_UnknownWall_Front(x, y, 3);
				}

				direction_xy();

			} else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//West & 右折
				if(is_the_Wall_watched(x, y, 0) == 0){
					Add_UnknownWall_Front(x, y, 0);
				}

				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

			} else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//West & 左折
				if(is_the_Wall_watched(x, y, 2) == 0){
					Add_UnknownWall_Front(x, y, 2);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
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
				direction_xy();
			}
			break;
		}
		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {

			break;
		}

	}
	x = sx;	//元に戻す
	y = sy;
	direction_count = s_direction;

}

void Search_UnknownWall_Pass_R(char hikisuu_goal_x, char hikisuu_goal_y) {	//仮想足立により、道壁を記録する
	volatile char ii,sx,sy, s_direction;
	sx = x;
	sy = y;
	s_direction = direction_count;

	direction_count = 0;		//スタート地点を考えている。
	x = 0;
	y = 1;
	q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
	Reset_Unknown_Wall();

	while (1) {
		switch (direction_count) {
		case 0:
			if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//North & 右折
				if(is_the_Wall_watched(x, y, 1) == 0){
					Add_UnknownWall_Front(x, y, 1);
				}
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
//				myprintf("direc=0,right\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);

			}else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//North & 直進
				if(is_the_Wall_watched(x, y, 0) == 0){
					Add_UnknownWall_Front(x, y, 0);
				}
				direction_xy();
//				myprintf("direc=0,straight\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);

			} else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//North & 左折
				if(is_the_Wall_watched(x, y, 3) == 0){
					Add_UnknownWall_Front(x, y, 3);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
//				myprintf("direc=0,left\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);

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
//				myprintf("direc=0,else\r\n");
//				myprintf("x=%d,y=%d\r\n",x,y);
				direction_xy();
			}
			break;

		case 1:
			if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//East & 右折
				if(is_the_Wall_watched(x, y, 2) == 0){
					Add_UnknownWall_Front(x, y, 2);
				}

				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

			}else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//East & 直進
				if(is_the_Wall_watched(x, y, 1) == 0){
					Add_UnknownWall_Front(x, y, 1);
				}

				direction_xy();

			} else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//East & 左折
				if(is_the_Wall_watched(x, y, 0) == 0){
					Add_UnknownWall_Front(x, y, 0);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
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
				direction_xy();
			}
			break;

		case 2:
			if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//South & 右折
				if(is_the_Wall_watched(x, y, 3) == 0){
					Add_UnknownWall_Front(x, y, 3);
				}

				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

			}else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//South & 直進
				if(is_the_Wall_watched(x, y, 2) == 0){
					Add_UnknownWall_Front(x, y, 2);
				}

				direction_xy();

			} else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//South & 左折
				if(is_the_Wall_watched(x, y, 1) == 0){
					Add_UnknownWall_Front(x, y, 1);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
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
				direction_xy();
			}
			break;

		case 3:
			if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//West & 右折
				if(is_the_Wall_watched(x, y, 0) == 0){
					Add_UnknownWall_Front(x, y, 0);
				}

				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

			}else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//West & 直進
				if(is_the_Wall_watched(x, y, 3) == 0){
					Add_UnknownWall_Front(x, y, 3);
				}

				direction_xy();

			} else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//West & 左折
				if(is_the_Wall_watched(x, y, 2) == 0){
					Add_UnknownWall_Front(x, y, 2);
				}

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
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
				direction_xy();
			}
			break;
		}
		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {

			break;
		}

	}
	x = sx;	//元に戻す
	y = sy;
	direction_count = s_direction;

}


void new_serch_algorithm(char hikisuu_goal_x, char hikisuu_goal_y, char start_x,
		char start_y, char para_mode) {		//10/28改訂。
	volatile float V_search, ACCEL_search, V_max;
	volatile char aaa, straight_count = 0, i, goal_size = 0, unknown_wall_finish = 0;

	if (para_mode == 1) {
		V_search = 500.0;
		ACCEL_search = 5000.0;
		V_max = 900.0;
		aaa = 0;	//小回り速度500
	} else if (para_mode == 2) {
		V_search = 650.0;
		ACCEL_search = 6500.0;
		V_max = 1000.0;
		aaa = 1;	//小回り速度650
	}
	column_temp[0] |= 1;
	goal_size = 0;

	x = start_x;
	y = start_y;
	wait(300); 			//励磁直後は少し待つ！

	test_daikei(90.0, V_search, ACCEL_search, 0.0, V_search, 0);
	direction_xy();
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
		Search_UnknownWall_Pass(goal_x, goal_y);//探索中のpassに基づいて未知壁をセットする
		Search_UnknownWall_Pass_R(goal_x, goal_y);//探索中のpassに基づいて未知壁をセットする
		Reset_Temp_Goal();						//xxx 多分合っているが、大きい迷路で確認したいね
		Set_Temp_Goal();											//通った未知壁情報をもとにゴール座標をセット
		q_new_walk_map_maker(0, 0, goal_size, x, y);				//goal_size==0の場合、(0,0)はゴール座標としてセットされない。
//		q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
		LED1 = 0;

		if (map[x][y] == 255) {		//閉じ込められたらスタートに戻る or fail-safe
			if (unknown_wall_finish == 0) {
				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				ideal_balance_velocity = 0;
				wait(300);
				LED_motion1();

				goal_size = 1;	//これでスタート地点がゴールとして設定される(goal_size=0ではゴール判定されない為)

				LED1 = 1;
				Search_UnknownWall_Pass(goal_x, goal_y);//探索中のpassに基づいて未知壁をセットする
				Search_UnknownWall_Pass_R(goal_x, goal_y);//探索中のpassに基づいて未知壁をセットする
				Reset_Temp_Goal();					//xxx 多分合っているが、大きい迷路で確認したいね
				Set_Temp_Goal();						//通った未知壁情報をもとにゴール座標をセット
				q_new_walk_map_maker(0, 0, goal_size, x, y);//goal_size==0の場合、(0,0)はゴール座標としてセットされない。
				LED1 = 0;
				balance_distance = 0.0;
				switch (direction_count) {
				case 0:
					if (y < 15 && map[x][y + 1] < map[x][y]&& is_Exist_Wall(x, y, 0) == 0) {	//North & 直進
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						wait(100);
						test_daikei(180.0, V_search, 3000.0, 0.0, V_search,1);
						direction_xy();
					} else if (x < 15 && map[x + 1][y] < map[x][y]&& is_Exist_Wall(x, y, 1) == 0) {//North & 右折
						if (direction_count == 3) {
							direction_count = 0;
						} else {
							direction_count++;
						}
						test_turn(90, 5000, 2500, -1.0, 0.0);	//右回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);	//
						direction_xy();

					} else if (x > 0 && map[x - 1][y] < map[x][y]&& is_Exist_Wall(x, y, 3) == 0) {//North & 左折
						if (direction_count == 0) {
							direction_count = 3;
						} else {
							direction_count--;
						}
						test_turn(90, 5000, 2500, 1.0, 0.0);	//左回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);	//
						direction_xy();
					}else {
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
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
						direction_xy();
					}
					break;

				case 1:
					if (x < 15 && map[x + 1][y] < map[x][y]&& is_Exist_Wall(x, y, 1) == 0) {	//East & 直進
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, 3000.0, 0.0, V_search,1);
						direction_xy();
					} else if (y > 0 && map[x][y - 1] < map[x][y]&& is_Exist_Wall(x, y, 2) == 0) {//East & 右折
						if (direction_count == 3) {
							direction_count = 0;
						} else {
							direction_count++;
						}
						test_turn(90, 5000, 2500, -1.0, 0.0);		//右回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);		//
						direction_xy();

					} else if (y < 15 && map[x][y + 1] < map[x][y]&& is_Exist_Wall(x, y, 0) == 0) {//East & 左折
						if (direction_count == 0) {
							direction_count = 3;
						} else {
							direction_count--;
						}
						test_turn(90, 5000, 2500, 1.0, 0.0);		//左回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);		//
						direction_xy();
					}else {
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
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(150);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
						direction_xy();
					}

					break;

				case 2:
					if (y > 0 && map[x][y - 1] < map[x][y]&& is_Exist_Wall(x, y, 2) == 0) {		//South & 直進
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, 3000.0, 0.0, V_search,1);
						direction_xy();
					} else if (x > 0 && map[x - 1][y] < map[x][y]&& is_Exist_Wall(x, y, 3) == 0) {//South & 右折
						if (direction_count == 3) {
							direction_count = 0;
						} else {
							direction_count++;
						}
						test_turn(90, 5000, 2500, -1.0, 0.0);		//右回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);		//
						direction_xy();

					} else if (x < 15 && map[x + 1][y] < map[x][y]&& is_Exist_Wall(x, y, 1) == 0) {//South & 左折
						if (direction_count == 0) {
							direction_count = 3;
						} else {
							direction_count--;
						}
						test_turn(90, 5000, 2500, 1.0, 0.0);		//左回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);		//
						direction_xy();
					}else {
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
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(150);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
						direction_xy();
					}

					break;

				case 3:
					if (x > 0 && map[x - 1][y] < map[x][y]&& is_Exist_Wall(x, y, 3) == 0) {		//West & 直進
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						wait(100);
						test_daikei(180.0, V_search, 3000.0, 0.0, V_search,1);
						direction_xy();
					} else if (y < 15 && map[x][y + 1] < map[x][y]&& is_Exist_Wall(x, y, 0) == 0) {//West & 右折
						if (direction_count == 3) {
							direction_count = 0;
						} else {
							direction_count++;
						}
						test_turn(90, 5000, 2500, -1.0, 0.0);		//右回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);		//
						direction_xy();

					} else if (y > 0 && map[x][y - 1] < map[x][y]&& is_Exist_Wall(x, y, 2) == 0) {//West & 左折
						if (direction_count == 0) {
							direction_count = 3;
						} else {
							direction_count--;
						}
						test_turn(90, 5000, 2500, 1.0, 0.0);		//左回転
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(100);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);		//
						direction_xy();
					}else {
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
						wait(150);
						reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
						ideal_balance_velocity = 0.0;
						balance_distance=0.0;
						wait(150);
						test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
						direction_xy();
					}

					break;
				}

//				LED1 =1;	//debug
//				LED_V1=1;	//debug
				unknown_wall_finish = 1;
			} else {
				fail_flag = 1;
			}
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
		}

		switch (direction_count) {
		case 0:
			if (y < 15&& map[x][y + 1]
			< map[x][y]&& sen.right_front < r_front_wall_judge) {	//North & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y < 15
							&& map[x][y + straight_count]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x, y + straight_count, 2) == 1
							&& is_Exist_Wall(x, y + straight_count, 2) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				}else{
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;

			} else if (x < 15&& map[x + 1][y]
			< map[x][y] && sen.right_side < r_wall_judge) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (x > 0&& map[x - 1][y]
			< map[x][y]&& sen.left_side < l_wall_judge) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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
				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 1:
			if (x
					< 15&& map[x+1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//East & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (x < 15
							&& map[x + straight_count][y]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x + straight_count, y, 3) == 1
							&& is_Exist_Wall(x + straight_count, y, 3) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				}else{
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
//				test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
//				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_side < r_wall_judge) {//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.left_side < l_wall_judge) {//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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
				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 2:
			if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_front < r_front_wall_judge) {//South & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y > 0
							&& map[x][y - straight_count]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x, y - straight_count, 0) == 1
							&& is_Exist_Wall(x, y - straight_count, 0) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				} else {
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
//				test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
//				direction_xy();

			} else if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_side < r_wall_judge) {//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (x
					< 15&& map[x+1][y] < map[x][y] && sen.left_side < l_wall_judge) {//South & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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

				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 3:
			if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//West & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (x > 0
							&& map[x - straight_count][y]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x - straight_count, y, 1) == 1
							&& is_Exist_Wall(x - straight_count, y, 1) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				}else{
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
//				direction_xy();
//				test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.right_side < r_wall_judge) {//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.left_side < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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
				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		}
		LED_V1 = 0;
		LED_V2 = 0;
		LED_V3 = 0;
		LED_V4 = 0;

		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			if (sen.right_front >= r_front_wall_judge) {	//前センサーの壁判断
				add_wall_front(x, y, direction_count);
			}
			if (sen.right_side >= r_wall_judge) {	//右センサーの壁判断
				add_wall_right(x, y, direction_count);
			}
			if (sen.left_side >= l_wall_judge) {	//左センサーの壁判断
				add_wall_left(x, y, direction_count);
			}
			break;
		}
	}
	test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
}



void adachihou_q(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search) {		//足立法を改造したもの
	volatile float r_before, l_before, r_after, l_after;
	volatile char goal_size;
	r_before = 10.5;
	r_after = 12.0;
	l_before = 7.8;
	l_after = 23.0;

	column_temp[0] |= 1;
	kabekire_right = 0;	//台形での誤作動防止のため
	kabekire_left = 0;	//台形での誤作動防止のため
	reverse_flag = 0;	//台形での誤作動防止のため

	x = start_x;
	y = start_y;
	wait(300); 			//励磁直後は少し待つ！

//	balance_distance = 0.0;	//xxx 足立法のバグの原因?

	q_dist_flag = 0;
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

//xxx 4マスゴール対応時に追加。既知壁情報
		goal_size=2;
		if(goal_size ==2){
			watched_wall_front(hikisuu_goal_x,     hikisuu_goal_y, 0);
			watched_wall_right(hikisuu_goal_x,     hikisuu_goal_y, 0);
			watched_wall_front(hikisuu_goal_x + 1, hikisuu_goal_y, 0);
			watched_wall_right(hikisuu_goal_x, hikisuu_goal_y + 1, 0);
		}


		if (x == goal_x && y == goal_y) {		//以下、クシつぶし

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

//		total_dist = 0.0;	DCでは不要
		LED1 = 1;
//		q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
		q_new_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y, goal_size, x, y);
		LED1 = 0;
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

		if ((x == hikisuu_goal_x && y == hikisuu_goal_y) ||
				(x == hikisuu_goal_x + 1 && y == hikisuu_goal_y) ||
				(x == hikisuu_goal_x     && y == hikisuu_goal_y + 1) ||
				(x == hikisuu_goal_x + 1 && y == hikisuu_goal_y + 1) ) {
			switch (direction_count) {

			case 0:		//North
				Pass_Goal_x = x;
				Pass_Goal_y = y + 1;
				break;
			case 1:		//East
				Pass_Goal_x = x + 1;
				Pass_Goal_y = y;
				break;
			case 2:		//South
				Pass_Goal_x = x;
				Pass_Goal_y = y - 1;
				break;
			case 3:		//West
				Pass_Goal_x = x - 1;
				Pass_Goal_y = y;
				break;
			}
			break;
		}
	}
	test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 0);
//	if (accident_flag == 0) {
//		distance3(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
//		wait(300);
//	}
}

void adachihou2_q(char hikisuu_goal_x, char hikisuu_goal_y, char start_x,
		char start_y, char para_mode) {		//正常な足立法。
	volatile float V_search, ACCEL_search, V_max;
	volatile char aaa, straight_count = 0, i;

	if (para_mode == 1) {
		V_search = 500.0;
		ACCEL_search = 6000.0;
		V_max = 900.0;
		aaa = 0;	//小回り速度500
	} else if (para_mode == 2) {
		V_search = 650.0;
		ACCEL_search = 6500.0;
		V_max = 1000.0;
		aaa = 1;	//小回り速度650
	}
	column_temp[0] |= 1;

	kabekire_right = 0;	//台形での誤作動防止のため
	kabekire_left = 0;	//台形での誤作動防止のため
	reverse_flag = 0;	//台形での誤作動防止のため

	x = start_x;
	y = start_y;
	wait(300); 			//励磁直後は少し待つ！

	q_dist_flag = 0;
	test_daikei(90.0, V_search, ACCEL_search, 0.0, V_search, 0);
	direction_xy();
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

		if (x == goal_x && y == goal_y) {		//以下、クシつぶし

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

//		total_dist = 0.0;	DCでは不要
		LED1 = 1;
		q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
		LED1 = 0;
		if (map[x][y] == 255) {		//閉じ込められたら自動で抜け出す。
			if (is_Exist_Wall(temp_goal_x, temp_goal_y, 0) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 1) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 2) == 1
					&& is_Exist_Wall(temp_goal_x, temp_goal_y, 3) == 1) {
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			kabekire_right=0;
			kabekire_left=0;
			reverse_flag = 0;

			fail_flag = 1;
		}

		switch (direction_count) {
		case 0:
			if (y < 15&& map[x][y + 1]
			< map[x][y]&& sen.right_front < r_front_wall_judge) {	//North & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y < 15
							&& map[x][y + straight_count]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x, y + straight_count, 2) == 1
							&& is_Exist_Wall(x, y + straight_count, 2) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				}else{
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;

			} else if (x < 15&& map[x + 1][y]
			< map[x][y] && sen.right_side < r_wall_judge) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (x > 0&& map[x - 1][y]
			< map[x][y]&& sen.left_side < l_wall_judge) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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
				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 1:
			if (x
					< 15&& map[x+1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//East & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (x < 15
							&& map[x + straight_count][y]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x + straight_count, y, 3) == 1
							&& is_Exist_Wall(x + straight_count, y, 3) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				}else{
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
//				test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
//				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_side < r_wall_judge) {//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.left_side < l_wall_judge) {//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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
				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 2:
			if (y
					> 0&& map[x][y-1] < map[x][y] && sen.right_front < r_front_wall_judge) {//South & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y > 0
							&& map[x][y - straight_count]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x, y - straight_count, 0) == 1
							&& is_Exist_Wall(x, y - straight_count, 0) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				} else {
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
//				test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
//				direction_xy();

			} else if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_side < r_wall_judge) {//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (x
					< 15&& map[x+1][y] < map[x][y] && sen.left_side < l_wall_judge) {//South & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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

				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		case 3:
			if (x
					> 0&& map[x-1][y] < map[x][y] && sen.right_front < r_front_wall_judge) {//West & 直進

				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (x > 0
							&& map[x - straight_count][y]
									== map[x][y] - straight_count
							&& is_the_Wall_watched(x - straight_count, y, 1) == 1
							&& is_Exist_Wall(x - straight_count, y, 1) == 0) {
					} else {
						straight_count--;	//直進数に変換
						break;
					}
				}
				if(straight_count > 1){
					test_daikei(180.0 * straight_count, V_max, 3000.0, V_search, V_search, 1);
				}else{
					test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);
				}

				for (i = 0; i < straight_count; i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
//				direction_xy();
//				test_daikei(180.0, V_search, 3000.0, V_search, V_search, 1);

			} else if (y
					< 15&& map[x][y+1] < map[x][y] && sen.right_side < r_wall_judge) {//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				slalom_2(turn[aaa].P_1_0.theta, turn[aaa].P_1_0.th1,
						turn[aaa].P_1_0.th2, 1000.0, turn[aaa].P_1_0.a_cc,
						turn[aaa].P_1_0.wise, turn[aaa].P_1_0.vel,
						turn[aaa].P_1_0.d_f, turn[aaa].P_1_0.d_r);	//小回り右スラローム
				direction_xy();

			} else if (y
					> 0&& map[x][y-1] < map[x][y] && sen.left_side < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				slalom_2(turn[aaa].P_1_8.theta, turn[aaa].P_1_8.th1,
						turn[aaa].P_1_8.th2, 1000.0, turn[aaa].P_1_8.a_cc,
						turn[aaa].P_1_8.wise, turn[aaa].P_1_8.vel,
						turn[aaa].P_1_8.d_f, turn[aaa].P_1_8.d_r);	//小回り左スラローム
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
				test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
				wait(150);
				test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
				wait(150);
				reverse_daikei(40.0, -120.0, -3000.0, 0.0, 0.0, 0);
				ideal_balance_velocity = 0.0;
				wait(150);
				test_daikei(180.0, V_search, ACCEL_search, 0.0, V_search, 1);//壁切れを読める前提なので180進むことにしている
				direction_xy();
			}
			break;

		}
		LED_V1 = 0;
		LED_V2 = 0;
		LED_V3 = 0;
		LED_V4 = 0;

		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			if (sen.right_front >= r_front_wall_judge) {	//前センサーの壁判断
				add_wall_front(x, y, direction_count);
			}
			if (sen.right_side >= r_wall_judge) {	//右センサーの壁判断
				add_wall_right(x, y, direction_count);
			}
			if (sen.left_side >= l_wall_judge) {	//左センサーの壁判断
				add_wall_left(x, y, direction_count);
			}
			break;
		}
	}
	test_daikei(90.0, V_search, ACCEL_search, V_search, 0.0, 0);
}

void move_xy_skew(char hikisuu_v_l, char hikisuu_direction){	//移動後の値を代入
	if(hikisuu_v_l==0){	//移動前level
		switch (hikisuu_direction){	//移動後の方向を代入
		case 0:
			y++;
			break;
		case 1:
			y++;
			break;
		case 3:
				//そのまま
			break;
		case 4:
			y--;
			break;
		case 5:
			x--;
			break;
		case 7:
			x--;
			y++;
			break;

		}

	}else{	//移動前vertcal
		switch (hikisuu_direction){	//移動後の方向を代入
		case 1:
			x++;
			break;
		case 2:
			x++;
			break;
		case 3:
			x++;
			y--;
			break;
		case 5:
			y--;
			break;
		case 6:
			x--;
			break;
		case 7:
			//そのまま
			break;
		}
	}
}

void MAKE_PASS_SKEW(char hikisuu_goal_x, char hikisuu_goal_y){
	volatile char l_v_judge = 0, nn = 0, nn_dir = 0, px, py, ii;
	px = x;
	py = y;
	x = 0;
	y = 0;
	for (ii = 0; ii < 200; ii++) {
		pass[ii] = 0;
	}

	direc[0] = 0;	//最初は北を向いているものとする。
	l_v_judge = 0;	//最初はlevelにいるものとする。
	pass[0] = 0;
	nn ++;

	skew_queue_walkmap_maker(hikisuu_goal_x, hikisuu_goal_y);

	while(1){
		myprintf("x=%d\n\r", x);
		myprintf("y=%d\n\r", y);
		myprintf("l_v_judge=%d\n\r", l_v_judge);
		myprintf("direc[%d]=%d\n\r", nn_dir, direc[nn_dir]);
		myprintf("pass[%d]=%d\n\r", nn, pass[nn]);
		myprintf("\n\r");

		if (l_v_judge == 0 && (direc[nn_dir] == 7 || direc[nn_dir] == 0 || direc[nn_dir] == 1)) {	//Case 1

			if (y < 14 && level[x][y + 1] == level[x][y] - 7) {	//level → level北向き

				direc[nn_dir+1] = 0;	//level 北向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 0;	//level

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (pass[nn] <= 30) {
						pass[nn] = pass[nn] + 2;	//直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 2;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == -1) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 2;

				} else {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 2;

				}

			}else if(x < 15 && y < 15 && vertical[x][y + 1] == level[x][y] - 5){		//level →　vertical北東向き
				direc[nn_dir+1] = 1;	//vertical 北東向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新

				l_v_judge = 1;	//vertical

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 201;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == 1) {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 201;

				} else {
					pass[nn+1] = 40;	//右90度
					nn++;
					pass[nn+1] = 201;

				}

			}else if(x > 0 && y < 15 && vertical[x-1][y + 1] == level[x][y] - 5){		//level →　vertival北西向き

				direc[nn_dir+1] = 7;	//vertical 北西向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 1;	//vertical

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため

					} else {
						pass[nn+1] = 201;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == 7) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 201;

				} else {
					pass[nn+1] = 50;	//左90度
					nn++;
					pass[nn+1] = 201;
				}
			}else{	//いずれにも当てはまらなければ終了
				myprintf("break case1\r\n");
				break;

			}

		} else if (l_v_judge == 0 && (direc[nn_dir] == 3 || direc[nn_dir] == 4 || direc[nn_dir] == 5)) {	//Case 2

			if(y < 15 && y > 0 && level[x][y - 1] == level[x][y] - 7){	//level → level南向き
				direc[nn_dir+1] = 4;	//level 南向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 0;	//level

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (pass[nn] <= 30) {
						pass[nn] = pass[nn] + 2;	//直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため

					} else {
						pass[nn+1] = 2;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == -1) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 2;
				} else {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 2;
				}

			}else if(x < 15 && y < 15 && vertical[x][y] == level[x][y] - 5){	//level →　vertical南東向き

				direc[nn_dir+1] = 3;	//vertical 南東向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 1;	//vertical
				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため

					} else {
						pass[nn+1] = 201;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == -1) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 201;
				} else {
					pass[nn+1] = 50;	//左90度
					nn++;
					pass[nn+1] = 201;
				}

			}else if(x > 0 && y < 15 && vertical[x-1][y] == level[x][y] - 5){		//level →　vertival南西向き
				direc[nn_dir+1] = 5;	//vertical 南西向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 1;	//vertical

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 201;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == 1) {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 201;
				} else {
					pass[nn+1] = 40;	//右90度
					nn++;
					pass[nn+1] = 201;
				}
			}else{	//いずれにも当てはまらなければ終了
				myprintf("break case2\r\n");
				break;

			}

		} else if (l_v_judge == 1 && (direc[nn_dir] == 1 || direc[nn_dir] == 2 || direc[nn_dir] == 3)) {	//Case 3
			if(x < 14 && vertical[x+1][y] == vertical[x][y] - 7){	//vertical → vertical東向き
				direc[nn_dir+1] = 2;	//vertical 東向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 1;	//vertical
				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (pass[nn] <= 30) {
						pass[nn] = pass[nn] + 2;	//直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため

					} else {
						pass[nn+1] = 2;
					}
				} else if (direc[nn_dir+1] - direc[nn_dir] == -1) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 2;

				} else {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 2;
				}

			}else if(x < 15 && y < 15 && level[x+1][y] == vertical[x][y] - 5){	//vertical →　level北東向き
				direc[nn_dir+1] = 1;	//level 北東向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 0;	//level

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 201;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == -1) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 201;
				} else {
					pass[nn+1] = 50;	//左90度
					nn++;
					pass[nn+1] = 201;
				}

			}else if(x < 15 && y > 0 && level[x+1][y-1] == vertical[x][y] - 5){	//vertical →　level南東向き
				direc[nn_dir+1] = 3;	//level 南東向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 0;	//level

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 201;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == 1) {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 201;
				} else {
					pass[nn+1] = 40;	//右90度
					nn++;
					pass[nn+1] = 201;
				}
			} else{	//いずれにも当てはまらなければ終了
				myprintf("break case3\r\n");
				break;

			}

		} else if (l_v_judge == 1 && (direc[nn_dir] == 5 || direc[nn_dir] == 6 || direc[nn_dir] == 7)) {	//Case 4
			if(x < 15 && x > 0 && vertical[x-1][y] == vertical[x][y] - 7){	//vertical → vertical西向き
				direc[nn_dir+1] = 6;	//vertical 東向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 1;	//vertical

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (pass[nn] <= 30) {
						pass[nn] = pass[nn] + 2;	//直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 2;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == -1) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 2;
				} else {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 2;
				}

			}else if(x < 15 && y< 15 && level[x][y] == vertical[x][y] - 5){
				direc[nn_dir+1] = 7;	//level 北西向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 0;	//level

				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 201;
					}

				} else if (direc[nn_dir+1] - direc[nn_dir] == 1) {
					pass[nn+1] = 45;	//右45度
					nn++;
					pass[nn+1] = 201;
				} else {
					pass[nn+1] = 40;	//右90度
					nn++;
					pass[nn+1] = 201;
				}

			}else if(x < 15 && y > 0 && level[x][y-1] == vertical[x][y] - 5){
				direc[nn_dir+1] = 5;	//vertical 南東向き
				move_xy_skew(l_v_judge, direc[nn_dir+1]);	//斜めmap用座標情報を更新
				l_v_judge = 0;	//level
				if (direc[nn_dir+1] - direc[nn_dir] == 0) {
					if (200 <= pass[nn] && pass[nn] <= 230) {
						pass[nn] = pass[nn] + 1;	//斜め直進
						nn--;	//次も直進ならnnのみ更新され、違うならnnは更新されないようにするため
					} else {
						pass[nn+1] = 201;
					}
				} else if (direc[nn_dir+1] - direc[nn_dir] == -1) {
					pass[nn+1] = 55;	//左45度
					nn++;
					pass[nn+1] = 201;
				} else {
					pass[nn+1] = 50;	//左90度
					nn++;
					pass[nn+1] = 201;
				}
			} else{	//いずれにも当てはまらなければ終了
				myprintf("break case4\r\n");
				break;

			}
		}

		//以下、ゴール処理
		/*if(y < 15 && (direc[nn_dir+1] ==3 || direc[nn_dir+1] == 4 || direc[nn_dir+1] == 5 ) && (level[x][y]==0)){
			myprintf("break case1\r\n");
			if (direc[nn_dir + 1] == 3) {
				last_p_i = nn + 2;
				pass[last_p_i] = 45;	//斜め→右45度ターン
			} else if (direc[nn_dir + 1] == 5) {
				last_p_i = nn + 2;
				pass[last_p_i] = 55;	//斜め→左45度ターン
			} else{
				last_p_i = nn + 1;
			}
			break;

		}else if(x < 15 && (direc[nn_dir+1] == 5 || direc[nn_dir+1] == 6 || direc[nn_dir+1] == 7 ) && (vertical[x][y]==0)){
			myprintf("break case2\r\n");
			if (direc[nn_dir + 1] == 5) {
				last_p_i = nn + 2;
				pass[last_p_i] = 45;	//斜め→右45度ターン
			} else if (direc[nn_dir + 1] == 7) {
				last_p_i = nn + 2;
				pass[last_p_i] = 55;	//斜め→左45度ターン
			} else{
				last_p_i = nn + 1;
			}
			break;

		}else if(y > 0 && (direc[nn_dir+1] == 7 || direc[nn_dir+1] == 0 || direc[nn_dir+1] == 1 ) && (level[x][y-1]==0)){
			myprintf("break case3r\n");
			if (direc[nn_dir + 1] == 7) {
				last_p_i = nn + 2;
				pass[last_p_i] = 45;	//斜め→右45度ターン
			} else if (direc[nn_dir + 1] == 1) {
				last_p_i = nn + 2;
				pass[last_p_i] = 55;	//斜め→左45度ターン
			} else{
				last_p_i = nn + 1;
			}
			break;

		}else if(x > 0 && (direc[nn_dir+1] == 1 || direc[nn_dir+1] == 2 || direc[nn_dir+1] == 3 ) && (vertical[x-1][y]==0)){
			myprintf("break case4\r\n");
			if (direc[nn_dir + 1] == 1) {
				last_p_i = nn + 2;
				pass[last_p_i] = 45;	//斜め→右45度ターン
			} else if (direc[nn_dir + 1] == 3) {
				last_p_i = nn + 2;
				pass[last_p_i] = 55;	//斜め→左45度ターン
			} else{
				last_p_i = nn + 1;
			}
			break;
		}*/
//		if(level[x][y]==0 || vertical[x][y]==0){
//			last_p_i = nn + 1;
//			break;
//		}

		nn_dir++;
		nn++;	//必ず更新する(直進のみマイナスされているはず)
	}

	if (((direc[nn_dir] == 3 || direc[nn_dir] == 7 ) && (l_v_judge == 0)) ||
			((direc[nn_dir] == 5 || direc[nn_dir] == 1 ) && (l_v_judge == 1))){
		myprintf("break 1!\r\n");
		last_p_i = nn + 1;
		pass[last_p_i] = 45;	//斜め→右45度ターン
	} else if (((direc[nn_dir] == 5 || direc[nn_dir] == 1 ) && (l_v_judge == 0)) ||
			((direc[nn_dir] == 7 || direc[nn_dir] == 3 ) && (l_v_judge == 1))){
		myprintf("break 2!\r\n");
		last_p_i = nn + 1;
		pass[last_p_i] = 55;	//斜め→左45度ターン
	}  else{
		last_p_i = nn + 1;
	}


	x = px;	//元に戻す
	y = py;

}

void CONVERT_SKEWMAP_PASS(){
	volatile int read_p_i, motion_count, adjust_straight, thin = 0, j=0, last_thin;
	volatile char ii;

	read_p_i = 0;
	motion_count = 0;
	adjust_straight = 0;

	for (ii = 0; ii < 100; ii++) {	//初期化してから使おうね！
		motion[ii] = 0;
	}

	while(1){
		 if (pass[read_p_i + 0] == 0 && pass[read_p_i + 1] == 45 && pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 40 && pass[read_p_i + 4] >= 201 && pass[read_p_i + 4] <= 230) {	//開幕右135°ターン→斜め
			motion[motion_count] = 94;
			motion_count++;
			adjust_straight = -1;
			read_p_i = read_p_i + 4;
			break;
		}  else if (pass[read_p_i + 0] == 0 && pass[read_p_i + 1] == 45 && pass[read_p_i + 2] == 201 && pass[read_p_i + 3] == 45
				&& pass[read_p_i + 4] >= 1 && pass[read_p_i + 4] <= 30) {	//開幕右大回り→直進
			motion[motion_count] = 64;
			motion_count++;
			adjust_straight = -1;
			read_p_i = read_p_i + 4;
			break;
		} else if (pass[read_p_i + 0] == 0 && pass[read_p_i + 1] == 45 && pass[read_p_i + 2] >= 201
				&& pass[read_p_i + 2] <= 230) {	//開幕左45°ターン→斜め
			motion[motion_count] = 84;
			motion_count++;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;
			break;
		} else if(pass[read_p_i + 0] == 0 && pass[read_p_i + 1] >= 1 && pass[read_p_i + 1] <= 30){
			motion[motion_count] = 255;
			motion_count++;
			read_p_i = read_p_i + 1;
			break;
		}else{
			read_p_i++;
		}
	}
	while(1){

		if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 45
				&& pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 40
				&& pass[read_p_i + 4] == 201
				&& pass[read_p_i + 5] == 45
				&& pass[read_p_i + 6] >= 1 && pass[read_p_i + 6] <= 30) {	//右の180度ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 124;
			adjust_straight = -1;
			read_p_i = read_p_i + 6;

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 55
				&& pass[read_p_i + 2] >= 201
				&& pass[read_p_i + 3] == 50
				&& pass[read_p_i + 4] == 201
				&& pass[read_p_i + 5] == 55
				&& pass[read_p_i + 6] >= 1 && pass[read_p_i + 6] <= 30){	//左の180度ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 125;
			adjust_straight = -1;
			read_p_i = read_p_i + 6;

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 45
				&& pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 45
				&& pass[read_p_i + 4] >= 1 && pass[read_p_i + 4] <= 30){	//右の大廻ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 114;
			adjust_straight = -1;	//ターンの後ろ側の直進分を記憶し、次のmotionの直進部に反映させる
			read_p_i = read_p_i + 4;	//次のpassの読み込み位置

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 55
				&& pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 55
				&& pass[read_p_i + 4] >= 1 && pass[read_p_i + 4] <= 30){	//左の大廻ターンの場合
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 115;
			adjust_straight = -1;	//ターンの後ろ側の直進分を記憶し、次のmotionの直進部に反映させる
			read_p_i = read_p_i + 4;	//次のpassの読み込み位置

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 45
				&& pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 40
				&& pass[read_p_i + 4] >= 201 && pass[read_p_i + 4] <= 230){	//右の135degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 144;
			adjust_straight = -1;	//0
			read_p_i = read_p_i + 4;

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 55
				&& pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 50
				&& pass[read_p_i + 4] >= 201 && pass[read_p_i + 4] <= 230){	//左の135degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 145;
			adjust_straight = -1;	//0
			read_p_i = read_p_i + 4;

		}else if(pass[read_p_i] >= 201 && pass[read_p_i] <= 230
				&& pass[read_p_i + 1] == 40
				&& pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 45
				&& pass[read_p_i + 4] >= 1 && pass[read_p_i + 4] <= 30){	//斜め→右135°ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			if(motion[motion_count]==200){
				motion[motion_count]=0;
			}
			motion_count++;
			motion[motion_count] = 164;
			adjust_straight = -1;
			read_p_i = read_p_i + 4;

		}else if(pass[read_p_i] >= 201 && pass[read_p_i] <= 230
				&& pass[read_p_i + 1] == 50
				&& pass[read_p_i + 2] == 201
				&& pass[read_p_i + 3] == 55
				&& pass[read_p_i + 4] >= 1 && pass[read_p_i + 4] <= 30){	//斜め→左135°ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			if(motion[motion_count]==200){
				motion[motion_count]=0;
			}
			motion_count++;
			motion[motion_count] = 165;
			adjust_straight = -1;
			read_p_i = read_p_i + 4;

		}else if(pass[read_p_i] >= 201 && pass[read_p_i] <= 230
				&& pass[read_p_i + 1] == 40
				&& pass[read_p_i+2] >= 201 && pass[read_p_i+2] <= 230){	//斜め→右V90°ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			if(motion[motion_count]==200){
				motion[motion_count]=0;
			}
			motion_count++;
			motion[motion_count] = 174;
			adjust_straight = -1;	//0
			read_p_i = read_p_i + 2;

		}else if(pass[read_p_i] >= 201 && pass[read_p_i] <= 230
				&& pass[read_p_i + 1] == 50
				&& pass[read_p_i+2] >= 201 && pass[read_p_i+2] <= 230){	//斜め→左V90°ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			if(motion[motion_count]==200){
				motion[motion_count]=0;
			}
			motion_count++;
			motion[motion_count] = 175;
			adjust_straight = -1;	//0
			read_p_i = read_p_i + 2;

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 45
				&& pass[read_p_i + 2] >= 201 && pass[read_p_i + 2] <= 230){//右の45degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 134;
			adjust_straight = -1;	//0
			read_p_i = read_p_i + 2;

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30
				&& pass[read_p_i + 1] == 55
				&& pass[read_p_i + 2] >= 201 && pass[read_p_i + 2] <= 230){	//左の45degターン→斜め
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			motion_count++;
			motion[motion_count] = 135;
			adjust_straight = -1;	//0
			read_p_i = read_p_i + 2;

		}else if(pass[read_p_i] >= 201 && pass[read_p_i] <= 230
				&& pass[read_p_i + 1] == 45
				&& pass[read_p_i + 2] >= 1 && pass[read_p_i + 2] <= 30){	//斜め→右45°ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			if(motion[motion_count]==200){
				motion[motion_count]=0;
			}
			motion_count++;
			motion[motion_count] = 154;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;

		}else if(pass[read_p_i] >= 201 && pass[read_p_i] <= 230
				&& pass[read_p_i + 1] == 55
				&& pass[read_p_i + 2] >= 1 && pass[read_p_i + 2] <= 30){	//斜め→左45°ターン
			motion[motion_count] = pass[read_p_i] - 1 + adjust_straight;
			if(motion[motion_count]==200){
				motion[motion_count]=0;
			}
			motion_count++;
			motion[motion_count] = 155;
			adjust_straight = -1;
			read_p_i = read_p_i + 2;

		}else if(pass[read_p_i] >= 201 && pass[read_p_i] <= 230){	//斜め直進
			motion[motion_count] = pass[read_p_i] + adjust_straight;
			read_p_i = read_p_i + 1;

		}else if(pass[read_p_i] >= 1 && pass[read_p_i] <= 30){		//直進
			motion[motion_count] = pass[read_p_i] + adjust_straight;
			read_p_i = read_p_i + 1;

		}else {
			read_p_i++;
		}
		if (read_p_i - 1 >= last_p_i) {
			motion[motion_count] = 241;
			break;
		}

		//xxx そのうちmyprintfをコメントアウトする
		myprintf("read_p_i=%d\n\r", read_p_i);
		myprintf("motion_count=%d\n\r", motion_count);
		myprintf("motion[%d]=%d\n\r", motion_count, motion[motion_count]);
		myprintf("last_p_i=%d\n\r", last_p_i);
		myprintf("\n\r");

		motion_count++;	//動作のカウント（斜め直線ではカウントアップしない）
	}
	last_p_i = read_p_i - 1;	//変換後のpass[]を最後の値とする

	thin=0;
	read_p_i = 0;
	while (read_p_i-1 != last_p_i) {
		if (motion[read_p_i] == 200){
			motion[read_p_i] = 0;
		}else if (motion[read_p_i] != 0) {
			motion[thin] = motion[read_p_i];
			thin++;
		} else if (motion[read_p_i] == 0) {

		}
		read_p_i++;
	}
	j = 0;
	last_thin = thin - 1;
	while (1) {	//thinが最後のmotion[last_thin]=241の次の値であるので、これらを一掃する
		if (thin + j <= last_p_i) {
			motion[thin + j] = 0;
			j++;
		} else {
			break;
		}
	}

	if(motion[last_thin]==241 && motion[last_thin-1] == 144){	//右Uターン＋停止へ変換
		motion[last_thin - 1] =  234;
		motion[last_thin] = 0;
	}else if(motion[last_thin]==241 && motion[last_thin-1] ==145){	//左Uターン＋停止へ変換
		motion[last_thin - 1] =  235;
		motion[last_thin] = 0;

	}else if(motion[last_thin]==241 && motion[last_thin-1] ==174){	//斜め→右135＋停止へ変換
		motion[last_thin - 1] =  238;
		motion[last_thin] = 0;

	}else if(motion[last_thin]==241 && motion[last_thin-1] ==175){	//斜め→左135＋停止へ変換
		motion[last_thin - 1] =  239;
		motion[last_thin] = 0;

	}else if(motion[last_thin]==241 && motion[last_thin-1] ==134){	//右大回り＋停止へ変換
		motion[last_thin - 1] =  232;
		motion[last_thin] = 0;

	}else if(motion[last_thin]==241 && motion[last_thin-1] ==135){	//左大回り＋停止へ変換
		motion[last_thin - 1] =  233;
		motion[last_thin] = 0;

	}else if(motion[last_thin]==241 && motion[last_thin-1] >= 201 && motion[last_thin-1] <=230){	//斜めでパスが終了する場合
		if(motion[last_thin-1] == 201){
			if(pass[last_p_i] == 45){	//斜め→右45ターンで終了する
				motion[last_thin - 1] =  236;	//斜め→右45ターンで終了する
			}else if(pass[last_p_i] == 55){	//斜め→左45ターンで終了する
				motion[last_thin - 1] =  237;	//斜め→左45ターンで終了する
			}
			motion[last_thin] = 0;
		}else{
			motion[last_thin - 1] =  motion[last_thin - 1] - 1;
			if(pass[last_p_i] == 45){	//斜め→右45ターンで終了する
				motion[last_thin] =  236;	//斜め→右45ターンで終了する
			}else if(pass[last_p_i] == 55){	//斜め→左45ターンで終了する
				motion[last_thin] =  237;	//斜め→左45ターンで終了する
			}
		}
	}

}

void convert_pass_skew() {
	volatile int read_p_i, motion_count, adjust_straight, thin = 0, s_count, j=0;
	volatile char ii;
	read_p_i = 0;
	motion_count = 0;
	adjust_straight = 0;

	for (ii = 0; ii < 100; ii++) {	//初期化してから使おうね！
		motion[ii] = 0;
	}

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
		} else if (pass[read_p_i] == 255 && pass[read_p_i + 1] >= 1	//これはうまく機能しないので除外20171002
				&& pass[read_p_i + 1] <= 30) {	//開幕直進の場合、直進区間を1区間だけ上乗せする。
			motion[motion_count] = 255;
//			pass[read_p_i + 1] = pass[read_p_i + 1] + 1;
			read_p_i++;
		} else if (pass[read_p_i] >= 1 && pass[read_p_i] <= 30
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

void exe_pass_EX(float hikisuu_vmax, float hikisuu_skew_vmax, float hikisuu_accel, char hikisuu_mode) {
//	volatile int read_P_I;
	read_P_I = 0;

	assign_parameters(hikisuu_mode);	//走行モードの決定
	if ((motion[read_P_I + 1] == 114)
			|| (motion[read_P_I + 1] == 124)
			|| (motion[read_P_I + 1] == 134)
			|| (motion[read_P_I + 1] == 144)
			|| (motion[read_P_I + 1] == 184)
			|| (motion[read_P_I + 1] == 154)
			|| (motion[read_P_I + 1] == 164)
			|| (motion[read_P_I + 1] == 174)) {	//壁切れを読み、次が右ターンの場合
		flags_kabekire.next_r=1;
		flags_kabekire.next_l=0;
		flags_kabekire.wait=1;
	} else if ((motion[read_P_I + 1] == 115)
			|| (motion[read_P_I + 1] == 125)
			|| (motion[read_P_I + 1] == 135)
			|| (motion[read_P_I + 1] == 145)
			|| (motion[read_P_I + 1] == 185)
			|| (motion[read_P_I + 1] == 155)
			|| (motion[read_P_I + 1] == 165)
			|| (motion[read_P_I + 1] == 175)) {	//壁切れを読み、次が左ターンの場合
		flags_kabekire.next_r=0;
		flags_kabekire.next_l=1;
		flags_kabekire.wait=1;
	} else {
		flags_kabekire.wait = 0;	//壁切れyomanai
		flags_kabekire.next_r=0;
		flags_kabekire.next_l=0;
	}

	//以下、最初の動作のみ別枠で行う。(尻当てによりスタート！)
	if (motion[read_P_I] >= 1 && motion[read_P_I] <= 30) {
		daikei_for_pass_EX(90.0 * motion[read_P_I] + 47.0, vel_high, hikisuu_accel, 0.0,
				vel_high, 1, 0);
	} else if (motion[read_P_I] == 64) {	//開幕大回りターン
		daikei_for_pass_EX(turn[bb].P_1_1.d_f + 47.0, turn[bb].P_1_1.vel, accel_normal, 0.0, turn[bb].P_1_1.vel, 1, 0);
		turn_for_pass(turn[bb].P_1_1.theta, turn[bb].P_1_1.th1,
				turn[bb].P_1_1.th2, 1000.0, turn[bb].P_1_1.a_cc,
				turn[bb].P_1_1.wise, turn[bb].P_1_1.vel,
				0.0, turn[bb].P_1_1.d_r);	//右大廻ターン

	} else if (motion[read_P_I] == 74) {	//開幕右小回り→1区間加速
		daikei_for_pass_EX(90.0 + 47.0, vel_low, accel_normal, 0.0, vel_low, 1, 0);
		slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, 13.0, 0.0);//右小回り(後距離0)  ????
		daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_low, vel_high, 1,
				0);	//加速
	} else if (motion[read_P_I] == 84) {	//開幕右45°ターン→斜め(初期速度は0.0)
		daikei_for_pass_EX(turn[cc].P_1_3.d_f + 47.0, turn[cc].P_1_3.vel, accel_normal, 0.0, turn[cc].P_1_3.vel, 1, 0);
		turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
				turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
				turn[cc].P_1_3.wise, turn[cc].P_1_3.vel,
				turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
				0.0, turn[cc].P_1_3.d_r, 1);	//右45°ターン→斜め

	} else if (motion[read_P_I] == 94) {	//開幕右135°ターン→斜め(初期速度は0.0)
		daikei_for_pass_EX(turn[cc].P_1_4.d_f + 47.0 , turn[cc].P_1_4.vel, accel_normal, 0.0, turn[cc].P_1_4.vel, 1, 0);
		turn_for_skew_pass(turn[cc].P_1_4.theta, turn[cc].P_1_4.th1,
				turn[cc].P_1_4.th2, 1000.0, turn[cc].P_1_4.a_cc,
				turn[cc].P_1_4.wise, turn[cc].P_1_4.vel,
				turn[cc].P_1_4.vel, turn[cc].P_1_4.vel,
				 0.0, turn[cc].P_1_4.d_r, 1);	//右135°ターン→斜め

	} else if (motion[read_P_I] == 253) {	//斜め無し・開幕ターンあり  学生大会で直した
		daikei_for_pass_EX(90.0 + 47.0, vel_low, accel_normal, 0.0, vel_low, 1, 0);//緩やかに加速する
	} else if (motion[read_P_I] == 255) {	//斜め無し・開幕ターン無し  学生大会で直した
		daikei_for_pass_EX(90.0 + 47.0, vel_high, accel_normal, 0.0, vel_high, 1, 0);

	}
	//以下、最初以外の動作
	while (1) {
		if (fail_flag == 1) {	//failセーフ
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			break;	//daikeiの関数を抜けられる
		}
		read_P_I++;
		if(read_P_I==13){		//debug
			sample_flag=1;		//debug
		}						//debug

		if ((motion[read_P_I + 1] == 114)
				|| (motion[read_P_I + 1] == 124)
				|| (motion[read_P_I + 1] == 134)
				|| (motion[read_P_I + 1] == 144)
				|| (motion[read_P_I + 1] == 184)
				|| (motion[read_P_I + 1] == 154)
				|| (motion[read_P_I + 1] == 164)
				|| (motion[read_P_I + 1] == 174)) {	//壁切れを読み、次が右ターンの場合
			flags_kabekire.next_r=1;
			flags_kabekire.next_l=0;
			flags_kabekire.wait=1;
		} else if ((motion[read_P_I + 1] == 115)
				|| (motion[read_P_I + 1] == 125)
				|| (motion[read_P_I + 1] == 135)
				|| (motion[read_P_I + 1] == 145)
				|| (motion[read_P_I + 1] == 185)
				|| (motion[read_P_I + 1] == 155)
				|| (motion[read_P_I + 1] == 165)
				|| (motion[read_P_I + 1] == 175)) {	//壁切れを読み、次が左ターンの場合
			flags_kabekire.next_r=0;
			flags_kabekire.next_l=1;
			flags_kabekire.wait=1;
		} else {
			flags_kabekire.wait = 0;	//壁切れyomanai
			flags_kabekire.next_r=0;
			flags_kabekire.next_l=0;
		}
		if (motion[read_P_I] <= 30 && motion[read_P_I] >= 1) {	//数値の区間の半分直進
			daikei_for_pass_EX(90.0 * motion[read_P_I], hikisuu_vmax,
					hikisuu_accel, vel_high, vel_high, 1, 1);
		}else if(motion[read_P_I] <= 230 && motion[read_P_I] >= 200){
			if((motion[read_P_I] - 200.0) >= 2){
				skew_kabekire.bit1=1;	//xxx 斜め壁切れ用。turn_skewは変更済み
				daikei_for_pass_EX(127.3 * (motion[read_P_I] - 200.0), hikisuu_skew_vmax,
						hikisuu_accel, turn[cc].P_1_3.vel, turn[cc].P_1_3.vel, 2, 1);		//2:斜め制御あり  1:壁切れ読む
				skew_kabekire.bit1=0;	//
			}else{
				skew_kabekire.bit1=1;	//xxx 斜め壁切れ用。turn_skewは変更済み
				daikei_for_pass_EX(127.3 * (motion[read_P_I] - 200.0), hikisuu_skew_vmax,
						hikisuu_accel, turn[cc].P_1_3.vel, turn[cc].P_1_3.vel, 0, 1);	//0:壁制御無し 1:壁切れ読む
				skew_kabekire.bit1=0;	//
			}
		}
		else {

			switch ((char) motion[read_P_I]) {
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
						turn[bb].P_1_10.d_r);	//左Uターン
				break;
			case 134:	//右45°ターン→斜め
				//xxx 斜めターン前後の壁制御はoff,ターン後の壁切れenableはon
				turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
						turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
						turn[cc].P_1_3.wise, turn[bb].P_1_3.vel,
						turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
						turn[cc].P_1_3.d_f + adjust_before_dist,
						turn[cc].P_1_3.d_r, 1);	//右45°ターン→斜め　1:後の壁切れ条件斜め
				break;
			case 135:	//左45°ターン→斜め
				turn_for_skew_pass(turn[cc].P_1_11.theta, turn[cc].P_1_11.th1,
						turn[cc].P_1_11.th2, 1000.0, turn[cc].P_1_11.a_cc,
						turn[cc].P_1_11.wise, turn[bb].P_1_11.vel,
						turn[cc].P_1_11.vel, turn[cc].P_1_11.vel,
						turn[cc].P_1_11.d_f + adjust_before_dist,
						turn[cc].P_1_11.d_r, 1);	//左45°ターン→斜め　1:後の壁切れ条件斜め
				break;
			case 144:	//右135°ターン→斜め
				turn_for_skew_pass(turn[cc].P_1_4.theta, turn[cc].P_1_4.th1,
						turn[cc].P_1_4.th2, 1000.0, turn[cc].P_1_4.a_cc,
						turn[cc].P_1_4.wise, turn[bb].P_1_4.vel,
						turn[cc].P_1_4.vel, turn[cc].P_1_4.vel,
						turn[cc].P_1_4.d_f + adjust_before_dist,
						turn[cc].P_1_4.d_r, 1);	//右135°ターン→斜め　1:後の壁切れ条件斜め
				break;
			case 145:	//左135°ターン→斜め
				turn_for_skew_pass(turn[cc].P_1_12.theta, turn[cc].P_1_12.th1,
						turn[cc].P_1_12.th2, 1000.0, turn[cc].P_1_12.a_cc,
						turn[cc].P_1_12.wise, turn[bb].P_1_12.vel,
						turn[cc].P_1_12.vel, turn[cc].P_1_12.vel,
						turn[cc].P_1_12.d_f + adjust_before_dist,
						turn[cc].P_1_12.d_r, 1);	//左135°ターン→斜め　1:後の壁切れ条件斜め
				break;
			case 154:	//斜め→右45°ターン
				turn_for_skew_pass(turn[cc].P_1_5.theta, turn[cc].P_1_5.th1,
						turn[cc].P_1_5.th2, 1000.0, turn[cc].P_1_5.a_cc,
						turn[cc].P_1_5.wise, turn[cc].P_1_5.vel,
						turn[cc].P_1_5.vel, turn[bb].P_1_5.vel,
						turn[cc].P_1_5.d_f + adjust_before_dist,
						turn[cc].P_1_5.d_r, 0);	//斜め→右45度ターン　0:後の壁切れ条件直進

				break;
			case 155:	//斜め→左45°ターン
				turn_for_skew_pass(turn[cc].P_1_13.theta, turn[cc].P_1_13.th1,
						turn[cc].P_1_13.th2, 1000.0, turn[cc].P_1_13.a_cc,
						turn[cc].P_1_13.wise, turn[cc].P_1_13.vel,
						turn[cc].P_1_13.vel, turn[bb].P_1_13.vel,
						turn[cc].P_1_13.d_f + adjust_before_dist,
						turn[cc].P_1_13.d_r, 0);	//斜め→左45度ターン　0:後の壁切れ条件直進
				break;
			case 164:	//斜め→右135°ターン
				turn_for_skew_pass(turn[cc].P_1_6.theta, turn[cc].P_1_6.th1,
						turn[cc].P_1_6.th2, 1000.0, turn[cc].P_1_6.a_cc,
						turn[cc].P_1_6.wise, turn[cc].P_1_6.vel,
						turn[cc].P_1_6.vel, turn[bb].P_1_6.vel,
						turn[cc].P_1_6.d_f + adjust_before_dist,
						turn[cc].P_1_6.d_r, 0);	//斜め→右135度ターン　0:後の壁切れ条件直進
				break;
			case 165:	//斜め→左135°ターン
				turn_for_skew_pass(turn[cc].P_1_14.theta, turn[cc].P_1_14.th1,
						turn[cc].P_1_14.th2, 1000.0, turn[cc].P_1_14.a_cc,
						turn[cc].P_1_14.wise, turn[cc].P_1_14.vel,
						turn[cc].P_1_14.vel, turn[bb].P_1_14.vel,
						turn[cc].P_1_14.d_f + adjust_before_dist,
						turn[cc].P_1_14.d_r, 0);	//斜め→左135度ターン　0:後の壁切れ条件直進
				break;
			case 174:	//斜め→右V90°ターン
				turn_for_skew_pass(turn[cc].P_1_7.theta, turn[cc].P_1_7.th1,
						turn[cc].P_1_7.th2, 1000.0, turn[cc].P_1_7.a_cc,
						turn[cc].P_1_7.wise, turn[cc].P_1_7.vel,
						turn[cc].P_1_7.vel, turn[cc].P_1_7.vel,
						turn[cc].P_1_7.d_f + adjust_before_dist,
						turn[cc].P_1_7.d_r, 1);	//斜め→右V90度ターン　1:後の壁切れ条件斜め
				break;
			case 175:	//斜め→左V90°ターン
				turn_for_skew_pass(turn[cc].P_1_15.theta, turn[cc].P_1_15.th1,
						turn[cc].P_1_15.th2, 1000.0, turn[cc].P_1_15.a_cc,
						turn[cc].P_1_15.wise, turn[cc].P_1_15.vel,
						turn[cc].P_1_15.vel, turn[cc].P_1_15.vel,
						turn[cc].P_1_15.d_f + adjust_before_dist,
						turn[cc].P_1_15.d_r, 1);	//斜め→左V90度ターン　1:後の壁切れ条件斜め
				break;
			case 184:	//減速+右ターン
				daikei_for_pass_EX(90.0, vel_high, hikisuu_accel,
						vel_high, vel_low, 1, 1);	//(壁切れyomu)
				slalom_2(turn[aa].P_1_0.theta, turn[aa].P_1_0.th1,
						turn[aa].P_1_0.th2, 1000.0, turn[aa].P_1_0.a_cc,
						turn[aa].P_1_0.wise, turn[aa].P_1_0.vel,
						turn[aa].P_1_0.d_f, turn[aa].P_1_0.d_r);	//右小回り
				break;
			case 185:	//減速+左ターン
				daikei_for_pass_EX(90.0, vel_high, hikisuu_accel,
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
				daikei_for_pass_EX(90.0, vel_high, hikisuu_accel, vel_low,
						vel_high, 1, 1);
				break;
			case 195:	//左ターン+加速
				slalom_2(turn[aa].P_1_8.theta, turn[aa].P_1_8.th1,
						turn[aa].P_1_8.th2, 1000.0, turn[aa].P_1_8.a_cc,
						turn[aa].P_1_8.wise, turn[aa].P_1_8.vel,
						turn[aa].P_1_8.d_f, turn[aa].P_1_8.d_r);	//左小回り
				daikei_for_pass_EX(90.0, vel_high, hikisuu_accel, vel_low,
						vel_high, 1, 1);
				break;
			case 232:	//右大廻ターン+停止
				turn_for_pass(turn[bb].P_1_1.theta, turn[bb].P_1_1.th1,
						turn[bb].P_1_1.th2, 1000.0, turn[bb].P_1_1.a_cc,
						turn[bb].P_1_1.wise, turn[bb].P_1_1.vel,
						turn[bb].P_1_1.d_f + adjust_before_dist, 0.0);	//右大廻ターン
				daikei_for_pass_EX(turn[bb].P_1_1.d_r, turn[bb].P_1_1.vel, accel_normal, turn[bb].P_1_1.vel,
						0.0, 1, 0);

				break;
			case 233:	//左大廻ターン+停止
				turn_for_pass(turn[bb].P_1_9.theta, turn[bb].P_1_9.th1,
						turn[bb].P_1_9.th2, 1000.0, turn[bb].P_1_9.a_cc,
						turn[bb].P_1_9.wise, turn[bb].P_1_9.vel,
						turn[bb].P_1_9.d_f + adjust_before_dist,0.0);	//左大廻ターン
				daikei_for_pass_EX(turn[bb].P_1_9.d_r, turn[bb].P_1_9.vel, accel_normal, turn[bb].P_1_9.vel,
						0.0, 1, 0);

				break;
			case 234:	//右Uターン+停止
				turn_for_pass(turn[bb].P_1_2.theta, turn[bb].P_1_2.th1,
						turn[bb].P_1_2.th2, 1000.0, turn[bb].P_1_2.a_cc,
						turn[bb].P_1_2.wise, turn[bb].P_1_2.vel,
						turn[bb].P_1_2.d_f + adjust_before_dist,0.0);	//右Uターン
				daikei_for_pass_EX(turn[bb].P_1_2.d_r, turn[bb].P_1_2.vel, accel_normal, turn[bb].P_1_2.vel,
						0.0, 1, 0);

				break;
			case 235:	//左Uターン+停止
				turn_for_pass(turn[bb].P_1_10.theta, turn[bb].P_1_10.th1,
						turn[bb].P_1_10.th2, 1000.0, turn[bb].P_1_10.a_cc,
						turn[bb].P_1_10.wise, turn[bb].P_1_10.vel,
						turn[bb].P_1_10.d_f + adjust_before_dist, 0.0);	//右Uターン
				daikei_for_pass_EX(turn[bb].P_1_10.d_r, turn[bb].P_1_10.vel, accel_normal, turn[bb].P_1_10.vel,
						0.0, 1, 0);

				break;
			case 236:	//斜め→右45°ターン+停止
				turn_for_skew_pass(turn[cc].P_1_5.theta, turn[cc].P_1_5.th1,
						turn[cc].P_1_5.th2, 1000.0, turn[cc].P_1_5.a_cc,
						turn[cc].P_1_5.wise, turn[cc].P_1_5.vel,
						turn[cc].P_1_5.vel, turn[cc].P_1_5.vel,
						turn[cc].P_1_5.d_f + adjust_before_dist, 0.0, 0);	//斜め→右45度ターン
				daikei_for_pass_EX(turn[cc].P_1_5.d_r, turn[cc].P_1_5.vel, accel_normal, turn[cc].P_1_5.vel,
						0.0, 1, 0);

				break;
			case 237:	//斜め→左45°ターン+停止
				turn_for_skew_pass(turn[cc].P_1_13.theta, turn[cc].P_1_13.th1,
						turn[cc].P_1_13.th2, 1000.0, turn[cc].P_1_13.a_cc,
						turn[cc].P_1_13.wise, turn[cc].P_1_13.vel,
						turn[cc].P_1_13.vel, turn[cc].P_1_13.vel,
						turn[cc].P_1_13.d_f + adjust_before_dist, 0.0, 0);	//斜め→左45度ターン
				daikei_for_pass_EX(turn[cc].P_1_13.d_r, turn[cc].P_1_13.vel, accel_normal, turn[cc].P_1_13.vel,
						0.0, 1, 0);

				break;
			case 238:	//斜め→右135°ターン+停止
				turn_for_skew_pass(turn[cc].P_1_6.theta, turn[cc].P_1_6.th1,
						turn[cc].P_1_6.th2, 1000.0, turn[cc].P_1_6.a_cc,
						turn[cc].P_1_6.wise, turn[cc].P_1_6.vel,
						turn[cc].P_1_6.vel, turn[cc].P_1_6.vel,
						turn[cc].P_1_6.d_f + adjust_before_dist, 0.0, 0);	//斜め→右135度ターン
				daikei_for_pass_EX(turn[cc].P_1_6.d_r, turn[cc].P_1_6.vel, accel_normal, turn[cc].P_1_6.vel,
						0.0, 1, 0);

				break;
			case 239:	//斜め→左135°ターン+停
				turn_for_skew_pass(turn[cc].P_1_14.theta, turn[cc].P_1_14.th1,
						turn[cc].P_1_14.th2, 1000.0, turn[cc].P_1_14.a_cc,
						turn[cc].P_1_14.wise, turn[cc].P_1_14.vel,
						turn[cc].P_1_14.vel, turn[cc].P_1_14.vel,
						turn[cc].P_1_14.d_f + adjust_before_dist, 0.0, 0);	//斜め→左135度ターン
				daikei_for_pass_EX(turn[cc].P_1_14.d_r, turn[cc].P_1_14.vel, accel_normal, turn[cc].P_1_14.vel,
						0.0, 1, 0);

				break;
			case 240:   //停止(90区間で最大限減速する)
				daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_high, 0.0,
						1, 0);
				break;
			case 241:   //停止(90.0区間で最大限減速する)
				daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_high, 0.0,
						1, 0);
				break;
			case 100:   //停止(90区間で)(斜め無しpass用)
				daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_high, 0.0,
						1, 0);
//				myprintf("test100\r\n");
				break;
			case 103:   //停止(90区間で)(斜め無しpass用)
				daikei_for_pass_EX(90.0, vel_low, accel_normal, vel_low, 0.0,
						1, 0);
				break;
			}
		}
		if (read_P_I == last_p_i) {

			break;
		}
	}

	wall_control = 0;	//ゴールで誤動作しないようにするため
	ideal_omega = 0.0;	//ゴールで誤動作しないようにするため
	ideal_balance_velocity = 0.0;
	wait(500);
	GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
	sensor_enable = 0;			//センサ切り
}

void GoalPosition_assign(char hikisuu_x, char hikisuu_y){	//任意のゴール座標代入

	goal_position[hikisuu_y] |= 0x8000 >> hikisuu_x;	//ゴール座標を追加できるようになる

}

void GoalPosition_remove(char hikisuu_x, char hikisuu_y){	//指定した座標のゴールビットを0にする
	volatile unsigned short remove_row = 0x8000;

	goal_position[hikisuu_y] = goal_position[hikisuu_y] & ~remove_row >> hikisuu_x;

}

short GoalPosition_check(char hikisuu_x, char hikisuu_y){	//代入値がゴール座標かどうかの判断
	volatile unsigned short judge_goal;
	judge_goal = goal_position[hikisuu_y] & 0x8000 >> hikisuu_x;
	if (judge_goal == 0x8000 >> hikisuu_x){
		judge_goal = 1;
	}else {
		judge_goal = 0;
	}
	return judge_goal;
}

char Compete_Near_Position(char current_posi, char position1, char position2){	//現在座標と任意の2点とを比較して、近いほうの座標を返す
	volatile unsigned char near_position, c_x, c_y, comp1_x, comp1_y, s_dist1, comp2_x, comp2_y, s_dist2;

	c_y = current_posi & 0x0f;       		// 引数から区画の座標を取り出す
	c_x = current_posi >> 4;
	comp1_y = position1 & 0x0f;       		// 引数から区画の座標を取り出す
	comp1_x = position1 >> 4;
	s_dist1 = abs(c_x - comp1_x)+ abs(c_y - comp1_y);
	comp2_y = position2 & 0x0f;       		// 引数から区画の座標を取り出す
	comp2_x = position2 >> 4;
	s_dist2 = abs(c_x - comp2_x)+ abs(c_y - comp2_y);
	if(s_dist1 <= s_dist2){
		near_position = position1;
	}else{
		near_position = position2;
	}
//xxx デバッグだから使用時にはコメントアウトすること(Compete_Near_Position)
	myprintf("s_dist1=%d\r\n",s_dist1);		//debug
	myprintf("s_dist2=%d\r\n",s_dist2);		//debug
	myprintf("near_posotion=%d\r\n",near_position);		//debug

	return near_position;
}

void Get_Possible_Near_Goal_Positions(char h_current_posi, char h_goal_number){	//xxx ちゃんと機能しているか要確認
	volatile unsigned char cr_x, cr_y, posi1 = 0, posi2 = 0, posi3 = 0, posi4 = 0, ii;

	cr_y = h_current_posi & 0x0f;       		// 引数から区画の座標を取り出す
	cr_x = h_current_posi >> 4;

	if(cr_x <= 7 && cr_y <= 7){	//現在地が南西ブロック
		posi1 = 255;	//一番遠い(15,15)を代入
		posi2 = 255;
		posi3 = 255;
		posi4 = 255;
	}else if(cr_x <= 7 && cr_y >= 8){ //現在地が北西ブロック
		posi1 = 240;	//一番遠い(15, 0)を代入
		posi2 = 240;
		posi3 = 240;
		posi4 = 240;
	}else if(cr_x >= 8 && cr_y <= 7){
		posi1 = 15;		//一番遠い( 0,15)を代入
		posi2 = 15;
		posi3 = 15;
		posi4 = 15;
	}else if(cr_x >= 8 && cr_y >= 8){
		posi1 = 0;		//一番遠い( 0, 0)を代入
		posi2 = 0;
		posi3 = 0;
		posi4 = 0;
	}

	for (ii = 0; ii < h_goal_number; ii++) {	//possible_tamp_goal[0]~[4]で、5点分比較できる

		if(Compete_Near_Position(h_current_posi, posi1,possible_tamp_goal[ii]) == possible_tamp_goal[ii]){

			posi4 = posi3;
			posi3 = posi2;
			posi2 = posi1;
			posi1 = Compete_Near_Position(h_current_posi, posi1,possible_tamp_goal[ii]);

		}else if(Compete_Near_Position(h_current_posi, posi2,possible_tamp_goal[ii]) == possible_tamp_goal[ii]){

			posi4 = posi3;
			posi3 = posi2;
			posi2 = Compete_Near_Position(h_current_posi, posi2,possible_tamp_goal[ii]);

		}else if(Compete_Near_Position(h_current_posi, posi3,possible_tamp_goal[ii]) == possible_tamp_goal[ii]){

			posi4 = posi3;
			posi3 = Compete_Near_Position(h_current_posi, posi3,possible_tamp_goal[ii]);

		}else if(Compete_Near_Position(h_current_posi, posi4,possible_tamp_goal[ii]) == possible_tamp_goal[ii]){

			posi4 = Compete_Near_Position(h_current_posi, posi4,possible_tamp_goal[ii]);

		}
	}
	//xxx デバッグだから使用時にはコメントアウトすること(Compete_Near_Position)
	myprintf("posi1=%d\r\n",posi1);
	myprintf("posi2=%d\r\n",posi2);
	myprintf("posi3=%d\r\n",posi3);
	myprintf("posi4=%d\r\n",posi4);

}

void q_new_walk_map_maker(char hikisuu_goal_x, char hikisuu_goal_y, char hikisuu_goal_size, char posi_x, char posi_y) {
	volatile unsigned short qx, qy, head, tail, q[257];
	volatile unsigned char i, j, kk=0;
	q_dist_flag = 1;

	for (i = 0; i < hikisuu_goal_size; i++) {	//複数マスゴールに対応
		for (j = 0; j < hikisuu_goal_size; j++) {
			GoalPosition_assign(hikisuu_goal_x + i, hikisuu_goal_y + j);
		}
	}

	if(GoalPosition_check(posi_x, posi_y) == 1){
		GoalPosition_remove(posi_x, posi_y);
	}

	for (qx = 0; qx <= x_size; qx++)		// マップの初期化,255の代入
			{
		for (qy = 0; qy <= y_size; qy++) {
			if(GoalPosition_check(qx,qy)==1){	//ゴール座標のビットが1ならば、0を代入
				map[qx][qy]=0;
				q[kk] = (qx * 16 + qy);		// 目標地点の座標を記憶
				kk++;
			}else{
				map[qx][qy] = 255;
			}
		}
	}
//	map[hikisuu_goal_x][hikisuu_goal_y] = 0;			// 目標地点に距離0を書き込む
//	map[10][10] = 0;	//test
//	map[5][1] = 0;	//test
//	q[0] = (hikisuu_goal_x * 16 + hikisuu_goal_y);		// 目標地点の座標を記憶
//	q[1] = (10 * 16 + 10);
//	q[2] = ( 5 * 16 +  1);

	head = 0;							// 先頭位置を初期化
	tail = kk;	//test			// 末尾位置

	while (head != tail)				// 配列の中身が空ならループを抜ける（更新できないとループを抜ける）
	{
//		if (fail_flag == 1) {	//failセーフ
//			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
//			LED_V1 = 1;
//			LED_V2 = 1;
//			LED_V3 = 1;
//			LED_V4 = 1;
//			break;	//daikeiの関数を抜けられる
//		}

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
	//xxx いづれmyprintfをコメントアウトする
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

}

void turn_angle_tuning(float h_totalangle, float h_angle1, float h_accel,
		float h_front, float h_rear, float h_c_wise, float h_vel, char h_count) {
	volatile float h_angle2;
	volatile char ii;

	h_angle2 = h_totalangle - h_angle1;

	test_daikei(180.0, h_vel, 6000, 0, h_vel, 0);
	sample_flag=0;
	for(ii=0;ii<h_count;ii++){
		slalom_for_tuning(h_totalangle, h_angle1, h_angle2, 2000.0, h_accel, h_c_wise, h_vel, h_front,
				h_rear);		//保存パラメタ
		test_daikei(63.6, h_vel, 6000, h_vel, h_vel, 0);
	}
	test_daikei(90.0, h_vel, 6000.0, h_vel, 0, 0);
	ideal_balance_velocity = 0.0;
	LED_V1 = 0;
	wait(1000);
	GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
	sensor_enable = 0;
}

void test_skew_wall_control() {
	volatile char Diff_Thre_test, Diff_Thre_test2;
//	LED_V1 = 0;	//debug
	LED_V2 = 0;	//debug
	LED_V3 = 0;	//debug
	LED_V4 = 0;	//debug
	LED2 = 0;	//debug

	Diff_Thre_test = 200;
	Diff_Thre_test2 = 50;

	if ((sen.right_side > SKEW_R_Threshold)
			&& (sen.left_side <= SKEW_L_Threshold)) {	//右壁ありの制御

		if (diff_average_sensor.right <= -1.0 * Diff_Thre_test) {//吸い込まれ対策,右壁の変化大のとき右制御を切る
			Error = 0.0;
//			LED_V2 = 1;	//debug
		} else {
			Error = (float) (-2.0 * (sen.right_side - SKEW_R_Refer));
			LED_V3 = 1;	//debug
		}

	} else if ((sen.right_side <= SKEW_R_Threshold)
			&& (sen.left_side > SKEW_L_Threshold)) {		//左壁ありの制御

		if (diff_average_sensor.left <= -1.0 * Diff_Thre_test) {//吸い込まれ対策,左壁の変化大のとき左制御を切る
			Error = 0.0;
//			LED_V2 = 1;
		} else {
			LED_V4 = 1;
			Error = (float) (2.0 * (sen.left_side - SKEW_L_Refer));
		}
	} else {	//両壁無い場合の制御⇒ここで前壁センサを用いて制御する

		if ((average_front_sen.right > SKEW_RFront_Threshold)
				&& (average_front_sen.left <= SKEW_LFront_Threshold)) {	//右壁ありの制御

			if (diff_average_front_sen.right <= -1.0 * Diff_Thre_test2) {//吸い込まれ対策,右壁の変化大のとき右制御を切る
				Error = 0.0;
				//			LED_V2 = 1;	//debug
			} else {
				Error = (float) (-2.0 * (average_front_sen.right - SKEW_RFront_Refer));
				LED_V3 = 1;	//debug
				LED_V2 = 1;	//debug
			}

		} else if ((average_front_sen.right <= SKEW_RFront_Threshold)
				&& (average_front_sen.left > SKEW_LFront_Threshold)) {		//左壁ありの制御

			if (diff_average_front_sen.left <= -1.0 * Diff_Thre_test2) {//吸い込まれ対策,左壁の変化大のとき左制御を切る
				Error = 0.0;
				//			LED_V2 = 1;
			} else {
				Error = (float) (2.0 * (average_front_sen.left - SKEW_LFront_Refer));
				LED_V4 = 1;
				LED2 = 1;
			}
		}else{
			Error = 0.0;
		}
	}

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
	buff_sen_right_front[0] = sen.right_front;
	buff_sen_left_front[0] = sen.left_front;

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

//		sen_count = 0;
	}
	//以下、更新後に現在の平均値を取得
	for (ii = 0; ii < number1 - 1; ii++) {
		buff_sen_right[ii + 1] = buff_sen_right[ii];
		buff_sen_left[ii + 1] = buff_sen_left[ii];
		average_sensor.right += buff_sen_right[ii];
		average_sensor.left += buff_sen_left[ii];
		buff_sen_right_front[ii + 1] = buff_sen_right_front[ii];
		buff_sen_left_front[ii + 1] = buff_sen_left_front[ii];
		average_front_sen.right +=  buff_sen_right_front[ii];
		average_front_sen.left +=  buff_sen_left_front[ii];
	}
	average_sensor.right = average_sensor.right / (float) number1;
	average_sensor.left = average_sensor.left / (float) number1;
	average_front_sen.right = average_front_sen.right / (float) number1;
	average_front_sen.left= average_front_sen.left / (float) number1;
	diff_average_front_sen.right = average_front_sen.right - buff_sen_right_front[number1-1];
	diff_average_front_sen.left = average_front_sen.left - buff_sen_left_front[number1-1];

	//以下、diffの算出
	if(sen_count == number2){
		diff_average_sensor.right = average_sensor.right - pre_buff_sen_right[0];
		diff_average_sensor.left = average_sensor.left - pre_buff_sen_left[0];
		sen_count = 0;
	}

}

void kabekire_for_pass() {	//1009pass用の壁切れ
	volatile float Diff_Thre, Sen_Judge_Value;
	if(skew_kabekire.bit1 == 1){
		Diff_Thre = 30.0;
		Sen_Judge_Value = 600.0;
	}else{
		Diff_Thre = DIFF_WALL_THRE;
		Sen_Judge_Value = 500.0;
	}

		if (flags_kabekire.next_r == 1) {
			if (diff_average_sensor.right <= -1.0 * Diff_Thre
					&& average_sensor.right < Sen_Judge_Value) {	//右の壁切れのみを検知
				if (kushi_judge == 2) {	//右が櫛である場合
					flags_kabekire.detect_r = 1;
					flags_kabekire.detect_l = 0;
				} else {	//右がただの壁切れの場合
					flags_kabekire.detect_r = 1;
					flags_kabekire.detect_l = 0;
				}
//				reverse_flag = 0;		//passでは不要
			}
		} else if (flags_kabekire.next_l == 1) {
			if (diff_average_sensor.left <= -1.0 * Diff_Thre
					&& average_sensor.left < Sen_Judge_Value) {	//左の壁切れのみを検知
				if (kushi_judge == 2) {	//左が櫛である場合
					flags_kabekire.detect_r = 0;
					flags_kabekire.detect_l = 1;
				} else {	//左がただの櫛である場合
					flags_kabekire.detect_r = 0;
					flags_kabekire.detect_l = 1;
				}
//				reverse_flag = 0;		//passでは不要
			}
		}
}

void initialize(){
	init_RSPI();
	refer_flag = 0;
	refer_count = 0;
	K_center.p=0.0;
	K_center.i=0.0;
	K_center.d=0.0;
	K_rot.p=0.0;
	K_rot.i=0.0;
	K_rot.d=0.0;
}

void get_log(int log_number){
//	Log1[log_number] = (float)sen.right_side;
//	Log2[log_number] = (float)sen.left_side;
//	Log3[log_number] = (float)average_front_sen.right;
//	Log4[log_number] = (float)average_front_sen.left;
//	Log5[log_number] = (float)Error;

	Log1[log_number] = (float)balance_distance;
	Log2[log_number] = (float)ideal_balance_distance;
	Log3[log_number] = (float)ideal_balance_velocity;
	Log4[log_number] = (float)x;
	Log5[log_number] = (float)y;
//	Log6[log_number] = (float)Error_wall.p;
//	Log7[log_number] = (float)sen.left_side;
//	Log8[log_number] = (float)average_sensor.left;
//	Log9[log_number] = (float)diff_average_sensor.left;
//	Log10[log_number] = (float)flags_kabekire.enable2;
//	Log11[log_number] = (float)flags_kabekire.wait;
}

void interrupt_CMT0() {
	int i, kabekire_error;
	volatile float K_wall_p;
	volatile unsigned char dir_r = 1, dir_l = 1;
	K_wall_p = K_wall.p;
	cmt_count++;
	if(wall_control==2){
		K_wall.p = K_skew_wall.p;
	}
	if (sensor_enable == 1) {
		sensor_ADconvert();
	}
	if (Erorr_center.i > 3500.0 && sen.right_front > 2400
			&& sen.left_front > 2400) {
		fail_flag = 1;
	}
	if (sample_flag == 1 /*&& sample_count < SAMPLE_NUMBER*/) {
		sample_count++;
		if(sample_count==10){
			get_log(log_count);
			log_count++;
			sample_count=0;
			if(log_count==LOG_NUMBER){
				sample_flag = 0;
				log_count = 0;
			}
		}
		LED3 = 1;
	}else if (sample_flag == 1 && sample_count == SAMPLE_NUMBER){
		LED3 = 0;
	}
	if(sample_flag == 2){
		sample_count ++;
		if(sample_count < SAMPLE_NUMBER){
			sample1[sample_count] = time_flag;
			sample2[sample_count] = sample_count;
		}else if(sample_count == SAMPLE_NUMBER){
			sample_flag = 0;
			sample_count = 0;
		}
	}
	if(kabekire_LED == 1){
		LED_count++;
		if(LED_count>=200){
			kabekire_LED = 0;
			LED_count=0;
			LED_V3=0;
			LED_V4=0;
		}
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
			refer_count = 0;
			refer_flag = 0;
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
	if(flags_kabekire.enable1==1){		//flags_kabekire.enable1:pass壁切れを読むために必要
		kabekire_for_pass();
	}
	if (wall_control == 1) {	//壁制御ON
		test_wall_control();	//壁制御（added in 20170923）

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
	} else if(wall_control==2){
		test_skew_wall_control();
	}else {
		Error = 0.0;
	}

//以下、重心を用いらた並進運動と回転運動の重ね合わせ
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
	K_wall.p = K_wall_p;

}

void task_exe(int first_number, int second_number, int therd_number) {//実行プログラムはこちらへ
	volatile int test, kk = 0, i, aaa, bbb, ccc, wall_gain=0;
	volatile float r_before, l_before, r_after, l_after, totalangle, angle1, angle2, accel,
			c_wise, vel, max_accel, max_vel;

	switch (first_number) {
	case 1:	//パラメータ調整用の第1層
		LED_motion3();
		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 2500)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		reference_fin = 0;
		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		sample_flag = 0;

		//以下、ターン調整-第2層
		switch (second_number) {
		case 1://90deg, clock-wise,
			vel=650.0;
			aa=1;
			cc=0;
			bb=0;
			test_daikei(90.0, vel, 6500.0, 0, vel, 0);
			slalom_2(turn[aa].P_1_0.theta, turn[aa].P_1_0.th1,
					turn[aa].P_1_0.th2, 1000.0, turn[aa].P_1_0.a_cc,
					turn[aa].P_1_0.wise, turn[aa].P_1_0.vel,
					turn[aa].P_1_0.d_f-1.5, turn[aa].P_1_0.d_r+3.0);	//小回り右スラローム
			test_daikei(90.0, vel, 6500.0, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 2:
			vel=650.0;
			aa=0;
			cc=0;
			bb=0;
			test_daikei(90.0, vel, 6500.0, 0, vel, 0);
			slalom_2(turn[aa].P_1_0.theta, turn[aa].P_1_0.th1,
					turn[aa].P_1_0.th2, 1000.0, turn[aa].P_1_0.a_cc,
					turn[aa].P_1_0.wise, turn[aa].P_1_0.vel,
					turn[aa].P_1_0.d_f, turn[aa].P_1_0.d_r);	//小回り右スラローム
			test_daikei(90.0, vel, 6500.0, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:
			vel=650.0;
			aa=1;
			cc=0;
			bb=0;
			test_daikei(90.0, vel, 6500.0, 0, vel, 0);
			slalom_2(turn[aa].P_1_8.theta, turn[aa].P_1_8.th1,
					turn[aa].P_1_8.th2, 1000.0, turn[aa].P_1_8.a_cc,
					turn[aa].P_1_8.wise, turn[aa].P_1_8.vel,
					turn[aa].P_1_8.d_f-3.0, turn[aa].P_1_8.d_r+1.0);	//小回り左スラローム
			test_daikei(90.0, vel, 6500.0, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			vel=1200.0;
			cc=7;
			bb=7;
			test_daikei(127.3, vel, 6500.0, 0, vel, 0);
			turn_for_skew_pass(turn[cc].P_1_15.theta, turn[cc].P_1_15.th1,
					turn[cc].P_1_15.th2, 2000.0, turn[cc].P_1_15.a_cc,
					turn[cc].P_1_15.wise, turn[cc].P_1_15.vel,
					turn[cc].P_1_15.vel, turn[cc].P_1_15.vel,
					1.0,
					20.0, 1);	//斜め→左V90度ターン　1:後の壁切れ条件斜め
			test_daikei(127.3, vel, 6500.0, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
			vel=1200.0;
			cc=7;
			bb=7;
			test_daikei(127.3, vel, 6500.0, 0, vel, 0);
			turn_for_skew_pass(turn[cc].P_1_15.theta, turn[cc].P_1_15.th1,
					turn[cc].P_1_15.th2, 2000.0, turn[cc].P_1_15.a_cc,
					turn[cc].P_1_15.wise, turn[cc].P_1_15.vel,
					turn[cc].P_1_15.vel, turn[cc].P_1_15.vel,
					10.0,
					10.0, 1);	//斜め→左V90度ターン　1:後の壁切れ条件斜め
			test_daikei(127.3, vel, 6500.0, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			vel=1200.0;
			cc=7;
			bb=7;
			test_daikei(127.3, vel, 6500.0, 0, vel, 0);
			turn_for_skew_pass(turn[cc].P_1_15.theta, turn[cc].P_1_15.th1,
					turn[cc].P_1_15.th2, 2000.0, turn[cc].P_1_15.a_cc,
					turn[cc].P_1_15.wise, turn[cc].P_1_15.vel,
					turn[cc].P_1_15.vel, turn[cc].P_1_15.vel,
					10.0,
					10.0, 1);	//斜め→左V90度ターン　1:後の壁切れ条件斜め
			test_daikei(127.3, vel, 6500.0, vel, 0, 0);
			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 7:
			turn_angle_tuning(179.5, 32.0, 8000.0, 5.0, 5.0, 1.0, 1200.0, 4);

			break;
		case 8:

			break;
		case 9:

			break;
		case 10:

			break;
		case 11:
			break;
		}

		break;

	case 2:	//ターン調整ⅱの第1層

		LED_motion2();
		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 2700)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		reference_fin = 0;
		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		sample_flag = 0;

		//以下、ターン調整-第2層
		switch (second_number) {
		case 1://90調整
			cc=1;
			bb=1;

			vel =650.0;
			flags_kabekire.next_r=1;
			flags_kabekire.next_l=0;
			flags_kabekire.wait=1;
			daikei_for_pass_EX(90.0 * 2.0, vel,
					6000.0, 0.0, vel, 1, 1);
			sample_flag=1;
			turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
					turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
					turn[cc].P_1_3.wise, turn[bb].P_1_3.vel,
					turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
					turn[cc].P_1_3.d_f + adjust_before_dist,
					turn[cc].P_1_3.d_r, 1);	//右45°ターン→斜め　1:後の壁切れ条件斜め
			daikei_for_pass_EX(127.3 * 7.0, 2000.0,
					6000.0, vel, vel, 2, 1);		//2:斜め制御あり  1:壁切れ読む
			test_daikei(127.3, vel, 6000, vel, 0, 0);

			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 2:
			cc=4;
			bb=4;

			vel =900.0;
			flags_kabekire.next_r=1;
			flags_kabekire.next_l=0;
			flags_kabekire.wait=1;
			daikei_for_pass_EX(90.0 * 2.0, vel,
					6000.0, 0.0, vel, 1, 1);
			sample_flag=1;
			turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
					turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
					turn[cc].P_1_3.wise, turn[bb].P_1_3.vel,
					turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
					turn[cc].P_1_3.d_f + adjust_before_dist,
					turn[cc].P_1_3.d_r, 1);	//右45°ターン→斜め　1:後の壁切れ条件斜め
			daikei_for_pass_EX(127.3 * 7.0, 2200.0,
					7000.0, vel, vel, 2, 1);		//2:斜め制御あり  1:壁切れ読む
			test_daikei(127.3, vel, 6000, vel, 0, 0);

			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:
			cc=4;
			bb=4;

			vel =900.0;
			flags_kabekire.next_r=1;
			flags_kabekire.next_l=0;
			flags_kabekire.wait=1;
			daikei_for_pass_EX(90.0 * 2.0, vel,
					6000.0, 0.0, vel, 1, 1);
			sample_flag=1;
			turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
					turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
					turn[cc].P_1_3.wise, turn[bb].P_1_3.vel,
					turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
					turn[cc].P_1_3.d_f + adjust_before_dist,
					turn[cc].P_1_3.d_r, 1);	//右45°ターン→斜め　1:後の壁切れ条件斜め
			daikei_for_pass_EX(127.3 * 7.0, 2500.0,
					7000.0, vel, vel, 2, 1);		//2:斜め制御あり  1:壁切れ読む
			test_daikei(127.3, vel, 6000, vel, 0, 0);

			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			cc=4;
			bb=4;

			vel =900.0;
			flags_kabekire.next_r=1;
			flags_kabekire.next_l=0;
			flags_kabekire.wait=1;
			daikei_for_pass_EX(90.0 * 2.0, vel,
					6000.0, 0.0, vel, 1, 1);
			sample_flag=1;
			turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
					turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
					turn[cc].P_1_3.wise, turn[bb].P_1_3.vel,
					turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
					turn[cc].P_1_3.d_f + adjust_before_dist,
					turn[cc].P_1_3.d_r, 1);	//右45°ターン→斜め　1:後の壁切れ条件斜め
			daikei_for_pass_EX(127.3 * 7.0, 2800.0,
					7500.0, vel, vel, 2, 1);		//2:斜め制御あり  1:壁切れ読む
			test_daikei(127.3, vel, 6000, vel, 0, 0);

			ideal_balance_velocity = 0.0;
			LED_V1 = 0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:

			break;
		case 6:

			break;
		case 7:

			break;
		case 8:
			break;
		case 11:
			break;
		}

		break;

	case 3:	//足立法調整用の第1層
		LED_motion2();
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
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				fail_flag = 1;
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
			}
			EI_keisuu = 1.0;
			LED5 = 1;
			sample_flag = 1;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.
			direction_count = 0;
			adachihou2_q(goal_x, goal_y, 0, 0, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
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
			adachihou2_q(0, 0, goal_x, goal_y, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;
			LED4 = 0;
			if(fail_flag==0){
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
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				fail_flag = 1;
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
			}
			EI_keisuu = 1.0;
			sample_flag = 0;
			direction_count = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
			adachihou2_q(goal_x, goal_y, 0, 0, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:	//

			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				fail_flag = 1;
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
			}
			EI_keisuu = 1.0;
			direction_count = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
			adachihou2_q(goal_x, goal_y, 0, 0, 2);	//重心速度650
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(500);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
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
			adachihou2_q( 0, 0, goal_x, goal_y, 2);	//重心速度650
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
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
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				fail_flag = 1;
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
			}
			EI_keisuu = 1.0;
			sample_flag = 0;
			direction_count = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
			adachihou2_q(goal_x, goal_y, 0, 0, 2);	//重心速度速いバージョン
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
		//4マスゴール対応探索の調整
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;

			while (reference_fin == 0)
				;
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				fail_flag = 1;
				LED_V1 = 1;
				LED_V2 = 1;
				LED_V3 = 1;
				LED_V4 = 1;
			}
			EI_keisuu = 1.0;
			sample_flag = 0;
			direction_count = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
			GoalPosition_assign(2, 2);
			GoalPosition_assign(5, 5);
			adachihou_q(goal_x, goal_y, 0, 0, 500.0, 5000.0);	//4マスゴール対応探索
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			//4マスゴール対応探索の調整
				sensor_enable = 1;
				wait(500);
				while (sen.right_side < 2500)
					;
				wait(1000);
				refer_flag = 1;	//リファレンス取得用フラグ
				gyro_enable = 1;

				while (reference_fin == 0)
					;
				reference_fin = 0;

				GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
				if (reference_omega == 0.0) {
					GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
					fail_flag = 1;
					LED_V1 = 1;
					LED_V2 = 1;
					LED_V3 = 1;
					LED_V4 = 1;
				}
				EI_keisuu = 1.0;
				sample_flag = 0;
				direction_count = 0;
				COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
				GoalPosition_assign(2, 2);
				GoalPosition_assign(5, 5);
				adachihou_q(goal_x, goal_y, 0, 0, 500.0, 5000.0);	//4マスゴール対応探索
				wall_control = 0;
				ideal_balance_velocity = 0.0;
				wait(1000);
				if(fail_flag==0){
					WALL_INFORMATION_save();
					WATCHED_WALL_INFORMATION_save();
				}

				adachihou2_q( 0, 0, x, y, 1);	//現在座標から探索再開
				wall_control = 0;
				ideal_balance_velocity = 0.0;
				wait(1000);
				if(fail_flag==0){
					WALL_INFORMATION_save();
					WATCHED_WALL_INFORMATION_save();
				}
				GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
				sensor_enable = 0;

			break;
		case 7:
			break;

		case 8:
			break;
		case 11:
			break;
		}

		break;

	case 4:		//斜め歩数マップ対応パス調整用―第1階層 //xxx  パスはここ
		LED_motion2();
		LED4 = 1;	//斜めmap対応パスの作成
		unknown_WALL_add();	//帰り探索のためには後にremoveする
		MAKE_PASS_SKEW(Pass_Goal_x, Pass_Goal_y);
		CONVERT_SKEWMAP_PASS();	//斜めmap対応パスに変換
		unknown_WALL_remove();	//探索続く場合　remove
		LED4 = 0;

		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 2500)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;
		while (reference_fin == 0)
			;
		reference_fin = 0;

		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}

		switch (second_number) {
		case 1:	//斜めパスパラメータ調整
			exe_pass_EX(1500.0, 800.0, 7000.0, 1);	//壁切れ改良型のパス
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 2:
			exe_pass_EX(1800.0, 1500.0, 6000.0, 1);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:
			exe_pass_EX(1500.0, 1200.0, 7000.0, 4);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			exe_pass_EX(2000.0, 1500.0, 7000.0, 4);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:
			exe_pass_EX(2000.0, 1500.0, 7000.0, 5);	//斜めで加速する
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 6:
			exe_pass_EX(2000.0, 1500.0, 7000.0, 6);	//斜めで加速する
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 7:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass_skew();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする

			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}
			exe_pass_EX(2700.0, 1200.0 ,9000.0, 2);	//斜めで加速する
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 8:
				unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
				WALL_INFORMATION_save();
				make_pass(Pass_Goal_x, Pass_Goal_y);
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
				reference_fin = 0;

				GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
				if (reference_omega == 0.0) {
					GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
				}

				exe_kamawari_pass_test(2800.0, 10000.0, 3);	//とりあえず、大回りの移植ができているかの確認
				ideal_balance_velocity = 0.0;
				wait(1000);
				GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
				sensor_enable = 0;

			break;
		case 9:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//正しい元情報に戻った

			break;
		case 10:
			LED5 = 1;	//斜めpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			WALL_INFORMATION_save();	//make_passのために保存する
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass_skew();	//斜め用のパスに変換
			skew_queue_walkmap_maker(Pass_Goal_x, Pass_Goal_y);
			skew_walkmap_display();
			LED5 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
			WALL_INFORMATION_save();	//元通り

			break;
		case 11:
			break;
		}
		break;

	case 5:		//直進、壁切れ調整用-第1層
		LED_motion2();
		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 2500)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		reference_fin = 0;

		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			fail_flag = 1;
			LED_V1 = 1;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 1;
		}

		switch (second_number) {
		case 1:	//新探索法の確認
			direction_count = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
			adachihou2_q(goal_x, goal_y, 0, 0, 1);	//重心速度650
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(500);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
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
			new_serch_algorithm(0,0,goal_x,goal_y,1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;

		case 2://4マス対応探索→新探索法の確認
			sample_flag = 0;
			direction_count = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
			adachihou_q(goal_x, goal_y, 0, 0, 500.0, 5000.0);	//4マスゴール対応探索
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			new_serch_algorithm(0, 0, x, y, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;

		case 3:
			sample_flag = 0;
			direction_count = 0;
			COPPY_SAVEDMAZE_TO_TEMP();	//クラッシュ後でも使用できるように.20171006
			adachihou_q(goal_x, goal_y, 0, 0, 500.0, 5000.0);	//4マスゴール対応探索
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			new_serch_algorithm(0, 0, x, y, 2);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			if(fail_flag==0){
				WALL_INFORMATION_save();
				WATCHED_WALL_INFORMATION_save();
			}
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:

			break;
		case 5:

			break;
		case 6:
			break;
		case 7:
			break;
		case 11:
			break;
		}
		break;

	case 6:	//北信越パス予備用-第1層	//xxx パス②はここ
		wall_gain = number_select();
		switch (wall_gain){
		case 0:
			vel=2000.0;	//0.15	//壁制御はP制御のみ
			break;
		case 1:
			vel=2400.0;
			break;
		case 2:
			vel=2600.0;
			break;
		case 3:
			vel=3000.0;
			break;
		case 4:
			vel=3400.0;
			break;
		case 5:
			vel=3600.0;
			break;
		case 6:
			vel=3800.0;
			break;
		case 7:
			vel=4000.0;
			break;
		}
		wall_gain = number_select();
		switch (wall_gain){
		case 0:
			max_accel=7000.0;	//0.15	//壁制御はP制御のみ
			break;
		case 1:
			max_accel=7500.0;
			break;
		case 2:
			max_accel=8000.0;
			break;
		case 3:
			max_accel=8500.0;
			break;
		case 4:
			max_accel=9000.0;
			break;
		case 5:
			max_accel=9500.0;
			break;
		case 6:
			max_accel=10000.0;
			break;
		}

		LED_motion2();
		switch (second_number) {		//以下、北信越pass-第2層

		case 1:

			mazedata_1112();
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass_skew();
			unknown_WALL_remove();	//探索が続く場合はremoveする
			LED4 = 0;

			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}

			exe_pass_EX(vel, 2400.0, 7000.0, 1);	//
//			exe_pass_EX(1500.0, 800.0, 7000.0, 1);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 2:
			mazedata_1112();
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
//			 WALL_INFORMATION_save();
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass_skew();
//			convert_pass();	//斜め無し用のパスに変換
			unknown_WALL_remove();	//探索が続く場合はremoveする
			LED4 = 0;

			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}

			exe_pass_EX(vel, 2400.0, max_accel, 4);	//とりあえず、大回りの移植ができているかの確認
//			exe_pass_EX(2000.0, 800.0, 7000.0, 1);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 3:
			mazedata_1112();
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
//			 WALL_INFORMATION_save();
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass_skew();
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
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}

			exe_pass_EX(vel, 2400.0, max_accel, 6);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 4:
			mazedata_1112();
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
//			 WALL_INFORMATION_save();
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass_skew();
//			convert_pass();	//斜め無し用のパスに変換
			unknown_WALL_remove();	//探索が続く場合はremoveする
			LED4 = 0;
			sensor_enable = 1;
			wait(500);
			while (sen.right_side < 2500)
				;
			wait(1000);
			refer_flag = 1;	//リファレンス取得用フラグ
			gyro_enable = 1;
			while (reference_fin == 0)
				;
			reference_fin = 0;

			GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
			if (reference_omega == 0.0) {
				GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			}


			exe_pass_EX(vel, 2500.0, max_accel, 7);	//とりあえず、大回りの移植ができているかの確認
//			exe_pass_EX(2000.0, 800.0, 7000.0, 1);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 5:

			break;
		case 6:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
//			 WALL_INFORMATION_save();
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
//			 WALL_INFORMATION_save();	//正しい元情報に戻った

			exe_pass_EX(2800.0, 800.0, 9000.0, 3);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 7:
			LED4 = 1;	//斜め無しpassの作成
			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
//			 WALL_INFORMATION_save();
			make_pass(Pass_Goal_x, Pass_Goal_y);
			convert_pass();	//斜め無し用のパスに変換
			LED4 = 0;

			unknown_WALL_remove();	//探索が続く場合はremoveする
//			 WALL_INFORMATION_save();	//正しい元情報に戻った

			exe_pass_EX(3000.0, 800.0, 10000.0, 3);	//とりあえず、大回りの移植ができているかの確認
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			sensor_enable = 0;

			break;
		case 8:
			break;
		case 11:
			break;
		}

		break;

	case 7:
		LED_motion2();
		//以下、表示系統調整-第2層
		switch (second_number) {
		case 1:
			mazedata_1112();
			LED4 = 1;	//斜め無しpassの作成
//			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			MAKE_PASS_SKEW(Pass_Goal_x, Pass_Goal_y);
			CONVERT_SKEWMAP_PASS();	//斜め無し用のパスに変換
//			unknown_WALL_remove();	//探索が続く場合はremoveする
			LED4 = 0;

			for (i = 0; i <= last_p_i; i++) {
				myprintf("pass[%d]=%d\r\n", i, pass[i]);
			}
			for (i = 0; i <= last_p_i; i++) {
				myprintf("motion[%d]=%d\r\n", i, motion[i]);
			}
			maze_display();
			saved_maze_display();
			skew_walkmap_display();
			walkmap_display();

			break;
		case 2:
//			myprintf("sample1, sample2\n\r");
//			for (kk = 1; kk < SAMPLE_NUMBER - 1; kk++) {
//				myprintf("%f, %f\n\r", sample1[kk], sample2[kk]);
//			}
			myprintf("log1, log2, log3, log4, log5, log6, log7, log8, log9\n\r");
			for (kk = 0; kk < LOG_NUMBER - 1; kk++) {
				myprintf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n\r", Log1[kk],
						Log2[kk], Log3[kk], Log4[kk], Log5[kk]/*, Log6[kk],
						Log7[kk], Log8[kk], Log9[kk]*/);
			}

			break;
		case 3:
			sensor_enable = 1;
			while (1) {
				if(switch_1==0){
					break;
				}
				myprintf(
						"sen.left_front=%d sen.left_side=%d sen.right_side=%d sen.right_front=%d\n\r",
						sen.left_front, sen.left_side, sen.right_side,
						sen.right_front);
			}

			break;
		case 4://迷路状法出力用
			for (i = 0; i < 15; i++) {
				myprintf("column_fix[%d]=%d;\r\n", i, column_fix[i]);
			}
			for (i = 0; i < 15; i++) {
				myprintf("row_fix[%d]=%d;\r\n", i, row_fix[i]);
			}
			for (i = 0; i < 15; i++) {
				myprintf("column_watched_fix[%d]=%d;\r\n", i, column_watched_fix[i]);
			}
			for (i = 0; i < 15; i++) {
				myprintf("row_watched_fix[%d]=%d;\r\n", i, row_watched_fix[i]);
			}

			break;
		case 5:
			Search_UnknownWall_Pass(7,7);//良さげ！
			walkmap_display();
			Unknown_Wall_Pass_display();
			Set_Temp_Goal();
			q_new_walk_map_maker(0,0,0,7,7);
			walkmap_display();

			break;
		case 6:
			walkmap_display();
			Unknown_Wall_Pass_display();

			break;
		case 7:
			maze_display();
			saved_maze_display();
			walkmap_display();

			break;
		case 8:

			for (i = 0; i <= last_p_i; i++) {
				myprintf("pass[%d]=%d\r\n", i, pass[i]);
			}
			for (i = 0; i <= last_p_i; i++) {
				myprintf("motion[%d]=%d\r\n", i, motion[i]);
			}
			unknown_WALL_add();
			maze_display();
			saved_maze_display();
			pass_walkmap_display();
			walkmap_display();
			skew_walkmap_display();

			break;
		case 9:
			myprintf("x=%d\r\n",x);
			myprintf("y=%d\r\n",y);
			break;

		case 10:

			break;
		case 11:
			break;

		}

		break;

	case 8:		//壁切れ調整
		LED_motion2();
		sensor_enable = 1;
		wait(500);
		while (sen.right_side < 3000)
			;
		wait(1000);
		refer_flag = 1;	//リファレンス取得用フラグ
		gyro_enable = 1;

		while (reference_fin == 0)
			;
		reference_fin = 0;

		GPT.GTSTR.BIT.CST0 = 1;		//カウント開始！
		if (reference_omega == 0.0) {
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
		}
		switch (second_number) {
		case 1:
			vel = 500.0;
			test_daikei(90.0 * 1.0, vel, 5000.0, 0.0, vel, 1);
			sample_flag=1;
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
			vel = 650.0;
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
		case 4:
			vel = 1000.0;
			test_daikei(90.0 * 1.0, vel, 6000.0, 0.0, vel, 1);
			test_daikei(90.0 * 4.0, vel, 5000.0, vel, vel, 1);
			test_daikei(180.0 * 1.0, vel, 6000.0, vel, 0.0, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			wall_control = 0;
			sensor_enable = 0;

			break;
		case 5:
			vel = 1200.0;
			test_daikei(180.0 * 1.0, vel, 6000.0, 0.0, vel, 1);
			test_daikei(90.0 * 4.0, vel, 5000.0, vel, vel, 1);
			test_daikei(180.0 * 1.0, vel, 6000.0, vel, 0.0, 1);
			wall_control = 0;
			ideal_balance_velocity = 0.0;
			wait(1000);
			GPT.GTSTR.BIT.CST0 = 0;		//カウント終了
			wall_control = 0;
			sensor_enable = 0;

			break;

		case 6:

			break;
		case 7:

			break;
		case 11:
			break;

		}

		break;

	case 9:
		LED_motion2();

		break;

	case 10:
		LED_motion2();
		initialize();

		break;

	case 11:
		LED_motion2();
		switch (second_number) {
		case 1:
			//座標変更モード!!
			goal_x = number_select();
			goal_y = number_select();
			myprintf("goal_x=%d  goal_y=%d\r\n", goal_x, goal_y);

			break;
		case 2://壁制御ゲイン
			wall_gain = number_select();
			switch (wall_gain){
			case 0:
				K_wall.p = 0.15;	//0.15	//壁制御はP制御のみ
				break;
			case 1:
				K_wall.p = 0.14;	//0.15	//壁制御はP制御のみ
				break;
			case 2:
				K_wall.p = 0.13;	//0.15	//壁制御はP制御のみ
				break;
			case 3:
				K_wall.p = 0.12;	//0.15	//壁制御はP制御のみ
				break;
			case 4:
				K_wall.p = 0.11;	//0.15	//壁制御はP制御のみ
				break;
			case 5:
				K_wall.p = 0.10;	//0.15	//壁制御はP制御のみ
				break;
			case 6:
				K_wall.p = 0.16;	//0.15	//壁制御はP制御のみ
				break;
			case 7:
				K_wall.p = 0.17;	//0.15	//壁制御はP制御のみ
				break;
			}

			break;
		case 3://斜め壁制御ゲイン
			wall_gain = number_select();
			switch (wall_gain){
			case 0:
				K_skew_wall.p = KP_SKEW_WALL;	//0.15	//壁制御はP制御のみ
				break;
			case 1:
				K_wall.p = 0.080;	//0.15	//壁制御はP制御のみ
				break;
			case 2:
				K_wall.p = 0.085;	//0.15	//壁制御はP制御のみ
				break;
			case 3:
				K_wall.p = 0.090;	//0.15	//壁制御はP制御のみ
				break;
			case 4:
				K_wall.p = 0.095;	//0.15	//壁制御はP制御のみ
				break;
			case 5:
				K_wall.p = 0.100;	//0.15	//壁制御はP制御のみ
				break;
			case 6:
				K_wall.p = 0.105;	//0.15	//壁制御はP制御のみ
				break;
			case 7:
				K_wall.p = 0.110;	//0.15	//壁制御はP制御のみ
				break;
			}

			break;
		case 4:
			temp_test_mazedata();

			saved_maze_display();
			pass_walkmap_display();
//			unknown_WALL_add();	//帰り探索のためには後でremoveする必要あり
			saved_maze_display();
//			skew_queue_walkmap_maker(7,7);
			MAKE_PASS_SKEW(7,7);
			for (i = 0; i <= last_p_i; i++) {
				myprintf("pass[%d]=%d\r\n", i, pass[i]);
			}
			CONVERT_SKEWMAP_PASS();
			for (i = 0; i <= last_p_i; i++) {
				myprintf("motion[%d]=%d\r\n", i, motion[i]);
			}

			skew_walkmap_display();
			pass_walkmap_display();
			saved_maze_display();

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
		case 11:
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
	K_wall.p = 0.15;	//0.15	//壁制御はP制御のみ
	K_wall.i = 0.0;
	K_wall.d = 0.0;
	K_skew_wall.p = KP_SKEW_WALL;
	goal_x = GOAL_X;	//ひとまずこれで代入。変更できるのは座標変更モードのみ。
	goal_y = GOAL_Y;	//ひとまずこれで代入。変更できるのは座標変更モードのみ。
	Pass_Goal_x = GOAL_X;
	Pass_Goal_y = GOAL_Y;

//	GoalPosition_assign(3,3);
//	GoalPosition_assign(7,7);
//	GoalPosition_assign(10,10);
//	GoalPosition_assign(15,3);
//	q_new_walk_map_maker(7,7,2 ,x,y);
//	walkmap_display();
//	GoalPosition_remove(7,7);
//	GoalPosition_remove(15,3);
//	q_new_walk_map_maker(7,7,2,x,y);
//	walkmap_display();

/*	temp_test_mazedata_3();
	direction_count = 2;
	Search_UnknownWall_Pass(7,7);//良さげ！
	walkmap_display();
	x=7;
	y=7;
	new_serch_algorithm(0,0,goal_x,goal_y,1);
	*/
/*	temp_test_mazedata_4();
	sample_flag=2;
	time_flag = 1;
	for(i=0;i<10;i++){
		Search_UnknownWall_Pass(goal_x, goal_y);			//探索中のpassに基づいて未知壁をセットする
		Set_Temp_Goal();									//通った未知壁情報をもとにゴール座標をセット
		q_new_walk_map_maker(0, 0, 0, x, y);				//goal_size==0の場合、(0,0)はゴール座標としてセットされない。
	}
	time_flag=0;*/
//	Search_UnknownWall_Pass(7,7);//良さげ！
//	walkmap_display();
//	Unknown_Wall_Pass_display();
//	Set_Temp_Goal();
//	q_new_walk_map_maker(0,0,0,7,6);
//	walkmap_display();


//	Compete_Near_Position(119, 51, 83);
//	possible_tamp_goal[1] = 119;
//	possible_tamp_goal[3] = 102;
//	possible_tamp_goal[2] = 85;
//	possible_tamp_goal[4] = 68;
//	possible_tamp_goal[0] = 51;
//	possible_tamp_goal[5] = 6;
//	possible_tamp_goal[6] = 7;
//	possible_tamp_goal[7] = 8;
//	Get_Possible_Near_Goal_Positions(31, 8);
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
