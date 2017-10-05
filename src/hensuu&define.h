/*
 * hensuu&define.h
 *
 *  Created on: 2016/02/20
 *      Author: Alex
 */

//defineの値はこのヘッダーファイルで変更する！ゴール座標などを簡単に変更できる！

#ifndef HENSUU_DEFINE_H_
#define HENSUU_DEFINE_H_

#define  motor_enable PE.DRL.BIT.B8
#define LED1_p PB.DR.BIT.B1
#define LED2_y PB.DR.BIT.B2
#define LED3_r PB.DR.BIT.B3
#define pi 3.141592
#define r_tire 25.96
#define l_tire 25.92
#define R_tread /*44.2*/43.65
#define r_tire_control_ON 25.96
#define l_tire_control_ON 25.92
#define SEN_r_ref 600
#define SEN_l_ref 500
#define r_threshold 200
#define l_threshold 250
#define r_front_threshold
#define l_front_threshold
#define DIFF_THRESHOLD 2
#define Kp_r 0.15
#define Kp_l 0.15
#define Kp_center 0.15
#define r_wall_judge 180
#define l_wall_judge 250
#define r_front_wall_judge 120
#define l_front_wall_judge
#define x_size 15
#define y_size 15
#define goal_x 1
#define goal_y 0
//#define V_FAST 2000
//#define ACCEL_FAST 3000
//#define V_search 480
//#define ACCEL_search 2000
#define STOP_count 180
//#define V_path 500
//#define ACCEL_path 2000
//#define STOP_count_pat 150

extern volatile int SEN, SEN_l_value, SEN_l_front_value, SEN_r_front_value,
		SEN_r_value, SEN_r_diff, SEN_l_diff, SEN_r_old_value, SEN_l_old_value,
		wall_control, SEN_l_front_value_ON, SEN_r_front_value_ON,
		SEN_l_value_ON, SEN_r_value_ON, SEN_l_front_value_OFF,
		SEN_r_front_value_OFF, SEN_l_value_OFF, SEN_r_value_OFF, sen_count ,
		mode_count, direction_count, x, y, Wall_Judge, watched_Wall_Judge,
		Mark_Judge, j, xj, yj, i, k, l, direction_number, s_count, test1,
		clash_count, p_i, kabe_cancel;
extern volatile unsigned long cmt_count;
extern volatile float Battery, speed, speed_r, speed_l, accel, angacc, omega,
		angle, speed_relative, speed_turn, total_dist, TGRA_r, TGRA_l, Error,
		Control, r_sen_thredhold, l_sen_thredhold;
extern volatile unsigned short colum[15] , row[15] ,
		colum_watched[15] , row_watched[15] , colum2[15] ,
		row2[15] , colum_watched2[15] , row_watched2[15] ,
		remove_row , remove_colum , q_s[257], q_v[257];
extern volatile unsigned char renewal_flag , accident_flag , q[257],
		map[16][16], map2[16][16], footmark[16][16] , straight_count , /*vector_map[16][16],*/
		straight_flag[16][16] , straight_map[16][16] ,
		flag_kabekire , qx, qy, interrupt_kabekire , count_kabekire_r ,
		pass[200], last_p_i, con_p_i, read_p_i, d_i ; // 区画の座標(0～255)を入れる配列 左下0、右下15、左上240、右上255
extern volatile short head, tail;          // 先頭位置, 末尾位置
extern volatile unsigned short int node_colum[16][16], node_row[16][16],
		node_dist_colum[16][16], node_dist_row[16][16], node_center[16][16];


#endif /* HENSUU_DEFINE_H_ */
