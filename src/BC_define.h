/*
 * BC_define.h
 *
 *  Created on: 2017/02/10
 *      Author: Alex
 */

#ifndef BC_DEFINE_H_
#define BC_DEFINE_H_

#define LED1 PORT2.DR.BIT.B2
#define LED2 PORT2.DR.BIT.B3
#define LED3 PORTB.DR.BIT.B1
#define LED4 PORTB.DR.BIT.B3
#define	LED5 PORTB.DR.BIT.B2
#define LED_V1 PORT7.DR.BIT.B5
#define LED_V2 PORT7.DR.BIT.B3
#define LED_V3 PORT7.DR.BIT.B2
#define LED_V4 PORT2.DR.BIT.B4
#define SEN_LED_r_s PORTB.DR.BIT.B4
#define SEN_LED_r_f PORTB.DR.BIT.B5
#define SEN_LED_l_f PORTB.DR.BIT.B6
#define SEN_LED_l_s PORTB.DR.BIT.B7
#define switch_1 PORTD.PORT.BIT.B6
#define switch_2 PORTD.PORT.BIT.B7
#define pi 3.141592
#define d_tire 22.25 //22.15(20171028に改訂)
#define m_body 0.100	//kg単位
#define Kp 0.001
#define Kd 0.0001
#define Ki 0.00005
#define Kp_rot 0.001 //0.001
#define Kd_rot 0.0012 //0.0012
#define Ki_rot 0.1 //0.1
#define KP_SKEW_WALL 0.105	//0.075
//#define Kp_wall 0.00005
#define DIFF_THRESHOLD 8.0//15.0(10_29)
#define DIFF_WALL_THRE 40.0//100.0(10_29)
#define sen_right_refer 1370 //2000
#define sen_left_refer 1900 //1520
#define r_threshold 900//700//1000　　//xxx 高すぎるので、部室で低くすること(左はこのままでOKかも)
#define l_threshold 1000//800//900
#define SKEW_R_Threshold 2000	//2500
#define SKEW_L_Threshold 2600	//2800
#define SKEW_R_Refer 2400//3000
#define SKEW_L_Refer 3100	//3100
#define SKEW_RFront_Threshold 800	//
#define SKEW_LFront_Threshold 800	//
#define SKEW_RFront_Refer 650	//
#define SKEW_LFront_Refer 650	//
#define r_wall_judge 800
#define l_wall_judge 800
#define r_front_wall_judge 1000//110
#define x_size 15
#define y_size 15
#define GOAL_X 0
#define GOAL_Y 6
#define SAMPLE_NUMBER 1	//800
#define LOG_NUMBER 350//150

volatile typedef struct { /* 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する */
	float right;
	float left;
} float_rightleft_t;
volatile typedef struct { /* 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する */
	signed int right;
	signed int left;
} int_rightleft_t;
volatile typedef struct { /* 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する */
	signed short int High;
	signed short int Low;
	signed short int SUM;
} float_highlow_t;
volatile typedef struct { /* 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する */
	signed int right_side;
	signed int left_side;
	signed int right_front;
	signed int left_front;
} int_sensor_t;
volatile typedef struct { /* 構造体の型枠を定義して，同時にそれを型名 velocty_t として定義する */
	float p;
	float i;
	float d;
} float_pid_t;
volatile typedef struct {
	unsigned short temp;
	unsigned short fixed;
} unsigned_short_fix_t;
volatile typedef struct turn_parameters { /*構造体turn_parametersを宣言し、変数としてturn_parameters_tを確保する*/
	float theta;	//ターン角度
	float th1;	//角速度加速区間
	float th2;	//角速度減速区間
	float a_cc;	//角加速度
	float wise;	//回転方向
	float d_f;	//前距離
	float d_r;	//後距離
	float vel;	//重心速度
	char skew;		//ターンは斜め走行か
	char enable;	//パラメータは使えるか
} turn_parameters_t;

volatile typedef struct turn_velocities {
	struct turn_parameters P_1_0;
	struct turn_parameters P_1_1;
	struct turn_parameters P_1_2;
	struct turn_parameters P_1_3;
	struct turn_parameters P_1_4;
	struct turn_parameters P_1_5;
	struct turn_parameters P_1_6;
	struct turn_parameters P_1_7;
	struct turn_parameters P_1_8;
	struct turn_parameters P_1_9;
	struct turn_parameters P_1_10;
	struct turn_parameters P_1_11;
	struct turn_parameters P_1_12;
	struct turn_parameters P_1_13;
	struct turn_parameters P_1_14;
	struct turn_parameters P_1_15;

} turn_velocities_t;

volatile typedef struct sbits {
	unsigned char bit1 :1;	//最下位ビット
	unsigned char bit2 :1;
	unsigned char bit3 :1;
	unsigned char bit4 :1;
	unsigned char bit5 :1;
	unsigned char bit6 :1;
	unsigned char bit7 :1;
	unsigned char bit8 :1;	//最上位ビット
} bits_t;

volatile typedef struct skabekire {
	unsigned char next_r :1;	//読むべき方向//最下位ビット
	unsigned char next_l :1;	//読むべき方向
	unsigned char enable1 :1;//
	unsigned char enable2 :1;
	unsigned char wait :1;
	unsigned char detect_r :1;	//右を検出
	unsigned char detect_l :1;	//左を検出
	unsigned char detected :1;	//最上位ビット
} kabekire_t;

extern volatile unsigned int queue[700];
extern volatile unsigned char sample_flag, refer_flag, gyro_enable,
		reference_fin, wall_control, ei_flag_center, ei_flag_rot,
		direction_count, x, y, sensor_enable, q_dist_flag, kabekire_right,
		kabekire_left, kabeiri_right, kabeiri_left, kabekire_enable,
		kabekire_enable_2, kushi_judge, pass[200] , motion[100] ,
		last_p_i, reverse_flag, pass_kabekire_straight, wait_kabekire,
		kabekire_read_flag, fail_flag, sen_fail_flag, fail_count, ei_flag_wall,
		goal_x, goal_y, temp_goal_x, temp_goal_y, daikei_mode, aa, bb, cc;
extern volatile int sen_bat, sen_l_f_ON, sen_l_s_ON, sen_r_f_ON, sen_r_s_ON,
		sen_l_f_OFF, sen_l_s_OFF, sen_r_f_OFF, sen_r_s_OFF, gptcount_l,
		gptcount_r, sample_count, refer_count, buff_sen_right[10],
		buff_sen_left[10], sen_count, jkl;
extern volatile unsigned short map[16][16], row_temp[15], column_temp[15],
		row_fix[15], column_fix[15], row_watched_fix[15],
		column_watched_fix[15], row_watched_temp[15], column_watched_temp[15],
		unknown_wall_row[15], unknown_wall_column[15], level[16][15],
		vertical[15][16], read_P_I;

extern volatile long cmt_count;
extern volatile float Battery, dutty_r , dutty_l , omega , angle ,
		reference_omega , diff_omega[3] , diff_kabe[3] ,
		ideal_omega , ideal_omega2 , ideal_angle, ideal_angacc ,
		balance_velocity, balance_distance , ideal_balance_velocity ,
		ideal_balance_accel , ideal_balance_distance ,
		diff_balance_velocity[3], Error, Kp_wall_r, Kp_wall_l, EI_keisuu ,
		EP_keisuu , pre_buff_sen_right[10] ,
		pre_buff_sen_left[10] , pre_kabe_ave_right ,
		pre_kabe_ave_left , pre_ave_ave_right ,
		pre_ave_ave_left , d_i , adjust_before_dist ,
		adjust_distance ,vel_low, vel_high, accel_normal;
extern volatile float_rightleft_t velocity, total_distance, accel, old_velocity,
		ideal_velocity, ideal_accel, ideal_distance, diff_velocity[3], dutty,
		dutty_foward, average_sensor, diff_average_sensor, average_sensor_old;
extern volatile int_rightleft_t old_tcnt, tcnt, diff_tcnt, motor_direc;
extern volatile float_highlow_t gyro_x, gyro_y, gyro_z, accel_x, accel_y,
		accel_z;
extern volatile float_pid_t K_center, K_rot, Erorr_center, Erorr_rot;
extern volatile int_sensor_t sen;
extern volatile unsigned_short_fix_t a;
extern volatile const turn_velocities_t turn[10];
extern volatile kabekire_t flags_kabekire;

/*
 extern volatile unsigned char sample_flag, refer_flag, gyro_enable,
 reference_fin, direction_count, x, y;
 extern volatile unsigned short row[15], colum[15], Wall_Judge_watched,
 row_watched_temp[15], column_watched_temp[15], map[16][16];
 extern volatile float_rightleft_t velocity, total_distance, accel, old_velocity,
 ideal_velocity, ideal_accel, diff_velocity[3], dutty, dutty_foward;
 extern volatile int_rightleft_t old_tcnt, tcnt, diff_tcnt, motor_direc;
 extern volatile float_highlow_t gyro_x, gyro_y, gyro_z, accel_x, accel_y,
 accel_z;
 extern volatile int_sensor_t sen;

 extern volatile int sen_bat, gptcount_l, gptcount_r, sample_count, refer_count;
 extern volatile float Battery, dutty_r, dutty_l, omega, angle, reference_omega,
 ideal_angacc;
 extern volatile long cmt_count;
 */

//extern volatile signed short int sample[2500];
//extern volatile float sample[2500] ;
#endif /* BC_DEFINE_H_ */
