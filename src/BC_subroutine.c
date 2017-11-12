/*
 * BC_subroutine.c
 *
 *  Created on: 2017/05/05
 *      Author: Alex
 */
#include "iodefine.h"
#include "BC_subroutine.h"
#include "BC_define.h"
#include "math.h"

void battery_ADconvert() {
	SYSTEM.MSTPCRA.BIT.MSTPA16 = 0; //12bitAD1ストップ解除
	S12AD1.ADANS.BIT.CH = 3;	//シングル＆AN103なので
	S12AD1.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD1.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_bat = S12AD1.ADDR3;
	Battery = 5.0 * 2.0 / 1.0 * sen_bat / 4096.0;
	S12AD1.ADCSR.BIT.ADST = 0;
}

void sensor_ADconvert() {
	volatile int kk = 0;
	SYSTEM.MSTPCRA.BIT.MSTPA17 = 0; //12bitAD0ストップ解除

	SEN_LED_r_f = 0;
	SEN_LED_l_f = 0;
	SEN_LED_r_s = 0;
	SEN_LED_l_s = 0;
	//以下消灯時のセンサ値
	S12AD0.ADANS.BIT.CH = 1;	//シングル＆AN001なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_r_f_OFF = S12AD0.ADDR1;
	S12AD0.ADCSR.BIT.ADST = 0;

	S12AD0.ADANS.BIT.CH = 2;	//シングル＆AN002なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_l_f_OFF = S12AD0.ADDR2;
	S12AD0.ADCSR.BIT.ADST = 0;

	S12AD0.ADANS.BIT.CH = 0;	//シングル＆AN000なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_r_s_OFF = S12AD0.ADDR0A;
	S12AD0.ADCSR.BIT.ADST = 0;

	S12AD0.ADANS.BIT.CH = 3;	//シングル＆AN003なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_l_s_OFF = S12AD0.ADDR3;
	S12AD0.ADCSR.BIT.ADST = 0;

	SEN_LED_r_s = 1;
	SEN_LED_l_s = 0;
	SEN_LED_r_f = 0;
	SEN_LED_l_f = 1;
	while (1) {	//原因不明のforが機能しない問題
		kk++;
		if (kk >= 20) {
			kk = 0;
			break;
		}
	}
//	for (kk = 0; kk++; kk < 10000)
//		;	//LEDが点くまでの待ち時間
	//以下点灯時の"対角線"センサ値
	S12AD0.ADANS.BIT.CH = 0;	//シングル＆AN000なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_r_s_ON = S12AD0.ADDR0A;
	S12AD0.ADCSR.BIT.ADST = 0;

	S12AD0.ADANS.BIT.CH = 2;	//シングル＆AN002なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_l_f_ON = S12AD0.ADDR2;
	S12AD0.ADCSR.BIT.ADST = 0;

	SEN_LED_r_s = 0;
	SEN_LED_l_s = 1;
	SEN_LED_r_f = 1;
	SEN_LED_l_f = 0;
	while (1) {	//原因不明のforが機能しない問題
		kk++;
		if (kk >= 20) {
			kk = 0;
			break;
		}
	}

	S12AD0.ADANS.BIT.CH = 3;	//シングル＆AN003なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_l_s_ON = S12AD0.ADDR3;
	S12AD0.ADCSR.BIT.ADST = 0;

	//以下点灯時の"前"センサ値
	S12AD0.ADANS.BIT.CH = 1;	//シングル＆AN001なので
	S12AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	while (S12AD0.ADCSR.BIT.ADST == 1)
		; //AD変換完了まで待つ
	sen_r_f_ON = S12AD0.ADDR1;
	S12AD0.ADCSR.BIT.ADST = 0;

	sen.right_front = sen_r_f_ON - sen_r_f_OFF;		//センサ値の計算
	sen.left_front = sen_l_f_ON - sen_l_f_OFF;
	sen.right_side = sen_r_s_ON - sen_r_s_OFF;
	sen.left_side = sen_l_s_ON - sen_l_s_OFF;
	SEN_LED_r_f = 0;
	SEN_LED_l_f = 0;
	SEN_LED_r_s = 0;
	SEN_LED_l_s = 0;
}

void motor_direction(char direction_left, char direction_right) { //正転・・・1,逆回転・・・0
	//以下、モータードライバの出力方向の設定に使用
	PORT7.DDR.BIT.B0 = 1;		//STBポート
	PORT9.DDR.BIT.B3 = 1;		//右モータ、IN1
	PORT9.DDR.BIT.B4 = 1;		//右モータ、IN2
	PORT7.DR.BIT.B0 = 1;		//STBポート-high(STB_h,IN1_l,IN2_hでCW)
	PORT9.DDR.BIT.B2 = 1;		//左モータ、IN1
	PORT9.DDR.BIT.B1 = 1;		//左モータ、IN2
	if (direction_right == 1) {
		PORT9.DR.BIT.B3 = 0;		//右モータ、IN1-low
		PORT9.DR.BIT.B4 = 1;		//右モータ、IN2-high
		motor_direc.right = 1;	//現在の回転方向を保存！
	} else {
		PORT9.DR.BIT.B3 = 1;
		PORT9.DR.BIT.B4 = 0;
		motor_direc.right = 0;
	}
	if (direction_left == 1) {
		PORT9.DR.BIT.B2 = 0;		//左モータ、IN1-low
		PORT9.DR.BIT.B1 = 1;		//左モータ、IN2-high
		motor_direc.left = 1;
	} else {
		PORT9.DR.BIT.B2 = 1;
		PORT9.DR.BIT.B1 = 0;
		motor_direc.left = 0;
	}
}

void wait(int cnt) {
	cmt_count = 0;
	while (cmt_count <= cnt) {
	}
}

void interrupt_GPT0_GTCCRA() {
	dutty.left = 120;
	GPT0.GTCCRA = dutty.left;
	LED1 = 1;
	GPT0.GTST.BIT.TCFA = 0;	//フラグクリアのはず・・・
}

void interrupt_GPT0_GTCCRB() {
	LED1 = 1;
}

int mode_select(void) {
	volatile int mode_count = 0, k;
	while (1) {
		if (switch_1 == 0 && mode_count == 0) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 1;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			mode_count = 1;
		} else if (switch_1 == 0 && mode_count == 1) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 1;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			mode_count = 2;
		} else if (switch_1 == 0 && mode_count == 2) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 1;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			mode_count = 3;
		} else if (switch_1 == 0 && mode_count == 3) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 1;
			LED2 = 0;
			LED1 = 0;
			mode_count = 4;
		} else if (switch_1 == 0 && mode_count == 4) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 1;
			LED1 = 0;
			mode_count = 5;
		} else if (switch_1 == 0 && mode_count == 5) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 1;
			mode_count = 6;
		} else if (switch_1 == 0 && mode_count == 6) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 1;
			LED_V2 = 1;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			mode_count = 7;
		} else if (switch_1 == 0 && mode_count == 7) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			mode_count = 8;
		} else if (switch_1 == 0 && mode_count == 8) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 1;
			LED_V4 = 1;
			LED2 = 0;
			LED1 = 0;
			mode_count = 9;
		} else if (switch_1 == 0 && mode_count == 9) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 1;
			LED2 = 1;
			LED1 = 0;
			mode_count = 10;
		} else if (switch_1 == 0 && mode_count == 10) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 1;
			LED1 = 1;
			mode_count = 11;
		} else if (switch_2 == 0) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			break;
		} else if (switch_1 == 0 && mode_count == 11) {
			mode_count = 0;
		}
	}
	return mode_count;
}

int number_select(void) {
	volatile int number_count = 0, k;
	balance_distance =0.0;
	while (1) {
		if (balance_distance >=5.0 && number_count == 0) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 1;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			number_count = 1;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 1) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 1;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			number_count = 2;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 2) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 1;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			number_count = 3;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 3) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 1;
			LED2 = 0;
			LED1 = 0;
			number_count = 4;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 4) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 1;
			LED1 = 0;
			number_count = 5;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 5) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 1;
			number_count = 6;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 6) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 1;
			LED_V2 = 1;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			number_count = 7;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 7) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			number_count = 8;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 8) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 1;
			LED_V4 = 1;
			LED2 = 0;
			LED1 = 0;
			number_count = 9;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 9) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 1;
			LED2 = 1;
			LED1 = 0;
			number_count = 10;
			balance_distance =0.0;
		} else if (balance_distance >=5.0 && number_count == 10) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 1;
			LED1 = 1;
			number_count = 11;
			balance_distance =0.0;
		}else if (balance_distance >=5.0 && number_count == 11) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 1;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			number_count = 12;
			balance_distance =0.0;
		}else if (balance_distance >=5.0 && number_count == 12) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 1;
			LED2 = 0;
			LED1 = 0;
			number_count = 13;
			balance_distance =0.0;
		}else if (balance_distance >=5.0 && number_count == 13) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 1;
			LED_V4 = 1;
			LED2 = 1;
			LED1 = 0;
			number_count = 14;
			balance_distance =0.0;
		}else if (balance_distance >=5.0 && number_count == 14) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 1;
			LED2 = 1;
			LED1 = 1;
			number_count = 15;
			balance_distance =0.0;
		}else if (balance_distance >=5.0 && number_count == 15) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 1;
			LED_V2 = 1;
			LED_V3 = 1;
			LED_V4 = 1;
			LED2 = 0;
			LED1 = 0;
			number_count = 16;
			balance_distance =0.0;
		} else if (switch_2 == 0) {
			for (k = 0; k <= 2000000; k++) {
			}
			LED_V1 = 0;
			LED_V2 = 0;
			LED_V3 = 0;
			LED_V4 = 0;
			LED2 = 0;
			LED1 = 0;
			balance_distance =0.0;
			break;
		} else if (balance_distance >=5.0 && number_count == 16) {
			number_count = 0;
			balance_distance =0.0;
		}
	}
	return number_count - 1;	//初期番号を0としてカウントする場合を考えている
}

void watched_wall_front(int hikisuu_x, int hikisuu_y,
		int hikisuu_direction_count) {
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_y == 15) {
			break;
		}
		row_watched_temp[hikisuu_y] |= 32768 >> hikisuu_x;
		break;
	case 1:		//East
		if (hikisuu_x == 15) {
			break;
		}
		column_watched_temp[hikisuu_x] |= 1 << hikisuu_y;
		break;
	case 2:		//South
		if (hikisuu_y == 0) {
			break;
		}
		row_watched_temp[hikisuu_y - 1] |= 32768 >> hikisuu_x;
		break;
	case 3:		//West
		if (hikisuu_x == 0) {
			break;
		}
		column_watched_temp[hikisuu_x - 1] |= 1 << hikisuu_y;
		break;
	}
}

void watched_wall_right(int hikisuu_x, int hikisuu_y,
		int hikisuu_direction_count) {
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_x == 15) {
			break;
		}
		column_watched_temp[hikisuu_x] |= 1 << hikisuu_y;
		break;
	case 1:		//East
		if (hikisuu_y == 0) {
			break;
		}
		row_watched_temp[hikisuu_y - 1] |= 32768 >> hikisuu_x;
		break;
	case 2:		//South
		if (hikisuu_x == 0) {
			break;
		}
		column_watched_temp[hikisuu_x - 1] |= 1 << hikisuu_y;
		break;
	case 3:		//West
		if (hikisuu_y == 15) {
			break;
		}
		row_watched_temp[hikisuu_y] |= 32768 >> hikisuu_x;
		break;
	}
}

void watched_wall_left(int hikisuu_x, int hikisuu_y,
		int hikisuu_direction_count) {
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_x == 0) {
			break;
		}
		column_watched_temp[hikisuu_x - 1] |= 0x0001 << hikisuu_y;
		break;
	case 1:		//East
		if (hikisuu_y == 15) {
			break;
		}
		row_watched_temp[hikisuu_y] |= 0x8000 >> hikisuu_x;
		break;
	case 2:		//South
		if (hikisuu_x == 15) {
			break;
		}
		column_watched_temp[hikisuu_x] |= 0x0001 << hikisuu_y;
		break;
	case 3:		//West
		if (hikisuu_y == 0) {
			break;
		}
		row_watched_temp[hikisuu_y - 1] |= 0x8000 >> hikisuu_x;
		break;
	}
}

char is_Exist_Wall(char hikisuu_x, char hikisuu_y, char hikisuu_direction_count) {
	volatile unsigned short Wall_Judge = 0;
	switch (hikisuu_direction_count) {
	case 0:	//North
		if(hikisuu_y==y_size){
			break;
		}
		Wall_Judge = row_temp[hikisuu_y] & (0x8000 >> hikisuu_x);
		if (Wall_Judge == (0x8000 >> hikisuu_x)) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 1:	//East
		if(hikisuu_x==x_size){
			break;
		}
		Wall_Judge = column_temp[hikisuu_x] & 0x0001 << hikisuu_y;
		if (Wall_Judge == 0x0001 << hikisuu_y) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 2:	//South
		if(hikisuu_y==0){
			break;
		}
		Wall_Judge = row_temp[hikisuu_y - 1] & 0x8000 >> hikisuu_x;
		if (Wall_Judge == 0x8000 >> hikisuu_x) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 3:	//West
		if(hikisuu_x==0){
			break;
		}
		Wall_Judge = column_temp[hikisuu_x - 1] & 0x0001 << hikisuu_y;
		if (Wall_Judge == 0x0001 << hikisuu_y) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	}
	return Wall_Judge;
}

char is_saved_wall_exist(char hikisuu_x, char hikisuu_y,
		char hikisuu_direction_count) {
	volatile unsigned short Wall_Judge = 0;
	switch (hikisuu_direction_count) {
	case 0:	//North
//		if(hikisuu_y==y_size){
//			break;
//		}
		Wall_Judge = row_fix[hikisuu_y] & (0x8000 >> hikisuu_x);
		if (Wall_Judge == (0x8000 >> hikisuu_x)) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 1:	//East
//		if(hikisuu_x==x_size){
//			break;
//		}
		Wall_Judge = column_fix[hikisuu_x] & 0x0001 << hikisuu_y;
		if (Wall_Judge == 0x0001 << hikisuu_y) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 2:	//South
//		if(hikisuu_y==0){
//			break;
//		}
		Wall_Judge = row_fix[hikisuu_y - 1] & 0x8000 >> hikisuu_x;
		if (Wall_Judge == 0x8000 >> hikisuu_x) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 3:	//West
//		if(hikisuu_x==0){
//			break;
//		}
		Wall_Judge = column_fix[hikisuu_x - 1] & 0x0001 << hikisuu_y;
		if (Wall_Judge == 0x0001 << hikisuu_y) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	}
	return Wall_Judge;
}

void test_hidaritehou_slalom(int hikisuu_goal_x, int hikisuu_goal_y) {//ゴール座標ありの左手法

	direction_count = 0;
	wall_control = 1;
	x = 0;
	y = 1;
	test_daikei(90.0, 500.0, 3000.0, 0.0, 500.0, 1);

	while (1) {
		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
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
		if (sen.left_side < l_wall_judge) {
			if (direction_count == 0) {
				direction_count = 3;
			} else {
				direction_count--;
			}
			direction_xy();
			test_slalom(90.0, 6000.0, 4500.0, 1.0, 500.0, 5.0, 15.0);	//左折
		} else if (sen.right_front < r_front_wall_judge) {
			direction_xy();
			test_daikei(180.0, 500.0, 3000.0, 500.0, 500.0, 1);
		} else if (sen.right_side < r_wall_judge) {
			if (direction_count == 3) {
				direction_count = 0;
			} else {
				direction_count++;
			}
			direction_xy();
			test_slalom(90.0, 6000.0, 4500.0, -1.0, 500.0, 7.0, 15.0);	//右折
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
			test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 0);
			wait(300);
			test_turn(180, 5000, 2500, -1.0, 0.0);//(float hikisuu_angle, float omega_max, float hikisuu_angacc,float unclock_wise, float hikisuu_balance_velocity)
			wait(300);
			test_daikei(90.0, 500.0, 3000.0, 0.0, 500.0, 0);
		}
	}
	test_daikei(90.0, 500.0, 3000.0, 500.0, 0.0, 1);
	ideal_balance_velocity = 0.0;
	ideal_omega = 0.0;
	wait(300);
}

void add_wall_front(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_y == 15) {
			break;
		}
		row_temp[hikisuu_y] |= 32768 >> hikisuu_x;
		break;
	case 1:		//East
		if (hikisuu_x == 15) {
			break;
		}
		column_temp[hikisuu_x] |= 1 << hikisuu_y;
		break;
	case 2:		//South
		if (hikisuu_y == 0) {
			break;
		}
		row_temp[hikisuu_y - 1] |= 32768 >> hikisuu_x;
		break;
	case 3:		//West
		if (hikisuu_x == 0) {
			break;
		}
		column_temp[hikisuu_x - 1] |= 1 << hikisuu_y;
		break;
	}
}
void add_wall_right(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_x == 15) {
			break;
		}
		column_temp[hikisuu_x] |= 1 << hikisuu_y;
		break;
	case 1:		//East
		if (hikisuu_y == 0) {
			break;
		}
		row_temp[hikisuu_y - 1] |= 32768 >> hikisuu_x;
		break;
	case 2:		//South
		if (hikisuu_x == 0) {
			break;
		}
		column_temp[hikisuu_x - 1] |= 1 << hikisuu_y;
		break;
	case 3:		//West
		if (hikisuu_y == 15) {
			break;
		}
		row_temp[hikisuu_y] |= 32768 >> hikisuu_x;
		break;
	}
}
void add_wall_left(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_x == 0) {
			break;
		}
		column_temp[hikisuu_x - 1] |= 0x0001 << hikisuu_y;
		break;
	case 1:		//East
		if (hikisuu_y == 15) {
			break;
		}
		row_temp[hikisuu_y] |= 0x8000 >> hikisuu_x;
		break;
	case 2:		//South
		if (hikisuu_x == 15) {
			break;
		}
		column_temp[hikisuu_x] |= 0x0001 << hikisuu_y;
		break;
	case 3:		//West
		if (hikisuu_y == 0) {
			break;
		}
		row_temp[hikisuu_y - 1] |= 0x8000 >> hikisuu_x;
		break;
	}
}
void add_saved_wall_front(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {	//パスを生成するために必要20171002
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_y == 15) {
			break;
		}
		row_fix[hikisuu_y] |= 32768 >> hikisuu_x;
		break;
	case 1:		//East
		if (hikisuu_x == 15) {
			break;
		}
		column_fix[hikisuu_x] |= 1 << hikisuu_y;
		break;
	case 2:		//South
		if (hikisuu_y == 0) {
			break;
		}
		row_fix[hikisuu_y - 1] |= 32768 >> hikisuu_x;
		break;
	case 3:		//West
		if (hikisuu_x == 0) {
			break;
		}
		column_fix[hikisuu_x - 1] |= 1 << hikisuu_y;
		break;
	}
}
void add_saved_wall_right(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {	//パスを生成するために必要20171002
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_x == 15) {
			break;
		}
		column_fix[hikisuu_x] |= 1 << hikisuu_y;
		break;
	case 1:		//East
		if (hikisuu_y == 0) {
			break;
		}
		row_fix[hikisuu_y - 1] |= 32768 >> hikisuu_x;
		break;
	case 2:		//South
		if (hikisuu_x == 0) {
			break;
		}
		column_fix[hikisuu_x - 1] |= 1 << hikisuu_y;
		break;
	case 3:		//West
		if (hikisuu_y == 15) {
			break;
		}
		row_fix[hikisuu_y] |= 32768 >> hikisuu_x;
		break;
	}
}
void add_saved_wall_left(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {	//パスを生成するために必要20171002
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_x == 0) {
			break;
		}
		column_fix[hikisuu_x - 1] |= 0x0001 << hikisuu_y;
		break;
	case 1:		//East
		if (hikisuu_y == 15) {
			break;
		}
		row_fix[hikisuu_y] |= 0x8000 >> hikisuu_x;
		break;
	case 2:		//South
		if (hikisuu_x == 15) {
			break;
		}
		column_fix[hikisuu_x] |= 0x0001 << hikisuu_y;
		break;
	case 3:		//West
		if (hikisuu_y == 0) {
			break;
		}
		row_fix[hikisuu_y - 1] |= 0x8000 >> hikisuu_x;
		break;
	}
}

char is_the_Wall_watched(char hikisuu_x, char hikisuu_y,
		char hikisuu_direction_count) {
	volatile unsigned short Wall_Judge_watched = 0;
	switch (hikisuu_direction_count) {
	case 0:	//North
		Wall_Judge_watched = row_watched_temp[hikisuu_y] & 0x8000 >> hikisuu_x;
		if (Wall_Judge_watched == 0x8000 >> hikisuu_x) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	case 1:	//East
		Wall_Judge_watched = column_watched_temp[hikisuu_x]
				& 0x0001 << hikisuu_y;
		if (Wall_Judge_watched == 0x0001 << hikisuu_y) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	case 2:	//South
		Wall_Judge_watched = row_watched_temp[hikisuu_y - 1]
				& 0x8000 >> hikisuu_x;
		if (Wall_Judge_watched == 0x8000 >> hikisuu_x) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	case 3:	//West
		Wall_Judge_watched = column_watched_temp[hikisuu_x - 1]
				& 0x0001 << hikisuu_y;
		if (Wall_Judge_watched == 0x0001 << hikisuu_y) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	}
	return Wall_Judge_watched;
}

char is_saved_wall_watched(char hikisuu_x, char hikisuu_y,
		char hikisuu_direction_count) {	//パスを作成時、unknown wall add & remove に必要
	volatile unsigned short Wall_Judge_watched = 0;
	switch (hikisuu_direction_count) {
	case 0:	//North
		Wall_Judge_watched = row_watched_fix[hikisuu_y] & 0x8000 >> hikisuu_x;
		if (Wall_Judge_watched == 0x8000 >> hikisuu_x) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	case 1:	//East
		Wall_Judge_watched = column_watched_fix[hikisuu_x]
				& 0x0001 << hikisuu_y;
		if (Wall_Judge_watched == 0x0001 << hikisuu_y) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	case 2:	//South
		Wall_Judge_watched = row_watched_fix[hikisuu_y - 1]
				& 0x8000 >> hikisuu_x;
		if (Wall_Judge_watched == 0x8000 >> hikisuu_x) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	case 3:	//West
		Wall_Judge_watched = column_watched_fix[hikisuu_x - 1]
				& 0x0001 << hikisuu_y;
		if (Wall_Judge_watched == 0x0001 << hikisuu_y) {
			Wall_Judge_watched = 1;
		} else {
			Wall_Judge_watched = 0;
		}
		break;
	}
	return Wall_Judge_watched;
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


void temp_test_mazedata(){
//以下、学生大会迷路(watched wallの情報がないが、これは今回はパス生成に不要。)
	column_temp[0]=32009;
	column_temp[1]=60116;
	column_temp[2]=21818;
	column_temp[3]=43648;
	column_temp[4]=25884;
	column_temp[5]=6736;
	column_temp[6]=1488;
	column_temp[7]=14852;
	column_temp[8]=61826;
	column_temp[9]=16754;
	column_temp[10]=644;
	column_temp[11]=9544;
	column_temp[12]=10752;
	column_temp[13]=14096;
	column_temp[14]=30720;
	row_temp[0]=24574;
	row_temp[1]=44831;
	row_temp[2]=25326;
	row_temp[3]=189;
	row_temp[4]=51392;
	row_temp[5]=48605;
	row_temp[6]=22319;
	row_temp[7]=10836;
	row_temp[8]=54696;
	row_temp[9]=10837;
	row_temp[10]=21850;
	row_temp[11]=10804;
	row_temp[12]=5592;
	row_temp[13]=2848;
	row_temp[14]=1854;

	column_fix[0]=32009;
	column_fix[1]=60116;
	column_fix[2]=21818;
	column_fix[3]=43648;
	column_fix[4]=25884;
	column_fix[5]=6736;
	column_fix[6]=1488;
	column_fix[7]=14852;
	column_fix[8]=61826;
	column_fix[9]=16754;
	column_fix[10]=644;
	column_fix[11]=9544;
	column_fix[12]=10752;
	column_fix[13]=14096;
	column_fix[14]=30720;
	row_fix[0]=24574;
	row_fix[1]=44831;
	row_fix[2]=25326;
	row_fix[3]=189;
	row_fix[4]=51392;
	row_fix[5]=48605;
	row_fix[6]=22319;
	row_fix[7]=10836;
	row_fix[8]=54696;
	row_fix[9]=10837;
	row_fix[10]=21850;
	row_fix[11]=10804;
	row_fix[12]=5592;
	row_fix[13]=2848;
	row_fix[14]=1854;
}

void temp_test_mazedata_2(){
	column_fix[0]=845;
	column_fix[1]=896;
	column_fix[2]=576;
	column_fix[3]=1440;
	column_fix[4]=194;
	column_fix[5]=226;
	column_fix[6]=270;
	column_fix[7]=799;
	column_fix[8]=0;
	column_fix[9]=0;
	column_fix[10]=0;
	column_fix[11]=0;
	column_fix[12]=0;
	column_fix[13]=0;
	column_fix[14]=0;
	row_fix[0]=0;
	row_fix[1]=31232;
	row_fix[2]=32256;
	row_fix[3]=0;
	row_fix[4]=64256;
	row_fix[5]=24576;
	row_fix[6]=22016;
	row_fix[7]=10496;
	row_fix[8]=3072;
	row_fix[9]=2304;
	row_fix[10]=57344;
	row_fix[11]=0;
	row_fix[12]=0;
	row_fix[13]=0;
	row_fix[14]=0;
	column_watched_fix[0]=2047;
	column_watched_fix[1]=2036;
	column_watched_fix[2]=2036;
	column_watched_fix[3]=2036;
	column_watched_fix[4]=759;
	column_watched_fix[5]=1015;
	column_watched_fix[6]=927;
	column_watched_fix[7]=799;
	column_watched_fix[8]=0;
	column_watched_fix[9]=0;
	column_watched_fix[10]=0;
	column_watched_fix[11]=0;
	column_watched_fix[12]=0;
	column_watched_fix[13]=0;
	column_watched_fix[14]=0;
	row_watched_fix[0]=34560;
	row_watched_fix[1]=65280;
	row_watched_fix[2]=65280;
	row_watched_fix[3]=65280;
	row_watched_fix[4]=65280;
	row_watched_fix[5]=64512;
	row_watched_fix[6]=65024;
	row_watched_fix[7]=47872;
	row_watched_fix[8]=48896;
	row_watched_fix[9]=65280;
	row_watched_fix[10]=61440;
	row_watched_fix[11]=0;
	row_watched_fix[12]=0;
	row_watched_fix[13]=0;
	row_watched_fix[14]=0;

}

void temp_test_mazedata_3(){
	column_temp[0]=1;
	column_temp[1]=2;
	column_temp[2]=4;
	column_temp[3]=0;
	column_temp[4]=0;
	column_temp[5]=100;
	column_temp[6]=154;
	column_temp[7]=116;
	column_temp[8]=4;
	column_temp[9]=0;
	column_temp[10]=14;
	column_temp[11]=0;
	column_temp[12]=0;
	column_temp[13]=0;
	column_temp[14]=0;
	row_temp[0]=2624;
	row_temp[1]=32832;
	row_temp[2]=25536;
	row_temp[3]=96;
	row_temp[4]=0;
	row_temp[5]=256;
	row_temp[6]=0;
	row_temp[7]=768;
	row_temp[8]=0;
	row_temp[9]=0;
	row_temp[10]=0;
	row_temp[11]=0;
	row_temp[12]=0;
	row_temp[13]=0;
	row_temp[14]=0;
	column_watched_temp[0]=7;
	column_watched_temp[1]=6;
	column_watched_temp[2]=6;
	column_watched_temp[3]=2;
	column_watched_temp[4]=2;
	column_watched_temp[5]=230;
	column_watched_temp[6]=254;
	column_watched_temp[7]=126;
	column_watched_temp[8]=14;
	column_watched_temp[9]=14;
	column_watched_temp[10]=14;
	column_watched_temp[11]=0;
	column_watched_temp[12]=0;
	column_watched_temp[13]=0;
	column_watched_temp[14]=0;
	row_watched_temp[0]=65504;
	row_watched_temp[1]=65504;
	row_watched_temp[2]=25568;
	row_watched_temp[3]=480;
	row_watched_temp[4]=768;
	row_watched_temp[5]=768;
	row_watched_temp[6]=768;
	row_watched_temp[7]=512;
	row_watched_temp[8]=0;
	row_watched_temp[9]=0;
	row_watched_temp[10]=0;
	row_watched_temp[11]=0;
	row_watched_temp[12]=0;
	row_watched_temp[13]=0;
	row_watched_temp[14]=0;
}

void temp_test_mazedata_4(){
	column_temp[0]=233;
	column_temp[1]=724;
	column_temp[2]=474;
	column_temp[3]=564;
	column_temp[4]=18;
	column_temp[5]=138;
	column_temp[6]=10;
	column_temp[7]=271;
	column_temp[8]=0;
	column_temp[9]=0;
	column_temp[10]=0;
	column_temp[11]=0;
	column_temp[12]=0;
	column_temp[13]=0;
	column_temp[14]=0;
	row_temp[0]=22528;
	row_temp[1]=41984;
	row_temp[2]=21504;
	row_temp[3]=34560;
	row_temp[4]=26624;
	row_temp[5]=0;
	row_temp[6]=3072;
	row_temp[7]=14848;
	row_temp[8]=58112;
	row_temp[9]=12288;
	row_temp[10]=0;
	row_temp[11]=0;
	row_temp[12]=0;
	row_temp[13]=0;
	row_temp[14]=0;

}

void mazedata_1112(){
	column_fix[0]=16383;
	column_fix[1]=28612;
	column_fix[2]=44010;
	column_fix[3]=28628;
	column_fix[4]=34987;
	column_fix[5]=17349;
	column_fix[6]=63487;
	column_fix[7]=0;
	column_fix[8]=0;
	column_fix[9]=0;
	column_fix[10]=0;
	column_fix[11]=0;
	column_fix[12]=0;
	column_fix[13]=0;
	column_fix[14]=0;
	row_fix[0]=4096;
	row_fix[1]=26624;
	row_fix[2]=5120;
	row_fix[3]=11264;
	row_fix[4]=31232;
	row_fix[5]=1024;
	row_fix[6]=0;
	row_fix[7]=2048;
	row_fix[8]=1024;
	row_fix[9]=2048;
	row_fix[10]=13312;
	row_fix[11]=512;
	row_fix[12]=1024;
	row_fix[13]=31744;
	row_fix[14]=16384;
	column_watched_fix[0]=65535;
	column_watched_fix[1]=65533;
	column_watched_fix[2]=65535;
	column_watched_fix[3]=65535;
	column_watched_fix[4]=57343;
	column_watched_fix[5]=63487;
	column_watched_fix[6]=63487;
	column_watched_fix[7]=0;
	column_watched_fix[8]=0;
	column_watched_fix[9]=0;
	column_watched_fix[10]=0;
	column_watched_fix[11]=0;
	column_watched_fix[12]=0;
	column_watched_fix[13]=0;
	column_watched_fix[14]=0;
	row_watched_fix[0]=48640;
	row_watched_fix[1]=65024;
	row_watched_fix[2]=65024;
	row_watched_fix[3]=65024;
	row_watched_fix[4]=65024;
	row_watched_fix[5]=65024;
	row_watched_fix[6]=65024;
	row_watched_fix[7]=65024;
	row_watched_fix[8]=65024;
	row_watched_fix[9]=65024;
	row_watched_fix[10]=62976;
	row_watched_fix[11]=65024;
	row_watched_fix[12]=65024;
	row_watched_fix[13]=65024;
	row_watched_fix[14]=65024;

	Pass_Goal_x = 1;
	Pass_Goal_y = 0;
}

void LED_display(){
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
}

void LED_motion1(){
	volatile char ii;
	for(ii=0;ii<2;ii++){
		wait(100);
		LED_V1 = 1;		//LED_V1
		wait(100);
		LED_V2 = 1;	//LED_V2
		wait(100);
		LED_V3 = 1;	//LED_V3
		wait(100);
		LED_V4 = 1;

		wait(200);
		LED_V1 = 0;
		LED_V2 = 0;
		LED_V3 = 0;
		LED_V4 = 0;
	}
}
void LED_motion2(){
	volatile char ii;
	for(ii=0;ii<1;ii++){
		wait(80);
		LED_V1 = 1;		//LED_V1
		wait(80);
		LED_V2 = 1;	//LED_V2
		wait(80);
		LED_V3 = 1;	//LED_V3
		wait(80);
		LED_V4 = 1;

		wait(100);
		LED_V1 = 0;
		LED_V2 = 0;
		LED_V3 = 0;
		LED_V4 = 0;
	}
}

void LED_motion3(){
	volatile char ii;
	for(ii=0;ii<1;ii++){
		wait(80);
		LED_V4 = 1;		//LED_V1
		wait(80);
		LED2 = 1;	//LED_V2

		wait(100);
		LED_V4 = 0;
		LED2 = 0;

	}
}

