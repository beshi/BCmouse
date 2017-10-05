/*
 * BC_subroutine.c
 *
 *  Created on: 2017/05/05
 *      Author: Alex
 */
#include "iodefine.h"
#include "BC_subroutine.h"
#include "BC_define.h"


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
		Wall_Judge = row_temp[hikisuu_y] & (0x8000 >> hikisuu_x);
		if (Wall_Judge == (0x8000 >> hikisuu_x)) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 1:	//East
		Wall_Judge = column_temp[hikisuu_x] & 0x0001 << hikisuu_y;
		if (Wall_Judge == 0x0001 << hikisuu_y) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 2:	//South
		Wall_Judge = row_temp[hikisuu_y - 1] & 0x8000 >> hikisuu_x;
		if (Wall_Judge == 0x8000 >> hikisuu_x) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 3:	//West
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
		Wall_Judge = row_temp[hikisuu_y] & (0x8000 >> hikisuu_x);
		if (Wall_Judge == (0x8000 >> hikisuu_x)) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 1:	//East
		Wall_Judge = column_temp[hikisuu_x] & 0x0001 << hikisuu_y;
		if (Wall_Judge == 0x0001 << hikisuu_y) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 2:	//South
		Wall_Judge = row_temp[hikisuu_y - 1] & 0x8000 >> hikisuu_x;
		if (Wall_Judge == 0x8000 >> hikisuu_x) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	case 3:	//West
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
void add_saved_wall_right(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {	//パスを生成するために必要20171002
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
void add_saved_wall_left(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {	//パスを生成するために必要20171002
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
	column_temp[15]=65535;
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
	row_temp[15]=32009;

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
	column_temp[15]=65535;
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
	row_temp[15]=32009;
}

