/*
 * daikei.c
 *
 *  Created on: 2017/10/09
 *      Author: Alex
 */
#include "iodefine.h"
#include "BC_daikei.h"
#include "BC_define.h"
#include "math.h"

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


