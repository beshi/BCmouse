/*
 * BC_pass_old.c
 *
 *  Created on: 2017/10/21
 *      Author: Alex
 */
#include "iodefine.h"
#include "BC_define.h"
#include "BC_pass_old.h"
#include "BC_subroutine.h"


void Add_UnknownWall_Front(int hikisuu_x, int hikisuu_y, int hikisuu_direction_count) {	//パスを生成するために必要20171002
	switch (hikisuu_direction_count) {
	case 0:		//North
		if (hikisuu_y == 15) {
			break;
		}
		unknown_wall_row[hikisuu_y] |= 32768 >> hikisuu_x;
		break;
	case 1:		//East
		if (hikisuu_x == 15) {
			break;
		}
		unknown_wall_column[hikisuu_x] |= 1 << hikisuu_y;
		break;
	case 2:		//South
		if (hikisuu_y == 0) {
			break;
		}
		unknown_wall_row[hikisuu_y - 1] |= 32768 >> hikisuu_x;
		break;
	case 3:		//West
		if (hikisuu_x == 0) {
			break;
		}
		unknown_wall_column[hikisuu_x - 1] |= 1 << hikisuu_y;
		break;
	}
}

void Reset_Unknown_Wall(){
	volatile unsigned char jj;

	for(jj=0;jj<15;jj++){
		unknown_wall_row[jj]=0;
		unknown_wall_column[jj]=0;
	}

}

void Set_Temp_Goal(){
	volatile unsigned char px,py,jj;

	for(py=0;py<15;py++){
		for(px=0;px<=15;px++){
			if(is_Exist_Unknown_Wall(px, py, 0) == 1){
				GoalPosition_assign(px, py);
				GoalPosition_assign(px, py + 1);
			}
		}
	}
	for(py=0;py<=15;py++){
		for(px=0;px<15;px++){
			if (is_Exist_Unknown_Wall(px, py, 1) == 1){
				GoalPosition_assign(px, py);
				GoalPosition_assign(px + 1, py);
			}
		}
	}

}

void Reset_Temp_Goal(){
	volatile unsigned char px,py;

	for(py=0;py<=15;py++){
		for(px=0;px<=15;px++){
			GoalPosition_remove(px,py);
		}
	}
}

char is_Exist_Unknown_Wall(char hikisuu_x, char hikisuu_y, char hikisuu_direction_count) {
	volatile unsigned short Wall_Judge = 0;
	switch (hikisuu_direction_count) {
	case 0:	//North
		if(hikisuu_y==y_size){
			break;
		}
		Wall_Judge = unknown_wall_row[hikisuu_y] & (0x8000 >> hikisuu_x);
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
		Wall_Judge = unknown_wall_column[hikisuu_x] & 0x0001 << hikisuu_y;
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
		Wall_Judge = unknown_wall_row[hikisuu_y - 1] & 0x8000 >> hikisuu_x;
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
		Wall_Judge = unknown_wall_column[hikisuu_x - 1] & 0x0001 << hikisuu_y;
		if (Wall_Judge == 0x0001 << hikisuu_y) {
			Wall_Judge = 1;
		} else {
			Wall_Judge = 0;
		}
		break;
	}
	return Wall_Judge;
}


void make_pass(int hikisuu_goal_x, int hikisuu_goal_y) {		//パスの作成（簡易版）
	volatile int p_i;
	volatile char ii;
	for (ii = 0; ii < 200; ii++) {	//初期化してから使おうね！
		pass[ii] = 0;
	}

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

void convert_pass() {
	volatile signed short read_p_i = 0, motion_count = 0, adjust_straight = 0, thin = 0, j=0;/*motion_count = 1, //20171002に変更*/
	volatile char ii;
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

void exe_kamawari_pass_test(float hikisuu_vmax, float hikisuu_accel, char para_mode) {
	volatile char aa, bb, cc;	//パラメータ選択  aa:小回りの速度  bb:大回り、Uターンの速度  cc:
	volatile int read_p_i;
	volatile float vel_low, vel_high, accel_normal;
	read_p_i = 0;

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
		accel_normal = 5000.0;
		aa = 1;	//速度650
		bb = 3;	//速度800
		cc = 1;	//斜め速度650

	} else if (para_mode == 3) {
		vel_low = 650.0;
		vel_high = 650.0;
		accel_normal = 5000.0;
		aa = 2;	//速度750
		bb = 1;	//速度650
		cc = 1;	//斜め速度650

	} else if (para_mode == 4) {
		vel_low = 750.0;
		vel_high = 800.0;
		accel_normal = 6000.0;
		aa = 2;	//速度750
		bb = 3;	//速度900
		cc = 1;	//斜め速度650

	}
	//以下、最初の動作のみ別枠で行う。
	if (pass[read_p_i] == 255) {	//斜め無し・開幕ターン無し  学生大会で直した
		daikei_for_pass_EX(90.0 + 47.0, vel_high, accel_normal, 0.0, vel_high, 1, 0);
	}
	//以下、最初以外の動作
	while (1) {
		if (fail_flag == 1) {	//failセーフ
			GPT.GTSTR.BIT.CST0 = 0;		//failセーフ
			break;	//daikeiの関数を抜けられる
		}

		read_p_i++;
		if (pass[read_p_i] <= 30 && pass[read_p_i] >= 1) {	//数値の区間の半分直進
			daikei_for_pass_EX(90.0 * pass[read_p_i], hikisuu_vmax,
					hikisuu_accel, vel_high, vel_high, 1, 1);
		} else {

			switch ((char) pass[read_p_i]) {
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
			case 100:   //停止(90区間で)(斜め無しpass用)
				daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_high,
						0.0, 1, 0);
//				myprintf("test100\r\n");
				break;
			}
		}
		if (read_p_i == last_p_i) {

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

/*void temp_exe_pass_EX(float hikisuu_vmax, float hikisuu_accel, char hikisuu_mode) {	//斜め直線での加速アリバージョン
	read_P_I = 0;

	assign_parameters(hikisuu_mode);	//走行モードの決定

	//以下、最初の動作のみ別枠で行う。(尻当てによりスタート！)
	if (motion[read_P_I] >= 1 && motion[read_P_I] <= 30) {
		daikei_for_pass_EX(90.0 * motion[read_P_I] + 47.0, vel_high, hikisuu_accel, 0.0,
				vel_high, 1, 0);
	} else if (motion[read_P_I] == 74) {	//開幕右小回り→1区間加速
		daikei_for_pass_EX(90.0 + 47.0, vel_low, accel_normal, 0.0, vel_low, 1, 0);
		slalom_2(90.0, 22.5, 67.5, 1000.0, 5500, -1.0, 500.0, 13.0, 0.0);//右小回り(後距離0)  ????
		daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_low, vel_high, 1,
				0);	//加速
	} else if (motion[read_P_I] == 84) {	//開幕右45°ターン→斜め(初期速度は0.0)
		daikei_for_pass_EX(turn[cc].P_1_3.d_f + 47.0, turn[cc].P_1_3.vel, accel_normal, 0.0, turn[cc].P_1_3.vel, 0, 0);
		turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
				turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
				turn[cc].P_1_3.wise, turn[cc].P_1_3.vel,
				turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
				0.0, turn[cc].P_1_3.d_r);	//右45°ターン→斜め

	} else if (motion[read_P_I] == 94) {	//開幕右135°ターン→斜め(初期速度は0.0)
		daikei_for_pass_EX(turn[cc].P_1_4.d_f + 47.0 , turn[cc].P_1_4.vel, accel_normal, 0.0, turn[cc].P_1_4.vel, 0, 0);
		turn_for_skew_pass(turn[cc].P_1_4.theta, turn[cc].P_1_4.th1,
				turn[cc].P_1_4.th2, 1000.0, turn[cc].P_1_4.a_cc,
				turn[cc].P_1_4.wise, turn[cc].P_1_4.vel,
				turn[cc].P_1_4.vel, turn[cc].P_1_4.vel,
				 0.0, turn[cc].P_1_4.d_r);	//右135°ターン→斜め

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
		if(read_P_I==2){		//debug
			sample_flag=1;		//debug
		}						//debug

		if ((motion[read_P_I + 1] == 114) || (motion[read_P_I + 1] == 124)
				|| (motion[read_P_I + 1] == 134)
				|| (motion[read_P_I + 1] == 144)
				|| motion[read_P_I + 1] == 184) {	//壁切れを読み、次が右ターンの場合
			flags_kabekire.next_r=1;
			flags_kabekire.next_l=0;
			flags_kabekire.wait=1;
		} else if ((motion[read_P_I + 1] == 115)
				|| (motion[read_P_I + 1] == 125)
				|| (motion[read_P_I + 1] == 135)
				|| (motion[read_P_I + 1] == 145)
				|| motion[read_P_I + 1] == 185) {	//壁切れを読み、次が左ターンの場合
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
			daikei_for_pass_EX(127.3 * (motion[read_P_I] - 200.0), 1200.0,
					hikisuu_accel, 650.0, 650.0, 0, 0);
		}
		else {

			switch ((char) motion[read_P_I]) {
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
				turn_for_skew_pass(turn[cc].P_1_3.theta, turn[cc].P_1_3.th1,
						turn[cc].P_1_3.th2, 1000.0, turn[cc].P_1_3.a_cc,
						turn[cc].P_1_3.wise, turn[bb].P_1_3.vel,
						turn[cc].P_1_3.vel, turn[cc].P_1_3.vel,
						turn[cc].P_1_3.d_f + adjust_before_dist,
						turn[cc].P_1_3.d_r);	//右45°ターン→斜め
				break;
			case 135:	//左45°ターン→斜め
				turn_for_skew_pass(turn[cc].P_1_11.theta, turn[cc].P_1_11.th1,
						turn[cc].P_1_11.th2, 1000.0, turn[cc].P_1_11.a_cc,
						turn[cc].P_1_11.wise, turn[bb].P_1_11.vel,
						turn[cc].P_1_11.vel, turn[cc].P_1_11.vel,
						turn[cc].P_1_11.d_f + adjust_before_dist,
						turn[cc].P_1_11.d_r);	//左45°ターン→斜め
				break;
			case 144:	//右135°ターン→斜め
				turn_for_skew_pass(turn[cc].P_1_4.theta, turn[cc].P_1_4.th1,
						turn[cc].P_1_4.th2, 1000.0, turn[cc].P_1_4.a_cc,
						turn[cc].P_1_4.wise, turn[bb].P_1_4.vel,
						turn[cc].P_1_4.vel, turn[cc].P_1_4.vel,
						turn[cc].P_1_4.d_f + adjust_before_dist,
						turn[cc].P_1_4.d_r);	//右135°ターン→斜め
				break;
			case 145:	//左135°ターン→斜め
				turn_for_skew_pass(turn[cc].P_1_12.theta, turn[cc].P_1_12.th1,
						turn[cc].P_1_12.th2, 1000.0, turn[cc].P_1_12.a_cc,
						turn[cc].P_1_12.wise, turn[bb].P_1_12.vel,
						turn[cc].P_1_12.vel, turn[cc].P_1_12.vel,
						turn[cc].P_1_12.d_f + adjust_before_dist,
						turn[cc].P_1_12.d_r);	//左135°ターン→斜め
				break;
			case 154:	//斜め→右45°ターン
				turn_for_skew_pass(turn[cc].P_1_5.theta, turn[cc].P_1_5.th1,
						turn[cc].P_1_5.th2, 1000.0, turn[cc].P_1_5.a_cc,
						turn[cc].P_1_5.wise, turn[cc].P_1_5.vel,
						turn[cc].P_1_5.vel, turn[bb].P_1_5.vel,
						turn[cc].P_1_5.d_f + adjust_before_dist,
						turn[cc].P_1_5.d_r);	//斜め→右45度ターン

				break;
			case 155:	//斜め→左45°ターン
				turn_for_skew_pass(turn[cc].P_1_13.theta, turn[cc].P_1_13.th1,
						turn[cc].P_1_13.th2, 1000.0, turn[cc].P_1_13.a_cc,
						turn[cc].P_1_13.wise, turn[cc].P_1_13.vel,
						turn[cc].P_1_13.vel, turn[bb].P_1_13.vel,
						turn[cc].P_1_13.d_f + adjust_before_dist,
						turn[cc].P_1_13.d_r);	//斜め→左45度ターン
				break;
			case 164:	//斜め→右135°ターン
				turn_for_skew_pass(turn[cc].P_1_6.theta, turn[cc].P_1_6.th1,
						turn[cc].P_1_6.th2, 1000.0, turn[cc].P_1_6.a_cc,
						turn[cc].P_1_6.wise, turn[cc].P_1_6.vel,
						turn[cc].P_1_6.vel, turn[bb].P_1_6.vel,
						turn[cc].P_1_6.d_f + adjust_before_dist,
						turn[cc].P_1_6.d_r);	//斜め→右135度ターン
				break;
			case 165:	//斜め→左135°ターン
				turn_for_skew_pass(turn[cc].P_1_14.theta, turn[cc].P_1_14.th1,
						turn[cc].P_1_14.th2, 1000.0, turn[cc].P_1_14.a_cc,
						turn[cc].P_1_14.wise, turn[cc].P_1_14.vel,
						turn[cc].P_1_14.vel, turn[bb].P_1_14.vel,
						turn[cc].P_1_14.d_f + adjust_before_dist,
						turn[cc].P_1_14.d_r);	//斜め→左135度ターン
				break;
			case 174:	//斜め→右V90°ターン
				turn_for_skew_pass(turn[cc].P_1_7.theta, turn[cc].P_1_7.th1,
						turn[cc].P_1_7.th2, 1000.0, turn[cc].P_1_7.a_cc,
						turn[cc].P_1_7.wise, turn[cc].P_1_7.vel,
						turn[cc].P_1_7.vel, turn[cc].P_1_7.vel,
						turn[cc].P_1_7.d_f + adjust_before_dist,
						turn[cc].P_1_7.d_r);	//斜め→右V90度ターン
				break;
			case 175:	//斜め→左V90°ターン
				turn_for_skew_pass(turn[cc].P_1_15.theta, turn[cc].P_1_15.th1,
						turn[cc].P_1_15.th2, 1000.0, turn[cc].P_1_15.a_cc,
						turn[cc].P_1_15.wise, turn[cc].P_1_15.vel,
						turn[cc].P_1_15.vel, turn[cc].P_1_15.vel,
						turn[cc].P_1_15.d_f + adjust_before_dist,
						turn[cc].P_1_15.d_r);	//斜め→左V90度ターン
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
						turn[bb].P_1_1.d_f + adjust_before_dist,
						turn[bb].P_1_1.d_r);	//右大廻ターン
				daikei_for_pass_EX(90.0, turn[bb].P_1_1.vel, accel_normal, turn[bb].P_1_1.vel,
						0.0, 1, 0);

				break;
			case 233:	//左大廻ターン+停止
				turn_for_pass(turn[bb].P_1_9.theta, turn[bb].P_1_9.th1,
						turn[bb].P_1_9.th2, 1000.0, turn[bb].P_1_9.a_cc,
						turn[bb].P_1_9.wise, turn[bb].P_1_9.vel,
						turn[bb].P_1_9.d_f + adjust_before_dist,
						turn[bb].P_1_9.d_r);	//左大廻ターン
				daikei_for_pass_EX(90.0, turn[bb].P_1_9.vel, accel_normal, turn[bb].P_1_9.vel,
						0.0, 1, 0);

				break;
			case 234:	//右Uターン+停止
				turn_for_pass(turn[bb].P_1_2.theta, turn[bb].P_1_2.th1,
						turn[bb].P_1_2.th2, 1000.0, turn[bb].P_1_2.a_cc,
						turn[bb].P_1_2.wise, turn[bb].P_1_2.vel,
						turn[bb].P_1_2.d_f + adjust_before_dist,
						turn[bb].P_1_2.d_r);	//右Uターン
				daikei_for_pass_EX(90.0, turn[bb].P_1_2.vel, accel_normal, turn[bb].P_1_2.vel,
						0.0, 1, 0);

				break;
			case 235:	//左Uターン+停止
				turn_for_pass(turn[bb].P_1_10.theta, turn[bb].P_1_10.th1,
						turn[bb].P_1_10.th2, 1000.0, turn[bb].P_1_10.a_cc,
						turn[bb].P_1_10.wise, turn[bb].P_1_10.vel,
						turn[bb].P_1_10.d_f + adjust_before_dist,
						turn[bb].P_1_10.d_r);	//右Uターン
				daikei_for_pass_EX(90.0, turn[bb].P_1_10.vel, accel_normal, turn[bb].P_1_10.vel,
						0.0, 1, 0);

				break;
			case 236:	//斜め→右45°ターン+停止
				turn_for_skew_pass(turn[cc].P_1_5.theta, turn[cc].P_1_5.th1,
						turn[cc].P_1_5.th2, 1000.0, turn[cc].P_1_5.a_cc,
						turn[cc].P_1_5.wise, turn[cc].P_1_5.vel,
						turn[cc].P_1_5.vel, turn[bb].P_1_5.vel,
						turn[cc].P_1_5.d_f + adjust_before_dist,
						turn[cc].P_1_5.d_r);	//斜め→右45度ターン
				daikei_for_pass_EX(90.0, turn[cc].P_1_5.vel, accel_normal, turn[cc].P_1_5.vel,
						0.0, 1, 0);

				break;
			case 237:	//斜め→左45°ターン+停止
				turn_for_skew_pass(turn[cc].P_1_13.theta, turn[cc].P_1_13.th1,
						turn[cc].P_1_13.th2, 1000.0, turn[cc].P_1_13.a_cc,
						turn[cc].P_1_13.wise, turn[cc].P_1_13.vel,
						turn[cc].P_1_13.vel, turn[bb].P_1_13.vel,
						turn[cc].P_1_13.d_f + adjust_before_dist,
						turn[cc].P_1_13.d_r);	//斜め→左45度ターン
				daikei_for_pass_EX(90.0, turn[cc].P_1_13.vel, accel_normal, turn[cc].P_1_13.vel,
						0.0, 1, 0);

				break;
			case 238:	//斜め→右135°ターン+停止
				turn_for_skew_pass(turn[cc].P_1_6.theta, turn[cc].P_1_6.th1,
						turn[cc].P_1_6.th2, 1000.0, turn[cc].P_1_6.a_cc,
						turn[cc].P_1_6.wise, turn[cc].P_1_6.vel,
						turn[cc].P_1_6.vel, turn[bb].P_1_6.vel,
						turn[cc].P_1_6.d_f + adjust_before_dist,
						turn[cc].P_1_6.d_r);	//斜め→右135度ターン
				daikei_for_pass_EX(90.0, turn[cc].P_1_6.vel, accel_normal, turn[cc].P_1_6.vel,
						0.0, 1, 0);

				break;
			case 239:	//斜め→左135°ターン+停
				turn_for_skew_pass(turn[cc].P_1_14.theta, turn[cc].P_1_14.th1,
						turn[cc].P_1_14.th2, 1000.0, turn[cc].P_1_14.a_cc,
						turn[cc].P_1_14.wise, turn[cc].P_1_14.vel,
						turn[cc].P_1_14.vel, turn[bb].P_1_14.vel,
						turn[cc].P_1_14.d_f + adjust_before_dist,
						turn[cc].P_1_14.d_r);	//斜め→左135度ターン
				daikei_for_pass_EX(90.0, turn[cc].P_1_14.vel, accel_normal, turn[cc].P_1_14.vel,
						0.0, 1, 0);

				break;
			case 240:   //停止(90区間で最大限減速する)
				daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_high,
						0.0, 1, 0);

				break;
			case 100:   //停止(90区間で)(斜め無しpass用)
				daikei_for_pass_EX(90.0, vel_high, accel_normal, vel_high,
						0.0, 1, 0);
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
}*/




