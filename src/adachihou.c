/*
 * adachihou.c
 *
 *  Created on: 2016/02/20
 *      Author: Alex
 */

#include "iodefine.h"
#include "adachihou.h"
#include "hensuu&define.h"	//defineのために入れる

//void distance3(float hikisuu_dist, float vmax, float hikisuu_accel, float v_0,
//		float vterm, int kabeseigyo);
//void wait(int hikisuu);
//void turn_unclock3(float hikisuu_angle, float vmax2, float hikisuu_accel2,
//		float v_0_2, float vterm2);
//void turn_clock3(float hikisuu_angle, float vmax2, float hikisuu_accel2,
//		float v_0_2, float vterm2);

void adachihou_q2_kichikukan(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search, float hikisuu_angacc,
		float hikisuu_angle1_r, float hikisuu_angle2_r, float hikisuu_angle1_l,
		float hikisuu_angle2_l, float offset_r1, float offset_r2,
		float offset_l1, float offset_l2) {    //正常な足立法。
	colum2[0] |= 1;

//	colum[0] = 1;	←これやばいやつ！
//	direction_count = 0;
	footmark[0][0] = 1;

	x = start_x;
	y = start_y;
	direction_xy();		//スタート方向に初期値＋1する.これでスタート座標を直接代入できる
//	q_walk_map2_maker(hikisuu_goal_x, hikisuu_goal_y);
	motor_enable = 1;
	wait(300); 			//励磁直後は少し待つ！
	PE.DRL.BIT.B0 = 0;	//B0=0で正回転
	PE.DRL.BIT.B4 = 1;	//B4=1で正回転
	distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
	setReached(x, y);
	watched_wall_front_2(0, 0, 0);		//初期に読めない壁だが、読んだことにする
	watched_wall_right_2(0, 0, 0);		//初期に読めない壁だが、読んだことにする

	while (1) {
		if (map2[x][y] == 255) {		//閉じ込められたら自動で抜け出す。
			accident_flag = 1;
		}

		total_dist = 0.0;	//スラローム関数の前にはこれが必要

		if (SEN_r_front_value > 850 && SEN_l_front_value > 850
				&& clash_count >= 1) {		//ぶつかり続けるとカウント追加
			clash_count++;
			if (clash_count >= 3) {
				accident_flag = 1;
			}
		} else {
			clash_count = 0;
		}
		if (accident_flag == 1) {
			break;
		}
		if (SEN_r_front_value >= r_front_wall_judge) {	//前センサーの壁判断
			add_wall_front_2(x, y, direction_count);
		}
		if (SEN_r_value >= r_wall_judge) {	//右センサーの壁判断
			add_wall_right_2(x, y, direction_count);
		}
		if (SEN_l_value >= l_wall_judge) {	//左センサーの壁判断
			add_wall_left_2(x, y, direction_count);
		}
		watched_wall_front_2(x, y, direction_count);
		watched_wall_right_2(x, y, direction_count);
		watched_wall_left_2(x, y, direction_count);

		if (x == goal_x && y == goal_y) {		//以下、クシつぶし

		} else if (x == goal_x + 1 && y == goal_y) {

		} else if (x == goal_x + 1 && y == goal_y + 1) {

		} else if (x == goal_x && y == goal_y + 1) {
		} else {

			if (x > 0 && y < 15 && is_the_Wall_watched_2(x, y, 3) == 1	//左上横壁追加
			&& is_the_Wall_watched_2(x, y + 1, 3) == 1
					&& is_the_Wall_watched_2(x, y, 0) == 1
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_Exist_Wall_2(x, y + 1, 3) == 0
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_the_Wall_watched_2(x - 1, y, 0) == 0) {
				add_wall_front_2(x - 1, y, 0);
				watched_wall_front_2(x - 1, y, 0);
			}
			if (x < 15 && y < 15 && is_the_Wall_watched_2(x, y, 1) == 1	//右上横壁追加
			&& is_the_Wall_watched_2(x, y + 1, 1) == 1
					&& is_the_Wall_watched_2(x, y, 0) == 1
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_Exist_Wall_2(x, y + 1, 1) == 0
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_the_Wall_watched_2(x + 1, y, 0) == 0) {
				add_wall_front_2(x + 1, y, 0);
				watched_wall_front_2(x + 1, y, 0);
			}
			if (x < 15 && y > 0 && is_the_Wall_watched_2(x, y, 1) == 1	//右下横壁追加
			&& is_the_Wall_watched_2(x, y - 1, 1) == 1
					&& is_the_Wall_watched_2(x, y, 2) == 1
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_Exist_Wall_2(x, y - 1, 1) == 0
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_the_Wall_watched_2(x + 1, y - 1, 0) == 0) {
				add_wall_front_2(x + 1, y - 1, 0);
				watched_wall_front_2(x + 1, y - 1, 0);
			}
			if (x > 0 && y > 0 && is_the_Wall_watched_2(x, y, 3) == 1	//左下横壁追加
			&& is_the_Wall_watched_2(x, y - 1, 3) == 1
					&& is_the_Wall_watched_2(x, y, 2) == 1
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_Exist_Wall_2(x, y - 1, 3) == 0
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_the_Wall_watched_2(x - 1, y - 1, 0) == 0) {
				add_wall_front_2(x - 1, y - 1, 0);
				watched_wall_front_2(x - 1, y - 1, 0);
			}
			if (x > 0 && y < 15 && is_the_Wall_watched_2(x - 1, y, 0) == 1//左上縦壁追加
			&& is_the_Wall_watched_2(x, y, 0) == 1
					&& is_the_Wall_watched_2(x, y, 3) == 1
					&& is_Exist_Wall_2(x - 1, y, 0) == 0
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_the_Wall_watched_2(x, y + 1, 3) == 0) {
				add_wall_left_2(x, y + 1, 0);
				watched_wall_left_2(x, y + 1, 0);
			}
			if (x < 15 && y < 15 && is_the_Wall_watched_2(x, y, 0) == 1	//右上縦壁追加
			&& is_the_Wall_watched_2(x + 1, y, 0) == 1
					&& is_the_Wall_watched_2(x, y, 1) == 1
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_Exist_Wall_2(x + 1, y, 0) == 0
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_the_Wall_watched_2(x, y + 1, 1) == 0) {
				add_wall_right_2(x, y + 1, 0);
				watched_wall_right_2(x, y + 1, 0);
			}
			if (x < 15 && y > 0 && is_the_Wall_watched_2(x, y, 2) == 1	//右下縦壁追加
			&& is_the_Wall_watched_2(x + 1, y, 2) == 1
					&& is_the_Wall_watched_2(x, y, 1) == 1
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_Exist_Wall_2(x + 1, y, 2) == 0
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_the_Wall_watched_2(x, y - 1, 1) == 0) {
				add_wall_right_2(x, y - 1, 0);
				watched_wall_right_2(x, y - 1, 0);
			}
			if (x > 0 && y > 0 && is_the_Wall_watched_2(x - 1, y, 2) == 1//左下縦壁追加
			&& is_the_Wall_watched_2(x, y, 2) == 1
					&& is_the_Wall_watched_2(x, y, 3) == 1
					&& is_Exist_Wall_2(x - 1, y, 2) == 0
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_the_Wall_watched_2(x, y - 1, 3) == 0) {
				add_wall_left_2(x, y - 1, 0);
				watched_wall_left_2(x, y - 1, 0);
			}
		}
//		LED2_y = 1;
		q_walk_map2_maker(hikisuu_goal_x, hikisuu_goal_y);
//		LED2_y = 0;
		switch (direction_count) {
		case 0:
			if (y < 15 && map2[x][y + 1] == map2[x][y] - 1
					&& footmark[x][y + 1] == 1) {	//まず、既知区間かどうかの条件
				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y < 15
							&& map2[x][y + straight_count]
									== map2[x][y] - straight_count
							&& footmark[x][y + straight_count] == 1  && is_Exist_Wall_2(x, y + straight_count, 2) == 0) {
					} else {
						break;
					}
				}
				distance4(180 * (straight_count - 1), V_search + 120,
						ACCEL_search, V_search, V_search, 1);
				for (i = 0; i < (straight_count - 1); i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
			} else if (y < 15&& map2[x][y + 1]
			< map2[x][y]&& SEN_r_front_value < r_front_wall_judge) {//North & 直進
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
				direction_xy();

			} else if (x < 15&& map2[x + 1][y]
			< map2[x][y]&& SEN_r_value < r_wall_judge) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				LED1_p = 1;
				slalom_R2(90, hikisuu_angacc, hikisuu_angle1_r,
						hikisuu_angle2_r, V_search, offset_r1, offset_r2, 1);//右折//(角度、最高角速度、円弧開始角、円弧終了角、重心速度、オフセット、壁制御)
				direction_xy();
				LED1_p = 0;
			} else if (x > 0&& map2[x - 1][y]
			< map2[x][y]&& SEN_l_value < l_wall_judge) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}

				slalom_L2(90, hikisuu_angacc, hikisuu_angle1_l,
						hikisuu_angle2_l, V_search, offset_l1, offset_l2, 1);//左折
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 1;
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;
			}
			break;

		case 1:
			if (y < 15 && map2[x + 1][y] == map2[x][y] - 1
					&& footmark[x + 1][y] == 1) {	//まず、既知区間かどうかの条件
				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y < 15
							&& map2[x + straight_count][y]
									== map2[x][y] - straight_count
							&& footmark[x + straight_count][y] == 1 && is_Exist_Wall_2(x + straight_count, y, 3) == 0) {
					} else {
						break;
					}
				}
				distance4(180 * (straight_count - 1), V_search + 120,
						ACCEL_search, V_search, V_search, 1);
				for (i = 0; i < (straight_count - 1); i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
			} else if (x
					< 15&& map2[x+1][y] < map2[x][y] && SEN_r_front_value < r_front_wall_judge) {//East & 直進
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
				direction_xy();

			} else if (y
					> 0&& map2[x][y-1] < map2[x][y] && SEN_r_value < r_wall_judge) {//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				LED1_p = 1;
				slalom_R2(90, hikisuu_angacc, hikisuu_angle1_r,
						hikisuu_angle2_r, V_search, offset_r1, offset_r2, 1);//右折//(角度、最高角速度、円弧開始角、円弧終了角、重心速度、オフセット、壁制御)
				direction_xy();
				LED1_p = 0;

			} else if (y
					< 15&& map2[x][y+1] < map2[x][y] && SEN_l_value < l_wall_judge) {//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				LED1_p = 1;
				slalom_L2(90, hikisuu_angacc, hikisuu_angle1_l,
						hikisuu_angle2_l, V_search, offset_l1, offset_l2, 1);//左折
				direction_xy();
				LED1_p = 0;
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 1;
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;

			}
			break;

		case 2:
			if (y < 15 && map2[x][y - 1] == map2[x][y] - 1
					&& footmark[x][y - 1] == 1) {	//まず、既知区間かどうかの条件
				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y < 15
							&& map2[x][y - straight_count]
									== map2[x][y] - straight_count
							&& footmark[x][y - straight_count] == 1 && is_Exist_Wall_2(x, y - straight_count, 0) == 0) {
					} else {
						break;
					}
				}
				distance4(180 * (straight_count - 1), V_search + 120,
						ACCEL_search, V_search, V_search, 1);
				for (i = 0; i < (straight_count - 1); i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
			} else if (y
					> 0&& map2[x][y-1] < map2[x][y] && SEN_r_front_value < r_front_wall_judge) {//South & 直進
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
				direction_xy();
			} else if (x
					> 0&& map2[x-1][y] < map2[x][y] && SEN_r_value < r_wall_judge) {//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}

				slalom_R2(90, hikisuu_angacc, hikisuu_angle1_r,
						hikisuu_angle2_r, V_search, offset_r1, offset_r2, 1);//右折//(角度、最高角速度、円弧開始角、円弧終了角、重心速度、オフセット、壁制御)
				direction_xy();

			} else if (x
					< 15&& map2[x+1][y] < map2[x][y] && SEN_l_value < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}

				slalom_L2(90, hikisuu_angacc, hikisuu_angle1_l,
						hikisuu_angle2_l, V_search, offset_l1, offset_l2, 1);//左折
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 1;
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;

			}
			break;

		case 3:
			if (y < 15 && map2[x - 1][y] == map2[x][y] - 1
					&& footmark[x - 1][y] == 1) {	//まず、既知区間かどうかの条件
				while (1) {	//何区画既知区間を直進するか数える
					straight_count++;
					if (y < 15
							&& map2[x - straight_count][y]
									== map2[x][y] - straight_count
							&& footmark[x - straight_count][y] == 1 && is_Exist_Wall_2(x - straight_count, y, 1) == 0) {
					} else {
						break;
					}
				}
				distance4(180 * (straight_count - 1), V_search + 120,
						ACCEL_search, V_search, V_search, 1);
				for (i = 0; i < (straight_count - 1); i++) {	//座標を移動分だけ更新
					direction_xy();
				}
				straight_count = 0;
			} else if (x
					> 0&& map2[x-1][y] < map2[x][y] && SEN_r_front_value < r_front_wall_judge) {//South & 直進
				direction_xy();
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
			} else if (y
					< 15&& map2[x][y+1] < map2[x][y] && SEN_r_value < r_wall_judge) {//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}

				slalom_R2(90, hikisuu_angacc, hikisuu_angle1_r,
						hikisuu_angle2_r, V_search, offset_r1, offset_r2, 1);//右折//(角度、最高角速度、円弧開始角、円弧終了角、重心速度、オフセット、壁制御)
				direction_xy();

			} else if (y
					> 0&& map2[x][y-1] < map2[x][y] && SEN_l_value < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}

				slalom_L2(90, hikisuu_angacc, hikisuu_angle1_l,
						hikisuu_angle2_l, V_search, offset_l1, offset_l2, 1);//左折
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 1;
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				interrupt_kabekire = 2;
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;

			}
			break;

		}
		setReached(x, y);

		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}

	}
	if (accident_flag == 0) {
		distance3(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
		MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
		MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
		wait(300);
		motor_enable = 0;
	} else {
		motor_enable = 0;
	}
}
void adachihou_q2(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search) {		//正常な足立法。
	colum2[0] |= 1;
//	colum[0] = 1;	←これやばいやつ！
//	direction_count = 0;
	footmark[0][0] = 1;

	x = start_x;
	y = start_y;
	motor_enable = 1;
	wait(300); 			//励磁直後は少し待つ！
	PE.DRL.BIT.B0 = 0;	//B0=0で正回転
	PE.DRL.BIT.B4 = 1;	//B4=1で正回転
	distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
	setReached(x, y);
	watched_wall_front_2(0, 0, 0);		//初期に読めない壁だが、読んだことにする
	watched_wall_right_2(0, 0, 0);		//初期に読めない壁だが、読んだことにする

	while (1) {
		if (map2[x][y] == 255) {		//閉じ込められたら自動で抜け出す。
			accident_flag = 1;
		}
		if (SEN_r_front_value > 850 && SEN_l_front_value > 850
				&& clash_count >= 1) {		//ぶつかり続けるとカウント追加
			clash_count++;
			if (clash_count >= 3) {
				accident_flag = 1;
			}
		} else {
			clash_count = 0;
		}
		if (accident_flag == 1) {
			break;
		}
		if (SEN_r_front_value >= r_front_wall_judge) {	//前センサーの壁判断
			add_wall_front_2(x, y, direction_count);
		}
		if (SEN_r_value >= r_wall_judge) {	//右センサーの壁判断
			add_wall_right_2(x, y, direction_count);
		}
		if (SEN_l_value >= l_wall_judge) {	//左センサーの壁判断
			add_wall_left_2(x, y, direction_count);
		}
		watched_wall_front_2(x, y, direction_count);
		watched_wall_right_2(x, y, direction_count);
		watched_wall_left_2(x, y, direction_count);

		if (x == goal_x && y == goal_y) {		//以下、クシつぶし

		} else if (x == goal_x + 1 && y == goal_y) {

		} else if (x == goal_x + 1 && y == goal_y + 1) {

		} else if (x == goal_x && y == goal_y + 1) {
		} else {

			if (x > 0 && y < 15 && is_the_Wall_watched_2(x, y, 3) == 1	//左上横壁追加
			&& is_the_Wall_watched_2(x, y + 1, 3) == 1
					&& is_the_Wall_watched_2(x, y, 0) == 1
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_Exist_Wall_2(x, y + 1, 3) == 0
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_the_Wall_watched_2(x - 1, y, 0) == 0) {
				add_wall_front_2(x - 1, y, 0);
				watched_wall_front_2(x - 1, y, 0);
			}
			if (x < 15 && y < 15 && is_the_Wall_watched_2(x, y, 1) == 1	//右上横壁追加
			&& is_the_Wall_watched_2(x, y + 1, 1) == 1
					&& is_the_Wall_watched_2(x, y, 0) == 1
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_Exist_Wall_2(x, y + 1, 1) == 0
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_the_Wall_watched_2(x + 1, y, 0) == 0) {
				add_wall_front_2(x + 1, y, 0);
				watched_wall_front_2(x + 1, y, 0);
			}
			if (x < 15 && y > 0 && is_the_Wall_watched_2(x, y, 1) == 1	//右下横壁追加
			&& is_the_Wall_watched_2(x, y - 1, 1) == 1
					&& is_the_Wall_watched_2(x, y, 2) == 1
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_Exist_Wall_2(x, y - 1, 1) == 0
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_the_Wall_watched_2(x + 1, y - 1, 0) == 0) {
				add_wall_front_2(x + 1, y - 1, 0);
				watched_wall_front_2(x + 1, y - 1, 0);
			}
			if (x > 0 && y > 0 && is_the_Wall_watched_2(x, y, 3) == 1	//左下横壁追加
			&& is_the_Wall_watched_2(x, y - 1, 3) == 1
					&& is_the_Wall_watched_2(x, y, 2) == 1
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_Exist_Wall_2(x, y - 1, 3) == 0
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_the_Wall_watched_2(x - 1, y - 1, 0) == 0) {
				add_wall_front_2(x - 1, y - 1, 0);
				watched_wall_front_2(x - 1, y - 1, 0);
			}
			if (x > 0 && y < 15 && is_the_Wall_watched_2(x - 1, y, 0) == 1//左上縦壁追加
			&& is_the_Wall_watched_2(x, y, 0) == 1
					&& is_the_Wall_watched_2(x, y, 3) == 1
					&& is_Exist_Wall_2(x - 1, y, 0) == 0
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_the_Wall_watched_2(x, y + 1, 3) == 0) {
				add_wall_left_2(x, y + 1, 0);
				watched_wall_left_2(x, y + 1, 0);
			}
			if (x < 15 && y < 15 && is_the_Wall_watched_2(x, y, 0) == 1	//右上縦壁追加
			&& is_the_Wall_watched_2(x + 1, y, 0) == 1
					&& is_the_Wall_watched_2(x, y, 1) == 1
					&& is_Exist_Wall_2(x, y, 0) == 0
					&& is_Exist_Wall_2(x + 1, y, 0) == 0
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_the_Wall_watched_2(x, y + 1, 1) == 0) {
				add_wall_right_2(x, y + 1, 0);
				watched_wall_right_2(x, y + 1, 0);
			}
			if (x < 15 && y > 0 && is_the_Wall_watched_2(x, y, 2) == 1	//右下縦壁追加
			&& is_the_Wall_watched_2(x + 1, y, 2) == 1
					&& is_the_Wall_watched_2(x, y, 1) == 1
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_Exist_Wall_2(x + 1, y, 2) == 0
					&& is_Exist_Wall_2(x, y, 1) == 0
					&& is_the_Wall_watched_2(x, y - 1, 1) == 0) {
				add_wall_right_2(x, y - 1, 0);
				watched_wall_right_2(x, y - 1, 0);
			}
			if (x > 0 && y > 0 && is_the_Wall_watched_2(x - 1, y, 2) == 1//左下縦壁追加
			&& is_the_Wall_watched_2(x, y, 2) == 1
					&& is_the_Wall_watched_2(x, y, 3) == 1
					&& is_Exist_Wall_2(x - 1, y, 2) == 0
					&& is_Exist_Wall_2(x, y, 2) == 0
					&& is_Exist_Wall_2(x, y, 3) == 0
					&& is_the_Wall_watched_2(x, y - 1, 3) == 0) {
				add_wall_left_2(x, y - 1, 0);
				watched_wall_left_2(x, y - 1, 0);
			}
		}

		total_dist = 0.0;

		q_walk_map2_maker(hikisuu_goal_x, hikisuu_goal_y);

		switch (direction_count) {
		case 0:
			if (y < 15&& map2[x][y + 1]
			< map2[x][y]&& SEN_r_front_value < r_front_wall_judge) {//North & 直進
				LED2_y = 1;
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
				direction_xy();

				LED2_y = 0;
			} else if (x < 15&& map2[x + 1][y]
			< map2[x][y]&& SEN_r_value < r_wall_judge) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				LED1_p = 1;

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				LED1_p = 0;
			} else if (x > 0&& map2[x - 1][y]
			< map2[x][y]&& SEN_l_value < l_wall_judge) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_unclock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
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
				LED1_p = 1;

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;

			}
			break;

		case 1:
			if (x
					< 15&& map2[x+1][y] < map2[x][y] && SEN_r_front_value < r_front_wall_judge) {//East & 直進
				LED1_p = 1;
				LED3_r = 1;
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
				direction_xy();
				LED1_p = 0;
				LED3_r = 0;

			} else if (y
					> 0&& map2[x][y-1] < map2[x][y] && SEN_r_value < r_wall_judge) {//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();

			} else if (y
					< 15&& map2[x][y+1] < map2[x][y] && SEN_l_value < l_wall_judge) {//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_unclock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;

			}
			break;

		case 2:
			if (y
					> 0&& map2[x][y-1] < map2[x][y] && SEN_r_front_value < r_front_wall_judge) {//South & 直進

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
				direction_xy();
			} else if (x
					> 0&& map2[x-1][y] < map2[x][y] && SEN_r_value < r_wall_judge) {//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();

			} else if (x
					< 15&& map2[x+1][y] < map2[x][y] && SEN_l_value < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_unclock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;

			}
			break;

		case 3:
			if (x
					> 0&& map2[x-1][y] < map2[x][y] && SEN_r_front_value < r_front_wall_judge) {//South & 直進
				direction_xy();
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(180.0, V_search, ACCEL_search, V_search, V_search,
						1);
			} else if (y
					< 15&& map2[x][y+1] < map2[x][y] && SEN_r_value < r_wall_judge) {//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();

			} else if (y
					> 0&& map2[x][y-1] < map2[x][y] && SEN_l_value < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_unclock3(90.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance_q(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				turn_clock3(180.0, V_search, ACCEL_search, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(STOP_count);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, V_search, ACCEL_search, 150.0, V_search, 1);
				direction_xy();
				footmark[x][y] = 10;

			}
			break;

		}
		setReached(x, y);

		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}

	}
	if (accident_flag == 0) {
		distance3(90.0, V_search, ACCEL_search, V_search, 150.0, 1);
		MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
		MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
		wait(300);
		motor_enable = 0;
	} else {
		motor_enable = 0;

	}
}

void adachihou(int hikisuu_goal_x, int hikisuu_goal_y) {
	colum[0] |= 1;
//	colum[0] = 1;
//	direction_count = 0;
	footmark[0][0] = 1;

	x = 0;
	y = 1;
	distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);

	if (SEN_r_front_value > r_front_wall_judge) {	//前センサーの壁判断
		add_wall_front(x, y, direction_count);
	}
	if (SEN_r_value > r_wall_judge) {	//右センサーの壁判断
		add_wall_right(x, y, direction_count);
	}
	if (SEN_l_value > l_wall_judge) {	//左センサーの壁判断
		add_wall_left(x, y, direction_count);
	}

	while (1) {

		if (SEN_r_front_value > r_front_wall_judge) {	//前センサーの壁判断
			add_wall_front(x, y, direction_count);
		}
		if (SEN_r_value > r_wall_judge) {	//右センサーの壁判断
			add_wall_right(x, y, direction_count);
		}
		if (SEN_l_value > l_wall_judge) {	//左センサーの壁判断
			add_wall_left(x, y, direction_count);
		}
		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}

//		q_walk_map_maker(hikisuu_goal_x, hikisuu_goal_y);
//		myprintf("y=%d",y);
		switch (direction_count) {
		case 0:
			if (map[x][y + 1]
					< map[x][y]&& y<15 && SEN_r_front_value < r_front_wall_judge) {	//North & 直進
				direction_xy();
				LED2_y = 1;
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(180.0, 400.0, 2000.0, 400.0, 400.0, 1);
				LED2_y = 0;
			} else if (map[x + 1][y]
					< map[x][y]&& x<15 && SEN_r_value < r_wall_judge) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				LED1_p = 1;

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
				LED1_p = 0;
			} else if (map[x - 1][y]
					< map[x][y]&& x>0 && SEN_l_value < l_wall_judge) {//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_unclock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
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
				LED1_p = 1;
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(180.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
			}
			break;

		case 1:
			if (direction_count
					== 1&& map[x+1][y] < map[x][y] && x<15 && SEN_r_front_value < r_front_wall_judge) {	//East & 直進
				direction_xy();
				LED1_p = 1;
				LED3_r = 1;
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(180.0, 400.0, 2000.0, 400.0, 400.0, 1);
				LED1_p = 0;
				LED3_r = 0;

			} else if (direction_count
					== 1&& map[x][y-1] < map[x][y] && y>0 && SEN_r_value < r_wall_judge) {//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);

			} else if (direction_count
					== 1&& map[x][y+1] < map[x][y] && y<15 && SEN_l_value < l_wall_judge) {	//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_unclock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);

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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(180.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
			}
			break;

		case 2:
			if (direction_count
					== 2&& map[x][y-1] < map[x][y] && y>0 && SEN_r_front_value < r_front_wall_judge) {//South & 直進
				direction_xy();
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(180.0, 400.0, 2000.0, 400.0, 400.0, 1);
			} else if (direction_count
					== 2&& map[x-1][y] < map[x][y] && x>0 && SEN_r_value < r_wall_judge) {//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);

			} else if (direction_count
					== 2&& map[x+1][y] < map[x][y] && x<15 && SEN_l_value < l_wall_judge) {	//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_unclock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(180.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
			}
			break;

		case 3:
			if (direction_count
					== 3&& map[x-1][y] < map[x][y] && x>0 && SEN_r_front_value < r_front_wall_judge) {//South & 直進
				direction_xy();
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(180.0, 400.0, 2000.0, 400.0, 400.0, 1);
			} else if (direction_count
					== 3&& map[x][y+1] < map[x][y] && y<15 && SEN_r_value < r_wall_judge) {	//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);

			} else if (direction_count
					== 3&& map[x][y-1] < map[x][y] && y>0 && SEN_l_value < l_wall_judge) {//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_unclock3(90.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
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

				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				turn_clock3(180.0, 400.0, 2000.0, 150.0, 150.0);
				MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
				MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
				wait(300);
				PE.DRL.BIT.B0 = 0;	//B0=0で正回転
				PE.DRL.BIT.B4 = 1;	//B4=1で正回転
				distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
			}
			break;

		}

		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}

		setReached(x, y);
	}
	distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
	MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
	MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
	wait(300);
	motor_enable = 0;
}

