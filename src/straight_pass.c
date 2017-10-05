/*
 * straight_pass.c
 *
 *  Created on: 2016/03/02
 *      Author: Alex
 */

#include "straight_pass.h"
#include "iodefine.h"
#include "hensuu&define.h"

void adachihou_straight_map(int hikisuu_goal_x, int hikisuu_goal_y);
void adachihou_straight(int hikisuu_goal_x, int hikisuu_goal_y, int V_FAST,
		int V_path, int ACCEL_FAST, int ACCEL_path, int STOP_count_pat);


void adachihou_straight_map(int hikisuu_goal_x, int hikisuu_goal_y) {//足立法のシュミレーションにより、パスの時の直線を数える。
	colum[0] |= 1;
//	colum[0] = 1;
	direction_count = 0;		//スタート地点を考えている。
	footmark[0][0] = 1;
	straight_count = 0;
	x = 0;
	y = 1;

	while (1) {
		switch (direction_count) {
		case 0:
			if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//North & 直進
				direction_xy();
				straight_flag[x][y] = 1;

			} else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//North & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				straight_flag[x][y] = 0;

			} else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//North & 左折

				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				straight_flag[x][y] = 0;

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
				straight_flag[x][y] = 0;

			}
			break;

		case 1:
			if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//East & 直進
				direction_xy();
				straight_flag[x][y] = 1;

			} else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//East & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				straight_flag[x][y] = 0;

			} else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//East & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				straight_flag[x][y] = 0;

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
				straight_flag[x][y] = 0;

			}
			break;

		case 2:
			if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//South & 直進
				direction_xy();
				straight_flag[x][y] = 1;

			} else if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//South & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				straight_flag[x][y] = 0;

			} else if (x < 15 && map[x + 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 1) == 0) {	//South & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				straight_flag[x][y] = 0;

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
				straight_flag[x][y] = 0;

			}
			break;

		case 3:
			if (x > 0 && map[x - 1][y] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 3) == 0) {	//West & 直進
				direction_xy();
				straight_flag[x][y] = 1;

			} else if (y < 15 && map[x][y + 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 0) == 0) {	//West & 右折
				if (direction_count == 3) {
					direction_count = 0;
				} else {
					direction_count++;
				}
				direction_xy();
				straight_flag[x][y] = 0;

			} else if (y > 0 && map[x][y - 1] == map[x][y] - 1
					&& is_Exist_Wall(x, y, 2) == 0) {	//West & 左折
				if (direction_count == 0) {
					direction_count = 3;
				} else {
					direction_count--;
				}
				direction_xy();
				straight_flag[x][y] = 0;

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
				straight_flag[x][y] = 0;

			}
			break;

		}
		if (straight_flag[x][y] == 1) {
			straight_count++;
			if (direction_count == 0) {		//関数化する！しなくてもいいか…
				straight_map[x][y - (straight_count)] = straight_count;
			} else if (direction_count == 1) {
				straight_map[x - straight_count][y] = straight_count;
			} else if (direction_count == 2) {
				straight_map[x][y + straight_count] = straight_count;
			} else if (direction_count == 3) {
				straight_map[x + straight_count][y] = straight_count;
			}

		} else {
			straight_count = 0;

			if (direction_count == 0) {		//必要ないが一応わかりやすくするために書いておく…
				straight_map[x][y - (straight_count)] = straight_count;
			} else if (direction_count == 1) {
				straight_map[x - straight_count][y] = straight_count;
			} else if (direction_count == 2) {
				straight_map[x][y + straight_count] = straight_count;
			} else if (direction_count == 3) {
				straight_map[x + straight_count][y] = straight_count;
			}

		}
		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}

	}
}

void adachihou_straight(int hikisuu_goal_x, int hikisuu_goal_y, int V_FAST,
		int V_path, int ACCEL_FAST, int ACCEL_path, int STOP_count_pat) {
	colum[0] |= 1;
//	colum[0] = 1;	←これやばいやつ！
	direction_count = 0;
	footmark[0][0] = 1;

	x = 0;
	y = 1;
	PE.DRL.BIT.B0 = 0;	//B0=0で正回転
	PE.DRL.BIT.B4 = 1;	//B4=1で正回転
	motor_enable = 1;
	wait(300); 			//励磁直後は少し待つ！

	distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);

	while (1) {
		if (map[x][y] == 255) {		//閉じ込められたら自動で抜け出す。
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
		if (straight_map[x][y] >= 1) {
			s_count = straight_map[x][y];
			distance3(180 * (s_count), V_FAST, ACCEL_FAST, V_path, V_path, 1);
			for (l = 1; l <= s_count; l++) {
				direction_xy();
				setReached(x, y);
			}
			if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
				break;
			}
		} else if (straight_map[x][y] < 2) {

			switch (direction_count) {
			case 0:
//				if (straight_map[x][y + 1] >= 2) {
//					distance3(180 * straight_count, V_FAST, ACCEL_FAST, 400.0,
//							400.0,1);
//					for(l = 1; l <= straight_map[x][y]; l++){
//						direction_xy();
//					}
//				} else {

				if (y < 15&& map[x][y + 1]
				< map[x][y]&& SEN_r_front_value < r_front_wall_judge) {	//North & 直進
					direction_xy();
					LED2_y = 1;
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(180.0, V_path, ACCEL_path, V_path, V_path, 1);
					LED2_y = 0;
				} else if (x < 15&& map[x + 1][y]
				< map[x][y]&& SEN_r_value < r_wall_judge) {	//North & 右折
					if (direction_count == 3) {
						direction_count = 0;
					} else {
						direction_count++;
					}
					direction_xy();
					LED1_p = 1;

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
					LED1_p = 0;
				} else if (x > 0&& map[x - 1][y]
				< map[x][y]&& SEN_l_value < l_wall_judge) {	//North & 左折

					if (direction_count == 0) {
						direction_count = 3;
					} else {
						direction_count--;
					}
					direction_xy();

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_unclock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
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
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(180.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
				}
//				}
				break;

			case 1:
//				if (straight_map[x + 1][y] >= 2) {
//					distance3(180 * straight_count, V_FAST, ACCEL_FAST, 400.0,
//							400.0,1);
//					for (k = 1; k <= straight_count; k++) {
//						direction_xy();
//					}
//				} else {

				if (x
						< 15&& map[x+1][y] < map[x][y] && SEN_r_front_value < r_front_wall_judge) {	//East & 直進
					direction_xy();
					LED1_p = 1;
					LED3_r = 1;
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(180.0, V_path, ACCEL_path, V_path, V_path, 1);
					LED1_p = 0;
					LED3_r = 0;

				} else if (y
						> 0&& map[x][y-1] < map[x][y] && SEN_r_value < r_wall_judge) {//East & 右折
					if (direction_count == 3) {
						direction_count = 0;
					} else {
						direction_count++;
					}
					direction_xy();

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);

				} else if (y
						< 15&& map[x][y+1] < map[x][y] && SEN_l_value < l_wall_judge) {	//East & 左折
					if (direction_count == 0) {
						direction_count = 3;
					} else {
						direction_count--;
					}
					direction_xy();

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_unclock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);

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
					distance3(90.0, ACCEL_path, ACCEL_path, ACCEL_path, 150.0,
							1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(180.0, ACCEL_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
				}
//				}
				break;

			case 2:
//				if (straight_map[x][y - 1] >= 2) {
//					distance3(180 * straight_count, V_FAST, ACCEL_FAST, 400.0,
//							400.0,1);
//					for (k = 1; k <= straight_count; k++) {
//						direction_xy();
//					}
//				} else {

				if (y
						> 0&& map[x][y-1] < map[x][y] && SEN_r_front_value < r_front_wall_judge) {//South & 直進
					direction_xy();
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(180.0, V_path, ACCEL_path, V_path, V_path, 1);
				} else if (x
						> 0&& map[x-1][y] < map[x][y] && SEN_r_value < r_wall_judge) {//South & 右折
					if (direction_count == 3) {
						direction_count = 0;
					} else {
						direction_count++;
					}
					direction_xy();

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, V_path, 1);

				} else if (x
						< 15&& map[x+1][y] < map[x][y] && SEN_l_value < l_wall_judge) {	//West & 左折
					if (direction_count == 0) {
						direction_count = 3;
					} else {
						direction_count--;
					}
					direction_xy();

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_unclock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
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
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(180.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
				}
//				}
				break;

			case 3:
//				if (straight_map[x - 1][y] >= 2) {
//					distance3(180 * straight_count, V_FAST, ACCEL_FAST, 400.0,
//							400.0,1);
//					for (k = 1; k <= straight_count; k++) {
//						direction_xy();
//					}
//				} else {

				if (x
						> 0&& map[x-1][y] < map[x][y] && SEN_r_front_value < r_front_wall_judge) {//South & 直進
					direction_xy();
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(180.0, V_path, ACCEL_path, V_path, V_path, 1);
				} else if (y
						< 15&& map[x][y+1] < map[x][y] && SEN_r_value < r_wall_judge) {	//West & 右折
					if (direction_count == 3) {
						direction_count = 0;
					} else {
						direction_count++;
					}
					direction_xy();

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);

				} else if (y
						> 0&& map[x][y-1] < map[x][y] && SEN_l_value < l_wall_judge) {//West & 左折
					if (direction_count == 0) {
						direction_count = 3;
					} else {
						direction_count--;
					}
					direction_xy();

					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_unclock3(90.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
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
					distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					turn_clock3(180.0, V_path, ACCEL_path, 150.0, 150.0);
					MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
					MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
					wait(STOP_count_pat);
					PE.DRL.BIT.B0 = 0;	//B0=0で正回転
					PE.DRL.BIT.B4 = 1;	//B4=1で正回転
					distance3(90.0, V_path, ACCEL_path, 150.0, V_path, 1);
				}
//				}
				break;

			}

		}
		if (x == hikisuu_goal_x && y == hikisuu_goal_y) {
			break;
		}
		setReached(x, y);

	}
	if (accident_flag == 0) {
		distance3(90.0, V_path, ACCEL_path, V_path, 150.0, 1);
		MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
		MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
		wait(300);
		motor_enable = 0;
	} else {
		motor_enable = 0;
	}
}

