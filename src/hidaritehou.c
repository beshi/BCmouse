/*
 * hidaritehou.c
 *
 *  Created on: 2016/03/02
 *      Author: Alex
 */
#include "iodefine.h"
#include "hidaritehou.h"
#include "hensuu&define.h"


void hidaritehou_slalom(int hikisuu_goal_x, int hikisuu_goal_y, int V_search,
		int ACCEL_search, float hikisuu_angacc, float hikisuu_angle1,
		float hikisuu_angle2, float offset) {	//ゴール座標ありの左手法
	colum[0] = 1;
	direction_count = 0;
	footmark[0][0] = 1;
	motor_enable = 1;
	wait(300); 			//励磁直後は少し待つ！
	PE.DRL.BIT.B0 = 0;	//B0=0で正回転
	PE.DRL.BIT.B4 = 1;	//B4=1で正回転
	MTU2.TSTR.BIT.CST0 = 1;		//ステータスレジスタ　停止
	MTU2.TSTR.BIT.CST1 = 1;		//ステータスレジスタ　停止

	x = 0;
	y = 1;
	distance3(90.0, 480.0, 2000.0, 150.0, 480.0, 1);
	setReached(x, y);
	//	if (SEN_r_front_value >= r_front_wall_judge) {	//前センサーの壁判断
	//		add_wall_front(x, y, direction_count);
	//	}
	//	if (SEN_r_value >= r_wall_judge) {	//右センサーの壁判断
	//		add_wall_right(x, y, direction_count);
	//	}
	//	if (SEN_l_value >= l_wall_judge) {	//左センサーの壁判断
	//		add_wall_left(x, y, direction_count);
	//	}

	while (1) {
		if (accident_flag == 1) {
			break;
		}
		if (SEN_l_value < l_wall_judge) {
			if (direction_count == 0) {
				direction_count = 3;
			} else {
				direction_count--;
			}
			direction_xy();
			slalom_L2(90, hikisuu_angacc, hikisuu_angle1, hikisuu_angle2,
					V_search, offset, 0);	//左折

		} else if (SEN_r_front_value < r_front_wall_judge) {
			direction_xy();
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			//			wall_control = 1;
			distance3(180.0, 480.0, 2000.0, 480.0, 480.0, 1);
		} else if (SEN_r_value < r_wall_judge) {
			if (direction_count == 3) {
				direction_count = 0;
			} else {
				direction_count++;
			}
			direction_xy();

			slalom_R2(90, hikisuu_angacc, hikisuu_angle1, hikisuu_angle2,
					V_search, offset, 0);	//左折
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
			distance3(90.0, 480.0, 2000.0, 480.0, 150.0, 1);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			turn_clock3(180.0, 480.0, 2000.0, 150.0, 150.0);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			distance3(90.0, 480.0, 2000.0, 150.0, 480.0, 1);
		}
		setReached(x, y);
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
	}
	distance3(90.0, 480.0, 2000.0, 480.0, 150.0, 1);
	MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
	MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
	wait(300);
	motor_enable = 0;
}

void hidaritehou2() {	//速度が連続的な左手法
	distance3(90.0, 600.0, 500.0, 150.0, 400.0, 1);
	while (1) {
		if (SEN_l_value < 150) {
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			distance3(90.0, 400.0, 1500.0, 400.0, 150.0, 1);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			turn_unclock3(90.0, 400.0, 1000.0, 150.0, 150.0);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			wall_control = 1;
			distance3(90.0, 400.0, 1500.0, 150.0, 400.0, 1);
		} else if (SEN_r_front_value < 130) {
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
//			wall_control = 1;
			distance3(180.0, 400.0, 1500.0, 400.0, 400.0, 1);
		} else if (SEN_r_value < 150) {
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			distance3(90.0, 400.0, 1500.0, 400.0, 150.0, 1);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			turn_clock3(90.0, 400.0, 1000.0, 150.0, 150.0);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			wall_control = 1;
			distance3(90.0, 400.0, 1500.0, 150.0, 400.0, 1);
			PB.DR.BIT.B2 = 1;
			LED2_y = 0;
		} else {
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			distance3(90.0, 400.0, 1500.0, 400.0, 150.0, 1);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			turn_clock3(180.0, 400.0, 1000.0, 150.0, 150.0);
			MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
			MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
			wait(300);
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
			distance3(90.0, 400.0, 1500.0, 150.0, 400.0, 1);
		}
	}
}


void hidaritehou3(int hikisuu_goal_x, int hikisuu_goal_y) {	//ゴール座標ありの左手法
	colum[0] = 1;
	direction_count = 0;
	footmark[0][0] = 1;

	x = 0;
	y = 1;
	distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
	setReached(x, y);
//	if (SEN_r_front_value >= r_front_wall_judge) {	//前センサーの壁判断
//		add_wall_front(x, y, direction_count);
//	}
//	if (SEN_r_value >= r_wall_judge) {	//右センサーの壁判断
//		add_wall_right(x, y, direction_count);
//	}
//	if (SEN_l_value >= l_wall_judge) {	//左センサーの壁判断
//		add_wall_left(x, y, direction_count);
//	}

	while (1) {
		if (accident_flag == 1) {
			break;
		}
		if (SEN_l_value < l_wall_judge) {
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
			wall_control = 1;
			distance3(90.0, 400.0, 2000.0, 150.0, 400.0, 1);
		} else if (SEN_r_front_value < r_front_wall_judge) {
			direction_xy();
			PE.DRL.BIT.B0 = 0;	//B0=0で正回転
			PE.DRL.BIT.B4 = 1;	//B4=1で正回転
//			wall_control = 1;
			distance3(180.0, 400.0, 2000.0, 400.0, 400.0, 1);
		} else if (SEN_r_value < r_wall_judge) {
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
//			wall_control = 1;
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
		setReached(x, y);
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
	}
	distance3(90.0, 400.0, 2000.0, 400.0, 150.0, 1);
	MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
	MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
	wait(300);
	motor_enable = 0;
}



