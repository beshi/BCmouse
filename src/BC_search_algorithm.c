/*
 * BC_search_algorithm.c
 *
 *  Created on: 2017/10/24
 *      Author: Alex
 */

#include "iodefine.h"
#include "BC_daikei.h"
#include "BC_define.h"
#include "math.h"

void new_serch_algorithm(int hikisuu_goal_x, int hikisuu_goal_y, int start_x,
		int start_y, int V_search, int ACCEL_search) {		//足立法を改造したもの
	volatile float r_before, l_before, r_after, l_after;
	volatile char goal_size, unknown_wall_finish = 0;
	r_before = 10.5;
	r_after = 12.0;
	l_before = 7.8;
	l_after = 23.0;

	goal_size = 0;

	column_temp[0] |= 1;

	x = start_x;
	y = start_y;
	wait(300); 			//励磁直後は少し待つ！

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
		LED1 = 1;
		Search_UnknownWall_Pass(hikisuu_goal_x, hikisuu_goal_y);	//探索中のpassに基づいて未知壁をセットする
		Set_Temp_Goal();											//通った未知壁情報をもとにゴール座標をセット
		q_new_walk_map_maker(0, 0, goal_size, x, y);				//goal_size==0の場合、(0,0)はゴール座標としてセットされない。
		LED1 = 0;

		if (map[x][y] == 255) {		//閉じ込められたらスタートに戻る or fail-safe
			if(unknown_wall_finish == 0){
				goal_size = 1;	//これでスタート地点がゴールとして設定される
				LED1 =1;	//debug
				LED_V1=1;	//debug
				unknown_wall_finish = 1;
			}else{
				fail_flag = 1;
			}
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
				(x == hikisuu_goal_x + 1 && y == hikisuu_goal_y + 1) ||
				(x == hikisuu_goal_x + 1 && y == hikisuu_goal_y + 1) ) {
			switch (direction_count) {
			case 0:		//North
				goal_x = x;
				goal_y = y + 1;
				break;
			case 1:		//East
				goal_x = x + 1;
				goal_y = y;
				break;
			case 2:		//South
				goal_x = x;
				goal_y = y - 1;
				break;
			case 3:		//West
				goal_x = x - 1;
				goal_y = y;
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

