#include "iodefine.h"
#include "workshop.h"
#define pi 3.141592
#define R_tread /*44.2*/43.65
#define LED2_y PB.DR.BIT.B2
#define  motor_enable PE.DRL.BIT.B8

void daikei(float _dist, float _Vmax, float _Vmin, float _acc, int _wall) {
	int u;
	float d1 = (_Vmax * _Vmax - _Vmin * _Vmin) / (2 * _acc);
	total_dist = 0.0;
	speed = _Vmin;
	wall_control = _wall;
	if (_dist > d1 * 2) {
		while (1) {
			if (total_dist <= d1) {
				accel = _acc;
			} else if (total_dist <= _dist - d1) {
				accel = 0;
			} else if (total_dist <= _dist) {
				accel = -_acc;
			} else if (speed <= _Vmin) {
				accel = 0;
				break;
			}
		}
	} else {
		LED2_y = 1;
		while (1) {
			if (total_dist <= _dist / 2) {
				accel = _acc;
			} else if (total_dist <= _dist) {
				accel = -_acc;
			} else if (speed <= _Vmin) {
				accel = 0;
				break;
			}
		}
	}
	MTU2.TSTR.BIT.CST0 = 0;		//ステータスレジスタ　停止
	MTU2.TSTR.BIT.CST1 = 0;		//ステータスレジスタ　停止
	for(u=0;u<1000000;u++);
	motor_enable = 0;
}

void cho_sinchi(float _ang){
	int u;
	float dist = pi*R_tread*_ang/180;
	motor_enable = 1;
	for(u=0;u<1000000;u++);
	MTU2.TSTR.BIT.CST0 = 1;		//ステータスレジスタ　停止
	MTU2.TSTR.BIT.CST1 = 1;		//ステータスレジスタ　停止
	if(_ang > 0){
		PE.DRL.BIT.B0 = 1;		//右ホイール時計回りに回転
		PE.DRL.BIT.B4 = 1;		//左ホイール

	}else{
		PE.DRL.BIT.B0 = 0;		//右ホイール時計回りに回転
		PE.DRL.BIT.B4 = 0;		//左ホイール
		dist = -1*dist;
	}
	daikei(dist,450,150,2000,0);
}
