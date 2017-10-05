/*
 * BC_init.c
 *
 *  Created on: 2017/03/10
 *      Author: Alex
 */
#include "iodefine.h"
#include "BC_init.h"
#include "BC_define.h"

void init_CPU() {
	SYSTEM.SCKCR.BIT.ICK = 0;	//システムクロック　x8倍により、100MHz
	SYSTEM.SCKCR.BIT.PCK = 1;	//周辺動作クロック　x4倍により、50MHz
}

void init_sci(void) {
	int Cnt;
	int baud = 38400;			//ビットレート38400bps
	unsigned char tmp;

//	STB.CR3.BIT._SCI1 = 0;			//スタンバイ解除

	PORTD.DDR.BIT.B3 = 1;		//シリアルポートを設定-TXD1を出力
	PORTD.DDR.BIT.B5 = 0;		//シリアルポートを設定-RXD1を入力
	PORTD.ICR.BIT.B3 = 1;
	PORTD.ICR.BIT.B5 = 0;

	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0; //SCI1ストップ解除

	SCI1.SCR.BYTE = 0x00u; //SCI1停止
	SCI1.SMR.BYTE = 0x00u; //PCLK分周無し(n=0)、ストップビット1、パリティ無し、データ8bit、調歩同期式

	tmp = (unsigned char) (50000000 / 32 / baud) - 1;	//PCLK=50MHz,n=0
	SCI1.BRR = tmp;			//ビットレート設定

	for (Cnt = 0; Cnt < 1000; Cnt++)
		;	//1bit(@9600bps) Wait
	SCI1.SSR.BYTE &= ~0x78u; //エラーフラグクリア
	SCI1.SCR.BIT.TE = 1;		//送信許可
	SCI1.SCR.BIT.RE = 1;		//受信許可
}

void init_ADconvert() {
	//以下、Battery用のAD変換
	S12AD1.ADCSR.BIT.ADST = 0;	//スタンバイを0で設定
	S12AD1.ADCSR.BIT.TRGE = 0;	//外部トリガ禁止
	S12AD1.ADCSR.BIT.CKS = 2;	//ADCLK(A/D変換クロック)　PCLKの1/2倍により、25MHｚ
	S12AD1.ADCSR.BIT.ADIE = 0;	//変換後割り込み禁止
	S12AD1.ADCSR.BIT.ADCS = 0;	//シングルモード
	S12AD1.ADANS.BIT.CH = 3;	//シングル＆AN103なので
	S12AD1.ADCER.BIT.ADPRC = 0;	//12ビットで格納
	S12AD1.ADCER.BIT.ADRFMT = 0;	//右詰めでデータ格納
	S12AD1.ADSSTR = 3;	//変換時間は3/(50/8)=0.48us

	//以下、センサー用のAD変換
	S12AD0.ADCSR.BIT.ADST = 0;	//スタンバイを0で設定
	S12AD0.ADCSR.BIT.TRGE = 0;	//外部トリガ禁止
	S12AD0.ADCSR.BIT.CKS = 2;	//ADCLK(A/D変換クロック)　PCLKの1/2倍により、25MHｚ
	S12AD0.ADCSR.BIT.ADIE = 0;	//変換後割り込み禁止
	S12AD0.ADCSR.BIT.ADCS = 0;	//シングルモード
	S12AD0.ADCER.BIT.ADPRC = 0;	//12ビットで格納
	S12AD0.ADCER.BIT.ADRFMT = 0;	//右詰めでデータ格納
	S12AD0.ADSSTR = 3;	//変換時間は3/(50/8)=0.48us

	//以下、センサーLEDの出力設定
	PORTB.DDR.BIT.B4 = 1;
	PORTB.DDR.BIT.B5 = 1;
	PORTB.DDR.BIT.B6 = 1;
	PORTB.DDR.BIT.B7 = 1;
}

void init_CMT() {
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;

	CMT.CMSTR0.BIT.STR0 = 0;	//タイマー停止
	CMT0.CMCNT = 0;			//カウンタクリア
	CMT0.CMCOR = 6249;		//割り込み周期1ms
	CMT0.CMCR.BIT.CKS = 0;	//分周比8(PCLK/8=6.25MHz)
	CMT0.CMCR.BIT.CMIE = 1;	//割り込み許可
	ICU.IPR[04].BIT.IPR = 14;		//CMT0の割り込み優先度14
	ICU.IER[03].BIT.IEN4 = 1;		//割り込み要求許可

}

void init_MTU() {
	IOPORT.PFCMTU.BIT.TCLKS = 0;		//MTCLKピンとして設定
	//以下、MTCLKピンの入力バッファを有効化！これは必要だから注意！
	PORT3.ICR.BIT.B0 = 1;
	PORT3.ICR.BIT.B1 = 1;
	PORT3.ICR.BIT.B2 = 1;
	PORT3.ICR.BIT.B3 = 1;

	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;	//MTUストップ解除
	MTU.TSTRA.BIT.CST1 = 0;	//MTU1ストップ
	MTU.TSTRA.BIT.CST2 = 0;	//MTU2ストップ
//	MTU1.TCR.BIT.TPSC = 4;	//←入れてもいいが、不要みたいですね
//	MTU2.TCR.BIT.TPSC = 4;	//←入れてもいいが、不要みたいですね
	MTU1.TMDR1.BIT.MD = 4;		//位相計数モード1 right_motor
	MTU2.TMDR1.BIT.MD = 4;
	MTU1.TCNT = 32767;
	MTU2.TCNT = 32767;
	MTU.TSTRA.BIT.CST1 = 1;		//MTU1スタート
	MTU.TSTRA.BIT.CST2 = 1;		//MTU2スタート

	/*	MTU.TSTRA.BYTE = 0;		//以下不要
	 MTU.TSTRB.BYTE = 0;
	 MTU3.TCR.BIT.CCLR = 1; //TGRAのコンペアマッチでカウンタクリア
	 MTU3.TCR.BIT.CKEG = 0; //UPエッジでカウント
	 MTU3.TCR.BIT.TPSC = 2;	//ICLK/16=100/16=6.25MHzでカウント
	 MTU3.TMDR1.BIT.MD = 2;	//PWMモード1
	 MTU3.TMDR1.BIT.BFA = 1;	 //TGRAとTGRCはバッファ動作
	 MTU3.TGRA = 625;		//周期1msにてコンペアマッチ
	 MTU3.TGRB = 6;			//Duty0.01
	 MTU3.TGRC = 625;		//MTIOC4C TGRAへのバッファレジスタ(速度設定先)
	 MTU3.TIORH.BIT.IOA = 2;	//TGRA:初期値0、CMで1出力
	 MTU3.TIORH.BIT.IOB = 1;	//TGRB:初期値0、CMで0出力
	 MTU.TOERA.BIT.OE3B = 1;	//出力許可
	 MTU.TOERA.BIT.OE3D = 1;	//出力許可
	 PORT7.DDR.BIT.B1 = 1; //MTIOC3B(38pin)を出力に設定
	 PORT7.DDR.BIT.B4 = 1; //MTIOC3D(35pin)を出力に設定
	 //	ICU.IPR[0x57u].BIT.IPR = 3; //MTU3割り込み優先度設定←不要
	 //	ICU.IER[0x10u].BIT.IEN1 = 1; //MUT4 TGIA4割り込み
	 //	MTU.TSTRA.BIT.CST3=1;//MTU3スタート*/

}

void init_GPT() {
	IOPORT.PFDGPT.BIT.GPTS = 0;	//GTIOC0A-A,GTIOC0B-Aとして設定
	PORT7.DDR.BIT.B1 = 1;
	PORT7.DDR.BIT.B4 = 1;
	ICU.IER[15].BIT.IEN6 = 0;		//GTCCRAの割り込み許可
	ICU.IER[15].BIT.IEN7 = 0;

	SYSTEM.MSTPCRA.BIT.MSTPA7 = 0;	//凡用PWMモジュールストップ解除

	GPT0.GTCR.BIT.MD = 0;		//のこぎり波PWMモード
	GPT0.GTUDC.BIT.UD = 1;		//アップカウント
	GPT0.GTCR.BIT.TPCS = 2;		//カウントクロック分周比1/2(ICLK/2 = 12.5MHz)
	GPT0.GTPR = (125 - 1);		//PWM周期を100kHzとするためのカウント数
	GPT0.GTCNT = 0;				//カウンタ初期値
	GPT0.GTIOR.BIT.GTIOA = 6;	//表18.5を参照(出力値の設定)
	GPT0.GTIOR.BIT.GTIOB = 6;
	GPT0.GTONCR.BIT.OAE = 1;	//出力許可
	GPT0.GTONCR.BIT.OBE = 1;
	GPT0.GTCCRA = dutty_l;			//Duty比設定
	GPT0.GTCCRB = gptcount_r;
	GPT0.GTINTAD.BIT.GTINTA = 1;	//割り込み許可・・・
	GPT0.GTINTAD.BIT.GTINTB = 1;
//	GPT0.GTINTAD.BIT.GTINTPR = 3;

	ICU.DTCER[27].BIT.DTCE=0;	//不要な気が・・・
	ICU.SWINTR.BIT.SWINT=1;		//不要な気が・・・
	ICU.IPR[68].BIT.IPR = 12;
	ICU.IER[15].BIT.IEN6 = 1;		//GTCCRAの割り込み許可
	ICU.IER[15].BIT.IEN7 = 1;
//	ICU.DTCER[174].BIT.DTCE=1;		//DTC起動許可・・・
	GPT0.GTST.BIT.TCFA = 0;	//フラグクリアのはず・・・
	GPT0.GTST.BIT.TCFB = 0;	//フラグクリアのはず・・・

}

void init_RSPI() {
	int i;
	SYSTEM.MSTPCRB.BIT.MSTPB17 = 0;	//シリアルペリフェラルのストップ解除
	RSPI0.SPCR.BIT.SPE = 0; //RSPI機能は無効（初期設定）

	RSPI0.SPCR.BIT.SPMS = 0; //SPI動作、4線式
	RSPI0.SPCR.BIT.TXMD = 0; //全二重式シリアル通信
	RSPI0.SPCR.BIT.MSTR = 1; //マスターモード
	RSPI0.SPCMD0.BIT.CPHA = 1;
	RSPI0.SPCMD0.BIT.CPOL = 1;
	RSPI0.SPCMD0.BIT.SSLKP = 1; //バースト転送（SSL信号レベル保持）
	RSPI0.SPCMD0.BIT.SPB = 4; //データ長は8bit
	RSPI0.SPCMD0.BIT.LSBF = 0; //MSBファースト
	RSPI0.SPBR = 5;
	RSPI0.SPCMD0.BIT.BRDV = 2;	//分周比は48となり、ビットレートは1.04Mbpsとなる
	RSPI0.SPDCR.BIT.SLSEL = 1;		//SSL0のみ出力設定

	IOPORT.PFGSPI.BIT.SSL0E = 1;	//SSL0出力許可
	IOPORT.PFGSPI.BIT.RSPCKE = 1;		//RSPCK出力許可
	IOPORT.PFGSPI.BIT.MISOE = 1;		//MISO出力許可
	IOPORT.PFGSPI.BIT.MOSIE = 1;		//MOSI出力許可
	IOPORT.PFHSPI.BIT.RSPIS = 1;	//各端子をMISO-Bなどとして設定

	PORTA.DDR.BIT.B5 = 0;	//MISOより入力設定
	PORTB.DDR.BIT.B0 = 1;	//MOSIより出力設定
	PORTA.DDR.BIT.B4 = 1;	//RSPCKより出力設定
	PORTA.DDR.BIT.B3 = 1;	//SSL0より出力設定
	PORTA.ICR.BIT.B5 = 1;	//MISOの入力バッファを有効

//以下、ジャイロ読み取りの設定
	SPIWrite(0x6b, 0x80);	//resetする
	wait(500);
	SPIWrite(0x6b, 0x08);	//Power Management1：Sleep解除、温度センサー使用不可
	wait(10);
	SPIWrite(0x1b, 0x18);	//Gyroscope Measurements：gyroの最大レンジを±2000°に設定(16.4LSB/°/s)
	wait(10);
	SPIWrite(0x6a, 0x10);	//User Control：I2cを無効化する
	wait(10);
	SPIWrite(0x6c, 0xee);	//Power Management2：gyroはz軸のみ、加速度はy軸のみ使用可能
}

void init_INTERFACE(void) {
//以下、インターフェイスLEDの設定
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORT2.DDR.BIT.B3 = 1;
	PORT7.DDR.BIT.B3 = 1;
	PORT7.DDR.BIT.B2 = 1;
	PORT7.DDR.BIT.B5 = 1;
	PORT2.DDR.BIT.B4 = 1;
	PORT2.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
//以下、スイッチの設定
	PORTD.DDR.BIT.B6 = 0;
	PORTD.DDR.BIT.B7 = 0;
	PORTD.ICR.BIT.B6 = 1;
	PORTD.ICR.BIT.B7 = 1;
}
