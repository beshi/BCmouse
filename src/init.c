/*
 * init.c
 *
 *  Created on: 2016/02/17
 *      Author: Alex
 */
#include "iodefine.h"
#include "init.h"

void initCMT(void)	//CMT割込の設定
{
	STB.CR4.BIT._CMT = 0;	//CMTスタンバイ解除

	//  (1)コンペアマッチタイマスタートレジスタ（CMSTR）
	CMT.CMSTR.BIT.STR0 = 0;	// ステータスレジスタ　0：カウント停止, 1：カウント開始

	//  (2)コンペアマッチタイマコントロール／ステータスレジスタ（CMCSR）
	CMT0.CMCSR.BIT.CMIE = 1;     //割り込みイネーブル許可
	CMT0.CMCSR.BIT.CKS = 0;     //1/8（分周比は8）
	CMT0.CMCSR.BIT.CMF = 0;     //フラグをクリア
	CMT0.CMCOR = 3125;  //割り込み周期を1msに設定
	INTC.IPRJ.BIT._CMT0 = 12;  //割り込み優先度(15)

	STB.CR4.BIT._CMT = 0;	//CMTスタンバイ解除

	//  (1)コンペアマッチタイマスタートレジスタ（CMSTR）
	/*	 CMT.CMSTR.BIT.STR1 = 0;	// ステータスレジスタ　0：カウント停止, 1：カウント開始

	 //  (2)コンペアマッチタイマコントロール／ステータスレジスタ（CMCSR）
	 CMT1.CMCSR.BIT.CMIE = 1;     //割り込みイネーブル許可
	 CMT1.CMCSR.BIT.CKS = 0;     //1/8（分周比は8）
	 CMT1.CMCSR.BIT.CMF = 0;     //フラグをクリア
	 CMT1.CMCOR = 3125;  //割り込み周期を1msに設定
	 INTC.IPRJ.BIT._CMT1 = 13;  //割り込み優先度(15)*/
}

void initMTU(void) {
	STB.CR4.BIT._MTU2 = 0;  //スタンバイ解除
	MTU2.TSTR.BIT.CST0 = 0;  //スタートレジスタ 0:カウント停止, 1:カウント開始
	MTU2.TSTR.BIT.CST1 = 0;
	MTU20.TCR.BIT.TPSC = 1;  // 1/4
	MTU21.TCR.BIT.TPSC = 1;
	MTU20.TCR.BIT.CCLR = 1;  //TGRB1でコンペアマッチクリアに設定
	MTU21.TCR.BIT.CCLR = 1;
	MTU20.TCR.BIT.CKEG = 0;
	MTU21.TCR.BIT.CKEG = 0;
	MTU20.TIOR.BIT.IOA = 1;
	MTU21.TIOR.BIT.IOA = 1;
	MTU20.TIOR.BIT.IOB = 2;
	MTU21.TIOR.BIT.IOB = 2;
	MTU20.TIER.BIT.TGIEB = 1;
	MTU21.TIER.BIT.TGIEB = 1;
	INTC.IPRD.BIT._MTU20G = 14;
	INTC.IPRD.BIT._MTU21G = 15;
	MTU20.TGRA = 6250;
	MTU21.TGRA = 6250;
	MTU20.TGRB = 10;
	MTU21.TGRB = 10;
	MTU20.TMDR.BIT.MD = 3;
	MTU21.TMDR.BIT.MD = 3;
	PFC.PECRL1.BIT.PE1MD = 1;		//MTUで使う
	PFC.PECRL2.BIT.PE5MD = 1;		//MTUで使う
}

void initAD(void) {
	STB.CR4.BIT._AD0 = 0;
	AD0.ADCR.BIT.ADST = 0;
	AD0.ADCSR.BIT.ADCS = 0;
	AD0.ADCSR.BIT.TRGE = 0;
	AD0.ADCSR.BIT.CKSL = 0;
	AD0.ADCSR.BIT.ADIE = 0;
	AD0.ADCSR.BIT.ADM = 0;
	AD0.ADCSR.BIT.CH = 1;
	AD0.ADCSR.BIT.CH = 1;

	STB.CR4.BIT._AD1 = 0;
	AD1.ADCR.BIT.ADST = 0;
	AD1.ADCSR.BIT.ADCS = 0;
	AD1.ADCSR.BIT.TRGE = 0;
	AD1.ADCSR.BIT.CKSL = 0;
	AD1.ADCSR.BIT.ADIE = 0;
	AD1.ADCSR.BIT.ADM = 0;
	AD1.ADCSR.BIT.CH = 0;
	AD1.ADCSR.BIT.CH = 1;
	AD1.ADCSR.BIT.CH = 2;
	AD1.ADCSR.BIT.CH = 3;
}
void init_sci(void) {

	int baud = 38400;			//ビットレート38400bps
	unsigned char tmp;

	STB.CR3.BIT._SCI1 = 0;			//スタンバイ解除

	PFC.PACRL1.BIT.PA3MD = 1;		//シリアルポートを設定
	PFC.PACRL2.BIT.PA4MD = 1;	//シリアルポートを設定
	SCI1.SCSCR.BYTE = 0x00;		//送受信割り込み禁止

	SCI1.SCSMR.BYTE = 0; /* ASYNC、8bit、Parity-NONE、Stop-1、Clk = tmp	*/
	tmp = (unsigned char) (25000000 / 32 / baud) - 1;
	SCI1.SCBRR = tmp;			//ビットレート設定
	SCI1.SCSCR.BIT.TE = 1;		//送信許可
	SCI1.SCSCR.BIT.RE = 1;		//受信許可
}



