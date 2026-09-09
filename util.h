/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)?(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sample1.h 2416 2012-09-07 08:06:20Z ertl-hiro $
 */

#ifndef __UTIL_H__
#define __UTIL_H__

// 片方だけ有効にする
#define UTIL_EV3
//#define UTIL_SPIKE
#pragma once

#ifdef UTIL_SPIKE
# include "spikeapi.h"
#else
# include "ev3api.h"
#endif
#include <stdio.h>

/*************************************
  【重要】
  spike_printfは、USBがPCと繋がっていないとそこでハブがUSBの接続待ちになってしまう。
  ※ 一見暴走のように見える。

  そのため、USBを繋がないときには下の#defineを有効にしておき、spike_printfの中身を空にすること。
 *********************************** */
//#define NO_SPIKE_PRINTF

#ifdef UTIL_EV3
// EV3 LCDデバッグ用printf
void ev3_printf(const char *fmt, ...);
// EV3 LCD 行指定版printf
void ev3_printf_locate(unsigned char line, const char *fmt, ...);
#else
// Spike USB-シリアル デバッグ用printf
// #defineNO_SPIKE_PRINTF が有効なときは、カラ関数になる
void spike_printf(const char *fmt, ...);
#endif


/**
 * dly_tsk/tslp_tsk用の時間定義
 * ex) dly_tsk(1*MSEC) -> 1ms sleep
 *     dly_tsk(5*SEC) -> 5sec sleep
 *     tslp_tsk(3*SEC) -> 3sec sleep
 */
# define MSEC 1000
# define USEC 1
# define SEC 1000*1000

// Utils
typedef enum {
  STOP_FREE = 0,
  STOP_BRAKE = 1,
  STOP_NOHOLD = 2
} brake_t;


/********************************
 疑似台形移動（減速をしてより正確に移動）の設定
 #define GIJI_DAIKEI_IDOUを有効にしておくと
 最初の90％を通常速度、残りをパワー20でゆっくり回転角度（回転数）に合わせに行く
 *********************************/
# define GIJI_DAIKEI_IDOU   

/********************************************
 モーターペア関数
 EV3のステアリング・タンク同様に使える。
 停止時のブレーキ（回転角度保持）または惰性回転は、
 ステアリング回転数（角度）と
 ブレーキ関数で指定する。
 *******************************************/

// 最初に登録を行い、モーターペアの接続ポートを指定する。
// （ポートが間違っていると、Spikeの場合は救急車の音が出る）
#ifdef UTIL_EV3
 // EV3 ステアリング用モーター登録関数
void steering_register(motor_port_t mtr1, motor_port_t mtr2);
#endif
#ifdef UTIL_SPIKE
// Spike ステアリング用モーター登録関数
void steering_register(pbio_port_id_t mtr1, pbio_port_id_t mtr2);
#endif

// ステアリング
int steering_rot( int _steer, int _pwr, float rot, brake_t brake);
int steering_degree( int _steer, int _pwr, int degree, brake_t brake);

// ステアリング・オン
int steering_on( int _steer, int _pwr);

// タンク
int tank_rot(int _l_pwr, int _r_pwr, float rot);
int tank_degree(int _l_pwr, int _r_pwr, int deg);

// タンク・オン
int tank_on(int _l_pwr, int _r_pwr);

// ステアリングモーターの停止
int tank_stop(brake_t brake);
#ifndef steering_stop
// steering_stopという関数コールにもできる。
# define steering_stop tank_stop
#endif

/**
 * @fn バッテリー確認、LEDの最初の行に表示する
 * @param なし
 */
void disp_battery(void);


// カラーセンサーの接続設定を行う
int color_sensor_init(void);

#ifdef UTIL_SPIKE
/*
  @fn SPIKE専用 RGB検出版 色チェック
  @param 検出したいポート番号
 */
bool is_red(pbio_port_id_t sensor);
bool is_blue(pbio_port_id_t sensor);

/* PD制御によるライントレースのサンプル 停止条件は色 'R' 'B' のいずれか*/
void inetrace_single(pbio_port_id_t sensor, int speed, char _color);
#endif

#endif