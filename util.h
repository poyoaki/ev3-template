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

#pragma once
#include "ev3api.h"

#include <stdio.h>

/**
 * dly_tsk/tslp_tsk用の時間定義
 * ex) dly_tsk(1*MSEC) -> 1ms sleep
 *     dly_tsk(5*SEC) -> 5sec sleep
 *     tslp_tsk(3*SEC) -> 3sec sleep
 */
#define MSEC 1000
#define USEC 1
#define SEC 1000000

// Utils

typedef enum {
  STOP_FREE = 0,
  STOP_BRAKE = 1,
  STOP_NOHOLD = 2
} brake_t;


// LCDデバッグ用printf
void ev3_printf(const char *fmt, ...);

// 行指定版printf
void ev3_printf_locate(unsigned char line, const char *fmt, ...);

// ステアリング用モーター登録関数
void ev3_steering_register(motor_port_t mtr1, motor_port_t mtr2);

// 回転数のステアリング
int ev3_steering_rot(int _pwr, int _steer, float rot, brake_t brake);

// ステアリング・オン
int ev3_steering_on(int _pwr, int _steer);

// 回転数のタンク
int ev3_tank_rot(int _l_pwr, int _r_pwr, float l_rot, float r_rot);

// タンク・オン
int ev3_tank_on(int _l_pwr, int _r_pwr);

// ステアリングモーターの停止
int ev3_tank_stop(brake_t brake);
#ifndef ev3_steering_stop
// ev3_steering_stopという関数コールにもできる。
# define ev3_steering_stop ev3_tank_stop
#endif



#endif