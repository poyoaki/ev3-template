/**
 * @title EV3 テンプレートプログラム
 * This is a program used to test the whole platform.
 */

#include "ev3api.h"
#include "app.h"
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include "util.h"



/**
 * @name EV3 メインタスク
 */
void main_task(intptr_t unused) 
{
   ev3_lcd_set_font(EV3_FONT_MEDIUM) ;
   ev3_lcd_draw_string ("Template Program", 10, 10);

  // printfのテストプログラム
#if 0
  char tmp[1000];
  memset(tmp, 0, sizeof(tmp));
  for (;;){
    strcat(tmp, "aa");
    ev3_printf(tmp);
    tslp_tsk(500*MSEC);
  }
#endif

  /*
   * ステアリングユーティリティーのテストプログラム
   */
  
  // (1) ステアリング用モーターを登録する
  if (ev3_motor_config(EV3_PORT_B , LARGE_MOTOR) != E_OK) {
    ev3_lcd_draw_string ("PORT B Conn ERR", 0, 20);
    return;
  }
  if (ev3_motor_config(EV3_PORT_C , LARGE_MOTOR) != E_OK) {
    ev3_lcd_draw_string ("PORT C Conn ERR", 0, 20);
    return;
  }
  ev3_steering_register(EV3_PORT_B, EV3_PORT_C);

  // カラーセンサーの登録
  ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR);

  // (2) 動作テスト
#if 0
  // 直進、左折、後退
  ev3_steering_rot(50, 0, 1, STOP_NOHOLD);
  tslp_tsk(500*MSEC);
  ev3_tank_rot(-50, 50, 0.5, 0.5);
  tslp_tsk(1*SEC);
  ev3_steering_rot(50, 0, -1, true);
#endif
#if 0
  // 右大回り、左大回り、斜め右ステア、斜め左ステア
  ev3_tank_rot(50, 0, 1, 0);
  tslp_tsk(500*MSEC);
  ev3_tank_rot(0, 50, 0, 1);
  tslp_tsk(500*MSEC);
  ev3_steering_rot(50, 20, 2, STOP_FREE);
  ev3_steering_rot(70, -20, 2, STOP_NOHOLD);
#endif
#if 0
  // ライントレースのテスト
  while(1) {
    if (ev3_color_sensor_get_reflect(EV3_PORT_3) < 50) {
      ev3_steering_on(40, 20);
    }
    else {
      ev3_steering_on(40, -20);
    }
    if (ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_RED)
      break;
  }
  ev3_steering_stop(STOP_NOHOLD);
  
#endif
  
  return;
}

