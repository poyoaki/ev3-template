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

// (1) ステアリング用モーターを登録する
  if (ev3_motor_config(EV3_PORT_B , MEDIUM_MOTOR) != E_OK) {
    ev3_printf("PORT LEFT Conn ERR");
    return;
  }
  if (ev3_motor_config(EV3_PORT_C , MEDIUM_MOTOR) != E_OK) {
    ev3_printf("PORT RIGHT Conn ERR");
    return;
  }

  steering_register(EV3_PORT_B, EV3_PORT_C);
  // カラーセンサーの登録
  color_sensor_init();
  /*
   * ステアリングユーティリティーのテストプログラム
   */
  
  // (2) 動作テスト
#if 1
  // 直進、左折、後退
  while(1){
  steering_rot(0, 50, 1, STOP_NOHOLD);
//  tslp_tsk(500*MSEC);
//  tank_rot(-50, 50, 0.5);
//  tslp_tsk(1*SEC);
//  steering_rot(0, 50, -1, STOP_BRAKE);
    dly_tsk(1*SEC);
  }
#endif
#if 0
  // 右大回り、左大回り、斜め右ステア、斜め左ステア
  tank_rot(50, 0, 1);
  tslp_tsk(500*MSEC);
  tank_rot(0, 50, 1);
  tslp_tsk(500*MSEC);
  steering_rot(20, 50, 2, STOP_FREE);
  steering_rot(-20, 70, 2, STOP_NOHOLD);
#endif

#if 0
  // 黒線ライントレースのテスト
  while(1) 
  {
    ref = ev3_color_sensor_get_reflect(EV3_PORT_3);
    ev3_printf_locate(0, "ref:%03d", ref);
    if (ref < 50) {
      steering_on(30, 30);
    }
    else {
      steering_on(-20, 30);
    }
    if (ev3_color_sensor_get_color(EV3_PORT_3) == COLOR_RED)
      break;
  }
  steering_stop(STOP_NOHOLD);
  
#endif

#if 0
  // 赤線ライントレースのテスト
  while(1) {
    rgb_raw_t raw;
    ev3_printf_locate(0, "r%03d g%03d b%03d", raw.r, raw.g, raw.b);
    ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &raw);
    if (raw.g < 230) {
      steering_on(30, 30);
    }
    else {
      steering_on(-20, 30);
    }
  }
  steering_stop(STOP_NOHOLD);
  
#endif

  return;
}

