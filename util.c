/**
 * @title EV3 ユーティリティープログラム
 * This is a program for motoro steering.
 */

#include "ev3api.h"
#include "app.h"
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "util.h"

static motor_port_t _MtrL, _MtrR;

static int _Lcd_y;
static bool_t _Lcd_init = false;

void ev3_printf_init(void)
{
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  _Lcd_y = 0;
  _Lcd_init = true;
}
/**
 * @fn 178x128サイズのLCDに、さみだれ式に文字を表示する。1コール毎に改行する。
 * フォントサイズは
 * @param printfと同じ
 * font_small: 6x8  font_medium: 10x16 -> 最大17文字。
 */
void ev3_printf(const char *fmt, ...)
{
  char tmp[200];
  va_list args;
  int len;
  va_start(args, fmt);
  len = vsprintf(tmp, fmt, args);
  va_end(args);
  if (len < 18){
    int i;
    for (i=len; i<18; i++) {
      tmp[i] = ' ';
    }
    tmp[i] = '\0';
  }
    
  if (! _Lcd_init) {
    ev3_printf_init();
  }
  
  ev3_lcd_draw_string (tmp, 0, _Lcd_y);
  _Lcd_y += 16;
  if (_Lcd_y > 127-10)
    _Lcd_y = 0;
}

/**
 * @fn 178x128サイズのLCDに、行指定で文字を表示する。
 * フォントサイズは
 * @param 行数（0 to 10), printfと同じ
 * font_small: 6x8  font_medium: 10x16 -> 最大17文字。
 */
void ev3_printf_locate(unsigned char line, const char *fmt, ...)
{
  char tmp[200];
  va_list args;
  int len;
  if (line > 10)
    line = 10;
  
  va_start(args, fmt);
  len = vsprintf(tmp, fmt, args);
  va_end(args);
  
  if (len < 18){
    int i;
    for (i=len; i<18; i++) {
      tmp[i] = ' ';
    }
    tmp[i] = '\0';
  }

  if (! _Lcd_init) {
    ev3_printf_init();
  }
  
  ev3_lcd_draw_string (tmp, 0, line*16);
}


/**
 * @fn －100～100に数値を補正する
 */
int limit_100(int val)
{
  int tmp = val;
  if (tmp > 100)
    tmp = 100;
  if (tmp < -100)
    tmp = -100;
  return tmp;
}

/**
 * @fn ステアリングに使うモーター2コを登録する
 * @param 左モーターポート、右モーターポート
 */
void ev3_steering_register(motor_port_t mtr1, motor_port_t mtr2)
{
  _MtrL = mtr1;
  _MtrR = mtr2;
}

/**
 * @fn ステアリング関数(回転数)
 * @param パワー(-100-100）、ステアリング、回転数, ブレーキ
 * @detail モーター回転が終わるまでリターンしない。ステアリングは0にすると直進する。
 */  
int ev3_steering_rot(int _pwr, int _steer, float rot, brake_t brake)
{
  int l_deg, r_deg;  // 回転角度に変換
  int ercd;
  int l_pwr, r_pwr;
  int pwr, steer;
  pwr = limit_100(_pwr);
  steer = limit_100(_steer);
  l_pwr = pwr - steer/2;
  r_pwr = pwr + steer/2;

  l_deg = (int)(360.0*rot);
  r_deg = l_deg;

  // パワーマイナスの場合は、API未対応なので回転角度をマイナスにする
  if (l_pwr < 0){
    l_pwr *= -1;
    l_deg *= -1;
  }
  if (r_pwr < 0){
    r_pwr *= -1;
    r_deg *= -1;
  }
  
  // パワーが大きいほうの動作が終わったら、直ちに回転を止める
  if (abs(l_pwr) < abs(r_pwr)) {
    if (l_deg < 0) {
      ev3_motor_set_power(_MtrL, -l_pwr);
    }
    else {
      ev3_motor_set_power(_MtrL, l_pwr);
    }
    ercd = ev3_motor_rotate(_MtrR, r_deg, r_pwr, true);
  } else {
    if (r_deg < 0) {
      ev3_motor_set_power(_MtrR, -r_pwr);
    }
    else {
      ev3_motor_set_power(_MtrR, r_pwr);
    }
    ercd = ev3_motor_rotate(_MtrL, l_deg, l_pwr, true);
  }

  // ホールドしないブレーキ = 0.2秒後にロックを外す
  if (brake == STOP_NOHOLD) {
    ev3_motor_stop(_MtrR, true);
    ev3_motor_stop(_MtrL, true);
    tslp_tsk(200*MSEC);
    ev3_motor_stop(_MtrR, false);
    ev3_motor_stop(_MtrL, false);
  }
  else {
    // STOP_BREAKのときはタイヤがロックされる。
    ev3_motor_stop(_MtrR, brake);
    ev3_motor_stop(_MtrL, brake);
  }
  return ercd;
}

/**
 * @fn ステアリング(ON))
 * @param パワー
 * @param ステアリング（単純にパワーの差となる）
 */
int ev3_steering_on(int _pwr, int _steer)
{
  int l_pwr, r_pwr;
  int pwr, steer;
  int ercd;
  pwr = limit_100(_pwr);
  steer = limit_100(_steer);
  // ev3_motor_steer()は使わず、タンクに置きかえる
  
  l_pwr = pwr - steer/2;
  r_pwr = pwr + steer/2;
  ercd =  ev3_tank_on(l_pwr, r_pwr);
  return ercd;
}

/**
 * @fn タンク関数(回転数)
 * @param 左パワー(-100-100、右パワー、左回転数、右回転数
 * @detail モーター回転が終わるまでリターンしない。
 */  
int ev3_tank_rot(int _l_pwr, int _r_pwr, float l_rot, float r_rot)
{
  int ercd;
  int l_deg, r_deg; // 回転角度に変換
  int l_pwr, r_pwr;
  l_pwr = limit_100(_l_pwr);
  r_pwr = limit_100(_r_pwr);

  l_deg = (int)(360.0*l_rot);
  r_deg = (int)(360.0*r_rot);

  // パワーマイナスの場合は、API未対応なので回転角度をマイナスにする
  if (l_pwr < 0){
    l_pwr *= -1;
    l_deg *= -1;
  }
  if (r_pwr < 0){
    r_pwr *= -1;
    r_deg *= -1;
  }
  // 回転が長い方の動作が終わるまで待つ
  if (abs(l_deg) < abs(r_deg)) {
    ercd = ev3_motor_rotate(_MtrL, l_deg, l_pwr, false);
    ercd = ev3_motor_rotate(_MtrR, r_deg, r_pwr, true);
  }
  else {
    ercd = ev3_motor_rotate(_MtrR, r_deg, r_pwr, false);
    ercd = ev3_motor_rotate(_MtrL, l_deg, l_pwr, true);
  }
  return ercd;
}

/**
 * @fn タンク関数(On)
 * @param 左パワー(-100-100)、右パワー
 */  
int ev3_tank_on(int _l_pwr, int _r_pwr)
{
  int l_pwr, r_pwr;
  l_pwr = limit_100(_l_pwr);
  r_pwr = limit_100(_r_pwr);
  
  ev3_motor_set_power(_MtrL, l_pwr);
  return ev3_motor_set_power(_MtrR, r_pwr);
}

/**
 * @fne タンク関数(停止)
 * @param ブレーキタイプ
 */
int ev3_tank_stop(brake_t brake)
{
  bool_t tmp = brake;
  ER ercd;
  if (brake == STOP_NOHOLD)
    tmp = true;
  ev3_motor_stop(_MtrL, tmp);
  ercd = ev3_motor_stop(_MtrR, tmp);

  if (brake == STOP_NOHOLD) {
    tslp_tsk(200*MSEC);
    ev3_motor_stop(_MtrL, false);
    ercd = ev3_motor_stop(_MtrR, false);
  }
  return ercd;
}

