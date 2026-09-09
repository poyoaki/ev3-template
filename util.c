/**
 * @title EV3/Spike  ユーティリティープログラム
 * This is a program for motor steering/line-trace/battery monitor.
 */

#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "util.h"
#ifdef UTIL_SPIKE
# include "kernel_cfg.h"
# include <serial/newlib.h>
#endif

#ifdef UTIL_SPIKE
#include "spikeapi.h"
static pbio_port_id_t _MtrL, _MtrR;
pup_device_t *colorC1, *colorC2;
pup_motor_t *motorL, *motorR;
static int _Serial_init = false;
static FILE *fp;

// 本体はapp.c, 各ポートにつながっている物
extern pbio_port_id_t color_sensor1_port, color_sensor2_port, left_motor_port,
  right_motor_port, arm_motor_port;

/* USBシリアルコンソールを初期化する。 */
void spike_printf_init(void)
{
# ifndef NO_SPIKE_PRINTF  
  fp = serial_open_newlib_file(SIO_USB_PORTID);  
  _Serial_init = true;
#endif
}
/**
 * @fn LEGO Spike Only: USBシリアルコンソールに出力する。200文字制限あり
 * @param printfと同じ
  */
void spike_printf(const char *fmt, ...)
{
# ifndef NO_SPIKE_PRINTF
  char tmp[200];
  va_list args;
  memset(tmp, 0, sizeof(tmp));
  va_start(args, fmt);
  vsprintf(tmp, fmt, args);
  va_end(args);
    
  if (! _Serial_init) {
    spike_printf_init();
  }
  fputs(tmp, fp);
# endif
}
#else
//UTIL_EV3
/************************************
 EV3 declaration
 EV3用の情報保存用の変数などを定義
 ***********************************/

// カラーセンサーの参照用
int lineRport;
int lineLport;
int colorSideport;

static motor_port_t _MtrL, _MtrR;

static int _Lcd_y;
static bool_t _Lcd_init = false;

/* EV3 Printf用に初期化*/
void ev3_printf_init(void)
{
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  _Lcd_y = 0;
  _Lcd_init = true;
}
/**
 * @fn EV3 Only: 178x128サイズのLCDに、さみだれ式に文字を表示する。1コール毎に改行する。
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
 * @fn EV3 Only: 178x128サイズのLCDに、行指定で文字を表示する。
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
#endif

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
 * @fn valを[-abs(lim), abs(lim)]の範囲に丸める
 */
int limit_abs(int val, int lim)
{
  int l = abs(lim);
  if (val > l)
    return l;
  if (val < -l)
    return -l;
  return val;
}

#ifdef UTIL_SPIKE
/*******************************************
 P control gains used to converge the wheel
 angle onto the target after the coarse (bang-bang)
 approach phase. Tune P_KP / P_MIN_PWR on the
 actual robot to hit +-1 degree.
  P_KP        : proportional gain, [deg] error -> [%] power
  P_MIN_PWR   : minimum power needed to overcome
                the robot's own weight/friction
  P_TOLERANCE : error considered "arrived" [deg]
 *******************************************/
# define P_KP        0.5f
# define P_MIN_PWR   15
# define P_TOLERANCE 1
# define P_TIMEOUT   (1*SEC)  // shuusoku shinai baai no anzen timeout [us]

/* Convert a remaining-angle error into a motor power command.
   Saturates at +-max_pwr, and is floored at +-P_MIN_PWR so the
   motor actually moves against the robot's own weight. */
static int p_power(int err, int max_pwr)
{
  int pwr = (int)(P_KP * err);
  int limit = abs(max_pwr);
  if (pwr > limit)
    pwr = limit;
  else if (pwr < -limit)
    pwr = -limit;
  else if (pwr > 0 && pwr < P_MIN_PWR)
    pwr = P_MIN_PWR;
  else if (pwr < 0 && pwr > -P_MIN_PWR)
    pwr = -P_MIN_PWR;
  return pwr;
}

/* Servo motorL/motorR onto target_l/target_r [deg] (signed, relative
   to the last pup_motor_reset_count()) using P control, then apply
   the requested brake mode. max_l_pwr/max_r_pwr are the powers that
   were originally commanded (used as the saturation limit, and their
   sign is irrelevant since target_l/target_r already carry direction). */
static void p_stop2(int target_l, int max_l_pwr, int target_r, int max_r_pwr, brake_t brake)
{
  int l_err, r_err;
  SYSTIM t0, t1;
  get_tim(&t0);
  do {
    l_err = target_l - pup_motor_get_count(motorL);
    r_err = target_r - pup_motor_get_count(motorR);
    pup_motor_set_power(motorL, p_power(l_err, max_l_pwr));
    pup_motor_set_power(motorR, p_power(r_err, max_r_pwr));
    get_tim(&t1);
  } while ((abs(l_err) > P_TOLERANCE || abs(r_err) > P_TOLERANCE) && (SYSTIM)(t1 - t0) < P_TIMEOUT);

  spike_printf("p_stop2 err L:%d R:%d\n", l_err, r_err);

  if (brake == STOP_FREE) {
    pup_motor_stop(motorL);
    pup_motor_stop(motorR);
  } else {
    pup_motor_brake(motorL);
    pup_motor_brake(motorR);
    if (brake == STOP_BRAKE) {
      pup_motor_hold(motorL);
      pup_motor_hold(motorR);
    }
  }
}
#endif

/**
 * @fn ステアリングに使うモーター2コを登録する
 * @param 左モーターポート、右モーターポート
 */
#ifdef UTIL_SPIKE
void steering_register(pbio_port_id_t mtr1, pbio_port_id_t mtr2)
#else
void steering_register(motor_port_t mtr1, motor_port_t mtr2)
#endif
{
  _MtrL = mtr1;
  _MtrR = mtr2;
#ifdef UTIL_SPIKE
  motorL = pup_motor_get_device(_MtrL);
  motorR = pup_motor_get_device(_MtrR);
  pup_motor_setup(motorL, PUP_DIRECTION_COUNTERCLOCKWISE, true); // 左は反時計回りが正
  pup_motor_setup(motorR, PUP_DIRECTION_CLOCKWISE, true);// 右は時計回りが正
#endif
#if 0
// (1) ステアリング用モーターを登録する
  if (ev3_motor_config(_MtrL , MEDIUM_MOTOR) != E_OK) {
    ev3_printf("PORT LEFT Conn ERR");
    return;
  }
  if (ev3_motor_config(_MtrR , MEDIUM_MOTOR) != E_OK) {
    ev3_printf("PORT RIGHT Conn ERR");
    return;
  }
#endif
}

/**
 * @fn ステアリング関数(回転数)
 * @param パワー(-100-100）、ステアリング、回転数, ブレーキ
 * @detail モーター回転が終わるまでリターンしない。ステアリングは0にすると直進する。
 */  
int steering_rot( int _steer, int _pwr, float rot, brake_t brake)
{
  int deg;  // 回転角度に変換
  deg = (int)(360.0*rot);
  return steering_degree(_steer, _pwr, deg, brake);
}

/**
 * @fn ステアリング関数(角度指定)
 * @param パワー(-100-100）、ステアリング、角度, ブレーキ
 * @detail モーター回転が終わるまでリターンしない。ステアリングは0にすると直進する。
 */  
int steering_degree(int _steer, int _pwr, int deg, brake_t brake)
{
  int ercd;
  int l_pwr, r_pwr;
  int pwr, steer, diff;
  int abs_degree, target_l, target_r;

#ifdef UTIL_EV3
  int l_deg, r_deg;
  // Mモーターでステアリングを行う場合、左が反時計、右が時計回りになるのを考慮する
  if (ev3_motor_get_type(_MtrL) == MEDIUM_MOTOR)
    l_deg = -deg;
  else
    l_deg = deg;
  r_deg = deg;
#endif
  pwr = limit_100(_pwr);
  steer = limit_100(_steer);

  // steer=0:直進 / steer=±50:片輪停止(ある輪を軸に旋回) / steer=±100:その場でスピン(両輪逆回転、同パワー)
  // 減算のみで求める。steer>=0なら右輪、steer<0なら左輪をpwrからdiff分だけ減らす。
  diff = pwr * abs(steer) / 50;
  if (steer >= 0) {
    l_pwr = pwr;
    r_pwr = limit_abs(pwr - diff, pwr);
  } else {
    r_pwr = pwr;
    l_pwr = limit_abs(pwr - diff, pwr);
  }
  
#ifdef UTIL_EV3
  // EV3: パワーマイナスの場合は、API未対応なので回転角度をマイナスにする
  if (l_pwr < 0){
    l_pwr *= -1;
    l_deg *= -1;
  }
  if (r_pwr < 0){
    r_pwr *= -1;
    r_deg *= -1;
  }
#endif

  abs_degree = abs(deg);
#ifdef UTIL_SPIKE
  // 回転角度のリセット
  pup_motor_reset_count(motorL);
  pup_motor_reset_count(motorR);
  // 各輪の目標角度（符号付き）。パワー0の輪は0度＝その場ホールド。
  target_l = (l_pwr > 0) ? abs_degree : (l_pwr < 0) ? -abs_degree : 0;
  target_r = (r_pwr > 0) ? abs_degree : (r_pwr < 0) ? -abs_degree : 0;

  spike_printf("steering_deg %d pow:%d(%d %d) deg:%d (%d %d) %d\n",steer, pwr, l_pwr, r_pwr, deg,target_l, target_r,  brake);
#endif

  // 回転の規則
  // （1）ステアリング角度が0のとき＝パワーが同じ時は、両方の角度（エンコーダー）が指定角度になるまで待つ。
  // （2）ステアリング角度がそれ以外＝パワーが異なるなら、どちらかの角度が指定の角度になったらモーター動作を止める。

  //（1）ステアリング角度が0のとき＝パワーが同じ時は、両方の角度（エンコーダー）が指定角度になるまで待つ。
  if (steer == 0) {
#ifdef UTIL_SPIKE
    bool stop_l=false, stop_r=false;
    ercd = pup_motor_set_power(motorL, l_pwr);
    ercd = pup_motor_set_power(motorR, r_pwr);
    while (stop_l == false || stop_r == false)
    {
      if (stop_l == false && abs(pup_motor_get_count(motorL)) >= abs_degree) {
        pup_motor_brake(motorL);
        stop_l = true;
      }
      if (stop_r == false && abs(pup_motor_get_count(motorR)) >= abs_degree) {
        pup_motor_brake(motorR);
        stop_r = true;
      }
    }
    // 後処理。P制御で目標角度(±1度)まで追い込む。
    p_stop2(target_l, l_pwr, target_r, r_pwr, brake);
#else
    ercd = ev3_motor_rotate(_MtrL, l_deg, l_pwr, false);
    ercd = ev3_motor_rotate(_MtrR, r_deg, r_pwr, true); // ブロッキング：Rモーターが指定角度動くまでここに留まる
#endif
  }
  else {
  // (2) 先に指定角度動ききったモーターがいたら、直ちに回転を止める
#ifdef UTIL_SPIKE
    bool stop=false;
    ercd = pup_motor_set_power(motorL, l_pwr);
    ercd = pup_motor_set_power(motorR, r_pwr);
    while (stop == false)
    {
      if (abs(pup_motor_get_count(motorL)) >= abs_degree ||
          abs(pup_motor_get_count(motorR)) >= abs_degree )
      {
        pup_motor_brake(motorL);
        pup_motor_brake(motorR);
        stop = true;
      }
    }
    // 後処理。P制御で両輪を目標角度(±1度)まで追い込む。
    p_stop2(target_l, l_pwr, target_r, r_pwr, brake);
#else // EV3
    int stop = false;
    ev3_motor_reset_counts(_MtrL);
    ev3_motor_reset_counts(_MtrR);
    ercd = ev3_motor_steer(_MtrL, _MtrR, pwr, steer );
    while (stop == false)
    {
      if (abs(ev3_motor_get_counts(_MtrL)) > abs_degree ||
          abs(ev3_motor_get_counts(_MtrR)) > abs_degree )
      {
        if (brake == STOP_BRAKE){
          ev3_motor_stop(_MtrL, true);
          ev3_motor_stop(_MtrL, true);
        }
        else {
          ev3_motor_stop(_MtrL, false);
          ev3_motor_stop(_MtrL, false);
        }
        stop = true;
      }
    }
#endif
  }
  return ercd;
}

/**
 * @fn ステアリング(ON))
 * @param パワー
 * @param ステアリング（単純にパワーの差となる）
 */
int steering_on( int _steer, int _pwr)
{
  int l_pwr, r_pwr;
  int pwr, steer, diff;
  int ercd;
  pwr = limit_100(_pwr);
  steer = limit_100(_steer);
  // タンクに置きかえる（steer=0:直進 / ±50:片輪停止 / ±100:その場スピン）。減算のみで求める。
  diff = pwr * abs(steer) / 50;
  if (steer >= 0) {
    l_pwr = pwr;
    r_pwr = limit_abs(pwr - diff, pwr);
  } else {
    r_pwr = pwr;
    l_pwr = limit_abs(pwr - diff, pwr);
  }
  ercd =  tank_on(l_pwr, r_pwr);
  return ercd;
}

/**
 * @fn タンク関数(回転数)
 * @param 左パワー(-100-100、右パワー、左パワー、回転数
 * @detail モーター回転が終わるまでリターンしない。
 */  
int tank_rot(int _l_pwr, int _r_pwr, float rot)
{
  int deg; // 回転角度に変換
  deg = (int)(360.0*rot);
  return tank_degree(_l_pwr, _r_pwr, deg);
}

/**
 * @fn タンク関数(角度指定)
 * @param 左パワー(-100-100、右パワー、角度
 * @detail モーター回転が終わるまでリターンしない。
 * Note: #define GIJI_DAIKEI_IDOUを有効にすると、停止するときに減速して擬似的な台形スピード移動になる
 */  
int tank_degree(int _l_pwr, int _r_pwr, int degree)
{
  int ercd = E_OK;
  int l_pwr, r_pwr;
#ifdef UTIL_EV3
  int l_deg, r_deg;
  l_deg = degree;
  r_deg = degree;
#else
  int abs_degree = abs(degree);
# ifdef GIJI_DAIKEI_IDOU
  int abs_degree2 = abs(degree)*9/10;

# else
  int abs_degree2 = abs_degree;
# endif
  int target_l, target_r;
#endif

  // 動作しない設定の時はすぐリターン
  if (degree == 0 || (_l_pwr == 0 && _r_pwr == 0))
  {
#ifdef UTIL_SPIKE
    spike_printf("tank_degree: parameter zero\n");
#else
    ev3_printf("tank_degree: parameter zero\n");
#endif
    return E_PAR;
  }

  l_pwr = limit_100(_l_pwr);
  r_pwr = limit_100(_r_pwr);

#ifdef UTIL_EV3
  // EV3のみ、パワーマイナスの場合はAPI未対応なので、回転角度をマイナスにする
  if (l_pwr < 0){
    l_pwr *= -1;
    l_deg *= -1;
  }
  if (r_pwr < 0){
    r_pwr *= -1;
    r_deg *= -1;
  }
#endif

#ifdef UTIL_SPIKE
  // 回転角度のリセット
  pup_motor_reset_count(motorL);
  pup_motor_reset_count(motorR);
  // 各輪の目標角度（符号付き）。パワー0の輪は0度＝その場ホールド。
  target_l = (l_pwr > 0) ? abs_degree : (l_pwr < 0) ? -abs_degree : 0;
  target_r = (r_pwr > 0) ? abs_degree : (r_pwr < 0) ? -abs_degree : 0;
#endif

  // 回転の規則
  // （1）パワーの絶対値が同じ時は、両方の角度（エンコーダー）が指定角度になるまで待つ。
  // （2）パワーの絶対値があわない時＝どちらかの角度が先に指定の角度になったらモーター動作を止める。

  // （1）パワーの絶対値が同じ時は、両方の角度（エンコーダー）が指定角度になるまで待つ。
  if (abs(l_pwr) == abs(r_pwr))
  {
#ifdef UTIL_SPIKE

    bool stop=false;
    ercd = pup_motor_set_power(motorL, l_pwr);
    ercd = pup_motor_set_power(motorR, r_pwr);

    // 疑似台形移動の場合は、ここでは指定角度の90％までを指定されたスピードで動かす。
    while (stop != true)
    {
      if (abs(pup_motor_get_count(motorL)) >= abs_degree2 ||
          abs(pup_motor_get_count(motorR)) >= abs_degree2) {
        stop = true;
      }
    }
    pup_motor_stop(motorL);
    pup_motor_stop(motorR);

    // 後処理。P制御で目標角度(±1度)まで追い込む。
    p_stop2(target_l, l_pwr, target_r, r_pwr, STOP_BRAKE);
#else
    // EV3の場合は、APIで角度を指定できるのでそのまま使う。
    // 右輪はブロッキング処理にしてこの関数がすぐに抜けないようにする。
    ercd = ev3_motor_rotate(_MtrL, l_deg, l_pwr, false);
    ercd = ev3_motor_rotate(_MtrR, r_deg, r_pwr, true);
#endif
  }
  // （2）パワーの絶対値があわない時＝どちらかの角度が先に指定の角度になったらモーター動作を止める。
  else {
#ifdef UTIL_SPIKE
    bool stop=false;
    // 特に、どちらかのパワーが0のときはそのモーターをホールドしてずれないようにしておく。
    if (l_pwr == 0)
      pup_motor_hold(motorL);
    else
      pup_motor_set_power(motorL, l_pwr);
    if (r_pwr == 0)
      pup_motor_hold(motorR);
    else
      pup_motor_set_power(motorR, r_pwr);
    while (stop == false)
    {
      if (abs(pup_motor_get_count(motorL)) >= abs_degree2 ||
          abs(pup_motor_get_count(motorR)) >= abs_degree2)
      {
        pup_motor_brake(motorL);
        pup_motor_brake(motorR);
        stop = true;
      }
    }
    // 後処理。P制御で両輪を目標角度(±1度)まで追い込む。
    p_stop2(target_l, l_pwr, target_r, r_pwr, STOP_BRAKE);
#else
    ercd = ev3_motor_rotate(_MtrR, r_deg, r_pwr, false);
    ercd = ev3_motor_rotate(_MtrL, l_deg, l_pwr, true);
#endif
  }
  return ercd;
}

/**
 * @fn タンク関数(On)
 * @param 左パワー(-100-100)、右パワー
 */  
int tank_on(int _l_pwr, int _r_pwr)
{
  int l_pwr, r_pwr;
  l_pwr = limit_100(_l_pwr);
  r_pwr = limit_100(_r_pwr);
#ifdef UTIL_SPIKE
  pup_motor_set_power(motorL, l_pwr);
  return pup_motor_set_power(motorR, r_pwr);
#else
  ev3_motor_set_power(_MtrL, l_pwr);
  return ev3_motor_set_power(_MtrR, r_pwr);
#endif
}

/**
 * @fne タンク関数(停止)
 * @param ブレーキタイプ
 */
int tank_stop(brake_t brake)
{
  bool_t tmp = brake;
  ER ercd;
  if (brake != STOP_BRAKE)
    tmp = true;

  if (tmp){
#ifdef UTIL_SPIKE
    pup_motor_stop(motorL);
    ercd = pup_motor_stop(motorR);
#else
    ev3_motor_stop(_MtrL, tmp);
    ercd = ev3_motor_stop(_MtrR, tmp);
#endif
    return ercd;
  }
#ifdef UTIL_SPIKE
  pup_motor_brake(motorL);
  ercd = pup_motor_brake(motorR);
#else
  ev3_motor_stop(_MtrL, true);
  ercd = ev3_motor_stop(_MtrR, true);
#endif  
  return ercd;
}

/**
 * @fne バッテリー確認、LEDの最初の行に表示する
 * @param なし
 */
void disp_battery(void)
{
  int ma, mv;
#ifdef UTIL_SPIKE
  ma = hub_battery_get_current();
  mv = hub_battery_get_current();
  spike_printf("%d[mA] %d[mV]\r", ma, mv);
#else
  ma = ev3_battery_current_mA();
  mv = ev3_battery_voltage_mV();
  ev3_printf_locate(0, "%d[mA] %d[mV]", ma, mv);
#endif
}

/**
 * @fn カラーセンサーの接続設定を行う。この関数の中身を変えること
 * @param なし
 */
extern 
int color_sensor_init(void)
{
#ifdef UTIL_SPIKE
  colorC1 = pup_color_sensor_get_device(color_sensor1_port);
#else  
  if (ev3_sensor_config(EV3_PORT_3, COLOR_SENSOR) != E_OK) 
    ev3_printf("Color-S 3 ERR");

  if (ev3_sensor_config(EV3_PORT_4, COLOR_SENSOR) != E_OK) 
    ev3_printf("Color-S 4 ERR");

  lineRport = EV3_PORT_3;
  lineLport = EV3_PORT_4;

  if (ev3_sensor_config(lineRport, COLOR_SENSOR) != E_OK)
    ev3_printf("Color-S 3 ERR");
  if (ev3_sensor_config(lineLport, COLOR_SENSOR) != E_OK)
    ev3_printf("Color-S 4 ERR");

    /*
  colorSideport= EV3_PORT_2;  
  if (ev3_sensor_config(colorSideport, COLOR_SENSOR) != E_OK)
    ev3_printf("Color-S 2 ERR");
  */
#endif
  return E_OK;
}

 #ifdef UTIL_SPIKE
 /*
  @fn SPIKE専用 RGB検出版 赤色チェック
  @param 検出したいポート番号
 */
bool is_red(pbio_port_id_t sensor)
{
  pup_color_rgb_t rgb;
  double all, r_rate, g_rate, b_rate;
  if (sensor == color_sensor1_port)
    rgb = pup_color_sensor_rgb(colorC1);
  else if (sensor == color_sensor2_port)
  {
//    rgb = pup_color_sensor_rgb(colorC2);
    spike_printf("C2 not assign\n");
  }
  else
  {
    spike_printf("CS not assign\n");
    return false;
  }
  all = rgb.r + rgb.g + rgb.b;
  r_rate = rgb.r / all;
  g_rate = rgb.g / all;
  b_rate = rgb.b / all;
//  spike_printf("R:%d G:%d B:%d\n", rgb.r, rgb.g, rgb.b);
//  spike_printf("Rl:%lf Gl:%lf Bl:%lf\n", r_rate, g_rate, b_rate);

  // 単純な判別方法として、赤みが他の色みよりも2倍強ければ赤と見なす。
  if (r_rate/2 > g_rate && r_rate/2 > b_rate)
    return true;
  else
    return false;
} 

/*
  @fn SPIKE専用 RGB検出版 青色チェック
  @param 検出したいポート番号
 */
bool is_blue(pbio_port_id_t sensor)
{
  pup_color_rgb_t rgb;
  double all, r_rate, g_rate, b_rate;
  if (sensor == color_sensor1_port)
    rgb = pup_color_sensor_rgb(colorC1);
  else 
    if (sensor == color_sensor2_port)
    {
//    rgb = pup_color_sensor_rgb(colorC2);
    spike_printf("C2 not assign\n");
    }
    else
    {
      spike_printf("CS not assign\n");
      return false;
    }
  all = rgb.r + rgb.g + rgb.b;
  r_rate = rgb.r / all;
  g_rate = rgb.g / all;
  b_rate = rgb.b / all;
//  spike_printf("R:%d G:%d B:%d\n", rgb.r, rgb.g, rgb.b);
//  spike_printf("Rl:%lf Gl:%lf Bl:%lf\n", r_rate, g_rate, b_rate);

  // 単純な判別方法として、青みが赤みよりの2倍、緑みよりも青みが少しでも強ければ青と見なす。
  if (b_rate/2 > r_rate && b_rate > g_rate)
    return true;
  else
    return false;
} 

/*
  @fn SPIKE専用 反射光チェック
  @param 検出したいポート番号
 */
int get_light_reflect(pbio_port_id_t sensor)
{
  if (sensor == color_sensor1_port)
  {
    int ref = pup_color_sensor_reflection(colorC1);
    return ref;
  }
  else 
    if (sensor == color_sensor2_port)
    {
//    rgb = pup_color_sensor_rgb(colorC2);
      spike_printf("C2 not assign\n");
      return -1;
    }
    else
    {
      spike_printf("CS not assign\n");
      return -1;
    }
} 

/**
 * @fn カラーセンサー1個のライントレース、PD制御（色で停止）
 * @param 使用するポート, スピード（0-100）、停止する色 'R' 'B' 
 */
void inetrace_single(pbio_port_id_t sensor, int speed, char _color)
{
  bool found = false;  
  int new, old = 0;
  int middle = 50;
  double Kp = 1.9;
  double Kd = 100;
  int direction;
    while (found == false)
    {
      if (_color == 'R')
        found = is_red(sensor);
      else 
        if (_color == 'B')
          found = is_blue(sensor);
      new = get_light_reflect(sensor);
      direction = (int)((middle - new)*Kp + (old - new)*Kd);
      //spike_printf("ref:%d dir:%d\n", new, direction);
      steering_on(direction, speed);
      old = new;
    }
    steering_stop(STOP_NOHOLD);
}

#else
// EV3 Version
int PD_trace(sensor_port_t port, int speed, colorid_t stop_color)
{
  int old= 0, new;
  float kP = 0.6;
  int kD = 10;

  while (ev3_color_sensor_get_color(port) != stop_color) {
    new = ev3_color_sensor_get_reflect(port);
    if (port == lineRport) {
      steering_on(speed, (60-new)*kP + (old-new)*kD);
    }
    else {
      steering_on(speed, (new-60)*kP + (new-old)*kD);
    }
  }
  steering_stop(STOP_BRAKE);
}
#endif

