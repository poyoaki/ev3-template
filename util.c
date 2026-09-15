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
    //ev3_lcd_fill_rect(0,0,178,128,EV3_LCD_WHITE);
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

#ifdef UTIL_EV3
/*******************************************
 steering_degree()/tank_degree() ramp-drive (EV3).
 ev3_motor_rotate()/ev3_motor_steer() drive the motors with
 their own built-in control loop, uncoordinated between the
 two wheels, so calling them independently lets the robot
 veer slightly during the initial acceleration burst. Driving
 both wheels ourselves via ev3_motor_set_power(), starting
 slow and ramping up, avoids that burst; the final approach
 is handed off to the shared P control (ev3_p_stop2).
  EV3_RAMP_DEG     : distance[deg] over which power ramps up
  EV3_START_PWR    : starting power[%] at the ramp's beginning
  EV3_CRUISE_PCT   : cruise ends at this %% of the target angle;
                      the rest is left for ev3_p_stop2() so it
                      has room to decelerate smoothly instead of
                      correcting a large overshoot
  EV3_STALL_TIMEOUT : if a wheel makes literally zero progress
                      for this long[us], the ramp gives up on it
                      (stuck/slipping wheel safety net -- without
                      this, a wheel that never reaches the cruise
                      target keeps the loop spinning forever)
  SYNC_KP           : while driving (both wheels commanded the same
                      power, e.g. steer=0), proportional gain that
                      turns the L/R encoder GAP[deg] (accumulated
                      position error, effectively an integral of the
                      speed error) into a power correction[%%] --
                      corrects a steady bias, but only after it has
                      already built up some position gap
  SYNC_KD           : gain that turns the L/R speed DIFFERENCE[deg
                      per SYNC_SPEED_WINDOW ticks] into a power
                      correction[%%] -- reacts to a developing
                      mismatch immediately, before SYNC_KP's gap even
                      appears, which is what actually keeps the two
                      wheels' speed equal while driving instead of
                      just resyncing their position after the fact
  SYNC_SPEED_WINDOW : how many loop iterations (x10ms) to measure the
                      speed difference over -- 1 tick is too short:
                      a small speed mismatch rounds to a 0-degree
                      difference at 10ms and the D-term sees nothing
                      but quantization noise
  SYNC_MAX_CORR     : cap on the total (P+D) correction[%%], so a
                      single bad reading can't kick power around too
                      hard
  EARLY_SYNC_DEG    : for the first this many degrees of travel,
                      static friction/backlash means the two wheels
                      may not break free from a dead stop at exactly
                      the same instant -- a one-time heading kick that
                      SYNC_KP/SYNC_KD (which only react to an ongoing
                      gap once both wheels are already moving) can't
                      undo. Below this distance, whichever wheel gets
                      ahead by more than EARLY_SYNC_TOLERANCE is held
                      at 0 power until the other one catches up.
  EARLY_SYNC_TOLERANCE : allowed gap[deg] before the early hold-back
                      above kicks in
 *******************************************/
# define EV3_RAMP_DEG      60
# define EV3_START_PWR     25
# define EV3_CRUISE_PCT    70
# define EV3_STALL_TIMEOUT (1*SEC)
# define SYNC_KP           0.6f
# define SYNC_KD           1.5f
# define SYNC_SPEED_WINDOW 5
# define SYNC_MAX_CORR     20
# define EARLY_SYNC_DEG       15
# define EARLY_SYNC_TOLERANCE 2

// 実機の左右モーターの個体差(ハードウェアのバラツキ)を補正する固定
// オフセット。steer=0の直進時、左を-LR_BARANCE/2、右を+LR_BARANCE/2する。
# define LR_BARANCE        2
#endif

/*******************************************
 P control gains used to converge the wheel
 angle onto the target after the coarse (bang-bang)
 approach phase. Tune P_KP / P_MIN_PWR on the
 actual robot to hit +-1 degree. Shared by both
 platforms; the motor-API-specific finisher
 (p_stop2 / ev3_p_stop2) below uses it.
  P_KP        : proportional gain, [deg] error -> [%] power
  P_MIN_PWR   : minimum power needed to overcome
                the robot's own weight/friction
  P_TOLERANCE : error considered "arrived" [deg]
  P_TIMEOUT   : abort if the error hasn't gotten any smaller for
                this long [us] (a stall/oscillation safety net --
                NOT a cap on how long the whole approach may take,
                since that legitimately varies with distance/power)
 *******************************************/
# define P_KP        0.5f
# define P_MIN_PWR   15
# define P_TOLERANCE 1
# define P_TIMEOUT   (1*SEC)

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
  else if (abs(err) > P_TOLERANCE * 3) {
    // ごく近距離ではP_MIN_PWRの下駄を履かせない。誤差が小さいのに
    // 無理やり最低出力まで持ち上げると、目標を追い越してすぐ逆方向にも
    // 同じだけ持ち上げ…を繰り返し、いつまでも収束しない振動(タイヤが
    // 小刻みに震えて止まって見える)に陥るため。
    if (pwr > 0 && pwr < P_MIN_PWR)
      pwr = P_MIN_PWR;
    else if (pwr < 0 && pwr > -P_MIN_PWR)
      pwr = -P_MIN_PWR;
  }
  return pwr;
}

#ifdef UTIL_SPIKE
/* Servo motorL/motorR onto target_l/target_r [deg] (signed, relative
   to the last pup_motor_reset_count()) using P control, then apply
   the requested brake mode. max_l_pwr/max_r_pwr are the powers that
   were originally commanded (used as the saturation limit, and their
   sign is irrelevant since target_l/target_r already carry direction). */
static void p_stop2(int target_l, int max_l_pwr, int target_r, int max_r_pwr, brake_t brake)
{
  int l_err, r_err;
  int best_l = -1, best_r = -1;
  SYSTIM l_t, r_t, t_now;
  get_tim(&l_t);
  r_t = l_t;
  do {
    l_err = target_l - pup_motor_get_count(motorL);
    r_err = target_r - pup_motor_get_count(motorR);
    pup_motor_set_power(motorL, p_power(l_err, max_l_pwr));
    pup_motor_set_power(motorR, p_power(r_err, max_r_pwr));
    // 各輪の誤差を別々に追跡する。片方の誤差だけ縮んでいると、合計値
    // では動かなくなったもう片方(スタック)を見逃してしまうため。
    get_tim(&t_now);
    if (best_l < 0 || abs(l_err) < best_l) {
      best_l = abs(l_err);
      l_t = t_now;
    }
    if (best_r < 0 || abs(r_err) < best_r) {
      best_r = abs(r_err);
      r_t = t_now;
    }
    if ((SYSTIM)(t_now - l_t) > P_TIMEOUT || (SYSTIM)(t_now - r_t) > P_TIMEOUT)
      break;
  } while (abs(l_err) > P_TOLERANCE || abs(r_err) > P_TOLERANCE);

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
#else
/* Mモーターでのステアリングは左右が鏡写しの取り付けになるため、
   ev3_motor_steerが内部でやっているのと同じく、l_signで左モーター
   だけ符号を反転してから読み書きする。l_sign/target_l/target_rは
   既にその補正込みの「論理的に+が前進」の値として渡すこと。 */
static void ev3_p_stop2(int target_l, int max_l_pwr, int target_r, int max_r_pwr, int l_sign, brake_t brake)
{
  int l_err, r_err;
  int ev3_brake = (brake == STOP_BRAKE);
  int best_l = -1, best_r = -1;
  SYSTIM l_t, r_t, t_now;
  get_tim(&l_t);
  r_t = l_t;
  ev3_printf("stop2");
  do {
    l_err = target_l - l_sign * ev3_motor_get_counts(_MtrL);
    r_err = target_r - ev3_motor_get_counts(_MtrR);
    ev3_motor_set_power(_MtrL, l_sign * p_power(l_err, max_l_pwr));
    ev3_motor_set_power(_MtrR, p_power(r_err, max_r_pwr));
    // 各輪の誤差を別々に追跡する。片方の誤差だけ縮んでいると、合計値
    // では動かなくなったもう片方(スタック)を見逃してしまうため。
    get_tim(&t_now);
    if (best_l < 0 || abs(l_err) < best_l) {
      best_l = abs(l_err);
      l_t = t_now;
    }
    if (best_r < 0 || abs(r_err) < best_r) {
      best_r = abs(r_err);
      r_t = t_now;
    }
    if ((SYSTIM)(t_now - l_t) > P_TIMEOUT || (SYSTIM)(t_now - r_t) > P_TIMEOUT)
      break;
    // set_powerを連呼するタイトループが他タスクを飢餓状態にしないよう、
    // 毎周期少しCPUを譲る。
    dly_tsk(10*MSEC);
  } while (abs(l_err) > P_TOLERANCE || abs(r_err) > P_TOLERANCE);

  ev3_printf("p_st2 L:%d R:%d", l_err, r_err);

  ev3_motor_stop(_MtrL, ev3_brake);
  ev3_motor_stop(_MtrR, ev3_brake);
}
#endif

#ifdef UTIL_EV3
/* tank_degree()のEV3粗動フェーズ。各輪を自分のエンコーダ量に応じて
   EV3_START_PWRから指定パワーまでランプさせながら、指定角度の70%
   (GIJI_DAIKEI_IDOU有効時。無効なら100%)まで走らせる。l_pwr/r_pwrは
   呼び出し側の意図した符号のまま使う(steering_degreeのl_signのような
   鏡写し補正はしない。既存のev3_motor_rotate版もしていなかった)。 */
static void ev3_ramp_drive(int l_pwr, int r_pwr, int cruise_deg, int l_sign, int sync)
{
  int stop = false;
  int abs_l = abs(l_pwr), abs_r = abs(r_pwr);
  int lp_sign = (l_pwr >= 0) ? 1 : -1;
  int rp_sign = (r_pwr >= 0) ? 1 : -1;
  int l_start = (EV3_START_PWR < abs_l) ? EV3_START_PWR : abs_l;
  int r_start = (EV3_START_PWR < abs_r) ? EV3_START_PWR : abs_r;
  int ramp_deg = EV3_RAMP_DEG;
  int last_l = -1, last_r = -1;
  int speed_prev_l = 0, speed_prev_r = 0, speed_tick = 0, speed_diff = 0;
  SYSTIM l_t, r_t, t_now;
  if (ramp_deg > cruise_deg / 2)
    ramp_deg = cruise_deg / 2;
  if (ramp_deg < 1)
    ramp_deg = 1;

  get_tim(&l_t);
  r_t = l_t;
  while (!stop) {
    int l_cnt = abs(ev3_motor_get_counts(_MtrL));
    int r_cnt = abs(ev3_motor_get_counts(_MtrR));
    int l_ramp, r_ramp, cur_l, cur_r;
    if (l_cnt >= cruise_deg || r_cnt >= cruise_deg) {
      stop = true;
      break;
    }
    // 各輪を別々に見て、動かすはずの輪(パワー0でない)が一定時間まったく
    // 進んでいなければスタック(スリップ/引っかかり)とみなして打ち切る。
    // 左右の合計で見ると、片方が正常に進んでいる間はもう片方が本当に
    // 止まっていても見逃してしまうため、必ず輪ごとに判定する。
    // パワー0の輪(意図的に静止させている)は対象外。
    get_tim(&t_now);
    if (l_cnt > last_l) {
      last_l = l_cnt;
      l_t = t_now;
    } else if (abs_l > 0 && (SYSTIM)(t_now - l_t) > EV3_STALL_TIMEOUT) {
      ev3_printf("ramp stall L:%d R:%d", l_cnt, r_cnt);
      stop = true;
      break;
    }
    if (r_cnt > last_r) {
      last_r = r_cnt;
      r_t = t_now;
    } else if (abs_r > 0 && (SYSTIM)(t_now - r_t) > EV3_STALL_TIMEOUT) {
      ev3_printf("ramp stall L:%d R:%d", l_cnt, r_cnt);
      stop = true;
      break;
    }
    l_ramp = (l_cnt < ramp_deg) ? l_cnt : ramp_deg;
    r_ramp = (r_cnt < ramp_deg) ? r_cnt : ramp_deg;
    cur_l = l_start + (abs_l - l_start) * l_ramp / ramp_deg;
    cur_r = r_start + (abs_r - r_start) * r_ramp / ramp_deg;
    if (cur_l > abs_l)
      cur_l = abs_l;
    if (cur_r > abs_r)
      cur_r = abs_r;
    // 走行中も左右のズレを見てパワー配分を微調整する(直進/その場スピン、
    // つまり左右同じパワーを指定したときだけ)。モーターの個体差で
    // 同じパワーでも速度が揃わないと、途中は斜めに進んで最終的に
    // ev3_p_stop2()の補正だけで正面に戻る形になってしまうため、進み
    // すぎている側を弱め遅れている側を強めて、走行中から真っすぐ
    // 進むようにする。l_pwr/r_pwrが異なる(円弧移動)ときは、そもそも
    // 左右が同じ速度になるのが正しくないので対象外にする。
    if (sync) {
      if (l_cnt < EARLY_SYNC_DEG || r_cnt < EARLY_SYNC_DEG) {
        // 起動直後は、個体差による動き出しのタイミングのズレが
        // ロボットの向きのズレとしてそのまま残ってしまう。ここだけは
        // 比例補正で「弱める」のではなく、進んでいる方を完全に
        // 足止めして遅れている方が追いつくのを待つ。
        if (l_cnt - r_cnt > EARLY_SYNC_TOLERANCE)
          cur_l = 0;
        else if (r_cnt - l_cnt > EARLY_SYNC_TOLERANCE)
          cur_r = 0;
      } else {
        int diff = l_cnt - r_cnt;
        int corr;
        // SYNC_SPEED_WINDOW周期ごとに直近の「進んだ量」の差(=速度差)を
        // 測る。1周期(10ms)だと角度が整数度でしか取れないため、小さな
        // 速度差はまるめで消えてノイズにしかならない。
        if (++speed_tick >= SYNC_SPEED_WINDOW) {
          speed_diff = (l_cnt - speed_prev_l) - (r_cnt - speed_prev_r);
          speed_prev_l = l_cnt;
          speed_prev_r = r_cnt;
          speed_tick = 0;
        }
        corr = (int)(diff * SYNC_KP + speed_diff * SYNC_KD);
        if (corr > SYNC_MAX_CORR)
          corr = SYNC_MAX_CORR;
        else if (corr < -SYNC_MAX_CORR)
          corr = -SYNC_MAX_CORR;
        cur_l -= corr / 2;
        cur_r += corr / 2;
        if (cur_l < 0)
          cur_l = 0;
        else if (cur_l > abs_l)
          cur_l = abs_l;
        if (cur_r < 0)
          cur_r = 0;
        else if (cur_r > abs_r)
          cur_r = abs_r;
      }
    }
    ev3_motor_set_power(_MtrL, l_sign * lp_sign * cur_l);
    ev3_motor_set_power(_MtrR, rp_sign * cur_r);
    // set_powerを連呼するタイトループが他タスク(モーターへの実際の
    // 送信を担う下位優先度タスクなど)を飢餓状態にしないよう、毎周期
    // 少しCPUを譲る。
    dly_tsk(10*MSEC);
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
  int ercd = E_OK;
  int l_pwr, r_pwr;
  int pwr, steer, diff;
  int abs_degree, target_l, target_r;
#ifdef UTIL_EV3
  int l_sign;
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
  // 直進時のみ、左右モーターの個体差を固定オフセットで補正する。
  // steer==0のときl_pwr/r_pwrはどちらもpwrちょうどなので、+側は
  // limit_abs(_, pwr)ではクランプされて効果が消えてしまう。ここは
  // 意図的な微調整なのでlimit_100で-100～100の範囲だけ守る。
  if (steer == 0) {
    l_pwr = limit_100(l_pwr - LR_BARANCE / 2);
    r_pwr = limit_100(r_pwr + LR_BARANCE / 2);
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
#else
  // EV3: Mモーターは左右が鏡写しの取り付けなので、左だけ符号反転すると
  // 「+が前進」に揃う(ev3_motor_steerが内部でやっている補正と同じ)。
  l_sign = (ev3_motor_get_type(_MtrL) == MEDIUM_MOTOR) ? -1 : 1;
  ev3_motor_reset_counts(_MtrL);
  ev3_motor_reset_counts(_MtrR);
  target_l = (l_pwr > 0) ? abs_degree : (l_pwr < 0) ? -abs_degree : 0;
  target_r = (r_pwr > 0) ? abs_degree : (r_pwr < 0) ? -abs_degree : 0;
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
    // EV3: ev3_motor_rotate()は1輪ずつ独立した制御ループで動くため、
    // 2回コールしても左右が同期せず、立ち上がりの個体差でわずかに
    // 曲がってしまう。SPIKE版と同じ(1)ゆっくり立ち上がる(2)指定パワーで
    // 巡航(3)P制御で追い込む、の3段階を共通のev3_ramp_drive()で行う。
    ev3_ramp_drive(l_pwr, r_pwr, abs_degree * EV3_CRUISE_PCT / 100, l_sign, 1);
    ev3_printf("ramp end");
    // (3) 後処理。P制御で目標角度(±1度)まで追い込む。
    ev3_p_stop2(target_l, l_pwr, target_r, r_pwr, l_sign, brake);
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
          ev3_motor_stop(_MtrR, true);
        }
        else {
          ev3_motor_stop(_MtrL, false);
          ev3_motor_stop(_MtrR, false);
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
  int abs_degree = abs(degree);
#ifdef UTIL_SPIKE
# ifdef GIJI_DAIKEI_IDOU
  int abs_degree2 = abs(degree)*9/10;
# else
  int abs_degree2 = abs_degree;
# endif
#endif
  int target_l, target_r;

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

#ifdef UTIL_SPIKE
  // 回転角度のリセット
  pup_motor_reset_count(motorL);
  pup_motor_reset_count(motorR);
#else
  ev3_motor_reset_counts(_MtrL);
  ev3_motor_reset_counts(_MtrR);
#endif
  // 各輪の目標角度（符号付き）。パワー0の輪は0度＝その場ホールド。
  target_l = (l_pwr > 0) ? abs_degree : (l_pwr < 0) ? -abs_degree : 0;
  target_r = (r_pwr > 0) ? abs_degree : (r_pwr < 0) ? -abs_degree : 0;

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
    // EV3: (1)ゆっくり立ち上がる (2)指定パワーで巡航 (3)P制御で追い込む。
    ev3_ramp_drive(l_pwr, r_pwr, abs_degree * EV3_CRUISE_PCT / 100, 1, 1);
    ev3_p_stop2(target_l, l_pwr, target_r, r_pwr, 1, STOP_BRAKE);
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
    // EV3: (1)ゆっくり立ち上がる (2)指定パワーで巡航 (3)P制御で追い込む。
    ev3_ramp_drive(l_pwr, r_pwr, abs_degree * EV3_CRUISE_PCT / 100, 1, 0);
    ev3_p_stop2(target_l, l_pwr, target_r, r_pwr, 1, STOP_BRAKE);
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

