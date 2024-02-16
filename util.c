/**
 * @title EV3 ���[�e�B���e�B�[�v���O����
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
 * @fn 178x128�T�C�Y��LCD�ɁA���݂��ꎮ�ɕ�����\������B1�R�[�����ɉ��s����B
 * �t�H���g�T�C�Y��
 * @param printf�Ɠ���
 * font_small: 6x8  font_medium: 10x16 -> �ő�17�����B
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
 * @fn 178x128�T�C�Y��LCD�ɁA�s�w��ŕ�����\������B
 * �t�H���g�T�C�Y��
 * @param �s���i0 to 10), printf�Ɠ���
 * font_small: 6x8  font_medium: 10x16 -> �ő�17�����B
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
 * @fn �|100�`100�ɐ��l��␳����
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
 * @fn �X�e�A�����O�Ɏg�����[�^�[2�R��o�^����
 * @param �����[�^�[�|�[�g�A�E���[�^�[�|�[�g
 */
void ev3_steering_register(motor_port_t mtr1, motor_port_t mtr2)
{
  _MtrL = mtr1;
  _MtrR = mtr2;
}

/**
 * @fn �X�e�A�����O�֐�(��]��)
 * @param �p���[(-100-100�j�A�X�e�A�����O�A��]��, �u���[�L
 * @detail ���[�^�[��]���I���܂Ń��^�[�����Ȃ��B�X�e�A�����O��0�ɂ���ƒ��i����B
 */  
int ev3_steering_rot(int _pwr, int _steer, float rot, brake_t brake)
{
  int l_deg, r_deg;  // ��]�p�x�ɕϊ�
  int ercd;
  int l_pwr, r_pwr;
  int pwr, steer;
  pwr = limit_100(_pwr);
  steer = limit_100(_steer);
  l_pwr = pwr - steer/2;
  r_pwr = pwr + steer/2;

  l_deg = (int)(360.0*rot);
  r_deg = l_deg;

  // �p���[�}�C�i�X�̏ꍇ�́AAPI���Ή��Ȃ̂ŉ�]�p�x���}�C�i�X�ɂ���
  if (l_pwr < 0){
    l_pwr *= -1;
    l_deg *= -1;
  }
  if (r_pwr < 0){
    r_pwr *= -1;
    r_deg *= -1;
  }
  
  // �p���[���傫���ق��̓��삪�I�������A�����ɉ�]���~�߂�
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

  // �z�[���h���Ȃ��u���[�L = 0.2�b��Ƀ��b�N���O��
  if (brake == STOP_NOHOLD) {
    ev3_motor_stop(_MtrR, true);
    ev3_motor_stop(_MtrL, true);
    tslp_tsk(200*MSEC);
    ev3_motor_stop(_MtrR, false);
    ev3_motor_stop(_MtrL, false);
  }
  else {
    // STOP_BREAK�̂Ƃ��̓^�C�������b�N�����B
    ev3_motor_stop(_MtrR, brake);
    ev3_motor_stop(_MtrL, brake);
  }
  return ercd;
}

/**
 * @fn �X�e�A�����O(ON))
 * @param �p���[
 * @param �X�e�A�����O�i�P���Ƀp���[�̍��ƂȂ�j
 */
int ev3_steering_on(int _pwr, int _steer)
{
  int l_pwr, r_pwr;
  int pwr, steer;
  int ercd;
  pwr = limit_100(_pwr);
  steer = limit_100(_steer);
  // ev3_motor_steer()�͎g�킸�A�^���N�ɒu��������
  
  l_pwr = pwr - steer/2;
  r_pwr = pwr + steer/2;
  ercd =  ev3_tank_on(l_pwr, r_pwr);
  return ercd;
}

/**
 * @fn �^���N�֐�(��]��)
 * @param ���p���[(-100-100�A�E�p���[�A����]���A�E��]��
 * @detail ���[�^�[��]���I���܂Ń��^�[�����Ȃ��B
 */  
int ev3_tank_rot(int _l_pwr, int _r_pwr, float l_rot, float r_rot)
{
  int ercd;
  int l_deg, r_deg; // ��]�p�x�ɕϊ�
  int l_pwr, r_pwr;
  l_pwr = limit_100(_l_pwr);
  r_pwr = limit_100(_r_pwr);

  l_deg = (int)(360.0*l_rot);
  r_deg = (int)(360.0*r_rot);

  // �p���[�}�C�i�X�̏ꍇ�́AAPI���Ή��Ȃ̂ŉ�]�p�x���}�C�i�X�ɂ���
  if (l_pwr < 0){
    l_pwr *= -1;
    l_deg *= -1;
  }
  if (r_pwr < 0){
    r_pwr *= -1;
    r_deg *= -1;
  }
  // ��]���������̓��삪�I���܂ő҂�
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
 * @fn �^���N�֐�(On)
 * @param ���p���[(-100-100)�A�E�p���[
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
 * @fne �^���N�֐�(��~)
 * @param �u���[�L�^�C�v
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

