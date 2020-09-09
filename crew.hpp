//
//  crew.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef crew_hpp
#define crew_hpp

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * 左コース/右コース向けの設定を定義します
 * デフォルトは左コース(ラインの右エッジをトレース)です
 */
#if defined(MAKE_RIGHT)
    static const int _LEFT = 0;
    #define _EDGE -1
#else
    static const int _LEFT = 1;
    #define _EDGE 1
#endif

//#include <cinttypes>
#include <cmath>
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Steering.h"
#include "Clock.h"
using namespace ev3api;
#include "utility.hpp"

// global variables
extern rgb_raw_t g_rgb;
extern hsv_raw_t g_hsv;
extern int16_t g_grayScale, g_grayScaleBlueless, test_mode;
extern int16_t g_angle, g_anglerVelocity;
extern bool b1, b2, b3, slalom_flg; // sano

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE          60  /* 白色の光センサ値 */
#define LIGHT_BLACK           3  /* 黒色の光センサ値 */
#define GS_LOST              90  // threshold to determine "line lost"
#define OLT_SKIP_PERIOD    1000 * 1000 // period to skip outlier test in miliseconds
#define OLT_INIT_PERIOD    3000 * 1000 // period before starting outlier test in miliseconds
#define FINAL_APPROACH_LEN  100  // final approch length in milimater
#define ANG_V_TILT           50  // threshold to determine "tilt"
#define SONAR_ALERT_DISTANCE 10  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  85  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */

#define TIRE_DIAMETER    100.0F  // diameter of tire in milimater
#define WHEEL_TREAD      150.0F  // distance between the right and left wheels
//#define P_CONST           0.38D  // PID constants determined by Ultimate Gain method
//#define I_CONST           0.06D
//#define D_CONST          0.027D

// sano：開始
#define P_CONST           0.85D
#define I_CONST            0.0000001D
#define D_CONST            0.5D
#define SPEED_NORM           53
#define GS_TARGET            47
// sano：終了

//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START_R     'R' // R-mode start command
#define CMD_START_r     'r' // R-mode start command
#define CMD_START_L     'L' // L-mode start command
#define CMD_START_l     'l' // L-mode start command

// machine state
#define ST_takingOff    0
#define ST_tracing_L    1
#define ST_tracing_R    3
#define ST_stopping_L   5
#define ST_stopping_R   6
#define ST_landing      7
#define ST_challenge_L     8
#define ST_challenge_R     9

#define ST_NAME_LEN     20  // maximum number of characters for a machine state name
const char stateName[][ST_NAME_LEN] = {
    "ST_takingOff",
    "ST_tracing_L",
    "ST_tracing_R",
    "ST_stopping_L",
    "ST_stopping_R",
    "ST_landing",
    "ST_challenge_L",
    "ST_slasom_R"
};

// event
#define EVT_cmdStart_L      0
#define EVT_cmdStart_R      1
#define EVT_touch_On        2
#define EVT_touch_Off       3
#define EVT_sonar_On        4
#define EVT_sonar_Off       5
#define EVT_bk2bl           8
#define EVT_bl2bk           9
#define EVT_line_lost       10
#define EVT_line_found      11
#define EVT_dist_reached    12
#define EVT_tilt            13
#define EVT_turnCnr         14
#define EVT_turnb3          15
#define EVT_slalom_reached  16
#define EVT_slalom_On       17
#define EVT_slalom_Off      18
#define EVT_obstcl_unkown   19
#define EVT_obstcl_detect   20
#define EVT_obstcl_angle    21
#define EVT_obstcl_infront  22
#define EVT_obstcl_reached  23
#define EVT_obstcl_avoidable 24
#define EVT_lost_on_the_line 25
#define EVT_lost_on_the_edge 26
#define EVT_NAME_LEN        25  // maximum number of characters for an event name
const char eventName[][EVT_NAME_LEN] = {
    "EVT_cmdStart_L",
    "EVT_cmdStart_R",
    "EVT_touch_On",
    "EVT_touch_Off",
    "EVT_sonar_On",
    "EVT_sonar_Off",
    "EVT_line_lost",
    "EVT_line_found",
    "EVT_dist_reached",
    "EVT_tilt",
    "EVT_turnCnr",
    "EVT_turnb3"

};

// pwmMode
#define Mode_speed_constant     1
#define Mode_speed_increaseL    2
#define Mode_speed_decreaseL    3
#define Mode_speed_increaseR    4
#define Mode_speed_decreaseR    5
#define Mode_speed_increaseLR   6
#define Mode_speed_decreaseLR   7
#define Mode_speed_incrsLdcrsR  8
#define Mode_speed_incrsRdcrsL  9

// FIR filter parameters
const int FIR_ORDER = 10;
//const double hn[FIR_ORDER+1] = { 2.993565708123639e-03, 9.143668394023662e-03, -3.564197579813870e-02, -3.996625085414179e-02, 2.852028479250662e-01, 5.600000000000001e-01, 2.852028479250662e-01, -3.996625085414179e-02, -3.564197579813870e-02, 9.143668394023662e-03, 2.993565708123639e-03 };
const double hn[FIR_ORDER+1] = { -1.247414986406201e-18, -1.270350182429102e-02, -2.481243022283666e-02, 6.381419731491805e-02, 2.761351394755998e-01, 4.000000000000000e-01, 2.761351394755998e-01, 6.381419731491805e-02, -2.481243022283666e-02, -1.270350182429102e-02, -1.247414986406201e-18 };

// moving average parameter
const int MA_CAP = 10;

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

#define M_2PI    (2.0 * M_PI)

class Radioman {
private:
    FILE*           bt;      /* Bluetoothファイルハンドル */
public:
    Radioman();
    void operate(); // method to invoke from the task handler
    ~Radioman();
};

class Observer {
private:
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          armMotor; //sano
    Motor*          tailMotor; //sano
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    GyroSensor*     gyroSensor;
    ColorSensor*    colorSensor;
    double distance, runDistance, azimuth, locX, locY, sonarDistance;
    int8_t process_count;
    int16_t traceCnt, prevGS, curRgbSum, prevRgbSum;
    int32_t prevAngL, prevAngR, notifyDistance, gsDiff, timeDiff;
    bool touch_flag, sonar_flag, lost_flag, frozen, line_over_flg, moveBack_flg, adjust_flg;

    rgb_raw_t cur_rgb;
    hsv_raw_t cur_hsv;
    FIR_Transposed<FIR_ORDER> *fir_r, *fir_g, *fir_b;
    MovingAverage<int32_t, MA_CAP> *ma;
    //OutlierTester*  ot_r;
    //OutlierTester*  ot_g;
    //OutlierTester*  ot_b;

    bool check_touch(void);
    bool check_sonar(void);
    bool check_sonar(int16_t sonar_alert_dist_from, int16_t sonar_alert_dist_to);
    bool check_lost(void);
    bool check_tilt(void);

    int curAngle; //sano
    int prevAngle; //sano
    bool right_angle;//sano


protected:
public:
    Observer();
    Observer(Motor* lm, Motor* rm, Motor* am, Motor* tm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs);
    void goOnDuty();
    void reset();
    void notifyOfDistance(int32_t delta);
    int32_t getDistance();
    int32_t getSonarDistance();
    int16_t getAzimuth();
    int16_t getDegree();
    int32_t getLocX();
    int32_t getLocY();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    void freeze();
    void unfreeze();
    ~Observer();
};

class Navigator {
private:
protected:
    int8_t forward;      /* 前後進命令 */
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    int8_t speed;
    int16_t         trace_pwmLR;
    Motor*          leftMotor;
    Motor*          rightMotor;
    PIDcalculator*  ltPid;
public:
    Navigator();
    void goOnDuty();
    virtual void haveControl() = 0;
    virtual void operate() = 0;
    void goOffDuty();
    virtual ~Navigator();
};

class AnchorWatch : public Navigator {
private:
    
protected:
public:
    AnchorWatch();
    AnchorWatch(Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~AnchorWatch();
};

class LineTracer : public Navigator {
private:
    int32_t motor_ang_l, motor_ang_r;
protected:
public:
    bool frozen;
    LineTracer();
    LineTracer(Motor* lm, Motor* rm, Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    void setSpeed(int8_t s);
    void freeze();
    void unfreeze();
    ~LineTracer();
};

class ChallengeRunner : public LineTracer {
private:
    int8_t pwmMode;
    int16_t count, procCount, traceCnt;
protected:
public:
    ChallengeRunner();
    ChallengeRunner(Motor* lm, Motor* rm, Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    void setPwmLR(int p_L,int p_R,int mode, int proc_count);
    void setPower(int 	power, int 	turnRatio);
    int8_t getPwmL();
    int8_t getPwmR();
    ~ChallengeRunner();
};

#include "SeesawCrimber.hpp"
#include "LimboDancer.hpp"

class Captain {
private:
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    ColorSensor*    colorSensor;
    GyroSensor*     gyroSensor;
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          tailMotor;
    Motor*          armMotor; //sano
    Steering*       steering;
    AnchorWatch*    anchorWatch;
    LineTracer*     lineTracer;
    ChallengeRunner*   challengeRuuner; // sano
    SeesawCrimber*  seesawCrimber;
    LimboDancer*    limboDancer;
    uint64_t curTime, prevTime;
protected:
public:
    Captain();
    void takeoff();
    void decide(uint8_t event);
    void triggerLanding();
    void land();
    ~Captain();
};

extern Captain*     captain;
extern Observer*    observer;
extern Navigator*   activeNavigator;
extern Clock*       clock;
extern uint8_t      state;

#endif /* crew_hpp */
