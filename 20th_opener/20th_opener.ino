//// loop()と，ここから呼び出される関数ではdelay()使用禁止
// 閾値設定
float Accel_out_threshold = 11.0;      //離床判定に用いるm/ss
float Altitude_out_threshold = 0.6;    //離床判定に用いるm/s (誤差を|1.0|m/sで考慮)
float accel_open_threshold = -11.0;    //開放判定に用いるm/ss
float altitude_open_threshold = -0.6;  //開放判定に用いるm/s (誤差を|1.0|m/sで考慮)
unsigned long maxaltime = 24000;       //頂点到達時間: 22.651s
unsigned long mecotime = 10000;        //燃焼時間：6.41s

//global judge
bool ready_judge = false;
bool emst = true;                        //開放禁止コマンド用状態表示用
bool mecotime_data_judge_ms = false;     //燃焼終了検知
bool maxaltitude_data_judge_ms = false;  //頂点到達検知
//global data
unsigned long time_data_ms = 0;  //離床判定タイマー(燃焼終了検知)
int nowphase = 0;                //now_state
int pinState_key = 0;            //key
int pinState_pwm_1 = 0;
int pinState_pwm_2 = 0;
float altitude_tmp_m = 0;       //高度データの一時保管
float altitude_per_time = 0;    //単位時間当たりの高度
float altitude_difference = 0;  //判定に用いるデータ格納(altitudedata_m-altitude_tmp_m)/0.01

//データ格納変数
float acceldata_mss = 0;
float altitudedata_m = 0;

//判定状態(true,false)
bool acceljudge_ground = false;     //加速度による離床判定
bool altitudejudge_ground = false;  //高度による離床判定
bool acceljudge_open = false;       //加速度による開放判定
bool altitudejudge_open = false;    //高度による開放判定
bool open_accel_time = false;       //加速度検知と燃焼終了
bool open_altitude_time = false;    //高度検知or頂点到達
//global
unsigned long time_100Hz = 0;  //100Hz
unsigned long time_speaker = 0;
unsigned long time_speaker_interbal = 0;
int count_speacker = 0;
int count_10Hz = 0;           //10Hz
int count = 0;                //1Hz処理を行うために用いる
int accel_ground_count = 0;   //加速度が閾値の条件を満たす回数をカウント
int altitude_open_count = 0;  //高度が閾値の条件を満たす回数をカウント
int altitude_count = 0;
int altitude_ground_count = 0;
int accel_open_count = 0;  //加速度が閾値の条件を満たす回数をカウント
void goCHECK();
void goREADY();

//テレメトリー
// uint16_t x = 0b0000000000000000;テレメトリのバイナリ化
uint32_t downlink = 0;
uint16_t damy = 1;
uint16_t downlink_key = 0;
uint16_t downlink_emst = 0;
uint16_t downlink_STM_1 = 0;  //state transition model
uint16_t downlink_STM_2 = 0;
uint16_t downlink_outground_accel = 0;
uint16_t downlink_outground_altitude = 0;
uint16_t downlink_open_accel = 0;
uint16_t downlink_open_altitude = 0;
uint16_t downlink_meco_time = 0;
uint16_t downlink_top_time = 0;
uint16_t downlink_pwm_1 = 0;
uint16_t downlink_pwm_2 = 0;

//LED
#define PWM_LED_RED 20   //GPIO20を使用する
#define PWM_LED_BLUE 21  //GPIO21を使用する
#define RGB_LED_RED 16
#define RGB_LED_BLUE 17
#define RGB_LED_GREEN 25
// digitalWrite(PWM_LED_RED, HIGH);
// digitalWrite(PWM_LED, LOW);

//specker
int pinNo = 29;  // 9番ピン設定
int i = 0;
//servo
#define PWM_SERVO_1 1  //GPIO5を使用する
#define PWM_SERVO_2 0  //GPIO6を使用する

//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT D1
#define CAN0_CS D0
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

// setup()ではdelay()使用可
void setup() {
  Serial.begin(115200);
  //key
  pinMode(28, INPUT_PULLUP);  // GPIO4ピンをプルアップ付き入力として設定
  //LED
  pinMode(PWM_LED_RED, OUTPUT);
  pinMode(PWM_LED_BLUE, OUTPUT);
  pinMode(RGB_LED_RED,OUTPUT);
  pinMode(RGB_LED_BLUE,OUTPUT);
  pinMode(RGB_LED_GREEN,OUTPUT);
  //servo
  pinMode(PWM_SERVO_1, OUTPUT);
  pinMode(PWM_SERVO_2, OUTPUT);
  //speacker
  pinMode(pinNo, OUTPUT);  // 9番ピンを出力設定
  //CAN
  CCP.begin();
  // デバッグ出力
  Serial.println("Opener Start");
  for (int i = 0; i < 1000; i++) {
    digitalWrite(pinNo, HIGH);  // ブザーを鳴らす
    delay(1);                   // 500ms停止
    digitalWrite(pinNo, LOW);   // ブザーを止める
    delay(1);                   // 1000停止
  }
}

void loop() {
  if (millis() - time_speaker >= 0.8) {
    if (i == 1) {
      if (millis() - time_speaker < 3000) {
        if (time_speaker_interbal == 0) {
          digitalWrite(pinNo, HIGH);  // ブザーを鳴らす
          time_speaker_interbal = 1;
        } else if (time_speaker_interbal == 1) {
          digitalWrite(pinNo, LOW);  // ブザーを止める
          time_speaker_interbal = 0;
        }
      }
    }
  }
  if (millis() - time_100Hz >= 10) {
    //100Hz処理
    time_100Hz += 10;
    count_10Hz++;  //100Hz処理されたときに+1
    if (count_10Hz > 10) {
      //10Hz
      //テレメトリ送信
      // Serial.print("downlink: ");
      // Serial.print(downlink_emst);
      // Serial.print(downlink_key);
      // Serial.print(downlink_STM_1);
      // Serial.print(downlink_STM_2);
      // Serial.print(downlink_outground_accel);
      // Serial.print(downlink_outground_altitude);
      // Serial.print(downlink_meco_time);
      // Serial.print(downlink_top_time);
      // Serial.print(downlink_open_accel);
      // Serial.print(downlink_open_altitude);
      // Serial.print(downlink_pwm_1);
      // Serial.println(downlink_pwm_2);
      downlink = (damy << 12) | (downlink_emst << 11) | (downlink_key << 10) | (downlink_STM_1 << 9) | (downlink_STM_2 << 8) | (downlink_outground_accel << 7) | (downlink_outground_altitude << 6) | (downlink_meco_time << 5) | (downlink_top_time << 4) | (downlink_open_accel << 3) | (downlink_open_altitude << 2) | (downlink_pwm_1 << 1) | (downlink_pwm_2 << 0);
      Serial.println(downlink);
      digitalWrite(PWM_LED_RED, HIGH);  //赤LED点灯
      CCP.uint32_to_device(CCP_downlink, downlink);
      digitalWrite(PWM_LED_RED, LOW);  //赤LED消灯
      count_10Hz = 0;
      count++;
      if (count > 10) {
        //1Hz
        // CCP.uint32_to_device(CCP_downlink, downlink);
        count = 0;
        //数値デバック
        // Serial.println(altitudedata_m);
        // Serial.println(acceldata_mss);
      }
    }

    //100Hzで実行する処理
    // //テレメトリ送信
    // Serial.print("downlink:");
    // Serial.print(downlink_emst);
    // Serial.print(downlink_key);
    // Serial.print(downlink_STM_1);
    // Serial.print(downlink_STM_2);
    // Serial.print(downlink_outground_accel);
    // Serial.print(downlink_outground_altitude);
    // Serial.print(downlink_meco_time);
    // Serial.print(downlink_top_time);
    // Serial.print(downlink_open_accel);
    // Serial.print(downlink_open_altitude);
    // Serial.print(downlink_pwm_1);
    // Serial.println(downlink_pwm_2);


    CCP.read_device();
    switch (CCP.id) {
      case CCP_opener_control:
        if (CCP.str_match(const_cast<char*>("CHECK"), 5)) goCHECK();
        if (CCP.str_match(const_cast<char*>("READY"), 5)) goREADY();
        if (CCP.str_match(const_cast<char*>("EMST"), 4)) emst = false;
        downlink_emst = 1;
        break;
      case CCP_A_accel_mss:
        acceldata_mss = CCP.data_fp16_2();  //加速度のZ軸方向の値を代入
        // Serial.println(acceldata_mss);
        break;
      case CCP_A_altitude_m:
        altitude_tmp_m = altitudedata_m;
        altitudedata_m = CCP.data_float();  //高度の値を代入
        altitude_difference = (altitudedata_m - altitude_tmp_m);
        // Serial.print("高度差");  //debag
        // Serial.println(altitude_difference);
        break;
      case CCP_parachute_fuse:
        if (CCP.str_match(const_cast<char*>("NOTOPEN"), 7)) emst = false;
        if (CCP.str_match(const_cast<char*>("CLEAR"), 5)) emst = true;
        break;
    }
  }
  if (nowphase == 0) {
    if ((pinState_key == LOW) && (ready_judge)) {
      goREADY();
      CCP.string_to_device(CCP_opener_state, (char*)"READY");
    }
  } else if (nowphase == 1) {
    if ((pinState_key == HIGH) || (!ready_judge)) {
      goCHECK();
      CCP.string_to_device(CCP_opener_state, (char*)"CHECK");
    }
  }
  //key
  pinState_key = digitalRead(28);  // GPIO4ピンの状態を読み取る
  if (downlink_key == 0) {
    if (pinState_key == LOW) {
      downlink_key = 1;
    }
  } else if (downlink_key == 1) {
    if (pinState_key == HIGH) {
      downlink_key = 0;
      goCHECK();
    }
  }
  //servo_1
  pinState_pwm_1 = digitalRead(PWM_SERVO_1);
  switch (downlink_pwm_1) {
    case 0:
      if (pinState_pwm_1 == HIGH) {
        downlink_pwm_1 = 1;
      }
      break;
    case 1:
      if (pinState_pwm_1 == LOW) {
        downlink_pwm_1 = 0;
      }
      break;
  }
  //servo_2
  pinState_pwm_2 = digitalRead(PWM_SERVO_2);
  switch (downlink_pwm_2) {
    case 0:
      if (pinState_pwm_2 == HIGH) {
        downlink_pwm_2 = 1;
      }
      break;
    case 1:
      if (pinState_pwm_2 == LOW) {
        downlink_pwm_2 = 0;
      }
      break;
  }
  //常に実行する処理
  //(R-F)加速度による離床判定altitude
  if ((acceldata_mss > Accel_out_threshold) && (!acceljudge_ground) && (nowphase >= 1)) {  //加速度による判定なし，READY以降が最低条件
    accel_ground_count++;
    // Serial.println(accel_ground_count);
    if (accel_ground_count >= 4) {  //5回以上
      // Serial.println("加速度検知");
      acceljudge_ground = true;
    }
  } else {
    accel_ground_count = 0;
  }

  //(R-F)高度による離床判定
  if ((altitude_difference > Altitude_out_threshold) && (!altitudejudge_ground) && (nowphase >= 1)) {  //高度による判定なし，READY以降が最低条件
    altitude_ground_count++;
    // Serial.println(altitude_ground_count);
    if (altitude_ground_count >= 4) {  //5回以上
      altitudejudge_ground = true;
      altitude_ground_count = 0;
      // Serial.println("----高度上昇--------------------------");
    }
  } else {
    altitude_ground_count = 0;
    // Serial.println(altitude_ground_count);
  }
  //開放判定用の加速度・高度検知
  if ((acceldata_mss < accel_open_threshold) && (!acceljudge_open) && (acceljudge_ground)) {
    accel_open_count++;
    if (accel_open_count > 4) {
      accel_open_count = 0;
      acceljudge_open = true;
      // Serial.println("加速度減少");
    }
  } else {
    accel_open_count = 0;
  }
  if ((altitude_difference < altitude_open_threshold) && (!altitudejudge_open) && (altitudejudge_ground)) {
    altitude_open_count++;
    if (altitude_open_count > 4) {
      altitude_open_count = 0;
      altitudejudge_open = true;
      // Serial.println("高度減少");
    }
  } else {
    altitude_open_count = 0;
  }

  switch (nowphase) {
    case 0:
      break;
    case 1:
      if ((acceljudge_ground) || (altitudejudge_ground)) {
        nowphase = 2;
        CCP.string_to_device(CCP_opener_state, (char*)"FLIGHT");
        // Serial.println("----------------READY to FLIGHT-------------");
        time_data_ms = millis();
      }
      break;
    case 2:
      if (millis() - time_data_ms > maxaltime) {  //時間による頂点到達検知
        maxaltitude_data_judge_ms = true;
      }
      if (millis() - time_data_ms > mecotime /*とりあえず燃焼時間5秒設定*/) {  //時間による燃焼終了検知
        mecotime_data_judge_ms = true;
      }
      if ((mecotime_data_judge_ms) && (acceljudge_open) && (!open_accel_time)) {  //燃焼終了と加速度検知
        open_accel_time = true;
        // Serial.println("--------燃焼終了と加速度検知------");
      }
      if (((maxaltitude_data_judge_ms) && (altitudejudge_open)) && (!open_altitude_time)) {
        open_altitude_time = true;
        // Serial.println("--------頂点到達と高度検知------");
      }
      if ((open_altitude_time) && (open_accel_time) && (emst)) {
        // Serial.println("--------servo_power_ON,OPENED--------------------");
        nowphase = 3;
        digitalWrite(PWM_SERVO_1, HIGH);  //servo1_open
        digitalWrite(PWM_SERVO_2, HIGH);  //servo2_open
        CCP.string_to_device(CCP_opener_state, (char*)"OPENED");
      }
      break;
    case 3:
      break;
  }
  //ダウンリンク
  if (!emst) {
    downlink_emst = 1;  //開放禁止コマンド
    digitalWrite(RGB_LED_RED,HIGH);
  } else {
    downlink_emst = 0;
    digitalWrite(RGB_LED_RED,LOW);
  }

  switch (nowphase) {
    case 0:
      downlink_STM_1 = 0;
      downlink_STM_2 = 0;
      break;
    case 1:
      downlink_STM_1 = 0;
      downlink_STM_2 = 1;
      break;
    case 2:
      downlink_STM_1 = 1;
      downlink_STM_2 = 0;
      break;
    case 3:
      downlink_STM_1 = 1;
      downlink_STM_2 = 1;
      break;
  }
  if (acceljudge_ground) downlink_outground_accel = 1;        //加速度による離床判定
  if (altitudejudge_ground) downlink_outground_altitude = 1;  //高度による離床判定
  if (mecotime_data_judge_ms) downlink_meco_time = 1;         //燃焼終了
  if (maxaltitude_data_judge_ms) downlink_top_time = 1;
  if (acceljudge_open) downlink_open_accel = 1;        //加速度による開放判定
  if (altitudejudge_open) downlink_open_altitude = 1;  //高度による開放判定

  //アップリンク
  //本番はコメントアウト//////////////////////////////////////////////////////////////////////////////////////////////////
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // シリアルモニタからの入力を読み取る
    input.trim();                                 // 入力の前後の空白を削除

    Serial.print("Received input: ");  // デバッグメッセージ
    Serial.println(input);
    if (input == "CHECK") {
      goCHECK();
    } else if (input == "READY") {
      ready_judge = true;
      goREADY();
    } else if (input == "EMST") {
      emst = false;
    } else if (input == "CANCEL") {
      emst = true;
    } else if (input == "OPEN") {
      digitalWrite(PWM_SERVO_1, HIGH);  //servo1_open
      digitalWrite(PWM_SERVO_2, HIGH);  //servo2_open
      digitalWrite(RGB_LED_BLUE,HIGH);
    } else if (input == "CLOSE") {
      digitalWrite(PWM_SERVO_1, LOW);  //servo1_close
      digitalWrite(PWM_SERVO_2, LOW);  //servo2_close
      digitalWrite(RGB_LED_BLUE,LOW);
    } else {
      Serial.println("Unknown command");  // 不明なコマンドの場合のデバッグメッセージ
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void goCHECK() {
  nowphase = 0;
  CCP.string_to_device(CCP_opener_state, const_cast<char*>("CHECK"));
  ready_judge = false;
  reset();
}
void goREADY() {
  nowphase = 1;
  time_speaker = millis();
  i = 1;
  CCP.string_to_device(CCP_opener_state, const_cast<char*>("READY"));
  ready_judge = true;
  reset();
}
void reset() {
  accel_ground_count = 0;   //加速度が閾値の条件を満たす回数をカウント
  altitude_open_count = 0;  //高度が閾値の条件を満たす回数をカウント
  altitude_ground_count = 0;
  accel_open_count = 0;
  altitude_open_count = 0;       //高度が閾値の条件を満たす回数をカウント
  acceljudge_ground = false;     //加速度による離床判定
  altitudejudge_ground = false;  //高度による離床判定
  acceljudge_open = false;       //加速度による開放判定
  altitudejudge_open = false;    //高度による開放判定
  open_accel_time = false;
  open_altitude_time = false;
  mecotime_data_judge_ms = false;     //燃焼終了検知
  maxaltitude_data_judge_ms = false;  //頂点到達検知
  time_data_ms = 0;                   //離床判定タイマー(燃焼終了検知)
  digitalWrite(PWM_SERVO_1, LOW);     //servo1_close
  digitalWrite(PWM_SERVO_2, LOW);     //servo2_close
  //テレメトリ
  downlink_STM_1 = 0;  //state transition model
  downlink_STM_2 = 0;
  downlink_open_accel = 0;
  downlink_open_altitude = 0;
  downlink_outground_accel = 0;
  downlink_outground_altitude = 0;
  downlink_meco_time = 0;
  downlink_top_time = 0;
  downlink_pwm_1 = 0;
  downlink_pwm_2 = 0;
  CCP.string_to_device(CCP_lift_off_judge, const_cast<char*>("reset--OK"));
}
