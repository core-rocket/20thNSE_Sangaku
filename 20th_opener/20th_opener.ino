//閾値設定
float accel_threshold = 110.0;           //離床判定に用いるm/ss
float altitude_threshold = 0.8;        //離床判定に用いるm/s
float accel_open_threshold = -110.0;     //開放判定に用いるm/ss
float altitude_open_threshold = -0.8;  //開放判定に用いるm/s
// float time_interval = 0.1;             //単位時間当たりの高度差を求めるための単位時間設定
//試験用に閾値を超えるように設定してある本番は0.1s(sensor部からデータが送られてくる間隔)
unsigned long maxaltime = 12000;
unsigned long mecotime = 5000;

//global judge
bool ready_judge = false;
bool emst = true;                        //開放禁止コマンド用状態表示用
bool mecotime_data_judge_ms = false;     //燃焼終了検知
bool maxaltitude_data_judge_ms = false;  //頂点到達検知
//global data
unsigned long time_data_ms = 0;  //離床判定タイマー(燃焼終了検知)
int nowphase = 0;                //now_state
int pinState = 0;                //key
float altitude_tmp_m = 0;        //高度データの一時保管
float altitude_difference = 0;     //判定に用いるデータ格納(altitudedata_m-altitude_tmp_m)/0.01
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
int count_10Hz = 0;            //10Hz
int count = 0;                 //1Hz処理を行うために用いる
int accel_ground_count = 0;    //加速度が閾値の条件を満たす回数をカウント
int altitude_open_count = 0;   //高度が閾値の条件を満たす回数をカウント
int altitude_ground_count = 0;
int accel_open_count = 0;  //加速度が閾値の条件を満たす回数をカウント

//テレメトリー
int downlink_key = 0;
int downlink_emst = 0;
int downlink_STM_1 = 0;  //state transition model
int downlink_STM_2 = 0;
int downlink_open_accel = 0;
int downlink_open_altitude = 0;
int downlink_outground_accel = 0;
int downlink_outground_altitude = 0;
int downlink_meco_time = 0;
int downlink_top_time = 0;

//CAN_id

//LED
#define PWM_LED_RED 20   //GPIO20を使用する
#define PWM_LED_BLUE 21  //GPIO21を使用する
// digitalWrite(PWM_LED_RED, HIGH);
// digitalWrite(PWM_LED, LOW);

//servo
#define PWM_SERVO_1 7  //GPIO5を使用する
#define PWM_SERVO_2 6  //GPIO6を使用する

//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT D1
#define CAN0_CS D0
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);



// setup()ではdelay()使用可
void setup() {
  Serial.begin(115200);
  //key
  pinMode(4, INPUT_PULLUP);  // GPIO4ピンをプルアップ付き入力として設定
  //LED
  pinMode(PWM_LED_RED, OUTPUT);
  pinMode(PWM_LED_BLUE, OUTPUT);
  //servo
  pinMode(PWM_SERVO_1, OUTPUT);
  pinMode(PWM_SERVO_2, OUTPUT);
  //CAN
  CCP.begin();
  // デバッグ出力
  Serial.println("Opener Start");
}

// loop()と，ここから呼び出される関数ではdelay()使用禁止
void loop() {
  if (millis() - time_100Hz >= 10) {
    time_100Hz += 10;
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
    // Serial.println(downlink_open_altitude);
    count_10Hz++;  //100Hz処理されたときに+1
    if (count_10Hz > 10) {
      //10Hz
      count_10Hz = 0;
      count++;
      if (count > 10) {
        //1Hz
        //テレメトリ送信
        Serial.print("downlink:");
        Serial.print(downlink_emst);
        Serial.print(downlink_key);
        Serial.print(downlink_STM_1);
        Serial.print(downlink_STM_2);
        Serial.print(downlink_outground_accel);
        Serial.print(downlink_outground_altitude);
        Serial.print(downlink_meco_time);
        Serial.print(downlink_top_time);
        Serial.print(downlink_open_accel);
        Serial.println(downlink_open_altitude);
        count = 0;
        //数値デバック
        // Serial.println(altitudedata_m);
        // Serial.println(acceldata_mss);
      }
    }

    //100Hzで実行する処理

    CCP.read_device();
    switch (CCP.id) {
      case CCP_opener_control:
        // if (CCP.str_match("CHECK", 5)) goCHECK();nowphase
        // if (CCP.str_match("READY", 5)) goREADY();
        // if (CCP.str_match("EMST", 4)) downlink_emst = 1;
        break;
      case CCP_A_accel_mss:
        acceldata_mss = CCP.data_fp16_2();  //加速度のZ軸方向の値を代入
        break;
      case CCP_A_altitude_m:
        altitude_tmp_m = altitudedata_m;
        altitudedata_m = CCP.data_float();  //高度の値を代入
        altitude_difference = (altitudedata_m - altitude_tmp_m);
        Serial.print("高度差");//debag
        Serial.println(altitude_difference);
        break;
    }
    switch (nowphase) {
      case 0:
        pinState = digitalRead(4);  // GPIO4ピンの状態を読み取る
        if ((pinState == LOW) || (ready_judge)) {
          nowphase = 1;
        }
        break;
    }
  }
  //常に実行する処理
  //(R-F)加速度による離床判定
  if ((acceldata_mss > accel_threshold) && (!acceljudge_ground) && (nowphase >= 1)) {
    accel_ground_count++;
    // Serial.println(accel_ground_count);
    if (accel_ground_count > 4) {
      // Serial.println("加速度検知");
      acceljudge_ground = true;
    }
  } else {
    accel_ground_count = 0;
  }
  //(R-F)高度による離床判定

  if ((altitude_difference > altitude_threshold) && (!altitudejudge_ground) && (nowphase >= 1)) {
    altitude_ground_count++;
    // Serial.println(altitude_ground_count);
    if (altitude_ground_count > 4) {
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
  //key
  if (downlink_key == 1) {
    pinState = digitalRead(4);  // GPIO4ピンの状態を読み取る
    if (pinState == HIGH) {
      gocheck();
      ready_judge = false;
      downlink_key = 0;
    }
  }
  switch (nowphase) {
    case 0:
      break;
    case 1:
      if ((acceljudge_ground) || (altitudejudge_ground)) {
        nowphase = 2;
        // Serial.println("----------------READY to FLIGHT-------------");
        time_data_ms = millis();
      }
      break;
    case 2:
      if (millis() - time_data_ms > maxaltime /*とりあえず燃焼時間12秒設定*/) {  //時間による頂点到達検知
        maxaltitude_data_judge_ms = true;
      }
      if (millis() - time_data_ms > mecotime /*とりあえず燃焼時間5秒設定*/) {  //時間による燃焼終了検知
        mecotime_data_judge_ms = true;
      }
      if ((mecotime_data_judge_ms) && (acceljudge_open) && (!open_accel_time)) {  //燃焼終了と加速度検知
        open_accel_time = true;
        // Serial.println("--------燃焼終了と加速度検知------");
      }
      if (((maxaltitude_data_judge_ms) || (altitudejudge_open)) && (!open_altitude_time)) {
        open_altitude_time = true;
        // Serial.println("--------頂点到達と高度検知------");
      }
      if ((open_altitude_time) && (open_accel_time) && (emst)) {
        // Serial.println("--------servo_power_ON,OPENED--------------------");
        nowphase = 3;
      }
      break;
    case 3:
      break;
  }
  //ダウンリンク
  if (!emst) {
    downlink_emst = 1;  //開放禁止コマンド
  } else {
    downlink_emst = 0;
  }
  if (pinState == LOW) {
    downlink_key = 1;
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
  if (acceljudge_ground) {
    downlink_outground_accel = 1;  //加速度による離床判定
  }
  if (altitudejudge_ground) {
    downlink_outground_altitude = 1;  //高度による離床判定
  }
  if (mecotime_data_judge_ms) {  //燃焼終了
    downlink_meco_time = 1;
  }
  if (maxaltitude_data_judge_ms) {  //頂点到達
    downlink_top_time = 1;
  }
  if (acceljudge_open) {
    downlink_open_accel = 1;  //加速度による開放判定
  }
  if (altitudejudge_open) {
    downlink_open_altitude = 1;  //高度による開放判定
  }
  //アップリンク
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // シリアルモニタからの入力を読み取る
    input.trim();                                 // 入力の前後の空白を削除

    Serial.print("Received input: ");  // デバッグメッセージ
    Serial.println(input);

    if (input == "CHECK") {
      gocheck();
    } else if (input == "READY") {
      ready_judge = true;
      goready();
    } else if (input == "EMST") {
      emst = false;
    } else if (input == "OPEN") {
      emst = true;
    } else {
      Serial.println("Unknown command");  // 不明なコマンドの場合のデバッグメッセージ
    }
  }
}
void gocheck() {
  nowphase = 0;
  ready_judge = false;
  reset();
}
void goready() {
  ready_judge = true;
  reset();
}
void reset() {
  accel_ground_count = 0;   //加速度が閾値の条件を満たす回数をカウント
  altitude_open_count = 0;  //高度が閾値の条件を満たす回数をカウント
  altitude_ground_count = 0;
  accel_open_count = 0;          //加速度が閾値の条件を満たす回数をカウント
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
  //テレメトリ
  downlink_STM_1 = 0;  //state transition model
  downlink_STM_2 = 0;
  downlink_open_accel = 0;
  downlink_open_altitude = 0;
  downlink_outground_accel = 0;
  downlink_outground_altitude = 0;
  downlink_meco_time = 0;
  downlink_top_time = 0;
}