//閾値設定
float accel_threshold = 6.0;           //離床判定に用いる
float altitude_threshold = 74.5;         //離床判定に用いる
float accel_open_threshold = -6.0;     //開放判定に用いる
float altitude_open_threshold = 76.5;  //開放判定に用いる
double voltage_threshold = 0;          //keyスイッチによる電圧検知

//global data
bool emst = true;                        //開放禁止コマンド用状態表示用
bool mecotime_data_judge_ms = false;     //燃焼終了検知
bool maxaltitude_data_judge_ms = false;  //頂点到達検知
unsigned long time_data_ms = 0;          //離床判定タイマー(燃焼終了検知)
int nowphase = 0;
//判定状態(true,false)
bool acceljudge_ground = false;     //加速度による離床判定
bool altitudejudge_ground = false;  //高度による離床判定
bool acceljudge_open = false;       //加速度による開放判定
bool altitudejudge_open = false;    //高度による開放判定
bool open_accel_time = false;
bool open_altitude_time = false;
//global
unsigned long time_100Hz = 0;  //100Hz
int count_10Hz = 0;            //10Hz
int count = 0;                 //1Hz処理を行うために用いる
int accel_temp_count = 0;      //加速度が閾値の条件を満たす回数をカウント
int altitude_count = 0;        //高度が閾値の条件を満たす回数をカウント
int accel_count = 0;           //加速度が閾値の条件を満たす回数をカウント
int altitude_open_count = 0;   //高度が閾値の条件を満たす回数をカウント

//テレメトリー
int downlink_key = 0;
int downlink_emst = 0;
int downlink_STM = 00;  //state transition model
int downlink_open_accel = 0;
int downlink_open_altitude = 0;
int downlink_outground_accel = 0;
int downlink_outground_altitude = 0;
int downlink_meco_time = 0;
int downlink_top_time = 0;
//データ格納変数
float acceldata_mss = 0;
float altitudedata_m = 0;
double voltagedata = 0;
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
  //LED
  Serial.begin(115200);
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
    count_10Hz++;  //100Hz処理されたときに+1
    if (count_10Hz > 10) {
      //10Hz
      count_10Hz = 0;
      count++;
      if (count > 10) {
        //1Hz
        count = 0;
        //数値デバック
        Serial.println(altitudedata_m);
        Serial.println(acceldata_mss);
      }
    }

    //100Hzで実行する処理
    CCP.read_device();
    switch (CCP.id) {
      case CCP_opener_control:
        // if (CCP.str_match("CHECK", 5)) goCHECK();
        // if (CCP.str_match("READY", 5)) goREADY();
        // if (CCP.str_match("EMST", 4)) downlink_emst = 1;
        break;
      case CCP_A_accel_mss:
        acceldata_mss = CCP.data_fp16_2();  //加速度のZ軸方向の値を代入
        break;
      case CCP_A_altitude_m:
        altitudedata_m = CCP.data_float();  //高度の値を代入
        break;
    }
    switch (nowphase) {
      case 0:
        break;
      case 1:
        //(R-F)加速度による離床判定
        if ((acceldata_mss > accel_threshold) && (!acceljudge_ground)) {
          accel_temp_count++;
          Serial.println(accel_temp_count);
          Serial.println("条件が満たしています");
          Serial.print("acceldata_mss: ");
          Serial.println(acceldata_mss);
          Serial.print("accel_threshold: ");
          Serial.println(accel_threshold);
          Serial.print("acceljudge_ground: ");
          Serial.println(acceljudge_ground);
          Serial.print("nowphase: ");
          Serial.println(nowphase);
          if (accel_temp_count > 4) {
            Serial.println("加速度検知");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            acceljudge_ground = true;
          }
        } else {
          accel_temp_count = 0;
        }
        //(R-F)高度による離床判定
        if ((altitudedata_m > altitude_threshold) && (!altitudejudge_ground)) {
          altitude_count++;
          if (altitude_count > 4) {
            altitudejudge_ground = true;
            altitude_count = 0;
            Serial.println("----高度上昇--------------------------");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
            Serial.println("-------------------\n----------------------\n-------------------------\n");
          }
        } else {
          altitude_count = 0;
        }
        break;
      case 2:
        if ((acceldata_mss < accel_open_threshold) && (!acceljudge_open)) {
          accel_count++;
          Serial.println("なんで？？？？？？？");
          Serial.println(accel_count);
          if (accel_count > 4) {
            accel_count = 0;
            acceljudge_open = true;
            Serial.println("加速度減少");
          }
        } else {
          accel_count = 0;
        }
        if ((altitudedata_m < altitude_open_threshold) && (!altitudejudge_open)) {
          altitude_open_count++;
          if (altitude_count > 4) {
            altitudejudge_open = true;
            Serial.println("高度下降");
          }
        } else {
          altitude_count = 0;
        }
        break;
      case 3:
        break;
    }
  }
  //常に実行する処理
  switch (nowphase) {
    case 0:
      break;
    case 1:
      if ((acceljudge_ground) && (altitudejudge_ground)) {
        nowphase = 2;
        Serial.println("----------------READY to FLIGHT-------------");
        time_data_ms = millis();
      }
      break;
    case 2:
      if (millis() - time_data_ms > 12000 /*とりあえず燃焼時間12秒設定*/) {  //時間による頂点到達検知
        maxaltitude_data_judge_ms = true;
      }
      if (millis() - time_data_ms > 5000 /*とりあえず燃焼時間5秒設定*/) {  //時間による燃焼終了検知
        mecotime_data_judge_ms = true;
      }
      if ((mecotime_data_judge_ms) && (acceljudge_open)) {  //燃焼終了と加速度検知
        open_accel_time = true;
        Serial.println("--------燃焼終了と加速度検知------");
      }
      if ((maxaltitude_data_judge_ms) || (altitudejudge_open)) {
        open_altitude_time = true;
        Serial.println("--------頂点到達と高度検知------");
      }
      if ((open_altitude_time) && (open_accel_time) && (emst)) {
        Serial.println("--------フライトフェーズ移行--------------------");
        nowphase = 3;
      }
      break;
    case 3:
      break;
  }
  //ダウンリンク
  if (acceljudge_ground) {
    downlink_outground_accel = 1;  //加速度による離床判定
  }
  if (altitudejudge_ground) {
    downlink_outground_altitude = 1;  //高度による離床判定
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
      goready();
    } else if (input == "EMST") {
      emst = false;
    } else {
      Serial.println("Unknown command");  // 不明なコマンドの場合のデバッグメッセージ
    }
  }
}
void gocheck() {
  nowphase = 0;
  reset();
}
void goready() {
  nowphase = 1;
  reset();
}
void reset() {
  accel_temp_count = 0;          //加速度が閾値の条件を満たす回数をカウント
  altitude_count = 0;            //高度が閾値の条件を満たす回数をカウント
  accel_count = 0;               //加速度が閾値の条件を満たす回数をカウント
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
}