// NJR4265 J1 移動体検知センサモジュールのテスト
// Arduino Nano Every で実験

#include <stdlib.h>
#include <ctype.h>
#include <string.h>

char Receive_Data[12]; //通信コマンドの受信用バッファ
int Receive_Data_Num; //通信コマンドの文字列の長さ及び受信フラグ

union  pari_t {
  uint8_t  dt ;
  struct {
    unsigned b0     :1;
    unsigned b1     :1;
    unsigned b2     :1;
    unsigned b3     :1;
    unsigned b4     :1;
    unsigned b5     :1;
    unsigned b6     :1;
    unsigned b7     :1;
  } ;
} ;

/*******************************************************************************
*  電源起動時とリセットの時だけのみ処理される関数(初期化と設定処理)            *
*******************************************************************************/
void setup() {
   // シリアル通信の設定
   
   // PC(シリアルモニタ)の設定
   Serial.begin(9600) ; 
   
   // ドップラーレーダーセンサーの設定
   // (RX=0 TX=1 BaudRate=9600bps Data=8bit Parity=odd Stop=1bit Flow=none)
   Serial1.begin(9600,SERIAL_8O1) ;

   Receive_Data_Num = -1;

   // SSR起動用信号ピン
   pinMode(2, OUTPUT);

}

/*******************************************************************************
*  繰り返し実行される処理の関数(メインの処理)                                  *
*******************************************************************************/
/*
 * void loop() {
  // 受信データ(センサデータ)がある時に真
  if (Serial1.available() > 0) {
    // Serial1(Sensor) to Serial(PC)
    int Sensor_Data = Serial1.read();
    Serial.write(Sensor_Data);
  }
  
  // データを受信した場合にのみシリアルモニタに
  // データを出力する
  if (Serial.available() > 0) {
    // 受信したデータの1バイトを読み取る
    int Incoming_Data = Serial.read();
  
    // 受信したデータをシリアルモニタに出力する
    Serial.println(Incoming_Data, DEC);
  }
}
*/

void loop() {
  int Ans;
  // 通信コマンドの受信
  Ans = Sensor_Receive();
  if (Ans != -1) {
    // 受信した通信コマンドに対する処理
    Sensor_Commands(Ans);
  }
  digitalWrite(2,HIGH); //2番ピンの出力をHIGH = 5Vにする
}

// 受信した通信コマンドに対して処理を行う
// num：受信コマンドの文字長さ
void Sensor_Commands(int num) {
  switch (Receive_Data[1]) {
    case 'W': // 起動完了
              Serial.println("起動完了");
              delay(5000);
              // 閾値の設定を行う(接近=5m/離反=5m)
              //Sensor_Threshold(500,500);
              break;
    case 'C': // 移動体が接近
              Serial.println("移動物体が接近");
              break;
    case 'L': // 移動体が離反
              Serial.println("移動物体が離反");
              break;
    case 'N': // 移動体が無い
              Serial.println("移動物体が無い");
              break;
    case 'E': // エラー
              Serial.println("エラー");
              break;
  }
}

/*
// 接近/離反時の閾値を設定する処理
// sp：接近時の閾値を指定(0-999cm)
// sm：離反時の閾値を指定(0-999cm)
void Sensor_Threshold(int sp,int sm)
{
     char buf[8] ;

     // 接近時の閾値を送信する
     sprintf(buf,"@SP%d\r\n",sp) ;
     Serial1.print(buf) ;
     // 離反時の閾値を送信する
     sprintf(buf,"@SM%d\r\n",sm) ;
     Serial1.print(buf) ;
}
*/

// 通信コマンドを受信する処理
// 受信したコマンドの文字長さを返す(CR/LFも含む)、未受信なら－１を返す
// 受信したコマンドの文字はReceiveDataバッファに格納する
int Sensor_Receive() {
  int Ans, Ret;
  char Dt;

  Ret = -1;
  while (1) {
    // 受信データがある時に真
    Ans = Serial1.available();
    if (Ans > 0) {
      Dt = Serial1.read();
      // 通信コマンドの開始
      if (Dt == '@') {
        Receive_Data_Num = 0;
      }
      if (Receive_Data_Num >= 0) {
        // 通信コマンドをバッファに格納
        Receive_Data[Receive_Data_Num] = Dt;
        Receive_Data_Num ++;
        // 通信コマンドの最後 (CR/LF)
        if (Dt == '\n' && Receive_Data[Receive_Data_Num - 2] == '\r') {
          Ret = Receive_Data_Num;
          Receive_Data_Num = -1;
          Serial.write(Receive_Data); // デバッグ用
          if (Serial.available() > 0) {
            // 受信したデータの1バイトを読み取る
            int Incoming_Data = Serial.read();
            // 受信したデータをシリアルモニタに出力する
            Serial.println(Incoming_Data, DEC);
          }
          break;
        }
      }
    } else {
      break;
    }
  }
  return Ret;
}
