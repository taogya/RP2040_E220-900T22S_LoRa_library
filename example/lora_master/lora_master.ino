#include "rp2040_e220900t22s.hpp"

CLoRa lora;
struct LoRaConfigItem_t config = {
  0x0000, // own_address 0
  0b011, // baud_rate 9600 bps
  0b10000, // air_data_rate SF:9 BW:125
  0b00, // subpacket_size 200
  0b1, // rssi_ambient_noise_flag 有効
  0b0, // transmission_pause_flag 有効
  0b00, // transmitting_power 13 dBm
  0x00, // own_channel 0
  0b1, // rssi_byte_flag 有効
  0b1, // transmission_method_type 固定送信モード
  0b0, // lbt_flag 有効
  0b011, // wor_cycle 2000 ms
  0x0000, // encryption_key 0
  0x0001, // target_address 0
  0x00, // target_channel 0
};
struct RecvFrameE220900T22S_t data;

/** prototype declaration **/
void LoRaRecvTask();
void LoRaSendTask();

void setup()
 {
  // シリアルモニター初期化
  SerialMon.begin(115200);
  while(!SerialMon);
  SerialMon.println("SerialMon Initialized.");
  
  // E220-900T22S(JP)へのLoRa初期設定
  if (lora.InitLoRaModule(config)) {
    SerialMon.println("init error");
    return;
  }
  SerialMon.println("LoRaModule Initialized.");

  // ノーマルモード(M0=0,M1=0)へ移行する
  lora.SwitchToNormalMode();

  SerialMon.println("Start LoRaRecvTask.");
  rp2040.fifo.push(1);
}

void loop() 
{
  // 受信タスク
  LoRaRecvTask();
}

void setup1()
{
  while(!rp2040.fifo.available());
  rp2040.fifo.pop();
  SerialMon.println("Start LoRaSendTask.");
}

void loop1() 
{
  // 送信タスク
  LoRaSendTask();
}

void LoRaRecvTask() 
{
  if (lora.RecieveFrame(&data) == 0) {
    SerialMon.println("recv data:");
    for (int i = 0; i < data.recv_data_len; i++) {
      SerialMon.print(char(data.recv_data[i]));
    }
    SerialMon.println();
    SerialMon.println("hex dump:");
    for (int i = 0; i < data.recv_data_len; i++) {
      SerialMon.print(data.recv_data[i], HEX); SerialMon.print(" ");
    }
    SerialMon.println();
    SerialMon.println("RSSI: " + String(data.rssi) + "dBm");
    SerialMon.println();

    SerialMon.flush();
  }

  delay(1);
}

void LoRaSendTask()
{
  int len = 0;
  char msg[200] = {0};
  int max_msg_len = 200;

  // コンソールから読み込む
  while (len < max_msg_len) {
    if (SerialMon.available() > 0) {
      char incoming_byte = SerialMon.read();
      if (0x20 <= incoming_byte && incoming_byte <= 0x7e){
        msg[len] = incoming_byte;
        len++;
      }
      // LF 受信で読み込み終了
      else if (incoming_byte == 0x0a) {
        break;
      }
      // 制御文字は除外
      else{
        continue;
      }
    }
    delay(1);
  }
  msg[len] = '\0';
  SerialMon.println(msg);

  if (lora.SendFrame(config, (uint8_t *)msg, strlen(msg)) == 0) {
    SerialMon.println("send succeeded.");
    SerialMon.println();
  } else {
    SerialMon.println("send failed.");
    SerialMon.println();
  }

  SerialMon.flush();

  delay(1);
}
