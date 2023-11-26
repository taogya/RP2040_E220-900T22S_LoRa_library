#include "rp2040_e220900t22s.hpp"
#include <vector>
#include <pico/mutex.h>

static recursive_mutex_t mutex;

// デフォルトコンフィグ値
LoRaConfigItem_t DEFAULT_LORA_CONFIG_ITEM = {
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
  0x0000, // target_address 0
  0x00, // target_channel 0
};

int CLoRa::InitLoRaModule(struct LoRaConfigItem_t &config) {
  int ret = 0;

  recursive_mutex_init(&mutex);
  
  // ポート初期化
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);
  pinMode(LoRa_AUXPin, INPUT);

  // シリアル初期化
  SerialLoRa.begin(LoRa_BaudRate, SERIAL_8N1);
  while(!SerialLoRa);
  delay(100);
  SerialMon.println("SerialLoRa Initialized.");

  // コンフィグモード(M0=1,M1=1)へ移行する
  SwitchToConfigurationMode();
  
  // Read Configuration
  std::vector<uint8_t> readCommand = {0xc1, 0x00, 0x08};
  SerialMon.println("# Command Request");
  if (recursive_mutex_enter_timeout_ms(&mutex, LoRa_Timeout_ms)) {
    for (auto i : readCommand) {
      SerialLoRa.write(i);
      SerialMon.print("0x"); SerialMon.print(i, HEX); SerialMon.print(" ");
    }
    SerialLoRa.flush();
    recursive_mutex_exit(&mutex);
  }
  SerialMon.println();

  delay(100);

  SerialMon.println("# Command Response");
  while (SerialLoRa.available()) {
    if (recursive_mutex_enter_timeout_ms(&mutex, LoRa_Timeout_ms)) {
      uint8_t data = SerialLoRa.read();
      SerialMon.print("0x"); SerialMon.print(data, HEX); SerialMon.print(" ");
      recursive_mutex_exit(&mutex);
    }
  }
  SerialMon.println();

  // Write Configuration
  std::vector<uint8_t> command = {0xc0, 0x00, 0x08};
  std::vector<uint8_t> response = {};

  // Register Address 00H, 01H
  uint8_t ADDH = config.own_address >> 8;
  uint8_t ADDL = config.own_address & 0xff;
  command.push_back(ADDH);
  command.push_back(ADDL);

  // Register Address 02H
  uint8_t REG0 = 0;
  REG0 = REG0 | (config.baud_rate << 5);
  REG0 = REG0 | (config.air_data_rate);
  command.push_back(REG0);

  // Register Address 03H
  uint8_t REG1 = 0;
  REG1 = REG1 | (config.subpacket_size << 6);
  REG1 = REG1 | (config.rssi_ambient_noise_flag << 5);
  REG1 = REG1 | (config.transmission_pause_flag << 4);
  REG1 = REG1 | (config.transmitting_power);
  command.push_back(REG1);

  // Register Address 04H
  uint8_t REG2 = config.own_channel;
  command.push_back(REG2);

  // Register Address 05H
  uint8_t REG3 = 0;
  REG3 = REG3 | (config.rssi_byte_flag << 7);
  REG3 = REG3 | (config.transmission_method_type << 6);
  REG3 = REG3 | (config.lbt_flag << 4);
  REG3 = REG3 | (config.wor_cycle);
  command.push_back(REG3);

  // Register Address 06H, 07H
  uint8_t CRYPT_H = config.encryption_key >> 8;
  uint8_t CRYPT_L = config.encryption_key & 0xff;
  command.push_back(CRYPT_H);
  command.push_back(CRYPT_L);

  SerialMon.println("# Command Request");
  if (recursive_mutex_enter_timeout_ms(&mutex, LoRa_Timeout_ms)) {
    for (auto i : command) {
      SerialLoRa.write(i);
      SerialMon.print("0x"); SerialMon.print(i, HEX); SerialMon.print(" ");
    }
    SerialLoRa.flush();
    recursive_mutex_exit(&mutex);
  }
  SerialMon.println();

  delay(100);

  SerialMon.println("# Command Response");
  while (SerialLoRa.available()) {
    if (recursive_mutex_enter_timeout_ms(&mutex, LoRa_Timeout_ms)) {
      uint8_t data = SerialLoRa.read();
      response.push_back(data);
      SerialMon.print("0x"); SerialMon.print(data, HEX); SerialMon.print(" ");
      recursive_mutex_exit(&mutex);
    }
  }
  SerialMon.println();

  if (response.size() != command.size()) {
    ret = 1;
  }

  return ret;
}

int CLoRa::RecieveFrame(struct RecvFrameE220900T22S_t *recv_frame) {
  int len = 0;
  uint8_t *start_p = recv_frame->recv_data;

  memset(recv_frame->recv_data, 0x00,
         sizeof(recv_frame->recv_data) / sizeof(recv_frame->recv_data[0]));

  while (1) {
    while (SerialLoRa.available()) {
      if (recursive_mutex_enter_timeout_ms(&mutex, LoRa_Timeout_ms)) {
        uint8_t ch = SerialLoRa.read();
        *(start_p + len) = ch;
        len++;
        recursive_mutex_exit(&mutex);
      }
      if (len > 200) {
        return 1;
      }
    }
    if ((SerialLoRa.available() == 0) && (len > 0)) {
      delay(10);
      if (SerialLoRa.available() == 0) {
        recv_frame->recv_data_len = len - 1;
        recv_frame->rssi = recv_frame->recv_data[len - 1] - 256;
        break;
      }
    }
    delay(100);
  }

  return 0;
}

int CLoRa::SendFrame(struct LoRaConfigItem_t &config, uint8_t *send_data,
                     int size) {
  uint8_t subpacket_size = 0;
  switch (config.subpacket_size) {
  case 0b00:
    subpacket_size = 200;
    break;
  case 0b01:
    subpacket_size = 128;
    break;
  case 0b10:
    subpacket_size = 64;
    break;
  case 0b11:
    subpacket_size = 32;
    break;
  default:
    subpacket_size = 200;
    break;
  }
  if (size > subpacket_size) {
    SerialMon.println("send data length too long");
    return 1;
  }
  uint8_t target_address_H = config.target_address >> 8;
  uint8_t target_address_L = config.target_address & 0xff;
  uint8_t target_channel = config.target_channel;

  uint8_t frame[3 + size] = {target_address_H, target_address_L,
                             target_channel};

  memmove(frame + 3, send_data, size);

#if 1 /* print debug */
  for (int i = 0; i < 3 + size; i++) {
    if (i < 3) {
      SerialMon.print(frame[i], HEX);
    } else {
      SerialMon.print(char(frame[i]));
    }
  }
  SerialMon.println();
#endif

  if (recursive_mutex_enter_timeout_ms(&mutex, LoRa_Timeout_ms)) {
    for (auto i : frame) {
      SerialLoRa.write(i);
    }
    SerialLoRa.flush();
    delay(100);
    while (SerialLoRa.available()) {
      while (SerialLoRa.available()) {
        SerialLoRa.read();
      }
      delay(100);
    }
    recursive_mutex_exit(&mutex);
  }

  return 0;
}

void CLoRa::SwitchToNormalMode(void) {
  digitalWrite(LoRa_ModeSettingPin_M0, 0);
  digitalWrite(LoRa_ModeSettingPin_M1, 0);
  delay(100);
}

void CLoRa::SwitchToWORSendingMode(void) {
  digitalWrite(LoRa_ModeSettingPin_M0, 1);
  digitalWrite(LoRa_ModeSettingPin_M1, 0);
  delay(100);
}

void CLoRa::SwitchToWORReceivingMode(void) {
  digitalWrite(LoRa_ModeSettingPin_M0, 0);
  digitalWrite(LoRa_ModeSettingPin_M1, 1);
  delay(100);
}

void CLoRa::SwitchToConfigurationMode(void) {
  digitalWrite(LoRa_ModeSettingPin_M0, 1);
  digitalWrite(LoRa_ModeSettingPin_M1, 1);
  delay(100);
}
