/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */

#include "amr_common/dongle/io_dongle.h"

#include <angles/angles.h>

#include <cstring>
#include <vector>

#define DEBUG_FLAG 0

namespace amr_dongle {

IoDongle::IoDongle(const std::string& key_name) { key_name_ = key_name; }

DongleData IoDongle::GetDecryptData(const DongleData& key_data,
                                    const DongleData& encrypt_data) {
  DongleData decrypt_data, key_data_tmp, encrypt_data_tmp;

  memcpy(encrypt_data_tmp.data, encrypt_data.data, sizeof(encrypt_data.data));
  memcpy(key_data_tmp.data, key_data.data, sizeof(key_data.data));

// 调试日志
#if DEBUG_FLAG
  for (int i = 0; i < sizeof(key_data_tmp.data); i++) {
    LOG_INFO("dongle_key[%d]: %x", i, key_data_tmp.data[i]);
  }
  for (int i = 0; i < sizeof(encrypt_data_tmp.data); i++) {
    LOG_INFO("encrypt_data[%d]: %x", i, encrypt_data_tmp.data[i]);
  }
#endif

  decode_fun(sizeof(encrypt_data_tmp.data), key_data_tmp.data,
             encrypt_data_tmp.data, decrypt_data.data);

#if DEBUG_FLAG
  for (int i = 0; i < sizeof(decrypt_data); i++) {
    LOG_WARN("decrypt_data[%d]: %x", i, decrypt_data.data[i]);
  }
#endif

  return decrypt_data;
}

DongleData IoDongle::GenerateEncryptData(const DongleData& key_data,
                                         const DongleData& raw_data) {
  DongleData data_temp, key_data_tmp, target_data;

  memcpy(key_data_tmp.data, key_data.data, sizeof(key_data.data));
  memcpy(data_temp.data, raw_data.data, sizeof(raw_data.data));
  encode_fun(sizeof(data_temp.data), key_data_tmp.data, data_temp.data,
             target_data.data);
#if DEBUG_FLAG
  for (int i = 0; i < sizeof(target_data.data); i++) {
    LOG_INFO("target_data[%d]: %x", i, target_data.data[i]);
  }
#endif
  return target_data;
}

// // 串口加密狗 解密后数据与系统时钟进行对比 小于系统时钟则认为存活
bool IoDongle::ComputeDongleAlive(const DongleData& decrypt_data) {
  uint32_t&& time_now = LocalTime2TimeInt();
  uint32_t decode_time = decrypt_data.data[0] * 1000000 +
                         decrypt_data.data[1] * 10000 +
                         decrypt_data.data[2] * 100 + decrypt_data.data[3];
  return decode_time > time_now;
}

bool IoDongle::ComputeDongleAlive(const DongleData& key_data,
                                  const DongleData& encrypt_data) {
  return true;
}

uint32_t IoDongle::LocalTime2TimeInt() {
  time_t now;
  struct tm* tm_now;
  time(&now);
  tm_now = localtime(&now);

  uint32_t time_int = (tm_now->tm_year + 1900) * 10000 +
                      (tm_now->tm_mon + 1) * 100 + tm_now->tm_mday;
  return time_int;
}

}  // namespace amr_dongle
