/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_INTERFACE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_INTERFACE_H_
#include <string.h>

#include <iostream>
#include <string>
#include <vector>

#include "amr_common/dongle/dongle_algorithm/sm4.h"
#include "amr_common/log_porting.h"
namespace amr_dongle {

struct DongleData {
  unsigned char data[16];

  bool operator!=(const DongleData& other) {
    for (uint8_t i = 0; i < 16; i++) {
      if (data[i] != other.data[i]) {
        return true;
      }
    }
    return false;
  }
  std::string to_string() {
    std::string ori_string;
    for (uint8_t i = 0; i < 16; i++) {
      ori_string += "-" + std::to_string(data[i]);
    }
    return ori_string;
  }

  DongleData() { std::memset(data, 0, sizeof(data)); }
};

class DongleInterface {
 public:
  virtual ~DongleInterface() {}

  // 外部输入已加密数据与秘钥 返回解密后数据
  virtual DongleData GetDecryptData(const DongleData& key_data,
                                    const DongleData& encrypt_data) = 0;

  // 外部输入秘钥与原始数据 返回加密后数据
  virtual DongleData GenerateEncryptData(const DongleData& key_data,
                                         const DongleData& raw_data) = 0;

  virtual bool ComputeDongleAlive(const DongleData& decrypt_data) = 0;

  // 外部输入已加密数据与秘钥 判断加密狗是否存活
  virtual bool ComputeDongleAlive(const DongleData& key_data,
                                  const DongleData& encrypt_data) = 0;
};

}  // namespace amr_dongle

#endif  // amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_INTERFACE_H_
