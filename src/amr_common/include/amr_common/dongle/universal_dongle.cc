/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */

#include "amr_common/dongle/universal_dongle.h"

#include <angles/angles.h>

#include <string>
#include <vector>

namespace amr_dongle {

UniversalDongle::UniversalDongle(const std::string& key_name) {
  key_name_ = key_name;
}

DongleData UniversalDongle::GetDecryptData(const DongleData& key_data,
                                           const DongleData& encrypt_data) {
  return DongleData();
}

// 加密传入数据 返回加密后的数据
DongleData UniversalDongle::GenerateEncryptData(const DongleData& key_data,
                                                const DongleData& raw_data) {
  return DongleData();
}

bool UniversalDongle::ComputeDongleAlive(const DongleData& decrypt_data) {
  return true;
}

bool UniversalDongle::ComputeDongleAlive(const DongleData& key_data,
                                  const DongleData& encrypt_data) {
  return true;
}

}  // namespace amr_dongle
