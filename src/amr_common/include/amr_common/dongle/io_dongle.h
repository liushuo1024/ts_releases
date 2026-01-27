/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_DONGLE_IO_DONGLE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_DONGLE_IO_DONGLE_H_
#include <angles/angles.h>

#include <string>
#include <vector>

#include "amr_common/dongle/dongle_interface.h"

namespace amr_dongle {

class IoDongle : public DongleInterface {
 public:
  explicit IoDongle(const std::string& key_name);

  DongleData GetDecryptData(const DongleData& key_data,
                            const DongleData& encrypt_data) override;

  // 加密传入数据 返回加密后的数据
  DongleData GenerateEncryptData(const DongleData& key_data,
                                 const DongleData& raw_data) override;
  bool ComputeDongleAlive(const DongleData& decrypt_data) override;

  bool ComputeDongleAlive(const DongleData& key_data,
                          const DongleData& encrypt_data) override;

 private:
  uint32_t LocalTime2TimeInt();

  std::string key_name_;
};

}  // namespace amr_dongle

#endif  // amr_COMMON_INCLUDE_amr_COMMON_DONGLE_IO_DONGLE_H_
