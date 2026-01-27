#ifndef amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "amr_common/dongle/dongle_interface.h"

namespace amr_dongle {

class Dongle : public DongleInterface {
 public:
  explicit Dongle(const std::string &key_name);

  static DongleData ConvertEncryptString(std::string encrypt_str) {
    DongleData encrypt_data;
    int available_cnt = 0;
    std::string tmp;
    for (std::string::iterator it = encrypt_str.begin();
         it != encrypt_str.end(); ++it) {
      if (*it != '-') {
        tmp.push_back(*it);
      }
      if (*it == '-' || it == encrypt_str.end() - 1) {
        encrypt_data.data[available_cnt++] =
            static_cast<unsigned char>(std::stoi(tmp));
        tmp.clear();
      }
    }

    return encrypt_data;
  }

  amr_dongle::DongleData GetDecryptData(
      const amr_dongle::DongleData &key_cpudata,
      const amr_dongle::DongleData &encrypt_cpudata) override;

  amr_dongle::DongleData GenerateEncryptData(
      const amr_dongle::DongleData &key_cpudata,
      const amr_dongle::DongleData &raw_cpudata) override;

  bool ComputeDongleAlive(
      const amr_dongle::DongleData &decrypt_data) override {
    return false;
  }

  bool ComputeDongleAlive(const DongleData &key_data,
                          const DongleData &encrypt_data) override;

 private:
  void NativeId(unsigned int *eax, unsigned int *ebx, unsigned int *ecx,
                unsigned int *edx);

  bool IdTake(int number);
  unsigned char HexChar(char c);
  uint8_t *GetId() {
    if (!IdTake(1)) {
      LOG_WARN("Get dongle number failed");
    }
    return total_id_;
  };

  /* data */
  std::string key_name_;
  std::string total_str_;
  unsigned eax, ebx, ecx, edx;
  unsigned char total_id_[16];
  const uint8_t key_cpudata_[16] = {0};
  amr_dongle::DongleData read_sernum_data;
};
}  // namespace amr_dongle

#endif