/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */

#include "amr_common/dongle/dongle.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <system_error>

#include "amr_common/log_porting.h"

using namespace std;

namespace amr_dongle {

Dongle::Dongle(const std::string &key_name) { key_name_ = key_name; }

namespace {
// 加密狗秘钥
const uint8_t kDiyDongleKey[8] = {'c', 'o', 't', 'e', 'k', '1', '2', '3'};

}  // namespace

//序列号解密函数
// key_cpudata：输入解密密钥
// encrypt_cpudata：输入需要解密的序列号数据
amr_dongle::DongleData Dongle::GetDecryptData(
    const amr_dongle::DongleData &key_cpudata,
    const amr_dongle::DongleData &encrypt_cpudata) {
  amr_dongle::DongleData decrypt_cpudata, key_cpudata_tmp,
      encrypt_cpudata_tmp;

  memcpy(encrypt_cpudata_tmp.data, encrypt_cpudata.data,
         sizeof(encrypt_cpudata.data));
  memcpy(key_cpudata_tmp.data, key_cpudata.data, sizeof(key_cpudata.data));
  amr_dongle::decode_fun(sizeof(encrypt_cpudata_tmp.data),
                           key_cpudata_tmp.data, encrypt_cpudata_tmp.data,
                           decrypt_cpudata.data);

  return decrypt_cpudata;
}
//序列号加密函数
// key_cpudata：输入解密密钥
// raw_cpudata：输入需要加密的序列号数据
amr_dongle::DongleData Dongle::GenerateEncryptData(
    const amr_dongle::DongleData &key_cpudata,
    const amr_dongle::DongleData &raw_cpudata) {
  amr_dongle::DongleData cpudata_temp, key_cpudata_tmp, target_cpudata;

  memcpy(key_cpudata_tmp.data, key_cpudata.data, sizeof(key_cpudata.data));
  memcpy(cpudata_temp.data, raw_cpudata.data, sizeof(raw_cpudata.data));
  amr_dongle::encode_fun(sizeof(cpudata_temp.data), key_cpudata_tmp.data,
                           cpudata_temp.data, target_cpudata.data);

  return target_cpudata;
}

bool Dongle::ComputeDongleAlive(const DongleData &key_data,
                                const DongleData &encrypt_data) {
  amr_dongle::DongleData decrypt_data, actual_data;
  decrypt_data = GetDecryptData(key_data, encrypt_data);
  memcpy(actual_data.data, GetId(), 16);
  if (actual_data != decrypt_data) {
    return false;
  }
  LOG_INFO("Dongle is already !!!");
  return true;
}

inline void Dongle::NativeId(unsigned int *eax, unsigned int *ebx,
                             unsigned int *ecx, unsigned int *edx) {
  /* ecx is often an input as well as an output. */
  asm volatile("cpuid"
               : "=a"(*eax), "=b"(*ebx), "=c"(*ecx), "=d"(*edx)
               : "0"(*eax), "2"(*ecx));
}

/*cpu唯一序列号后8位与主板后8位统一成16位唯一序列号*/
bool Dongle::IdTake(int number) {
  unsigned char cpu_id[8];
  unsigned char board_id[8];
  string board_str;
  char buf[9] = {0};
  /* 获取CPU Id */
  eax = number;  // processor CPU serial number
  Dongle::NativeId(&eax, &ebx, &ecx, &edx);
  cpu_id[0] = eax & 0x000000ff;
  cpu_id[1] = (eax & 0x0000ff00) >> 8;
  cpu_id[2] = (eax & 0x00ff0000) >> 16;
  cpu_id[3] = (eax & 0xff000000) >> 24;
  cpu_id[4] = edx & 0x000000ff;
  cpu_id[5] = (edx & 0x0000ff00) >> 8;
  cpu_id[6] = (edx & 0x00ff0000) >> 16;
  cpu_id[7] = (edx & 0xff000000) >> 24;

  // /*获取主板Id*/
  // ifstream infile;
  // system(
  //     "echo 123 |sudo -S dmidecode -t 2|grep Number > board.txt");
  //     //主板ID存入当前路径下的board.txt文件中
  // infile.open("board.txt", std::ios::in | std::ios::ate);
  // if (!infile) return false;
  // infile.seekg(-9, ios::end);  //光标定位至后8位ID前
  // ostringstream os;
  // os << infile.rdbuf();
  // board_str = os.str();
  // strcpy(buf, board_str.c_str());  //转成字符数组
  // for (size_t i = 0; i < 8; i++)   //转16进制数
  // {
  //   board_id[i] = HexChar(buf[i]);
  // }
  // infile.close();
  // remove("board.txt");  //删除文件

  /*序列号合并，CPU在前，自定义序列号在后*/
  for (int i = 0; i < 8; i++) {
    total_id_[i] = cpu_id[i];
    total_id_[i + 8] = kDiyDongleKey[i];
  }
  int total_length = sizeof(total_id_) / sizeof(total_id_[0]);
  if (!total_length) {
    return false;
  }
  return true;
}

unsigned char Dongle::HexChar(char c) {
  if ((c >= '0') && (c <= '9'))
    return c - '0' + 0x30;
  else if ((c >= 'A') && (c <= 'Z'))
    return c - 'A' + 0x41;
  else if ((c >= 'a') && (c < 'z'))
    return c - 'a' + 0x61;
  else
    return 0;
}

}  // namespace amr_dongle
