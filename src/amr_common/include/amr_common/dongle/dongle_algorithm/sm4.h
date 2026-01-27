#ifndef amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_ALGORITHM_SM4_H_
#define amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_ALGORITHM_SM4_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

namespace amr_dongle {
// #define uint8_t unsigned char
// #define uint32_t uint32_t
// 四字节转换成u32
void four_uCh2uLong(uint8_t *in, uint32_t *out);

void uLong2four_uCh(uint32_t in, uint8_t *out);  // u32转换成四字节

uint32_t move(uint32_t data, int length);  // 左移，保留丢弃位放置尾部

uint32_t func_key(
    uint32_t input);  // 先使用Sbox进行非线性变化，再将线性变换L置换为L'

uint32_t func_data(
    uint32_t input);  // 先使用Sbox进行非线性变化，再进行线性变换L

void print_hex(uint8_t *data, int len);  // 无符号字符数组转16进制打印

void encode_fun(uint8_t len, uint8_t *key, uint8_t *input,
                uint8_t *output);  // 加密函数

void decode_fun(uint8_t len, uint8_t *key, uint8_t *input,
                uint8_t *output);  // 解密函数
}  // namespace amr_dongle

#endif  // amr_COMMON_INCLUDE_amr_COMMON_DONGLE_DONGLE_ALGORITHM_SM4_H_
