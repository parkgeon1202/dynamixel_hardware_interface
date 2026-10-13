// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "dynamixel_hardware_interface/dynamixel/sync_read_rx_tracker.hpp"

#include <algorithm>

namespace dynamixel_hardware_interface
{
namespace
{

// Protocol 2.0 상태 패킷 배치: FF FF FD 00 | ID | LEN_L LEN_H | 0x55 | ERR | 데이터 | CRC(2).
// LEN 은 명령+에러+데이터+CRC 이므로 데이터 길이는 LEN - 4.
constexpr std::size_t kPktId = 4;
constexpr std::size_t kPktLengthL = 5;
constexpr std::size_t kPktLengthH = 6;
constexpr std::size_t kPktData = 9;
// SDK 의 RXPACKET_MAX_LEN.
constexpr std::size_t kRxPacketMaxLen = 1024;

}  // namespace

void SyncReadRxTracker::Configure(
  uint16_t start_address, uint16_t data_length, const std::vector<uint8_t> & ids)
{
  ids_ = ids;
  start_address_ = start_address;
  packet_.assign(kRxPacketMaxLen, 0);
  for (auto & data : data_) {data.clear();}
  missing_.fill(false);
  for (const auto id : ids_) {
    data_[id].assign(data_length, 0);
  }
  corrupt_count_ = 0;
}

int SyncReadRxTracker::Receive(
  dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler)
{
  for (const auto id : ids_) {missing_[id] = true;}

  const std::size_t expected = ids_.size();
  std::size_t received = 0;
  corrupt_count_ = 0;
  int result = COMM_RX_TIMEOUT;
  // 깨진 패킷도 한 자리로 센다. 나머지가 다 도착했으면 빠진 하나를 기다리며
  // 타임아웃을 태울 이유가 없다.
  while (received + corrupt_count_ < expected) {
    result = packet_handler->rxPacket(port_handler, packet_.data());
    if (result == COMM_RX_CORRUPT) {
      ++corrupt_count_;
      continue;
    }
    if (result != COMM_SUCCESS) {
      break;
    }

    const uint8_t id = packet_[kPktId];
    const uint16_t length = DXL_MAKEWORD(packet_[kPktLengthL], packet_[kPktLengthH]);
    auto & data = data_[id];
    // 이번 요청에 없던 ID, 이미 받은 ID, 길이가 다른 패킷은 버린다.
    if (!missing_[id] || data.empty() || length < 4 ||
      static_cast<std::size_t>(length - 4) != data.size())
    {
      continue;
    }
    std::copy_n(packet_.begin() + kPktData, data.size(), data.begin());
    missing_[id] = false;
    ++received;
  }

  if (received == expected) {
    return COMM_SUCCESS;
  }
  return corrupt_count_ > 0 ? COMM_RX_CORRUPT : result;
}

void SyncReadRxTracker::MarkAllMissing()
{
  for (const auto id : ids_) {missing_[id] = true;}
  corrupt_count_ = 0;
}

uint32_t SyncReadRxTracker::GetData(uint8_t id, uint16_t address, uint8_t size) const
{
  const auto & data = data_[id];
  if (address < start_address_) {return 0;}
  const std::size_t offset = address - start_address_;
  if (size > 4 || offset + size > data.size()) {return 0;}
  uint32_t value = 0;
  for (std::size_t k = 0; k < size; ++k) {
    value |= static_cast<uint32_t>(data[offset + k]) << (8 * k);
  }
  return value;
}

}  // namespace dynamixel_hardware_interface
