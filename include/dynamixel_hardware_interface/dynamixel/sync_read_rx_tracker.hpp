// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__SYNC_READ_RX_TRACKER_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__SYNC_READ_RX_TRACKER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_hardware_interface
{

/// 일반 SyncRead 의 수신 단계를 GroupSyncRead::rxPacket() 대신 맡아, 서보별
/// 응답 도착 여부를 남기는 진단용 부품.
///
/// SDK 쪽은 기다리던 ID 가 아닌 패킷을 버리고 첫 실패에서 바로 반환하므로,
/// 실패했을 때 어느 ID 가 빠졌고 그 뒤 ID 들은 도착했는지를 알 수 없다. 이 클래스는
/// 타임아웃까지 들어오는 패킷을 모두 받아 ID 별로 기록한다. 송신과 타임아웃 설정은
/// 그대로 호출자(Dynamixel)가 하고, 이 클래스는 수신과 데이터 보관만 한다.
///
/// 하드웨어 파라미터 track_sync_read_rx 가 true 일 때만 만들어진다. 없으면 드라이버는
/// 원래의 SDK 경로를 그대로 쓴다.
///
/// 버퍼는 Configure() 에서 한 번 크기를 잡고, Receive() 는 할당하지 않는다.
class SyncReadRxTracker
{
public:
  /// SyncRead 핸들러를 만들 때 한 번 부른다. 인덱스는 comm_id.
  void Configure(uint16_t start_address, uint16_t data_length, const std::vector<uint8_t> & ids);

  /// 송신 직후 부른다. 응답을 타임아웃까지 모두 받아 ID 별 도착 여부를 남긴다.
  /// 반환값은 SDK 의 COMM_* 코드: 전부 받았으면 COMM_SUCCESS, 깨진 패킷이 있었으면
  /// COMM_RX_CORRUPT, 아니면 마지막 수신 결과(보통 COMM_RX_TIMEOUT).
  int Receive(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port_handler);

  /// 송신 자체가 실패해 이번 틱에 아무 응답도 받지 못했을 때 부른다.
  void MarkAllMissing();

  /// GroupSyncRead::getData() 와 같은 규칙: 시작 주소 기준 오프셋, 리틀 엔디언.
  uint32_t GetData(uint8_t id, uint16_t address, uint8_t size) const;

  /// 마지막 Receive() 에서 이 comm_id 의 응답이 오지 않았으면 true.
  bool WasMissing(uint8_t id) const {return missing_[id];}
  /// 마지막 Receive() 에서 CRC 가 틀려 버린 패킷 수.
  std::size_t CorruptCount() const {return corrupt_count_;}
  /// Configure() 에 넘긴 순서(= 버스 응답 순서)의 ID 목록.
  const std::vector<uint8_t> & Ids() const {return ids_;}

private:
  std::vector<uint8_t> ids_;
  std::vector<uint8_t> packet_;
  std::array<std::vector<uint8_t>, 256> data_;
  std::array<bool, 256> missing_{};
  uint16_t start_address_ = 0;
  std::size_t corrupt_count_ = 0;
};

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__SYNC_READ_RX_TRACKER_HPP_
