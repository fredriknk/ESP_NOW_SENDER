#pragma once
#pragma pack(push, 1)
struct Telemetry {
  uint32_t msg_id;
  float temperature_c;
  float battery_v;
};
#pragma pack(pop)
