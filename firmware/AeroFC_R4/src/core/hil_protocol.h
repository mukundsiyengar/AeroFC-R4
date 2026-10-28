// hil_protocol.h - Aerovidya / UAV HIL serial protocol v1 (see uav-hil-simulator arduino_hil/HIL_PROTOCOL.md)
//
//   MAGIC "HI" | VERSION 1 | TYPE u8 | LENGTH u16 | SEQUENCE u32 | TIMESTAMP u64 (ns) | PAYLOAD | CRC32
//   all little-endian, CRC32 = zlib/IEEE over header + payload.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace hil {

enum MsgType : uint8_t {
  SENSOR_IMU = 0x01,        // <6f  ax ay az (m/s^2)  gx gy gz (rad/s)   Gazebo body frame FLU
  SENSOR_BARO = 0x02,       // <f   altitude (m)
  SENSOR_MAG = 0x03,        // <3f  mx my mz (Tesla)                     Gazebo body frame FLU
  SENSOR_GPS = 0x04,        // <2df latitude, longitude (deg), altitude (m)
  ACTUATOR_COMMAND = 0x10,  // <4f  motor1..4 normalized 0..1 (FC -> sim)
  CONTROL_INPUT = 0x11,     // <4f  throttle, roll, pitch, yaw (sim -> FC)
  HELLO = 0x20,
  HELLO_ACK = 0x21,
  START = 0x22,
  STOP = 0x23,
  HEARTBEAT = 0x24,
  STATUS = 0x25,            // FC telemetry (our layout, see telemetry.h); the official bridge ignores it
  ERROR_MSG = 0x26
};

static const uint8_t MAGIC0 = 0x48;  // 'H'
static const uint8_t MAGIC1 = 0x49;  // 'I'
static const uint8_t VERSION = 1;
static const size_t HEADER_SIZE = 18;
static const size_t CRC_SIZE = 4;
static const size_t MAX_PAYLOAD = 64;
static const size_t MAX_PACKET = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE;
static const char HELLO_TEXT[] = "UAV_HIL_V1";

uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc = 0);   // zlib.crc32 compatible (chainable)

struct Packet {
  uint8_t type = 0;
  uint16_t length = 0;
  uint32_t sequence = 0;
  uint64_t timestampNs = 0;
  uint8_t payload[MAX_PAYLOAD];
};

// Byte-at-a-time receiver.  Like the official bridge it re-synchronises by dropping one byte
// whenever the magic, version, length or CRC is wrong, so a corrupted byte never loses more than one packet.
class Parser {
 public:
  void push(uint8_t b);
  bool next(Packet &out);   // call repeatedly after push() until it returns false
  void reset() { n_ = 0; }
  uint32_t goodPackets = 0, crcErrors = 0, framingErrors = 0;

 private:
  void drop(size_t k);
  uint8_t buf_[MAX_PACKET];
  size_t n_ = 0;
};

// Builds a complete packet into out (needs HEADER_SIZE + len + 4 bytes). Returns its size, or 0 if it does not fit.
size_t build(uint8_t *out, size_t cap, uint8_t type, uint32_t seq, uint64_t timestampNs,
             const uint8_t *payload, uint16_t len);

// Little-endian helpers (safe for unaligned buffers)
float getF32(const uint8_t *p);
double getF64(const uint8_t *p);
void putF32(uint8_t *p, float v);
void putU16(uint8_t *p, uint16_t v);
void putI16(uint8_t *p, int16_t v);
uint16_t getU16(const uint8_t *p);
int16_t getI16(const uint8_t *p);

}  // namespace hil
