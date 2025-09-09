#include <ros/protobuffer_traits.h>
#include <ros/serialization_protobuffer.h>
#include <ros/ros.h>
#include <fcntl.h>      // open()
#include <termios.h>    // termios 串口配置
#include <unistd.h>     // read(), close()
#include <cstring>
#include <string>
#include <vector>

// protoc 生成的头
#include "wind_vel.pb.h"

// 协议常量（仅 1 个 float）
static constexpr uint8_t SYNC0 = 0xAA;
static constexpr uint8_t SYNC1 = 0x55;
static constexpr uint8_t EXPECT_LEN = 4;

// ---- CRC16(Modbus-RTU)，多项式 0xA001（反射形式）----
static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

// ---- 串口（阻塞）打开 + 配置成 115200、8N1、原始模式 ----
int open_serial_blocking(const std::string& dev) {
  // 阻塞模式：不加 O_NONBLOCK
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) return -1;

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) { ::close(fd); return -1; }

  // cfmakeraw：关闭行缓冲/回显/特殊字符处理，按字节“原样”收发
  cfmakeraw(&tio);

  // 8N1：8 数据位、无校验、1 停止位；允许读
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;   // 无校验
  tio.c_cflag &= ~CSTOPB;   // 1 停止位
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;       // 8 数据位

  // 固定 115200
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);

  // VMIN/VTIME：阻塞读，至少等到 1 字节；无超时（VTIME=0）
  // 想要“最多等 0.5s”，可设 VTIME=5（单位 0.1s）
  tio.c_cc[VMIN]  = 1;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tio) != 0) { ::close(fd); return -1; }
  tcflush(fd, TCIOFLUSH); // 清空缓冲

  return fd;
}

// ---- 超简状态机：按字节喂入，凑成一帧就返回 true 并输出 airspeed ----
struct Parser {
  enum State { FIND_AA, FIND_55, READ_LEN, READ_PAYLOAD, READ_CRC_LO, READ_CRC_HI } st = FIND_AA;
  uint8_t len = 0;
  uint8_t crc_lo = 0;
  std::vector<uint8_t> payload;

  bool feed(uint8_t b, float& out) {
    switch (st) {
      case FIND_AA:
        st = (b == SYNC0) ? FIND_55 : FIND_AA;
        break;
      case FIND_55:
        st = (b == SYNC1) ? READ_LEN : FIND_AA;
        break;
      case READ_LEN:
        len = b;
        if (len != EXPECT_LEN) { st = FIND_AA; break; }
        payload.clear(); payload.reserve(len);
        st = READ_PAYLOAD;
        break;
      case READ_PAYLOAD:
        payload.push_back(b);
        if (payload.size() == len) st = READ_CRC_LO;
        break;
      case READ_CRC_LO:
        crc_lo = b;
        st = READ_CRC_HI;
        break;
      case READ_CRC_HI: {
        uint8_t crc_hi = b;
        uint8_t tmp[1 + EXPECT_LEN];
        tmp[0] = len;
        std::memcpy(&tmp[1], payload.data(), EXPECT_LEN);
        uint16_t crc = crc16_modbus(tmp, sizeof(tmp));
        if ((crc & 0xFF) == crc_lo && ((crc >> 8) & 0xFF) == crc_hi) {
          float f = 0.f;
          static_assert(sizeof(float) == 4, "float not 4B");
          std::memcpy(&f, payload.data(), 4); // 小端 + IEEE-754 假设
          out = f;
          st = FIND_AA;
          return true;
        } else {
          st = FIND_AA; // CRC 错，丢弃重来
        }
        break;
      }
    }
    return false;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "airspeed_serial_node");
  ros::NodeHandle nh("~");

  std::string port = "/dev/ttyUSB0";
  std::string topic = "/airspeed/filtered";
  nh.param("port", port, port);
  //nh.param("topic", topic, topic);

  int fd = open_serial_blocking(port);
  if (fd < 0) {
    ROS_FATAL("Open %s failed", port.c_str());
    return 1;
  }
  ROS_INFO("Opened %s @115200 (blocking, 8N1)", port.c_str());

  ros::Publisher pub = nh.advertise<sensor::WindSpeed>(topic, 10);

  Parser parser;
  uint8_t byte;
  while (ros::ok()) {
    // 阻塞读：VMIN=1, VTIME=0 → 没有字节就一直等
    ssize_t n = ::read(fd, &byte, 1);
    if (n == 1) {
      float v = 0.f;
      if (parser.feed(byte, v)) {
        sensor::WindSpeed msg;
        msg.set_value_mps(v);
        pub.publish(msg);
      }
    }
    ros::spinOnce(); // 不会被 read 卡死（每读到字节就走一下）
  }

  ::close(fd);
  return 0;
}
