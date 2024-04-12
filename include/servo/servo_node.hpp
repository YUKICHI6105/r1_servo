#include <rclcpp/rclcpp.hpp>
#include <robomas_plugins/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>

struct ServoArray{
  uint16_t value[8] = {};//サーボの状態
  uint16_t upperValue[4] = {};//サーボの前半
  uint16_t lowerValue[4] = {};//サーボの後半
  std::vector<std::string> mode = {"Toggle","Toggle","Toggle","Toggle","Toggle","Toggle","Toggle","Toggle"};//ToggleかNomalか
  // std::vector<int64_t> button(8,0);//ボタン割り当て
  std::vector<int64_t> button = std::vector<int64_t>(8);
  uint8_t countvalve1[8] = {};//カウントバルブの状態
  uint8_t preButton[8] = {};//ボタンの前の状態
};
