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
  std::vector<int64_t> button = {2,2,2,4,1,1,1,1};
  uint8_t countvalve1[8] = {};//カウントバルブの状態
  uint8_t preButton[8] = {};//ボタンの前の状態
  std::vector<int64_t> ccr1 = {0,40000,20000,17000,30000,30000,18000,18000};//CCR1の値
  std::vector<int64_t> ccr2 = {0,20000,40000,32000,18000,18000,30000,30000};//CCR2の値
};
