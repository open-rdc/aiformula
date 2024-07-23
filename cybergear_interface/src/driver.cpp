#include "cybergear_interface/driver.hpp"
#include "cybergear_interface/cybergear_defs.h"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"

using namespace utils;
using namespace cybergear_defs;

namespace cybergear_interface{

Driver::Driver(const uint8_t master_id, const uint8_t target_id,
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can)
: master_id(master_id), target_id(target_id), publisher_can(publisher_can)
{

}

// セットアップ
void Driver::init_motor(const uint8_t run_mode){
    reset_motor();
    set_run_mode(run_mode);
}
void Driver::set_limit_speed(const float speed){
    write_float_data(target_id, ADDR::LIMIT_SPEED, speed, 0.0f, LIMIT::VEL_MAX);
}
void Driver::enable_motor(){
    uint8_t data[8] = {0x00};
    send_command(target_id, CMD::ENABLE, master_id, 8, data);
}

// 基幹情報変更
void Driver::change_motor_can_id(const uint8_t can_id){
  uint8_t data[8] = {0x00};
  uint16_t option = can_id << 8 | master_id;
  send_command(target_id, CMD::CHANGE_CAN_ID, option, 8, data);
}
void Driver::change_motor_boardrate(const uint8_t value){
  uint8_t data[8] = {0x00};
  data[0] = value;
  send_command(target_id, CMD::CHANGE_BOARDRATE, master_id, 8, data);
}

// 命令
void Driver::set_position_ref(const float position, const float min, const float max){
    write_float_data(target_id, ADDR::LOC_REF, position, min, max);
}
void Driver::set_speed_ref(const float speed, const float min, const float max){
    write_float_data(target_id, ADDR::SPEED_REF, speed, min, max);
}

void Driver::set_mech_position_to_zero(){
    uint8_t data[8] = {0x00};
    data[0] = 0x01;
    send_command(target_id, CMD::SET_MECH_POSITION_TO_ZERO, master_id, 8, data);
}


/*プライベート関数*/
// 設定
void Driver::reset_motor(){
    uint8_t data[8] = {0x00};
    send_command(target_id, CMD::RESET, master_id, 8, data);
}
void Driver::set_run_mode(const uint8_t mode){
    this->run_mode = mode;
    uint8_t data[8] = {0x00};
    data[0] = ADDR::RUN_MODE & 0x00FF;
    data[1] = ADDR::RUN_MODE >> 8;
    data[4] = run_mode;
    send_command(target_id, CMD::RAM_WRITE, master_id, 8, data);
}

// 通信
void Driver::write_float_data(const uint8_t id, const uint16_t addr, const float value, const float min, const float max){
    uint8_t data[8] = {0x00};
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;

    const float value_ = constrain(value, min, max);
    memcpy(&data[4], &value_, 4);
    send_command(id, CMD::RAM_WRITE, master_id, 8, data);
}
void Driver::send_command(const uint8_t id, const uint8_t cmd_id, const uint16_t option, const uint8_t len, uint8_t * data){
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    // 出版
    msg_can->canid = cmd_id << 24 | option << 8 | id;
    msg_can->candlc = len;
    msg_can->eff = true;

    for(int i=0; i<msg_can->candlc; i++) msg_can->candata[i] = data[i];
    publisher_can->publish(*msg_can);
}

}
