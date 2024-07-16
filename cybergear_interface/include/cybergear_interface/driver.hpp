#pragma once

#include <rclcpp/rclcpp.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

namespace cybergear_interface{

class Driver{
public:
    Driver(const uint8_t master_id, const uint8_t target_id,
        rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can);

    // セットアップ
    void init_motor(const uint8_t run_mode);
    void set_limit_speed(const float speed);
    void enable_motor();

    // 命令
    void set_position_ref(const float position, const float min, const float max);
    void set_speed_ref(const float speed, const float min, const float max);
    void set_mech_position_to_zero();

    // 基幹情報変更
    void change_motor_can_id(const uint8_t can_id);
    void change_motor_boardrate(const uint8_t value);

    // 取得
    const uint8_t get_run_mode() const { return this->run_mode; }

private:
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;

    void reset_motor();
    void set_run_mode(const uint8_t mode);

    void write_float_data(const uint8_t id, const uint16_t addr, const float value, const float min, const float max);
    void send_command(const uint8_t can_id, const uint8_t cmd_id, const uint16_t option, const uint8_t len, uint8_t * data);


    // 定数
    const uint8_t master_id;
    const uint8_t target_id;

    // 変数
    uint8_t run_mode;
};
}
