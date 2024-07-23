#pragma once

namespace cybergear_defs{

/*命令*/
namespace CMD{
    constexpr uint8_t POSITION = 1;
    constexpr uint8_t RESPONSE = 2;
    constexpr uint8_t ENABLE = 3;
    constexpr uint8_t RESET = 4;
    constexpr uint8_t SET_MECH_POSITION_TO_ZERO = 6;
    constexpr uint8_t CHANGE_CAN_ID = 7;
    constexpr uint8_t RAM_READ = 17;
    constexpr uint8_t RAM_WRITE = 18;
    constexpr uint8_t GET_MOTOR_FAIL = 21;
    constexpr uint8_t CHANGE_BOARDRATE = 22;
}

/*アドレス*/
namespace ADDR{
    constexpr uint16_t RUN_MODE = 0x7005;
    constexpr uint16_t IQ_REF = 0x7006;
    constexpr uint16_t SPEED_REF = 0x700A;
    constexpr uint16_t LIMIT_TORQUE = 0x700B;
    constexpr uint16_t CURRENT_KP = 0x7010;
    constexpr uint16_t CURRENT_KI = 0x7011;
    constexpr uint16_t CURRENT_FILTER_GAIN = 0x7014;
    constexpr uint16_t LOC_REF = 0x7016;
    constexpr uint16_t LIMIT_SPEED = 0x7017;
    constexpr uint16_t LIMIT_CURRENT = 0x7018;
    constexpr uint16_t MECH_POS = 0x7019;
    constexpr uint16_t IQF = 0x701A;
    constexpr uint16_t MECH_VEL = 0x701B;
    constexpr uint16_t VBUS = 0x701C;
    constexpr uint16_t ROTATION = 0x701D;
    constexpr uint16_t LOC_KP = 0x701E;
    constexpr uint16_t SPD_KP = 0x701F;
    constexpr uint16_t SPD_KI = 0x7020;
}

/*モード*/
namespace MODE{
    constexpr uint8_t MOTION = 0x00;
    constexpr uint8_t POSITION = 0x01;
    constexpr uint8_t SPEED = 0x02;
    constexpr uint8_t CURRENT = 0x03;
}

/*リミット*/
namespace LIMIT{
    constexpr float VEL_MIN = -30.f;
    constexpr float VEL_MAX = 30.f;
}

}  // namespace cybergear_defs
