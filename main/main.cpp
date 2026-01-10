#include <cctype>
#include <iostream>


enum control_mode {
    Duty_Cycle_Set = 0x2050080,
    Speed_Set = 0x2050480,
    Smart_Velocity_Set = 0x20504C0,
    Position_Set = 0x2050C80,
    Voltage_Set = 0x2051080,
    Current_Set = 0x20510C0,
    Smart_Motion_Set = 0x2051480,
    //Position_Set = 0x2050C80,
    Heartbeat_Set = 0x2052C80,
    Device_Specific_Heartbeat = 0x2052480
};

constexpr uint32_t CAN_EFF_FLAG = 0x80000000;

enum status_frame_id {
    status_0 = 0x2051800,
    status_1 = 0x2051840,
    status_2 = 0x205B880,
    status_3 = 0x20518C0,
    status_4 = 0x2051900
};

struct can_frame {
    uint32_t can_id;
};

// Heartbeat Frame
const uint32_t HEARTBEAT_ID = 0x2052C80;  // 0x2052C80
const uint8_t HEARTBEAT_DATA[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
const uint8_t HEARTBEAT_SIZE = 8;


int main() {
    std::cout << "hello, world\n";

}