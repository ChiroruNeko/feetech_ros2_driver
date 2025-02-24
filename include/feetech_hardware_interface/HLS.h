#pragma once

//-------EPROM(read only)--------
#define HLS_FIRMWARE_VER_L 0
#define HLS_FIRMWARE_VER_H 1
#define HLS_MODEL_L 3
#define HLS_MODEL_H 4

//-------EPROM(read/write)--------
#define HLS_ID 5
#define HLS_BAUD_RATE 6
#define HLS_SECONDARY_ID 7
#define HLS_RESPONSE_STATUS_LEVEL 8
#define HLS_MIN_ANGLE_LIMIT_L 9
#define HLS_MIN_ANGLE_LIMIT_H 10
#define HLS_MAX_ANGLE_LIMIT_L 11
#define HLS_MAX_ANGLE_LIMIT_H 12
#define HLS_MAX_TEMPERATURE_LIMIT 13
#define HLS_MAX_INPUT_VOLT 14
#define HLS_MIN_INPUT_VOLT 15
#define HLS_MAX_TORQUE_L 16
#define HLS_MAX_TORQUE_H 17
#define HLS_PHASE 18
#define HLS_UNLOADING_CONDITION 19
#define HLS_LED_ALARM_CONDITION 20
#define HLS_P_COEF 21
#define HLS_D_COEF 22
#define HLS_I_COEF 23
#define HLS_MINIMUM_STARTUP_FORCE 24
#define HLS_INTEGRATION_LIMIT 25

#define HLS_CW_DEAD 26   // CW: Clockwise TODO: Match the name on the memory table
#define HLS_CCW_DEAD 27  // CCW: CounterClockwise

#define HLS_PROTECTION_CURRENT_L 28
#define HLS_PROTECTION_CURRENT_H 29
#define HLS_ANGULAR_RESOLUTION 30

#define HLS_OFS_L 31  // Position Correction
#define HLS_OFS_H 32

#define HLS_MODE 33

#define HLS_PROTECTIVE_TORQUE 34
#define HLS_PROTECTION_TIME 35
#define HLS_OVERLOAD_TORQUE 36
#define HLS_SPEED_CLOSED_LOOP_P_COEF 37
#define HLS_OVER_CURRENT_PROTECTION_TIME 38
#define HLS_VELOCITY_CLOSED_LOOP_I_COEF 39

//-------SRAM(read/write)--------
#define HLS_TORQUE_ENABLE 40
#define HLS_ACC 41
#define HLS_GOAL_POSITION_L 42
#define HLS_GOAL_POSITION_H 43
#define HLS_GOAL_TORQUE_L 44
#define HLS_GOAL_TORQUE_H 45
#define HLS_GOAL_SPEED_L 46
#define HLS_GOAL_SPEED_H 47
#define HLS_TORQUE_LIMIT_L 48
#define HLS_TORQUE_LIMIT_H 49
#define HLS_LOCK 55

//-------SRAM(read only)--------
#define HLS_PRESENT_POSITION_L 56
#define HLS_PRESENT_POSITION_H 57
#define HLS_PRESENT_SPEED_L 58
#define HLS_PRESENT_SPEED_H 59
#define HLS_PRESENT_LOAD_L 60
#define HLS_PRESENT_LOAD_H 61
#define HLS_PRESENT_VOLTAGE 62
#define HLS_PRESENT_TEMPERATURE 63
#define HLS_MOVING 66
#define HLS_PRESENT_CURRENT_L 69
#define HLS_PRESENT_CURRENT_H 70
