#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#pragma pack(push, 4)

namespace QNX2ROSProtocol{
struct ImuData{
    uint32_t timestamp;
    union{
        float buffer_float[9];
        uint8_t buffer_byte[3][12];
        struct{
            float angle_roll,angle_pitch,angle_yaw;
            float angular_velocity_roll,angular_velocity_pitch,angular_velocity_yaw;
            float acc_x,acc_y,acc_z;
        };
    };
};

struct ImuDataReceived{
    int code;                              ///< Command code  
    int size;                              ///< Command value                             
    int cons_code;                         ///< Command type
    struct ImuData data;
};

/// @brief robot state structe implementation
struct RobotState {
  int robot_basic_state;                 ///< Basic motion state of robot
  int robot_gait_state;                  ///< Robot gait information
  double rpy[3];                         ///< IMU angular 
  double rpy_vel[3];                     ///< IMU angular velocity
  double xyz_acc[3];                     ///< IMU acceleration
  double pos_world[3];                   ///< Position of robot in world coordinate system
  double vel_world[3];                   ///< The speed of robot in the world coordinate system
  double vel_body[3];                    ///< Speed of robot in body coordinate system
  unsigned touch_down_and_stair_trot;    ///< This function has not been activated for the time being. This data is only used to occupy the position
  bool is_charging;                      ///< Not opened temporarily
  unsigned error_state;                  ///< Not opened temporarily
  int robot_motion_state;                ///< Robot action status
  double battery_level;                  ///< Battery Percentage
  int task_state;                        ///< Not opened temporarily
  bool is_robot_need_move;               ///< When the robot is standing in place, whether it is pushed to switch into motion mode
  bool zero_position_flag;               ///< Zero return flag bit
  double ultrasound[2];
};
struct RobotStateReceived {
  int code;                              ///< Command code  
  int size;                              ///< Command value                             
  int cons_code;                         ///< Command type
  struct RobotState data;
};

/// @brief Jonint state structe implementation
struct JointState {
  double LF_Joint;
  double LF_Joint_1;
  double LF_Joint_2;
  double RF_Joint;
  double RF_Joint_1;
  double RF_Joint_2;
  double LB_Joint;
  double LB_Joint_1;
  double LB_Joint_2;
  double RB_Joint;
  double RB_Joint_1;
  double RB_Joint_2;
};
struct JointStateReceived {
  int code;
  int size;
  int cons_code;
  struct JointState data;
};

/// @brief handlestate state structe implementation
struct HandleState {
  double left_axis_forward;              ///< Left rocker y-axis,        range: - 1~1
  double left_axis_side;                 ///< Left rocker x-axis,         range: - 1~1
  double right_axis_yaw;                 ///< right rocker y-axis,        range: - 1~1
  double goal_vel_forward;               ///< Target linear speed in x direction
  double goal_vel_side;                  ///< Target linear speed in y direction
  double goal_vel_yaw;                   ///< Target Yaw angular velocity
};
struct HandleStateReceived {
  int code;
  int size;
  int cons_code;
  struct HandleState data;
};

}
#pragma pack(pop)


#pragma pack(push,1)
namespace NX2APPProtocol{

constexpr uint8_t kHeaderSize = 2;
constexpr uint8_t kChannlSize = 16;
constexpr uint8_t kAxisChannlSize = 4;
constexpr uint8_t kAxisButtonSize = 2;
constexpr uint16_t kJoystickRange = 1000;
constexpr uint16_t kDefaultPort = 43893;
const uint8_t kHeader[kHeaderSize] = {0x55, 0x66};
namespace ControllerType{
  constexpr uint32_t kRetroid = 1;
}
namespace JoystickKeyStatus{
  constexpr bool kReleased = 0; /**< Key is released. */
  constexpr bool kPressed = 1;  /**< Key is pressed. */
}

/// physical botton data
class JoystickHead {
public:
  uint8_t stx[2]; /**< Start-of-text bytes for identifying the start of data. */
  uint8_t ctrl; /**< Control byte for additional flags or information. */
  uint16_t data_len; /**< Length of the data in the packet. */
  uint16_t seq; /**< Sequence number for tracking packet order or identifying duplicates. */
  uint8_t id; /**< Identifier for the type of controller. */
  uint16_t checksum; /**< CRC-16 checksum for data integrity verification. */
};

class Channels{
public:
    union{
        uint8_t data[kChannlSize * sizeof(uint16_t)]; /**< Array representing raw data channels. */
        struct {
            uint16_t buttons[kChannlSize - kAxisChannlSize - kAxisButtonSize]; /**< Array representing button values. */
            int16_t left_axis_x; /**< X-axis value of the left analog stick. */
            int16_t left_axis_y; /**< Y-axis value of the left analog stick. */
            int16_t right_axis_x; /**< X-axis value of the right analog stick. */
            int16_t right_axis_y; /**< Y-axis value of the right analog stick. */
            uint16_t axis_buttons[kAxisButtonSize]; /**< Array representing axis button values. */
        };
    };
};

struct JoystickChannelFrame : public JoystickHead, public Channels
{};

struct RetroidKeys {
    union {
        uint16_t value; /**< Union to represent the keys as a single value. */
        struct {
            uint8_t R1 : 1; /**< R1 button. */
            uint8_t L1 : 1; /**< L1 button. */
            uint8_t start : 1; /**< Start button. */
            uint8_t select : 1; /**< Select button. */

            uint8_t R2 : 1; /**< R2 button. */
            uint8_t L2 : 1; /**< L2 button. */

            uint8_t A : 1; /**< A button. */
            uint8_t B : 1; /**< B button. */
            uint8_t X : 1; /**< X button. */
            uint8_t Y : 1; /**< Y button. */

            uint8_t left : 1; /**< Left button on the directional pad. */
            uint8_t right : 1; /**< Right button on the directional pad. */
            uint8_t up : 1; /**< Up button on the directional pad. */
            uint8_t down : 1; /**< Down button on the directional pad. */

            uint8_t left_axis_button : 1; /**< Left analog stick button. */
            uint8_t right_axis_button : 1; /**< Right analog stick button. */
        };
    };
    union {
        float axis_values[4]; /**< Array representing axis values. */
        struct {
            float left_axis_x; /**< X-axis value of the left analog stick. */
            float left_axis_y; /**< Y-axis value of the left analog stick. */
            float right_axis_x; /**< X-axis value of the right analog stick. */
            float right_axis_y; /**< Y-axis value of the right analog stick. */
        };
    };
};
#pragma pack(pop)

}



#endif
