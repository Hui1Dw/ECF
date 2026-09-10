#ifndef MOTOR_CORE_MOTOR_TYPES_HPP_
#define MOTOR_CORE_MOTOR_TYPES_HPP_

#include <cstdint>

// DM motor parameter limits.
#ifndef PI
#define PI 3.1415926535f
#endif

#ifndef RAD_2_DEGREE
#define RAD_2_DEGREE 57.2957795f
#define DEGREE_2_RAD 0.01745329252f
#endif

#ifndef RPM_2_RAD_PER_SEC
#define RPM_2_RAD_PER_SEC 0.104719755f
#endif

#ifndef RPM_2_ANGLE_PER_SEC
#define RPM_2_ANGLE_PER_SEC 6.0f
#endif

#define DM_P_MIN (-12.5f)
#define DM_P_MAX 12.5f
#define DM_V_MIN (-45.0f)
#define DM_V_MAX 45.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f
#define DM_T_MIN (-10.0f)
#define DM_T_MAX 10.0f

// DM motor reduction ratio.
#define DM4310_RATIO 10
#define DM4340_RATIO 40
#define DM6220_RATIO 1
#define DM8006_RATIO 6
#define DM8009_RATIO 9
#define DM10010_RATIO 10
#define DM10010L_RATIO 10
#define DM2325_RATIO 25
#define DM3519_RATIO 19.2

typedef enum {
    DM4310 = 0,
    DM4340,
    DM6220,
    DM8006,
    DM8009,
    DM10010,
    DM10010L,
    DM2325,
    DM3519,
    DM3507 = DM3519,
    DM_TYPE_COUNT
} DM_MotorType_e;

typedef enum {
    DM_MIT_MODE = 0x00,
    DM_ENABLE = 0xFC,
    DM_DISABLE = 0xFD,
    DM_SAVE_POS_ZERO = 0xFE,
    DM_CLEAR_ERROR = 0xFB,
    DM_SET_ZERO = DM_SAVE_POS_ZERO  // legacy alias
} DM_Control_Mode_e;

typedef enum {
    DM_ERR_DISABLED = 0x0,           // 失能
    DM_ERR_ENABLED = 0x1,            // 使能
    DM_ERR_OVERVOLTAGE = 0x8,        // 超压
    DM_ERR_UNDERVOLTAGE = 0x9,       // 欠压
    DM_ERR_OVERCURRENT = 0xA,        // 过电流
    DM_ERR_MOS_OVERTEMP = 0xB,       // MOS过温
    DM_ERR_COIL_OVERTEMP = 0xC,      // 电机线圈过温
    DM_ERR_COMM_LOSS = 0xD,          // 通讯丢失
    DM_ERR_OVERLOAD = 0xE,           // 过载
    DM_ERR_UNKNOWN = 0xFF,           // 未知错误
} DM_ErrorCode_e;

// DM parameter read/write/store protocol (J10010L protocol).
constexpr uint16_t DM_PARAM_FRAME_ID = 0x7FF;
constexpr uint8_t DM_PARAM_READ = 0x33;
constexpr uint8_t DM_PARAM_WRITE = 0x55;
constexpr uint8_t DM_PARAM_STORE = 0xAA;
constexpr uint8_t DM_PARAM_RID_TMAX = 23;

typedef struct {
    float KP;
    float KD;
    float MAX_VEL;
    float MAX_POS;
    float MAX_TOR;
    float RATIO;
} DM_Para_Set;

typedef struct {
    float Position;
    float Velocity;
    float Current;
    float Torque;
    float T_MOS;
    float T_Rotor;
    DM_ErrorCode_e Error_Code;
} DM_Data_Info;

typedef struct {
    float pos;
    float vel;
    float kp;
    float kd;
    float torq;
} DM_MIT_Control_t;

enum class MotorDriverType {
    kDm = 0,
    kEncos = 1,
};

struct MotorCommand {
    float torque;
    float angle;
    float speed;
    float kp;
    float kd;
};

struct MotorFeedback {
    float torque;
    float pos;
    float vel;
    float mos_temp;
    float rotor_temp;
    uint8_t error_code;
    uint8_t enabled;
    uint8_t online;
};

#endif  // MOTOR_CORE_MOTOR_TYPES_HPP_
