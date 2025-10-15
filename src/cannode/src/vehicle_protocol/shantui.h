#pragma once

#include <cstdint>
#include <linux/can.h>
#include "../chassis_driver.h"
namespace cannode {
enum CanProtoId {
    CCU_CMD_1 = 0X18501E50, // 上位机→VCU 控制命令1 角度 模式 挡位
    CCU_CMD_2 = 0x18502E50, // 上位机→VCU 控制命令2 ON电 驻车 油门 制动 灯光

    V2HMI_VehState1 = 0X18FF2021, // VCU→上位机 挡位车速高压状态
    V2HMI_VehState3 = 0X18FF2221, // VCU→上位机 蓄电池状态
    ESW_VCU_Info1 = 0x04854001, // VCU→上位机 线控转向转速状态
    VCU_ACU_Info1 = 0x18501F50, // VCU→上位机 角度 模式 灯光
    VCU_ACU_Info2 = 0x18502F50, // VCU→上位机 动臂铲斗压力状态
    VCU_ACU_Info3 = 0x18503F50, // VCU→上位机 故障状态
    V2HMI_CoursePara2 = 0X18DE1021, // VCU→上位机 油门制动开度状态
    V2HMI_LMCU1 = 0X18140021, // VCU→上位机 左电机状态
    V2HMI_LMCU2 = 0X18140121, // VCU→上位机 左电机状态
    V2HMI_RMCU1 = 0X18141021, // VCU→上位机 右电机状态
    V2HMI_RMCU2 = 0X18141121, // VCU→上位机 右电机状态
    V2HMI_ISG1 = 0X18240021, // VCU→上位机 ISG液压电机状态
    V2HMI_ISG2 = 0X18240121, // VCU→上位机 ISG液压电机状态
};

#pragma pack(push, 1)

// ================================
// 上位机→VCU 控制命令定义
// ================================
// 上位机→VCU 控制命令1 (0X18501E50)
typedef struct CanCtrlCmd1 {
    // byte0
    uint8_t ACU_Life1 : 4;      // 整车上高压
    uint8_t rev1 : 4;      // 预留
    // byte1
    uint8_t ACU_TurnCmd;         // 转向命令
    // byte2-3
    uint16_t ACU_ArmCmd : 10;        // 动臂角度指令
    uint16_t rev2 : 2;        // 预留
    uint16_t ACU_ArmEn : 1;        // 预留
    uint16_t rev3 : 3;        // 预留
    // byte4-5
    uint16_t ACU_BucketCmd : 10;       // 铲斗角度指令
    uint16_t rev4 : 2;        // 预留
    uint16_t ACU_BucketEn : 1;       // 铲斗使能
    uint16_t rev5 : 3;       // 预留
    // byte6
    uint8_t ACU_Tgrspd;       // 目标车速
    // byte7
    uint8_t ACU_DrvModeReq : 2;        // 请求接管模式
    uint8_t ACU_DrvGear : 2;         // FNR档位开关
    uint8_t ACU_EStop : 1;      // 急停
    uint8_t ACU_RapidShake : 1; // 快速抖动
    uint8_t rev6 : 2;         // 预留
}ST0x18501E50Struct;

// 上位机→VCU 控制命令2 (0x18502E50) ON电 驻车 油门 制动 灯光
typedef struct CanCtrlCmd2 {
    // byte 0
    uint8_t ACU_Life2 : 4;           // 心跳, bit 0-3
    uint8_t ACU_KeyOn : 1;           // 钥匙开关, bit 4
    uint8_t ACU_KeyStart : 1;        // 起动开关, bit 5
    uint8_t ACU_ParkReq : 1;         // P档驻车请求, bit 6
    uint8_t ACU_Horn : 1;            // 喇叭, bit 7
    // byte 1-2
    uint16_t ACU_AccPedlPct : 12;  // 油门踏板请求开度 bit 8-19
    uint16_t ACU_TurnLeftLgt : 1;     // 左转向灯控制, bit 20
    uint16_t ACU_TurnRightLgt : 1;    // 右转向灯控制, bit 21
    uint16_t ACU_FWorkLgt : 1;        // 前工作灯控制, bit 22
    uint16_t ACU_BWorkLgt : 1;        // 后工作灯控制, bit 23
    // byte 3
    uint8_t ACU_BrakePct;             // 刹车踏板开度, 8位, bit 24-31 (整个字节)
    // byte 4-7 (预留，可选)
    uint8_t rev2[4];                  // 预留字节，防止访问越界或对齐问题
}ST0x18502E50Struct;

// ================================
// VCU→上位机 信息定义
// ================================
// VCU→上位机 (0X18FF2021) 挡位车速高压状态
typedef struct CanFeedbackStatus1 {
    // byte 0
    uint8_t V2HMI_St_CurrPRNDSt : 4;     // 当前方向档位信息, bit 0-3
    uint8_t rev1 : 4;                    // 预留
    // byte 1
    uint8_t V2HMI_St_CurrGearSt;         // 当前速度档位信息, bit 8-15 (整个字节)
    // byte 2
    uint8_t rev2;                        // 预留字节 (bit 16-23)
    // byte 3-4
    uint16_t V2HMI_V_VehicleSpeed;         // 车速字节
    // byte 5
    uint8_t rev3 : 3;                    // bit 40-42
    uint8_t V2HMI_St_VehicleFaultLvl : 2; // 整车故障等级, bit 43-44
    uint8_t V2HMI_St_EconomyState : 1;   // 节能模式, bit 45
    uint8_t rev4 : 2;
    // byte 6
    uint8_t rev5;
    // byte 7
    uint8_t V2HMI_St_VehLoadMode : 4; // 整车载重模式高2位, bit 56-59
    uint8_t V2HMI_St_VehHVSTM : 4;     // 整车高压状态低4位, bit 50-53
}ST0X18FF2021Struct;

// VCU→上位机 (0X18FF2221) 蓄电池状态
typedef struct CanFeedbackStatus2 {
    // byte 0
    uint16_t V2HMI_St_VehLVSt : 4;              // 整车低压状态, bit 0-3
    uint16_t V2HMI_U_24VBatteryVoltage : 12;    // 蓄电池电压 bit 4-15
    // byte 1-7 (预留)
    uint8_t rev[6];                            // 预留字节，填充到8字节
}ST0X18FF2221Struct;
// VCU→上位机 (0x04854001) 线控转向转速状态（注意：motorola格式）
typedef struct CanMotorStatus {
    // byte 0-1: Current (16位, 起始位8)
    uint8_t  Current_high;    // byte0, bit8-15 (高8位)
    uint8_t  Current_low;     // byte1, bit0-7  (低8位)
    // byte 2-3: ActRotSpd (16位, 起始位24)
    uint8_t  ActRotSpd_high;  // byte2, bit24-31
    uint8_t  ActRotSpd_low;   // byte3, bit16-23
    // byte 4-5: TrgRotSpd (16位, 起始位40)
    uint8_t  TrgRotSpd_high;  // byte4, bit40-47
    uint8_t  TrgRotSpd_low;   // byte5, bit32-39
    // byte 6-7: FaultCode (16位, 起始位56)
    uint8_t  FaultCode_high;  // byte6, bit56-63
    uint8_t  FaultCode_low;   // byte7, bit48-55
} ST0X04854001Struct;
// VCU→上位机 (0x18501F50) 角度 模式 灯光
typedef struct {
    // byte 0
    uint8_t  VCU_Life1      : 4;    // 心跳, bit 0-3
    uint8_t  VCU_FlgMdVeh   : 2;    // 驾驶模式, bit 4-5
    uint8_t  rev1           : 2;    // 预留, bit 6-7
    // byte 1
    uint8_t  VCU_TurnLeftLgtSt   : 1;  // 左转向灯状态, bit 8
    uint8_t  VCU_TurnRightLgtSt  : 1;  // 右转向灯状态, bit 9
    uint8_t  VCU_FWorkLgtSt      : 1;  // 前工作灯状态, bit 10
    uint8_t  VCU_BWorkLgtSt      : 1;  // 后工作灯状态, bit 11
    uint8_t  VCU_FarLgtSt        : 1;  // 远光灯状态, bit 12
    uint8_t  VCU_HonrSt          : 1;  // 喇叭状态, bit 13
    uint8_t  VCU_EStopSt         : 1;  // 急停状态, bit 14
    uint8_t  VCU_WorkLockSt      : 1;  // 工作装置锁定状态, bit 15
    // byte 2-3: VCU_ArmAgl (10位, bit 16-25)
    uint16_t VCU_ArmAgl     : 10;   // 动臂角度状态, bit 16-25
    uint16_t rev2           : 6;    // 预留, bit 26-31
    // byte 4-5: VCU_BucketAgl (10位, bit 32-41)
    uint16_t VCU_BucketAgl  : 10;   // 铲斗角度状态, bit 32-41
    uint16_t rev3           : 6;    // 预留, bit 42-47
    // byte 6-7: VCU_TurnLRAgl (10位, bit 48-57)
    uint16_t VCU_TurnLRAgl  : 10;   // 转向角度状态, bit 48-57
    uint16_t rev4           : 6;    // 预留, bit 58-63
} ST0X18501F50Struct;
// VCU→上位机 (0x18502F50) 动臂铲斗压力状态
typedef struct {
    // byte 0
    uint8_t  VCU_Life2         : 4;    // 心跳, bit 0-3
    uint8_t  rev1              : 4;    // 预留, bit 4-7
    // byte 1
    uint8_t  VCU_ArmUpPress;           // 动臂举升压力, bit 8-15 (8位)
    // byte 2
    uint8_t  VCU_ArmDwnPress;          // 动臂下降压力, bit 16-23 (8位)
    // byte 3
    uint8_t  VCU_BucketUpPress;        // 收斗压力, bit 24-31 (8位)
    // byte 4
    uint8_t  VCU_BucketDwnPress;       // 翻斗压力, bit 32-39 (8位)
    // byte 5
    uint8_t  VCU_PutDownSt     : 2;    // 一键放平状态, bit 40-41
    uint8_t  VCU_ScoopSt       : 2;    // 一键铲料状态, bit 42-43
    uint8_t  VCU_PutUpSt       : 2;    // 一键举升状态, bit 44-45
    uint8_t  VCU_UnloadSt      : 2;    // 一键卸料状态, bit 46-47
    // byte 6-7 (预留)
    uint8_t  rev2[2];                  // 预留字节, bit 48-63
} ST0X18502F50Struct;
// VCU→上位机 (0x18503F50) 故障状态
typedef struct {
    // byte 0
    uint8_t  VCU_Life3         : 4;    // 心跳, bit 0-3
    uint8_t  VCU_ACUOffLine    : 1;    // 无人控制器离线故障, bit 4
    uint8_t  VCU_RCUOffLine    : 1;    // 遥控控制器离线故障, bit 5
    uint8_t  VCU_ICMOffLine    : 1;    // 仪表离线故障, bit 6
    uint8_t  VCU_SensOffLine   : 1;    // 角度传感器离线故障, bit 7

    // byte 1
    uint8_t  VCU_HonrFlt       : 1;    // 喇叭输出故障, bit 8
    uint8_t  VCU_FWorkLgtFlt   : 1;    // 前工作灯输出故障, bit 9
    uint8_t  VCU_BWorkLgtFlt   : 1;    // 后工作灯输出故障, bit 10
    uint8_t  VCU_FarLgtFlt     : 1;    // 远光灯输出故障, bit 11
    uint8_t  VCU_LeftLgtFlt    : 1;    // 左转向灯输出故障, bit 12
    uint8_t  VCU_RightLgtFlt   : 1;    // 右转向灯输出故障, bit 13
    uint8_t  VCU_WorkLockFlt   : 1;    // 工作装置锁输出故障, bit 14
    uint8_t  VCU_NearLgtFlt    : 1;    // 近光灯输出故障, bit 15

    // byte 2
    uint8_t  VCU_WarmLgtFlt    : 1;    // 警示灯输出故障, bit 16
    uint8_t  VCU_ESWOffLine    : 1;    // 方向盘离线故障, bit 17
    uint8_t  VCU_RadarOffLine  : 1;    // 雷达离线故障, bit 18
    uint8_t  VCU_ECOOffLine    : 1;    // 整车VCU通讯失败, bit 19
    uint8_t  rev1              : 4;    // 预留, bit 20-23

    // byte 3-7 (预留)
    uint8_t  rev2[5];                  // 预留字节, bit 24-63

} ST0X18503F50Struct;
// VCU→上位机 (0x18DE1021) 油门踏板、制动踏板开度
typedef struct {
    // byte 0
    uint8_t  V2HMI_Pct_AccPedl;        // 油门踏板开度值, bit 0-7 (8位)
    // byte 1
    uint8_t  V2HMI_Pct_BrkPedl;        // 制动踏板开度值, bit 8-15 (8位)
    // byte 2-7 (预留)
    uint8_t  rev[6];                   // 预留字节, bit 16-63
} ST0X18DE1021Struct;
// VCU→上位机 (0x18140021) 左行走电机状态
typedef struct {
    // byte 0-1: V2HMI_N_LMCUActuSpeed (16位, bit 0-15)
    uint16_t V2HMI_N_LMCUActuSpeed;        // 左行走电机实际转速, bit 0-15
    // byte 2-3: V2HMI_Trq_LMCUActuTorque (16位, bit 16-31)
    uint16_t V2HMI_Trq_LMCUActuTorque;     // 左行走电机实际扭矩, bit 16-31
    // byte 4-5: V2HMI_U_LMCUActuVoltage (16位, bit 32-47)
    uint16_t V2HMI_U_LMCUActuVoltage;      // 左行走电机实际电压, bit 32-47
    // byte 6-7: V2HMI_I_LMCUActuCurrent (16位, bit 48-63)
    uint16_t V2HMI_I_LMCUActuCurrent;      // 左行走电机实际电流, bit 48-63
} ST0X18140021Struct;
// VCU→上位机 (0x18140121) 左行走电机状态
typedef struct {
    // byte 0
    uint8_t  V2HMI_Temp_LMoterTemp;         // 左行走电机温度, bit 0-7 (8位)
    // byte 1
    uint8_t  V2HMI_Temp_LIGBTTemp;          // 左行走电机逆变器温度, bit 8-15 (8位)
    // byte 2
    uint8_t  rev1          : 4;             // 预留, bit 16-19
    uint8_t  V2HMI_St_LMCUFaultLvl : 4;     // 左行走电机故障等级, bit 20-23
    // byte 3-7 (预留)
    uint8_t  rev2[5];                       // 预留字节, bit 24-63
} ST0X18140121Struct;
// VCU→上位机 (0x18141021) 右行走电机状态
typedef struct {
    // byte 0-1: V2HMI_N_RMCUActuSpeed (16位, bit 0-15)
    uint16_t V2HMI_N_RMCUActuSpeed;        // 右行走电机实际转速, bit 0-15
    // byte 2-3: V2HMI_Trq_RMCUActuTorque (16位, bit 16-31)
    uint16_t V2HMI_Trq_RMCUActuTorque;     // 右行走电机实际扭矩, bit 16-31
    // byte 4-5: V2HMI_U_RMCUActuVoltage (16位, bit 32-47)
    uint16_t V2HMI_U_RMCUActuVoltage;      // 右行走电机实际电压, bit 32-47
    // byte 6-7: V2HMI_I_RMCUActuCurrent (16位, bit 48-63)
    uint16_t V2HMI_I_RMCUActuCurrent;      // 右行走电机实际电流, bit 48-63
} ST0X18141021Struct;
// VCU→上位机 (0x18141121) 右行走电机状态
typedef struct {
    // byte 0
    uint8_t  V2HMI_Temp_RMoterTemp;         // 右行走电机温度, bit 0-7 (8位)
    // byte 1
    uint8_t  V2HMI_Temp_RIGBTTemp;          // 右行走电机逆变器温度, bit 8-15 (8位)
    // byte 2
    uint8_t  rev1          : 4;             // 预留, bit 16-19
    uint8_t  V2HMI_St_RMCUFaultLvl : 4;     // 右行走电机故障等级, bit 20-23
    // byte 3-7 (预留)
    uint8_t  rev2[5];                       // 预留字节, bit 24-63
} ST0X18141121Struct;
// VCU→上位机 (0X18240021) 液压工作电机状态
typedef struct {
    // byte 0-1: V2HMI_N_ISGActuSpeed (16位, bit 0-15)
    uint16_t V2HMI_N_ISGActuSpeed;        // 液压工作电机实际转速, bit 0-15

    // byte 2-3: V2HMI_Trq_ISGActuTorque (16位, bit 16-31)
    uint16_t V2HMI_Trq_ISGActuTorque;     // 液压工作电机实际扭矩, bit 16-31

    // byte 4-5: V2HMI_U_ISGActuVoltage (16位, bit 32-47)
    uint16_t V2HMI_U_ISGActuVoltage;      // 液压工作电机实际电压, bit 32-47

    // byte 6-7: V2HMI_I_ISGActuCurrent (16位, bit 48-63)
    uint16_t V2HMI_I_ISGActuCurrent;      // 液压工作电机实际电流, bit 48-63

} ST0X18240021Struct;
// VCU→上位机 (0X18240121) 液压工作电机状态
typedef struct {
    // byte 0
    uint8_t  V2HMI_Temp_ISGMoterTemp;         // 液压工作电机温度, bit 0-7 (8位)
    // byte 1
    uint8_t  V2HMI_Temp_ISGIGBTTemp;          // 液压工作电机逆变器温度, bit 8-15 (8位)
    // byte 2
    uint8_t  rev1              : 4;           // 预留, bit 16-19
    uint8_t  V2HMI_St_ISGFaultLvl : 4;        // 液压工作电机故障等级, bit 20-23
    // byte 3-7 (预留)
    uint8_t  rev2[5];                         // 预留字节, bit 24-63
} ST0X18240121Struct;

#pragma pack(pop)


union STCanFrameMsg {
  //cmd1 角度 模式 挡位
  ST0x18501E50Struct ST0x18501E50St;
  //cmd2 ON电 驻车 油门 制动 灯光
  ST0x18502E50Struct ST0x18502E50St;

  //info1 挡位车速高压状态
  ST0X18FF2021Struct ST0X18FF2021St;
  //info2 蓄电池状态
  ST0X18FF2221Struct ST0X18FF2221St;
  //info3 线控转向转速状态
  ST0X04854001Struct ST0X04854001St;
  //info4 角度 模式 灯光
  ST0X18501F50Struct ST0X18501F50St;
  //info5 动臂铲斗压力状态
  ST0X18502F50Struct ST0X18502F50St;
  //info6 故障状态
  ST0X18503F50Struct ST0X18503F50St;
  //info7 油门制动开度状态
  ST0X18DE1021Struct ST0X18DE1021St;
  //info8 左电机状态1
  ST0X18140021Struct ST0X18140021St;
  //info9 左电机状态2
  ST0X18140121Struct ST0X18140121St;
  //info10 右电机状态1
  ST0X18141021Struct ST0X18141021St;
  //info11 右电机状态2
  ST0X18141121Struct ST0X18141121St;
  //info12 ISG液压电机状态1
  ST0X18240021Struct ST0X18240021St;
  //info13 ISG液压电机状态2
  ST0X18240121Struct ST0X18240121St;
  uint8_t canFrameData[8];
};

class ShantuiCANParser: public ChassisCan
{
public:
    ShantuiCANParser(std::string canPort);
    ~ShantuiCANParser() = default;
    void initFuncMap() override;
    void SendCmdFunc() override;

    void sendCtrlInfo1();
    void sendCtrlInfo2();

    void handle0x18FF2021(struct Canframe *recvCanFrame);
    void handle0x18FF2221(struct Canframe *recvCanFrame);
    void handle0x04854001(struct Canframe *recvCanFrame);
    void handle0x18501F50(struct Canframe *recvCanFrame);
    
    void checkdatastructure();
private:
    control::canbus::Chassis::GearPosition gear_state_to_proto(uint8_t gear_state);
    control::canbus::Chassis::DrivingMode driving_mode_to_proto(uint8_t driving_mode);
    // 解析具体的数据字段
    //std::optional<ShantuiData> ParseDataField(const std::array<uint8_t, 8>& data);
};
} // namespace cannode
