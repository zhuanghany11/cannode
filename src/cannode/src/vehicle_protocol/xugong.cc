#if 0
void ChassisCan::Start()
{
    // 启动后进入自身线程执行 Run
    threadPool_.emplace_back(&ChassisCan::Run, this);
}
 
/**
 * @brief 启动CAN总线，并启动接收和发送线程,
 * @return void
*/
void ChassisCan::Run()
{
    // 尝试连接CAN总线，最多尝试3次
    int retryCount = 0;
    const int maxRetries = 3;
    
    while (retryCount < maxRetries && !is_connect_) {
        if (ConnectDevice()) {
            is_connect_ = true;
            std::cout << "CAN device connected successfully." << std::endl;
            break;
        } else {
            retryCount++;
            std::cout << "Failed to connect to CAN device. Retry " << retryCount << "/" << maxRetries << std::endl;
            if (retryCount < maxRetries) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    
    if (!is_connect_) {
        std::cerr << "Failed to connect to CAN device after " << maxRetries << " attempts." << std::endl;
        return;
    }
    std::cout << "start cannode process." << std::endl;
}
int ChassisCan::WriteCanInfo(uint32_t can_id, const void *data, canDemo::SocketCanInterface* socket_can) {
    CanFrame frame;
    struct can_frame can_frame;

    switch (can_id) {
        case HOST_TO_VCU_ID_1:
            pack_host_to_vcu1((HostToVCU1 *)data, frame.data);
            break;
        case HOST_TO_VCU_ID_2:
            pack_host_to_vcu2((HostToVCU2 *)data, frame.data);
            break;
        case HOST_TO_VCU_ID_B0EE:
            pack_host_to_vcu_b0ee((HostToVCUB0EE *)data, frame.data);
            break;
        case HOST_TO_VCU_ID_B1EF:
            pack_host_to_vcu_b1ef((HostToVCUB1EF *)data, frame.data);
            break;
        default:
            return -1; // 不支持的 ID
    }

    // TODO 最好是在参数列表中增加一个扩展帧 标准帧的flag
    if (can_id > 0x7FF) {
        frame.can_id = can_id | CAN_EFF_FLAG;
    }
    frame.dlc = 8; // 默认数据长度为 8 字节

    // 转换为socket_can格式
    can_frame.can_id = frame.can_id;
    can_frame.len = frame.dlc;
    memcpy(can_frame.data, frame.data, sizeof(can_frame.data));
    
    socket_can->WriteCan(can_frame);
    return 0; // 返回成功
}

void ChassisCan::ThreadCanSocketTx()
{
    auto last_time = std::chrono::steady_clock::now();
    const auto cycle_time = std::chrono::milliseconds(1000); // 100Hz = 10ms周期
    std::cout << "start write can process." << std::endl;
    while (rclcpp::ok()) {
        if (is_ready_) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);
            
            if (elapsed_time >= cycle_time) {
                last_time = current_time;
                
                // 发送Host到VCU的指令
                WriteCanInfo(HOST_TO_VCU_ID_1, &set_vehicle_info_, &socket_can_);
                WriteCanInfo(HOST_TO_VCU_ID_2, &set_boom_lift_info_, &socket_can_);
                WriteCanInfo(HOST_TO_VCU_ID_B0EE, &set_walk_motor_info_, &socket_can_);
                WriteCanInfo(HOST_TO_VCU_ID_B1EF, &set_hydraulic_motor_info_, &socket_can_);
                std::cout << "send VCU data, per 100ms." << std::endl;
            }
        }
        
        // 短暂休眠以满足100Hz频率要求
        std::this_thread::sleep_for(std::chrono::microseconds(1000)); // 1ms
    }
}

void ChassisCan::ThreadCanSocketRx()
{
    struct can_frame canFrame;
    struct timeval timestamp;
    int32_t readBytes;
    static uint8_t flag = 0;
    std::cout << "start read can process." << std::endl;
    while (rclcpp::ok()) {
        std::cout << "read can data." << std::endl;
        // 读取CAN数据
        if (socket_can_.ReadCan(canFrame, timestamp, readBytes) == 0) {
            canFrame.can_id &= CAN_EFF_MASK;
            std::cout << "data[" << (uint32_t)canFrame.data[0] << " " << (uint32_t)canFrame.data[1] << " "
                                 << (uint32_t)canFrame.data[2] << " " << (uint32_t)canFrame.data[3] << " "
                                 << (uint32_t)canFrame.data[4] << " " << (uint32_t)canFrame.data[5] << " "
                                 << (uint32_t)canFrame.data[6] << " " << (uint32_t)canFrame.data[7] << " " <<"]"<<std::endl;
            std::cout << "flag: " << (uint32_t)flag <<", is_ready_:" << (uint32_t)is_ready_ << std::endl;
            std::cout << std::hex << canFrame.can_id << std::endl; 

            // 根据CAN ID解析数据
            switch (canFrame.can_id) {
                case VCU_TO_HOST_ID_1:
                    parse_vcu_to_host1(canFrame.data, &vehicle_info_);
                    flag |= 0x1;
                    flag = 0xFF;
                    std::cout << "read can 01." << std::endl;
                    break;
                case VCU_TO_HOST_ID_2:
                    parse_vcu_to_host2(canFrame.data, &current_Info_);
                    flag |= 0x2;
                    break;
                case VCU_TO_HOST_ID_3:
                    parse_vcu_to_host3(canFrame.data, &pressure_Info_);
                    flag |= 0x4;
                    break;
                case VCU_TO_HOST_ID_4:
                    parse_vcu_to_host4(canFrame.data, &hydraulic_motor_Info_);
                    flag |= 0x8;
                    break;
                case VCU_TO_HOST_ID_5:
                    parse_vcu_to_host5(canFrame.data, &walk_motor_Info_);
                    flag |= 0x10;
                    break;
                case VCU_TO_HOST_ID_6:
                    parse_vcu_to_host6(canFrame.data, &status_Info_);
                    flag |= 0x20;
                    break;
                case VCU_TO_HOST_ID_9:
                    parse_vcu_to_host9(canFrame.data, &rear_front_motor_Info_);
                    flag |= 0x40;
                    break;
                case VCU_TO_HOST_ID_A:
                    parse_vcu_to_hostA(canFrame.data, &angle_Info_);
                    flag |= 0x80;
                    break;
                default:
                    // 不处理的ID
                    break;
            }
            // 收到VCU到Host的数据后，初始化Host到VCU的信息
            if (!is_ready_ && flag == 0xFF) {
                // 这里可以添加初始化逻辑
                is_ready_ = true;
                HostToVCUInfoInit();  // 使用Vcu上报的状态信息，给set_xxx_的值进行初始化
                std::cout << "Received VCU data, system is ready." << std::endl;
            }

            if (is_ready_) {  // 系统已就绪
                // 发布底盘状态消息
                PublishChassisStatus();
            }
        }
        
        // 短暂休眠以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::microseconds(1000)); // 1ms
    }
}

void ChassisCan::HostToVCUInfoInit()
{
    // 使用VCU上报的状态信息，初始化HostToVCU结构体
    
    // 初始化HostToVCU1
    set_vehicle_info_.high_voltage = vehicle_info_.high_voltage_status;
    set_vehicle_info_.parking_brake = vehicle_info_.parking_brake;
    set_vehicle_info_.emergency_mode = status_Info_.vehicle_mode_2 = 3 ? 0:1; // 默认值
    set_vehicle_info_.left_turn_signal = vehicle_info_.left_turn_signal;
    set_vehicle_info_.right_turn_signal = vehicle_info_.right_turn_signal;
    set_vehicle_info_.walk_motor_mode = vehicle_info_.walk_motor_mode;
    set_vehicle_info_.mode_switch = 0; // 默认值
    set_vehicle_info_.hydraulic_lock = vehicle_info_.hydraulic_lock;
    set_vehicle_info_.heartbeat_signal = vehicle_info_.heartbeat_status;
    set_vehicle_info_.turtle_rabbit_gear = vehicle_info_.turtle_rabbit_gear;
    set_vehicle_info_.work_light = vehicle_info_.work_light;
    set_vehicle_info_.low_voltage_signal = 1; // 默认值
    set_vehicle_info_.high_beam = status_Info_.high_beam;
    set_vehicle_info_.low_beam = status_Info_.low_beam;
    set_vehicle_info_.horn = vehicle_info_.horn_status;
    set_vehicle_info_.rotation_alarm = vehicle_info_.rotation_alarm;
    set_vehicle_info_.left_turn_valve = 0; // 默认值
    set_vehicle_info_.right_turn_valve = 0; // 默认值
    set_vehicle_info_.clear_error = 0; // 默认值
    set_vehicle_info_.hydraulic_motor_mode = status_Info_.hydraulic_motor_mode;
    set_vehicle_info_.turn_pwm_or_angle = 0; // 默认值
    
    // 初始化HostToVCU2 (动臂铲斗控制)
    set_boom_lift_info_.boom_lift_pwm = 0; // 默认值
    set_boom_lift_info_.boom_lower_pwm = 0; // 默认值
    set_boom_lift_info_.bucket_close_pwm = 0; // 默认值
    set_boom_lift_info_.bucket_open_pwm = 0; // 默认值
    
    // 初始化HostToVCUB0EE (行走电机控制)
    set_walk_motor_info_.hydraulic_motor_enable = walk_motor_Info_.walk_motor_enable;
    set_walk_motor_info_.hydraulic_motor_mode = status_Info_.hydraulic_motor_mode;
    set_walk_motor_info_.gear = vehicle_info_.gear_signal;
    set_walk_motor_info_.target_speed = 0; // 默认值
    set_walk_motor_info_.request_torque = 0; // 默认值
    set_walk_motor_info_.brake_valve_pwm = vehicle_info_.brake_control;
    
    // 初始化HostToVCUB1EF (液压电机控制)
    set_hydraulic_motor_info_.hydraulic_motor_enable = hydraulic_motor_Info_.hydraulic_motor_enable;
    set_hydraulic_motor_info_.request_torque = 0; // 默认值
    set_hydraulic_motor_info_.target_speed = 0; // 默认值
    set_hydraulic_motor_info_.work_mode = 0; // 默认值
}


// void ChassisCan::PublishChassisStatus()
// {
//     // 以100Hz频率发布ROS2消息
//     auto message = message::msg::ChassisStatus();
    
//     // 填充VCUToHost1数据
//     message.high_voltage_status = vehicle_info_.high_voltage_status;
//     message.parking_brake = vehicle_info_.parking_brake;
//     message.horn_status = vehicle_info_.horn_status;
//     message.left_turn_signal = vehicle_info_.left_turn_signal;
//     message.right_turn_signal = vehicle_info_.right_turn_signal;
//     message.walk_motor_mode = vehicle_info_.walk_motor_mode;
//     message.wet_brake_alarm = vehicle_info_.wet_brake_alarm;
//     message.emergency_stop = vehicle_info_.emergency_stop;
//     message.gear_signal = vehicle_info_.gear_signal;
//     message.rotation_alarm = vehicle_info_.rotation_alarm;
//     message.heartbeat_status = vehicle_info_.heartbeat_status;
//     message.brake_control = vehicle_info_.brake_control;
//     message.front_rear_angle = vehicle_info_.front_rear_angle;
//     message.battery_level = vehicle_info_.battery_level;
//     message.charging_status = vehicle_info_.charging_status;
//     message.hydraulic_lock = vehicle_info_.hydraulic_lock;
//     message.fault_level = vehicle_info_.fault_level;
//     message.turtle_rabbit_gear = vehicle_info_.turtle_rabbit_gear;
//     message.work_light = vehicle_info_.work_light;
//     message.vehicle_mode = vehicle_info_.vehicle_mode;
    
//     // 填充VCUToHost2数据
//     message.boom_lift_current = current_Info_.boom_lift_current;
//     message.boom_lower_current = current_Info_.boom_lower_current;
//     message.bucket_close_current = current_Info_.bucket_close_current;
//     message.bucket_open_current = current_Info_.bucket_open_current;
    
//     // 填充VCUToHost3数据
//     message.boom_big_pressure = pressure_Info_.boom_big_pressure;
//     message.boom_small_pressure = pressure_Info_.boom_small_pressure;
//     message.bucket_big_pressure = pressure_Info_.bucket_big_pressure;
//     message.bucket_small_pressure = pressure_Info_.bucket_small_pressure;
    
//     // 填充VCUToHost4数据
//     message.hydraulic_motor_speed = hydraulic_motor_Info_.hydraulic_motor_speed;
//     message.hydraulic_motor_torque = hydraulic_motor_Info_.hydraulic_motor_torque;
//     message.hydraulic_motor_current = hydraulic_motor_Info_.hydraulic_motor_current;
//     message.hydraulic_motor_enable = hydraulic_motor_Info_.hydraulic_motor_enable;
    
//     // 填充VCUToHost5数据
//     message.walk_motor_current = walk_motor_Info_.walk_motor_current;
//     message.walk_motor_torque = walk_motor_Info_.walk_motor_torque;
//     message.walk_motor_speed = walk_motor_Info_.walk_motor_speed;
//     message.walk_motor_enable = walk_motor_Info_.walk_motor_enable;
    
//     // 填充VCUToHost6数据
//     message.throttle_opening = status_Info_.throttle_opening;
//     message.brake_opening = status_Info_.brake_opening;
//     message.heartbeat_signal = status_Info_.heartbeat_signal;
//     message.can_loss_1 = status_Info_.can_loss_1;
//     message.can_loss_2 = status_Info_.can_loss_2;
//     message.can_loss_3 = status_Info_.can_loss_3;
//     message.can_loss_4 = status_Info_.can_loss_4;
//     message.walk_motor_fault = status_Info_.walk_motor_fault;
//     message.hydraulic_motor_fault = status_Info_.hydraulic_motor_fault;
//     message.boom_lift_valve_fault = status_Info_.boom_lift_valve_fault;
//     message.boom_lower_valve_fault = status_Info_.boom_lower_valve_fault;
//     message.bucket_close_valve_fault = status_Info_.bucket_close_valve_fault;
//     message.bucket_open_valve_fault = status_Info_.bucket_open_valve_fault;
//     message.foot_brake_valve_fault = status_Info_.foot_brake_valve_fault;
//     message.turn_valve_fault = status_Info_.turn_valve_fault;
//     message.low_beam = status_Info_.low_beam;
//     message.high_beam = status_Info_.high_beam;
//     message.hydraulic_motor_voltage = status_Info_.hydraulic_motor_voltage;
//     message.turn_valve_current = status_Info_.turn_valve_current;
//     message.hydraulic_motor_mode = status_Info_.hydraulic_motor_mode;
//     message.vehicle_mode_2 = status_Info_.vehicle_mode_2;
    
//     // 填充VCUToHost9数据
//     message.rear_motor_current = rear_front_motor_Info_.rear_motor_current;
//     message.rear_motor_voltage = rear_front_motor_Info_.rear_motor_voltage;
//     message.front_motor_current = rear_front_motor_Info_.front_motor_current;
//     message.front_motor_voltage = rear_front_motor_Info_.front_motor_voltage;
//     message.hydraulic_motor_current_2 = rear_front_motor_Info_.hydraulic_motor_current_2;
    
//     // 填充VCUToHostA数据
//     message.boom_angle = angle_Info_.boom_angle;
//     message.bucket_angle = angle_Info_.bucket_angle;
    
//     // 发布消息
//     chassis_status_pub_->publish(message);
// }

#endif