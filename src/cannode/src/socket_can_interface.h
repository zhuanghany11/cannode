/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: socket can interface declaration
 */

#ifndef SOCKET_CAN_INTERFACE_H
#define SOCKET_CAN_INTERFACE_H

#include <string>
#include <vector>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h> 
#include <linux/can.h>
#include <linux/can/raw.h>

namespace cannode {
// SocketCAN 封装：负责底层 CAN 套接字的打开/配置/发送/接收
class SocketCanInterface {
  public:
    explicit SocketCanInterface(const std::string &port);
    ~SocketCanInterface() = default;

    // 初始化 CAN 设备（打开 socket 并 bind 到指定接口）
    bool CanInit(const std::string& canDevice);
    // 设置接收超时
    bool SetRecvTimeout(const struct timeval& tv) const;

    // 设置单/多过滤器
    bool SetCanFiLter(const struct can_filter& filter) const;
    bool SetCanFilters(const std::vector<can_filter>& filters) const;
    // 读取 CAN 帧并返回时间戳
    std::int32_t ReadCan(can_frame& receiveFrame, struct timeval& tstamp, std::int32_t& readBytes) const;

    // 发送单帧（根据 ID 自动设置扩展帧标志）
    bool SendCanFrame(uint32_t canid, int dlc, uint8_t *data);
    bool SendCanFrame(struct can_frame &canframe);
    bool SendCanFrame(std::vector<can_frame> &canframes);
    // 关闭 CAN 套接字
    void CloseSocketCan();
    int32_t GetSocket() const
    {
        return fd_can;
    }
  public:
    int32_t fd_can = -1;
    std::string canInterface;
};
}  // namespace cannode

#endif
