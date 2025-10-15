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
class SocketCanInterface {
  public:
    explicit SocketCanInterface(const std::string &port);
    ~SocketCanInterface() = default;

    bool CanInit(const std::string& canDevice);
    bool SetRecvTimeout(const struct timeval& tv) const;

    bool SetCanFiLter(const struct can_filter& filter) const;
    bool SetCanFilters(const std::vector<can_filter>& filters) const;
    std::int32_t ReadCan(can_frame& receiveFrame, struct timeval& tstamp, std::int32_t& readBytes) const;

    bool SendCanFrame(uint32_t canid, int dlc, uint8_t *data);
    bool SendCanFrame(struct can_frame &canframe);
    bool SendCanFrame(std::vector<can_frame> &canframes);
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
