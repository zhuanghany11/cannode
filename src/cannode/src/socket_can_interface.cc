/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: socket can interface definition
 */
#include <iostream>
#include <sys/ioctl.h>
#include <cstring>
#include "socket_can_interface.h"

namespace cannode {
SocketCanInterface::SocketCanInterface(const std::string &port) : canInterface(port) {
  bool ret = CanInit(port);
  std::cout << "SocketCanInterface::CanInit ret = " << ret << std::endl;
}

bool SocketCanInterface::CanInit(const std::string& canDevice)
{
    fd_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);  // 创建套接字
    if (fd_can < 0) {
        std::cout << "SocketCanInterface::CanInit socket error" << std::endl;
        return false;
    }
    struct ifreq ifr = {};
    strncpy(ifr.ifr_name, canDevice.c_str(), sizeof(ifr.ifr_name) - 1);
    ifr.ifr_name[sizeof(ifr.ifr_name) - 1] = '\0';  // 确保字符串以null结尾
    if (ifr.ifr_name[0] == '\0') {
        CloseSocketCan();
        std::cout << "SocketCanInterface::CanInit ifr.ifr_name[0] == '\0'" << std::endl;
        return false;
    }
    int32_t ret = ioctl(fd_can, SIOCGIFINDEX, &ifr);  // 指定 can 设备
    if (ret < 0) {
        CloseSocketCan();
        std::cout << "SocketCanInterface::CanInit ioctl error" << std::endl;
        return false;
    }

    const int32_t canFdFlag = 1;
    ret = setsockopt(fd_can, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canFdFlag, static_cast<socklen_t>(sizeof(canFdFlag)));
    if (ret < 0) {
        CloseSocketCan();
        std::cout << "SocketCanInterface::CanInit setsockopt error" << std::endl;
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family = static_cast<__kernel_sa_family_t>(AF_CAN);
    addr.can_ifindex = ifr.ifr_ifindex;
    // 将套接字与 can 设备绑定
    ret = bind(fd_can, reinterpret_cast<sockaddr*>(&addr), static_cast<socklen_t>(sizeof(addr)));
    if (ret < 0) {
        CloseSocketCan();
        std::cout << "SocketCanInterface::CanInit bind error" << std::endl;
        return false;
    }

    const int32_t sendBuffLen = 32768;
    ret = setsockopt(fd_can, SOL_SOCKET, SO_TIMESTAMP, &sendBuffLen, static_cast<socklen_t>(sizeof(sendBuffLen)));
    if (ret < 0) {
        CloseSocketCan();
        std::cout << "SocketCanInterface::CanInit setsockopt error" << std::endl;
        return false;
    }

    return true;
}

//设置接收套接字的超时时间
bool SocketCanInterface::SetRecvTimeout(const struct timeval& tv) const
{
    const int32_t ret = setsockopt(fd_can, SOL_SOCKET, SO_RCVTIMEO, &tv, static_cast<socklen_t>(sizeof(tv)));
    if (ret < 0) {
        return false;
    }
    return true;
}

int32_t SocketCanInterface::ReadCan(can_frame& receiveFrame, struct timeval& tstamp, std::int32_t& readBytes) const
{
    iovec iov;
    iov.iov_base = static_cast<void*>(&receiveFrame);
    iov.iov_len = sizeof(receiveFrame);

    const std::uint32_t controlSize = 512U;
    char controlBuf[CMSG_SPACE(controlSize)];
    msghdr canMsg;
    canMsg.msg_name = nullptr;
    canMsg.msg_namelen = 0U;
    canMsg.msg_iov = &iov;
    canMsg.msg_iovlen = 1U;
    canMsg.msg_control = controlBuf;
    canMsg.msg_controllen = sizeof(controlBuf);
    canMsg.msg_flags = 0;

    readBytes = static_cast<int32_t>(recvmsg(fd_can, &canMsg, 0));
    if (readBytes < 0) {
        return -1;
    }
    struct cmsghdr* cmsg = CMSG_FIRSTHDR(&canMsg);
    if (cmsg != nullptr) {
        tstamp = *(reinterpret_cast<timeval*>(CMSG_DATA(cmsg)));
    }
    return 0;
}

bool SocketCanInterface::SendCanFrame(uint32_t canid, int dlc, uint8_t *data) {
  //  LOG_Channel_Debug(LOG_CHANNEL_NAME_CAN_ADAPTOR)
  //      << boost::format("Port[%s] SendCanFrame Id[0x%x]") %
  //             canInterface.c_str() % canid;
  //  LOG_Channel_DEBUG_HEX(LOG_CHANNEL_NAME_CAN_ADAPTOR, data, dlc, "Frame
  //  Data:");
  struct can_frame sendCanFrame;

  // TODO 最好是在参数列表中增加一个扩展帧 标准帧的flag
  if (canid > 0x7FF) {
    canid |= CAN_EFF_FLAG;
  }
  sendCanFrame.can_id = canid;
  sendCanFrame.can_dlc = dlc;
  std::memmove(sendCanFrame.data, data, dlc);
  uint16_t sendLen = static_cast<int32_t>(write(fd_can, &sendCanFrame, sizeof(struct can_frame)));
  if (sendLen != static_cast<int32_t>(sizeof(sendCanFrame))) {
    return false;
  }
  return true;
}

bool SocketCanInterface::SendCanFrame(can_frame &canframe) {
  return SendCanFrame(canframe.can_id, canframe.can_dlc, canframe.data);
}

bool SocketCanInterface::SendCanFrame(std::vector<can_frame> &canframes) {
  bool ret = true;

  for (auto &canframe : canframes) {
    if (!SendCanFrame(canframe)) {
      ret = false;
    }
  }

  return ret;
}



bool SocketCanInterface::SetCanFiLter(const struct can_filter& filter) const
{
    const auto filterSize = static_cast<socklen_t>(sizeof(filter));
    const int32_t ret = setsockopt(fd_can, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, filterSize);
    if (ret < 0) {
        return false;
    }
    return true;
}

bool SocketCanInterface::SetCanFilters(const std::vector<can_filter>& filters) const
{
    if (filters.empty()) {
        return false;
    }
    const auto itemSize = static_cast<socklen_t>(sizeof(can_filter));
    const auto filterSize = static_cast<socklen_t>(static_cast<socklen_t>(filters.size()) * itemSize);
    const int32_t ret = setsockopt(fd_can, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filterSize);
    if (ret < 0) {
        return false;
    }
    return true;
}

void SocketCanInterface::CloseSocketCan()
{
    if (fd_can > 0) {
        (void)close(fd_can);
        fd_can = -1;
    }
}
}  // namespace cannode
