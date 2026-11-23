
#include "can_driver/socketcan_interface.hpp"
#include <stdexcept>
#include <string>
// SocketcanInterface::SocketcanInterface()
// {
//     // Constructor for no mask definition
//     this->openSocket(DEFAULT_MSG_ID,DEFAULT_CAN_CHANNEL,DEFAULT_CANID_MASK, DEFAULT_READ_TIMOUT);
// }
SocketcanInterface::SocketcanInterface(canid_t msg_id)
{
    // Constructor for no mask definition
    this->openSocket(msg_id,DEFAULT_CAN_CHANNEL,DEFAULT_CANID_MASK, DEFAULT_READ_TIMOUT);
}
SocketcanInterface::SocketcanInterface(canid_t msg_id,uint8_t can_channel)
{
    // Constructor for no mask definition
    this->openSocket(msg_id,can_channel,DEFAULT_CANID_MASK, DEFAULT_READ_TIMOUT);
}
SocketcanInterface::SocketcanInterface(canid_t msg_id, uint8_t can_channel, uint16_t mask)
{
    // Constructor with mask definition
    this->openSocket(msg_id, can_channel, mask, DEFAULT_READ_TIMOUT);
}

SocketcanInterface::SocketcanInterface(canid_t msg_id, uint8_t can_channel, uint16_t mask, uint32_t timeout_microseconds)
{
    // Constructor with mask definition
    this->openSocket(msg_id, can_channel, mask, timeout_microseconds);
}

SocketcanInterface::~SocketcanInterface()
{
    // This might not ever be run
    close(this->s);
}

int SocketcanInterface::openSocket(canid_t msg_id, uint8_t can_channel, uint16_t mask, uint32_t timeout_microseconds)
{
    // Open socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        throw std::runtime_error("Cannot create CAN socket");
    }
    ifr = {};
    addr = {};
    if (can_channel == 0) {
        strcpy(ifr.ifr_name, "can0");
    } else {
        strcpy(ifr.ifr_name, "can1");
    }
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        close(s);
        throw std::runtime_error(std::string("CAN interface not found: ") + ifr.ifr_name);
    }
    can_filter filt;
    filt.can_id = msg_id;
    filt.can_mask = mask & CAN_SFF_MASK; // Mask to first 5 bits of 11 bit identifier
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt)) < 0)
    {
        printf("setsockopt failed\n");
    }

    int buf_size = 32;
    if (setsockopt(s, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0)
    {
        printf("setsockopt failed\n");
    }

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = timeout_microseconds;
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)))
    {
        printf("Set CAN socket option 'SO_RCVTIMEO' failed !\n");
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int err = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (err < 0)
    {
        close(s);
        throw std::runtime_error(std::string("Cannot bind CAN interface: ") + ifr.ifr_name);
    }

    return s;
}

int SocketcanInterface::readFrame(can_frame *frame)
{
    // Missing cyclic Iq feedback must not stall trajectory timing.
    return recv(this->s, frame, sizeof(*frame), MSG_DONTWAIT);
}


int SocketcanInterface::writeFrame(can_frame frame)
{
    return write(this->s, &frame, sizeof(frame));
}
