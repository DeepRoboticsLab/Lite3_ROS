/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes with UDP:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 */

#include "input.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include <string>


/// @brief constructor
/// @param private_nh ROS private handle for calling node.
/// @param port UDP port number
Input::Input(ros::NodeHandle private_nh) : private_nh_(private_nh) {
  private_nh.param<int>("local_port", port_, LOCAL_PORT_NUMBER);

  private_nh.param("remote_ip", devip_str_, std::string(""));
  private_nh.param<int>("remote_port", remote_port_, REMOTE_PORT_NUMBER);

  if (!devip_str_.empty())
    ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
}


/// @brief constructor
///  @param private_nh ROS private handle for calling node.
///  @param port UDP port number
InputSocket::InputSocket(ros::NodeHandle private_nh) : Input(private_nh) {
  sockfd_ = -1;

  if (!devip_str_.empty()) {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  // connect to UDP port
  ROS_INFO_STREAM("Opening UDP socket: port " << port_);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1) {
    perror("socket");  // TODO: ROS_ERROR errno
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port_);       // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    perror("bind");  // TODO: ROS_ERROR errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    return;
  }

  memset(&client_addr, 0, sizeof(client_addr));
  client_addr.sin_family = AF_INET;
  client_addr.sin_port = htons(remote_port_);
  client_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str());

  ROS_DEBUG("socket fd is %d\n", sockfd_);
}


/// @brief destructor 
InputSocket::~InputSocket(void) { (void)close(sockfd_); }


/// @brief Get one velodyne packet. 
int InputSocket::getPacket(uint8_t *pkt, size_t packet_size) {
  double time1 = ros::Time::now().toSec();

  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    // Unfortunately, the Linux kernel recvfrom() implementation
    // uses a non-interruptible sleep() when waiting for data,
    // which would cause this method to hang if the device is not
    // providing data.  We poll() the device first to make sure
    // the recvfrom() will not block.
    //
    // Note, however, that there is a known Linux kernel bug:
    //
    //   Under Linux, select() may report a socket file descriptor
    //   as "ready for reading", while nevertheless a subsequent
    //   read blocks.  This could for example happen when data has
    //   arrived but upon examination has wrong checksum and is
    //   discarded.  There may be other circumstances in which a
    //   file descriptor is spuriously reported as ready.  Thus it
    //   may be safer to use O_NONBLOCK on sockets that should not
    //   block.

    // poll() until input available
    do {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR) ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
        ROS_WARN("poll() timeout");
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
          (fds[0].revents & POLLNVAL))  // device error?
      {
        ROS_ERROR("poll() reports Velodyne error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(sockfd_, pkt, packet_size, 0,
                              (sockaddr *)&sender_address, &sender_address_len);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    } else if ((size_t)nbytes == packet_size) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break;  // done
    }

    ROS_INFO_STREAM("incomplete packet read: " << nbytes << " bytes");
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = ros::Time::now().toSec();

  return 0;
}


/// @brief end one velodyne packet.
/// @param pkt uint8_t
/// @param packet_size size_t
/// @return send successful return 0 else return 1.
int InputSocket::sendPacket(uint8_t *pkt, size_t packet_size) {
  ssize_t nbytes = sendto(sockfd_, pkt, packet_size, 0,
                          (struct sockaddr *)&client_addr, sizeof(client_addr));
  if (nbytes < 0) {
    perror("sendfail");
    ROS_INFO("sendfail");
    return 1;
  } else if ((size_t)nbytes == packet_size) {
    // send successful,
  }

  return 0;
}
