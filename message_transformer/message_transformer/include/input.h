/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Velodyne 3D LIDAR data input classes
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __VELODYNE_INPUT_H
#define __VELODYNE_INPUT_H

#include <netinet/in.h>
#include <pcap.h>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>

static const uint16_t LOCAL_PORT_NUMBER = 2368;   // default data port
static const uint16_t REMOTE_PORT_NUMBER = 8308;  // default position port

/** @brief Velodyne input base class */
class Input {
 public:
  Input(ros::NodeHandle private_nh);
  virtual ~Input() {}

  /** @brief Read one udp packet.
   *
   * @param pkt points to VelodynePacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int getPacket(uint8_t *pkt, size_t packet_size) = 0;

 protected:
  ros::NodeHandle private_nh_;
  int port_;
  int remote_port_;
  std::string devip_str_;
};

/** @brief Live Velodyne input from socket. */
class InputSocket : public Input {
 public:
  InputSocket(ros::NodeHandle private_nh);
  virtual ~InputSocket();

  virtual int getPacket(uint8_t *pkt, size_t packet_size);
  virtual int sendPacket(uint8_t *pkt, size_t packet_size);

  void setDeviceIP(const std::string &ip);

 private:
 private:
  int sockfd_;
  in_addr devip_;

  sockaddr_in client_addr;
};

#endif  // __VELODYNE_INPUT_H
