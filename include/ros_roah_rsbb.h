/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH RSBB Comm.
 *
 * RoAH RSBB Comm is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH RSBB Comm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH RSBB Comm.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ROS_ROAH_RSBB_H__
#define __ROS_ROAH_RSBB_H__

#include <string>

#include <ros/console.h>
#include <ros/param.h>
#include <ros/time.h>

#include "roah_rsbb.h"



namespace roah_rsbb
{
  inline void
  ros_to_proto_time (ros::Time const& ros, ::roah_rsbb_msgs::Time* proto)
  {
    proto->set_sec (ros.sec);
    proto->set_nsec (ros.nsec);
  }

  inline ros::Time
  proto_to_ros_time (::roah_rsbb_msgs::Time const& proto)
  {
    return ros::Time (proto.sec(), proto.nsec());
  }



  class RosErrorHandler
  {
    public:
      RosErrorHandler (std::string const& name,
                       protobuf_comm::ProtobufBroadcastPeer& channel) :
        name_ (name)
      {
        channel.signal_recv_error().connect (boost::bind (&RosErrorHandler::recv_error, this, _1, _2));
        channel.signal_send_error().connect (boost::bind (&RosErrorHandler::send_error, this, _1));
      }

      void
      unknown_msg (boost::asio::ip::udp::endpoint& endpoint,
                   uint16_t comp_id,
                   uint16_t msg_type,
                   std::shared_ptr<google::protobuf::Message> msg)
      {
        ROS_ERROR_STREAM ("On channel " << name_
                          << ": Unknown message from " << endpoint.address().to_string()
                          << ":" << endpoint.port()
                          << ", COMP_ID " << comp_id
                          << ", MSG_TYPE " << msg_type
                          << ", of type " << typeid (*msg).name());
      }

    private:
      std::string name_;

      void
      recv_error (boost::asio::ip::udp::endpoint& endpoint,
                  std::string msg)
      {
        ROS_ERROR_STREAM ("On channel " << name_
                          << ": Receive error from " << endpoint.address().to_string()
                          << ":" << endpoint.port()
                          << ": " << msg);
      }

      void
      send_error (std::string msg)
      {
        ROS_ERROR_STREAM ("On channel " << name_
                          << ": Send error: " << msg);
      }
  };

  typedef PublicChannel<RosErrorHandler> RosPublicChannel;
  typedef PrivateChannel<RosErrorHandler> RosPrivateChannel;
}



template<typename T> inline void
param_direct (std::string const& name,
              T const& def_value,
              T& var)
{
  if (! ros::param::getCached (name, var)) {
    var = def_value;
    ros::param::set (name, var);
  }
}



template<typename T> inline T
param_direct (std::string const& name,
              T const& def_value)
{
  T tmp;
  param_direct<T> (name, def_value, tmp);
  return tmp;
}

#endif
