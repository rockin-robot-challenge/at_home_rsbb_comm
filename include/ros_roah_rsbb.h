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
#include <memory>

#include <boost/make_shared.hpp>

#include <ros/console.h>
#include <ros/param.h>
#include <ros/time.h>
#include <ros/init.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

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



  class CallbackItem
    : public ros::CallbackInterface
  {
      boost::function<void() > f_;

    public:
      CallbackItem (boost::function<void() > const& f)
        : f_ (f)
      {
      }

      virtual CallResult call()
      {
        f_();
        return Success;
      }
  };



  class RosPublicChannel
    : public PublicChannel<RosErrorHandler>
  {
      boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon>) > rsbb_beacon_callback_;
      boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RobotBeacon>) > robot_beacon_callback_;
      boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::TabletBeacon>) > tablet_beacon_callback_;

      void
      receive_rsbb_beacon (boost::asio::ip::udp::endpoint& endpoint,
                           uint16_t comp_id,
                           uint16_t msg_type,
                           std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> msg)
      {
        ROS_DEBUG_STREAM ("Received RoahRsbbBeacon from " << endpoint.address().to_string()
                          << ":" << endpoint.port()
                          << ", COMP_ID " << comp_id
                          << ", MSG_TYPE " << msg_type);

        if (rsbb_beacon_callback_) {
          ros::getGlobalCallbackQueue()->addCallback (boost::make_shared<CallbackItem> (boost::bind (rsbb_beacon_callback_, endpoint, comp_id, msg_type, msg)));
        }
      }

      void
      receive_robot_beacon (boost::asio::ip::udp::endpoint& endpoint,
                            uint16_t comp_id,
                            uint16_t msg_type,
                            std::shared_ptr<const roah_rsbb_msgs::RobotBeacon> msg)
      {
        ROS_DEBUG_STREAM ("Received RobotBeacon from " << endpoint.address().to_string()
                          << ":" << endpoint.port()
                          << ", COMP_ID " << comp_id
                          << ", MSG_TYPE " << msg_type
                          << ", team_name: " << msg->team_name()
                          << ", robot_name: " << msg->robot_name());

        if (robot_beacon_callback_) {
          ros::getGlobalCallbackQueue()->addCallback (boost::make_shared<CallbackItem> (boost::bind (robot_beacon_callback_, endpoint, comp_id, msg_type, msg)));
        }
      }

      void
      receive_tablet_beacon (boost::asio::ip::udp::endpoint& endpoint,
                             uint16_t comp_id,
                             uint16_t msg_type,
                             std::shared_ptr<const roah_rsbb_msgs::TabletBeacon> msg)
      {
        ROS_DEBUG_STREAM ("Received TabletBeacon from " << endpoint.address().to_string()
                          << ":" << endpoint.port()
                          << ", COMP_ID " << comp_id
                          << ", MSG_TYPE " << msg_type);

        if (tablet_beacon_callback_) {
          ros::getGlobalCallbackQueue()->addCallback (boost::make_shared<CallbackItem> (boost::bind (tablet_beacon_callback_, endpoint, comp_id, msg_type, msg)));
        }
      }

    public:
      RosPublicChannel (std::string const& host,
                        unsigned short port)
        : PublicChannel<RosErrorHandler> (host, port)
      {
        signal_rsbb_beacon_received().connect (boost::bind (&RosPublicChannel::receive_rsbb_beacon, this, _1, _2, _3, _4));
        signal_robot_beacon_received().connect (boost::bind (&RosPublicChannel::receive_robot_beacon, this, _1, _2, _3, _4));
        signal_tablet_beacon_received().connect (boost::bind (&RosPublicChannel::receive_tablet_beacon, this, _1, _2, _3, _4));
      }

      void
      set_rsbb_beacon_callback (boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon>) > rsbb_beacon_callback)
      {
        rsbb_beacon_callback_ = rsbb_beacon_callback;
      }

      template<class T> void
      set_rsbb_beacon_callback (void (T::*callback) (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon>),
                                T* obj)
      {
        return set_rsbb_beacon_callback (boost::bind (callback, obj, _1, _2, _3, _4));
      }

      void
      set_robot_beacon_callback (boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RobotBeacon>) > robot_beacon_callback)
      {
        robot_beacon_callback_ = robot_beacon_callback;
      }

      template<class T> void
      set_robot_beacon_callback (void (T::*callback) (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RobotBeacon>),
                                 T* obj)
      {
        return set_robot_beacon_callback (boost::bind (callback, obj, _1, _2, _3, _4));
      }

      void
      set_tablet_beacon_callback (boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::TabletBeacon>) > tablet_beacon_callback)
      {
        tablet_beacon_callback_ = tablet_beacon_callback;
      }

      template<class T> void
      set_tablet_beacon_callback (void (T::*callback) (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::TabletBeacon>),
                                  T* obj)
      {
        return set_tablet_beacon_callback (boost::bind (callback, obj, _1, _2, _3, _4));
      }
  };



  class RosPrivateChannel
    : public PrivateChannel<RosErrorHandler>
  {
      boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::BenchmarkState>) > benchmark_state_callback_;
      boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RobotState>) > robot_state_callback_;

      void
      receive_benchmark_state (boost::asio::ip::udp::endpoint& endpoint,
                               uint16_t comp_id,
                               uint16_t msg_type,
                               std::shared_ptr<const roah_rsbb_msgs::BenchmarkState> msg)
      {
        ROS_DEBUG_STREAM ("Received BenchmarkState from " << endpoint.address().to_string()
                          << ":" << endpoint.port()
                          << ", COMP_ID " << comp_id
                          << ", MSG_TYPE " << msg_type
                          << ", benchmark_type: " << msg->benchmark_type());

        if (benchmark_state_callback_) {
          ros::getGlobalCallbackQueue()->addCallback (boost::make_shared<CallbackItem> (boost::bind (benchmark_state_callback_, endpoint, comp_id, msg_type, msg)));
        }
      }

      void
      receive_robot_state (boost::asio::ip::udp::endpoint& endpoint,
                           uint16_t comp_id,
                           uint16_t msg_type,
                           std::shared_ptr<const roah_rsbb_msgs::RobotState> msg)
      {
        ROS_DEBUG_STREAM ("Received RobotState from " << endpoint.address().to_string()
                          << ":" << endpoint.port()
                          << ", COMP_ID " << comp_id
                          << ", MSG_TYPE " << msg_type);

        if (robot_state_callback_) {
          ros::getGlobalCallbackQueue()->addCallback (boost::make_shared<CallbackItem> (boost::bind (robot_state_callback_, endpoint, comp_id, msg_type, msg)));
        }
      }

    public:
      RosPrivateChannel (std::string const& host,
                         unsigned short port,
                         std::string const& key,
                         std::string const& cypher)
        : PrivateChannel<RosErrorHandler> (host, port, key, cypher)
      {
        signal_benchmark_state_received().connect (boost::bind (&RosPrivateChannel::receive_benchmark_state, this, _1, _2, _3, _4));
        signal_robot_state_received().connect (boost::bind (&RosPrivateChannel::receive_robot_state, this, _1, _2, _3, _4));
      }

      void
      set_benchmark_state_callback (boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::BenchmarkState>) > benchmark_state_callback)
      {
        benchmark_state_callback_ = benchmark_state_callback;
      }

      template<class T> void
      set_benchmark_state_callback (void (T::*callback) (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::BenchmarkState>),
                                    T* obj)
      {
        return set_benchmark_state_callback (boost::bind (callback, obj, _1, _2, _3, _4));
      }

      void
      set_robot_state_callback (boost::function<void (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RobotState>) > robot_state_callback)
      {
        robot_state_callback_ = robot_state_callback;
      }

      template<class T> void
      set_robot_state_callback (void (T::*callback) (boost::asio::ip::udp::endpoint, uint16_t, uint16_t, std::shared_ptr<const roah_rsbb_msgs::RobotState>),
                                T* obj)
      {
        return set_robot_state_callback (boost::bind (callback, obj, _1, _2, _3, _4));
      }
  };
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
