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

#ifndef __ROAH_RSBB_H__
#define __ROAH_RSBB_H__

#include <boost/lexical_cast.hpp>

#include <protobuf_comm/peer.h>

#include "roah_rsbb_msgs.h"



namespace roah_rsbb
{
  void
  now (::roah_rsbb_msgs::Time* time_msg)
  {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::time_duration diff = now - boost::posix_time::from_time_t (0);
    time_msg->set_sec (diff.total_seconds());
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
    time_msg->set_nsec (diff.fractional_seconds());
#else
    time_msg->set_nsec (diff.fractional_seconds() * 1000);
#endif
  }



  class CerrErrorHandler
  {
    public:
      CerrErrorHandler (std::string const& name,
                        protobuf_comm::ProtobufBroadcastPeer& channel) :
        name_ (name)
      {
        channel.signal_recv_error().connect (boost::bind (&CerrErrorHandler::recv_error, this, _1, _2));
        channel.signal_send_error().connect (boost::bind (&CerrErrorHandler::send_error, this, _1));
      }

      void
      unknown_msg (boost::asio::ip::udp::endpoint& endpoint,
                   uint16_t comp_id,
                   uint16_t msg_type,
                   std::shared_ptr<google::protobuf::Message> msg)
      {
        std::cerr << "On channel " << name_
                  << ": Unknown message from " << endpoint.address().to_string()
                  << ":" << endpoint.port()
                  << ", COMP_ID " << comp_id
                  << ", MSG_TYPE " << msg_type
                  << ", of type " << typeid (*msg).name() << std::endl << std::flush;
      }

    private:
      std::string name_;

      void
      recv_error (boost::asio::ip::udp::endpoint& endpoint,
                  std::string msg)
      {
        std::cerr << "On channel " << name_
                  << ": Receive error from " << endpoint.address().to_string()
                  << ":" << endpoint.port()
                  << ": " << msg << std::endl << std::flush;
      }

      void
      send_error (std::string msg)
      {
        std::cerr << "On channel " << name_
                  << ": Send error: " << msg << std::endl << std::flush;
      }
  };



  template <typename ErrorHandler = CerrErrorHandler>
  class Channel :
    public protobuf_comm::ProtobufBroadcastPeer
  {
    public:
      Channel (std::string const& name,
               std::string const& host,
               unsigned short port) :
        protobuf_comm::ProtobufBroadcastPeer (host, port),
        name_ (name),
        host_ (host),
        port_ (port),
        error_handler_ (name, *this)
      {
        signal_received().connect (boost::bind (&Channel::recv_msg_base, this, _1, _2, _3, _4));
      }

      Channel (std::string const& name,
               std::string const& host,
               unsigned short port,
               std::string const& key,
               std::string const& cypher) :
        protobuf_comm::ProtobufBroadcastPeer (host, port, key, cypher),
        name_ (name),
        host_ (host),
        port_ (port),
        error_handler_ (name, *this)
      {
        signal_received().connect (boost::bind (&Channel::recv_msg_base, this, _1, _2, _3, _4));
      }

      std::string const& name() const
      {
        return name_;
      }

      std::string const& host() const
      {
        return host_;
      }

      unsigned short port() const
      {
        return port_;
      }

    protected:
      virtual bool
      recv_msg (boost::asio::ip::udp::endpoint& endpoint,
                uint16_t comp_id,
                uint16_t msg_type,
                std::shared_ptr<google::protobuf::Message> msg) = 0;

    private:
      std::string name_;
      std::string host_;
      unsigned short port_;
      ErrorHandler error_handler_;

      void
      recv_msg_base (boost::asio::ip::udp::endpoint& endpoint,
                     uint16_t comp_id,
                     uint16_t msg_type,
                     std::shared_ptr<google::protobuf::Message> msg)
      {
        if (! recv_msg (endpoint, comp_id, msg_type, msg)) {
          error_handler_.unknown_msg (endpoint, comp_id, msg_type, msg);
        }
      }
  };



  template <typename ErrorHandler = CerrErrorHandler>
  class PublicChannel :
    public Channel<ErrorHandler>
  {
    public:
      PublicChannel (std::string const& host,
                     unsigned short port) :
        Channel<ErrorHandler> ("roah_rsbb::PublicChannel(" + host + ":" + boost::lexical_cast<std::string> (port) + ")", host, port)
      {
        protobuf_comm::MessageRegister& mr = this->message_register();
        mr.add_message_type<roah_rsbb_msgs::RobotBeacon>();
        mr.add_message_type<roah_rsbb_msgs::RoahRsbbBeacon>();
      }

      typedef
      boost::signals2::signal<void (boost::asio::ip::udp::endpoint&,
                                    uint16_t,
                                    uint16_t,
                                    std::shared_ptr<const roah_rsbb_msgs::RobotBeacon>) >
      signal_robot_beacon_received_type;

      signal_robot_beacon_received_type& signal_robot_beacon_received()
      {
        return signal_robot_beacon_received_;
      }

      std::shared_ptr<const roah_rsbb_msgs::RobotBeacon>
      last_robot_beacon() const
      {
        std::shared_ptr<const roah_rsbb_msgs::RobotBeacon> ret;
        {
          std::lock_guard<std::mutex> lock (robot_beacon_mutex_);
          ret = last_robot_beacon_;
        }
        return ret;
      }

      typedef
      boost::signals2::signal<void (boost::asio::ip::udp::endpoint&,
                                    uint16_t,
                                    uint16_t,
                                    std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon>) >
      signal_rsbb_beacon_received_type;

      signal_rsbb_beacon_received_type& signal_rsbb_beacon_received()
      {
        return signal_rsbb_beacon_received_;
      }

      std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon>
      last_rsbb_beacon() const
      {
        std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> ret;
        {
          std::lock_guard<std::mutex> lock (rsbb_beacon_mutex_);
          ret = last_rsbb_beacon_;
        }
        return ret;
      }

    private:
      signal_robot_beacon_received_type signal_robot_beacon_received_;
      std::mutex robot_beacon_mutex_;
      std::shared_ptr<const roah_rsbb_msgs::RobotBeacon> last_robot_beacon_;

      signal_rsbb_beacon_received_type signal_rsbb_beacon_received_;
      std::mutex rsbb_beacon_mutex_;
      std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> last_rsbb_beacon_;

      bool
      recv_msg (boost::asio::ip::udp::endpoint& endpoint,
                uint16_t comp_id,
                uint16_t msg_type,
                std::shared_ptr<google::protobuf::Message> msg)
      {
        auto robot_beacon = std::dynamic_pointer_cast<roah_rsbb_msgs::RobotBeacon> (msg);
        if (robot_beacon) {
          {
            std::lock_guard<std::mutex> lock (robot_beacon_mutex_);
            last_robot_beacon_ = robot_beacon;
          }

          signal_robot_beacon_received_ (endpoint, comp_id, msg_type, robot_beacon);

          return true;
        }

        auto rsbb_beacon = std::dynamic_pointer_cast<roah_rsbb_msgs::RoahRsbbBeacon> (msg);
        if (rsbb_beacon) {
          {
            std::lock_guard<std::mutex> lock (rsbb_beacon_mutex_);
            last_rsbb_beacon_ = rsbb_beacon;
          }

          signal_rsbb_beacon_received_ (endpoint, comp_id, msg_type, rsbb_beacon);

          return true;
        }

        return false;
      }
  };


  template <typename ErrorHandler = CerrErrorHandler>
  class PrivateChannel :
    public Channel<ErrorHandler>
  {
    public:
      PrivateChannel (std::string const& host,
                      unsigned short port,
                      std::string const& key,
                      std::string const& cypher) :
        Channel<ErrorHandler> ("roah_rsbb::PrivateChannel(" + host + ":" + boost::lexical_cast<std::string> (port) + ")", host, port, key, cypher)
      {
        protobuf_comm::MessageRegister& mr = this->message_register();
        mr.add_message_type<roah_rsbb_msgs::BenchmarkState>();
        mr.add_message_type<roah_rsbb_msgs::RobotState>();
      }

      typedef
      boost::signals2::signal<void (boost::asio::ip::udp::endpoint&,
                                    uint16_t,
                                    uint16_t,
                                    std::shared_ptr<const roah_rsbb_msgs::BenchmarkState>) >
      signal_benchmark_state_received_type;

      signal_benchmark_state_received_type& signal_benchmark_state_received()
      {
        return signal_benchmark_state_received_;
      }

      std::shared_ptr<const roah_rsbb_msgs::BenchmarkState>
      last_benchmark_state() const
      {
        std::shared_ptr<const roah_rsbb_msgs::BenchmarkState> ret;
        {
          std::lock_guard<std::mutex> lock (benchmark_state_mutex_);
          ret = last_benchmark_state_;
        }
        return ret;
      }

      typedef
      boost::signals2::signal<void (boost::asio::ip::udp::endpoint&,
                                    uint16_t,
                                    uint16_t,
                                    std::shared_ptr<const roah_rsbb_msgs::RobotState>) >
      signal_robot_state_received_type;

      signal_robot_state_received_type& signal_robot_state_received()
      {
        return signal_robot_state_received_;
      }

      std::shared_ptr<const roah_rsbb_msgs::RobotState>
      last_robot_state() const
      {
        std::shared_ptr<const roah_rsbb_msgs::RobotState> ret;
        {
          std::lock_guard<std::mutex> lock (robot_state_mutex_);
          ret = last_robot_state_;
        }
        return ret;
      }

    private:
      signal_benchmark_state_received_type signal_benchmark_state_received_;
      std::mutex benchmark_state_mutex_;
      std::shared_ptr<const roah_rsbb_msgs::BenchmarkState> last_benchmark_state_;

      signal_robot_state_received_type signal_robot_state_received_;
      std::mutex robot_state_mutex_;
      std::shared_ptr<const roah_rsbb_msgs::RobotState> last_robot_state_;

      bool
      recv_msg (boost::asio::ip::udp::endpoint& endpoint,
                uint16_t comp_id,
                uint16_t msg_type,
                std::shared_ptr<google::protobuf::Message> msg)
      {
        auto benchmark_state = std::dynamic_pointer_cast<roah_rsbb_msgs::BenchmarkState> (msg);
        if (benchmark_state) {
          {
            std::lock_guard<std::mutex> lock (benchmark_state_mutex_);
            last_benchmark_state_ = benchmark_state;
          }

          signal_benchmark_state_received_ (endpoint, comp_id, msg_type, benchmark_state);

          return true;
        }

        auto robot_state = std::dynamic_pointer_cast<roah_rsbb_msgs::RobotState> (msg);
        if (robot_state) {
          {
            std::lock_guard<std::mutex> lock (robot_state_mutex_);
            last_robot_state_ = robot_state;
          }

          signal_robot_state_received_ (endpoint, comp_id, msg_type, robot_state);

          return true;
        }

        return false;
      }
  };
}

#endif
