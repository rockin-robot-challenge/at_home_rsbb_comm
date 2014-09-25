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

#include <boost/thread/thread.hpp>

#include "roah_rsbb.h"



using namespace std;
using namespace roah_rsbb;



const string TEAM_NAME = "DumpTeam";
const string ROBOT_NAME = "The Listener";
const string CRYPTO_KEY = "randomkey";
const string CRYPTO_CIPHER = "aes-128-cbc";



class DumpComm
{
    string host_;
    unsigned short port_;
    PublicChannel<> public_channel_;
    mutex private_channel_mutex_;
    shared_ptr<PrivateChannel<>> private_channel_;

    static void
    receive_benchmark_state (boost::asio::ip::udp::endpoint& endpoint,
                             uint16_t comp_id,
                             uint16_t msg_type,
                             shared_ptr<const roah_rsbb_msgs::BenchmarkState> msg)
    {
      cout << "Received BenchmarkState from " << endpoint.address().to_string()
           << ":" << endpoint.port()
           << ", COMP_ID " << comp_id
           << ", MSG_TYPE " << msg_type << endl
           << "  benchmark_type: " << msg->benchmark_type() << endl
           << "  benchmark_state: " << msg->benchmark_state() << endl
           << flush;
    }

    static void
    receive_robot_state (boost::asio::ip::udp::endpoint& endpoint,
                         uint16_t comp_id,
                         uint16_t msg_type,
                         shared_ptr<const roah_rsbb_msgs::RobotState> msg)
    {
      cout << "Received RobotState from " << endpoint.address().to_string()
           << ":" << endpoint.port()
           << ", COMP_ID " << comp_id
           << ", MSG_TYPE " << msg_type << endl
           << "  time: " << msg->time().sec() << "." << msg->time().nsec() << endl
           << "  messages_saved: " << msg->messages_saved() << endl
           << flush;
    }

    void
    receive_rsbb_beacon (boost::asio::ip::udp::endpoint& endpoint,
                         uint16_t comp_id,
                         uint16_t msg_type,
                         shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> rsbb_beacon)
    {
      cout << "Received RoahRsbbBeacon from " << endpoint.address().to_string()
           << ":" << endpoint.port()
           << ", COMP_ID " << comp_id
           << ", MSG_TYPE " << msg_type << endl;

      unsigned short connect_port = 0;
      for (auto const& bt : rsbb_beacon->benchmarking_teams()) {
        cout << "  team_name: " << bt.team_name()
             << ", robot_name: " << bt.robot_name()
             << ", rsbb_port: " << bt.rsbb_port() << endl;
        if ( (bt.team_name() == TEAM_NAME) && (bt.robot_name() == ROBOT_NAME)) {
          connect_port = bt.rsbb_port();
          // break; // Commented to show all entries
        }
      }

      lock_guard<mutex> lock (private_channel_mutex_);
      if (connect_port != (private_channel_ ? private_channel_->port() : 0)) {
        if (private_channel_) {
          cout << "Disconnecting private channel" << endl;
          private_channel_.reset();
        }
        if (connect_port) {
          cout << "Connecting private channel to " << endpoint.address().to_string()
               << ":" << connect_port << endl;
          private_channel_ = make_shared<PrivateChannel<CerrErrorHandler>> (endpoint.address().to_string(), connect_port, CRYPTO_KEY, CRYPTO_CIPHER);
          private_channel_->signal_benchmark_state_received().connect (&DumpComm::receive_benchmark_state);
          private_channel_->signal_robot_state_received().connect (&DumpComm::receive_robot_state);
        }
      }

      cout << flush;
    }

    static void
    receive_robot_beacon (boost::asio::ip::udp::endpoint& endpoint,
                          uint16_t comp_id,
                          uint16_t msg_type,
                          shared_ptr<const roah_rsbb_msgs::RobotBeacon> msg)
    {
      cout << "Received RobotBeacon from " << endpoint.address().to_string()
           << ":" << endpoint.port()
           << ", COMP_ID " << comp_id
           << ", MSG_TYPE " << msg_type << endl
           << "  team_name: " << msg->team_name() << endl
           << "  robot_name: " << msg->robot_name() << endl
           << "  time: " << msg->time().sec() << "." << msg->time().nsec() << endl
           << flush;
    }

  public:
    DumpComm (string const& host,
              unsigned short port) :
      host_ (host),
      port_ (port),
      public_channel_ (host, port),
      private_channel_mutex_(),
      private_channel_()
    {
      cout << "Connected public channel to " << host
           << ":" << port << endl << flush;
      public_channel_.signal_rsbb_beacon_received().connect (boost::bind (&DumpComm::receive_rsbb_beacon, this, _1, _2, _3, _4));
      public_channel_.signal_robot_beacon_received().connect (&DumpComm::receive_robot_beacon);
    }

    void run()
    {
      boost::posix_time::ptime loop_time = boost::posix_time::microsec_clock::universal_time();
      while (true) {
        shared_ptr<PrivateChannel<CerrErrorHandler>> private_channel;
        {
          lock_guard<mutex> lock (private_channel_mutex_);
          private_channel = private_channel_;
        }
        if (private_channel) {
          roah_rsbb_msgs::RobotState msg;
          roah_rsbb::now (msg.mutable_time());
          msg.set_messages_saved (0);
          cout << "Sending RobotState" << endl << flush;
          private_channel->send (msg);
        }
        else {
          roah_rsbb_msgs::RobotBeacon msg;
          msg.set_team_name (TEAM_NAME);
          msg.set_robot_name (ROBOT_NAME);
          roah_rsbb::now (msg.mutable_time());
          cout << "Sending RobotBeacon" << endl << flush;
          public_channel_.send (msg);
        }

        loop_time += boost::posix_time::milliseconds (1000);
        boost::this_thread::sleep (loop_time);
      }
    }
};



int
main (int argc,
      char** argv)
{
  DumpComm comm ("10.255.255.255", 6666);
  comm.run();
}
