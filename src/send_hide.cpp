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



int
main (int argc,
      char** argv)
{
  PublicChannel<> public_channel ("255.255.255.255", 6666);

  roah_rsbb_msgs::RoahRsbbBeacon beacon;
  beacon.mutable_devices_bell()->set_sec(0);
  beacon.mutable_devices_bell()->set_nsec(0);
  beacon.set_devices_switch_1 (false);
  beacon.set_devices_switch_2 (false);
  beacon.set_devices_switch_3 (false);
  beacon.set_devices_dimmer (0);
  beacon.set_devices_blinds (0);

  beacon.set_tablet_display_map (false);
  
  beacon.mutable_tablet_call_time()->set_sec(0);
  beacon.mutable_tablet_call_time()->set_nsec(0);
  beacon.mutable_tablet_position_time()->set_sec(0);
  beacon.mutable_tablet_position_time()->set_nsec(0);
  beacon.set_tablet_position_x (0);
  beacon.set_tablet_position_y (0);
  public_channel.send (beacon);

  boost::posix_time::ptime sleep_time = boost::posix_time::microsec_clock::universal_time();
  sleep_time += boost::posix_time::milliseconds (100);
  boost::this_thread::sleep (sleep_time);
}
