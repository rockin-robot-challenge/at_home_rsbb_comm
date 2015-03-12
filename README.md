RoAH RSBB Comm
==============

This repository contains the basic files needed to communicate with
the RoCKIn@Home Referee, Scoring and Benchmarking Box.

Here are provided:

- The proto files used to communicate;
- A reduced version of the protobuf_comm library by Tim Niemueller
(http://www.robocup-logistics.org/refbox), containing only the files
needed to communicate over UDP;
- A C++ header to ease the task of communicating specifically with
RoAH RSBB.

:warning: Please remember to always update right before the competitions!
```bash
cd roah_rsbb_comm/
git pull
```


## Dependencies

You need to have installed a C++11 compiler, CMake, Boost, Protobuf
and OpenSSL.

If you are using Ubuntu, install the dependencies with:
```bash
sudo apt-get install build-essential cmake libboost-all-dev libprotoc-dev protobuf-compiler libssl-dev
```

This was tested with Ubuntu 12.04.5 LTS (Precise Pangolin) and
14.04.1 LTS (Trusty Tahr).


## Compiling

To compile, use:
```bash
cmake .
make
```

Two libraries will be compiled, that you can use on your projects:
```
lib/libroah_rsbb_msgs.a
lib/libprotobuf_comm.a
```


## Using

You can use the proto files in any language, if you implement
communication to be compliant with protobuf_comm. Check the
documentation at http://www.robocup-logistics.org/refbox for a
full description. There is also a Java implementation available
there.

Read below to understand the proto files.


## Using roah_rsbb.h

This C++ header file provides a small decorator on top of
protobuf_comm specifically for RoAH. The classes `PublicChannel` and
`PrivateChannel` adapt the use of each channel specifically. They
provide a pooling interface for the last messages of the allowed
types received on each channel as well as a specific signal for each
one. Be careful with synchronization, the callbacks will be called
from different threads.


## Using ros_roah_rsbb.h

The classes `RosPublicChannel` and `RosPrivateChannel` provide
a way to use `PublicChannel` and `PrivateChannel` with error output
done by ROS. Include this file only if your project uses ROS.
If your project uses some other framework, you can use this file
as a base to adapt to your framework.


## Understanding the proto files

*The proto files follow the general idea of the ones in Robocup LLSF,
but adapted to the specifics of RoAH. Most notably, the idea of
having two teams is completely dropped, a private channel is open
for every robot that is participating in a benchmark.*

RSBB communication happens in two kinds of channels:
- Public channel: Used to advertise the RSBB and active robots. Only
one such channel can be active and can be used by everyone.
- Private channels: Used to communicate with a team during a
benchmark. Multiple private channels can be active if multiple
benchmarks are happening simultaneously. These channels are
encrypted using the teams private passwords.

In normal operation, the RSBB publishes `RoahRsbbBeacon` in the
public channel every second. Active team robots should publish
`RobotBeacon`, also every second.

The `RoahRsbbBeacon` contains:
- A list of robots that should be ready to start or are already
benchmarking. For each robot a port number is provided, where
it should setup its private channel.
- The state of the home automation devices and tablet application.
Note that while the output is public, the devices and tablet can
only be controlled by teams executing *Catering for Granny Annie’s Comfort*.

The `RobotBeacon` contains:
- The team name, used to identify the team.
- The robot name, used to identify the robot within the team. There
are no multi-robot benchmarks, so this is only used for identification
in the public display.
- The clock time in the moment the message is sent. This is used to
identify clock synchronization problems as soon as possible.

When a robot is expected to run a benchmark soon, a private channel
is created on the port specified by the RSBB. The RSBB will transmit
`BenchmarkState` every second. The robot should transmit
`RobotState` every second, and stop transmitting in the public channel.
When anything changes, these messages can be transmitted faster for a
while, 10 messages with a period of 50 milliseconds. Benchmark
progression is controlled by a state machine.

The `BenchmarkState` contains:
- The specific type of benchmark to run, identified by its initials.
- The state the RSBB is in (benchmark state).
- The time of the last message received from the robot, used as an
acknowledgement.
- The goal for *Object Manipulation Functionality*, only if it is the
benchmark running and the benchmark state is `GOAL_TX`.

The `RobotState` contains:
- The clock time in the moment the message is sent. This is used to
identify clock synchronization problems and acknowledgements.
- The number of messages saved as offline data. This will be displayed
by the RSBB but will not be used for anything else. Teams should set
this to the number of messages saved or any other integer that signals
that messages are being saved as offline data. This will be used to
raise a warning as soon as possible is data saving fails.
- The state the robot is in (robot state).
- Online data to be transmitted.
- Result of the *Object Perception Functionality*, only if that is
the benchmark running and the robot state is `RESULT_TX`.
- Commands for the home automation devices and tablet application,
only if the benchmark is *Catering for Granny Annie’s Comfort*.


## Understanding benchmark and robot states

The progression of the benchmark is controlled by two state machines,
one running on the RSBB controlling the benchmark state and another
running on the robot controlling the robot state. This is necessary
because the connection is unreliable and messages are likely to be
lost, so they must be retransmitted until the state is updated on the
other side.

The robot state machine should be updated when a state from the RSBB
is received, according to the following table. The top row contains
the current robot state. When the benchmark state contained in the
first column is received from the RSBB, the robot state should be
updated for the one in the table.

|                    | **STOP**                  | **PREPARING**           | **WAITING_GOAL**                         | **EXECUTING**           | **RESULT_TX**                      |
|-------------------:|:-------------------------:|:-----------------------:|:----------------------------------------:|:-----------------------:|:----------------------------------:|
|           **STOP** | Keep    <br/> `STOP`      | Halt  <br/> `STOP`      | Halt                <br/> `STOP`         | Halt  <br/> `STOP`      | End of benchmark <br/> `STOP`      |
|        **PREPARE** | Prepare <br/> `PREPARING` | Keep  <br/> `PREPARING` | Keep                <br/> `WAITING_GOAL` | Error <br/> `STOP`      | New goal         <br/> `PREPARING` |
|        **GOAL_TX** | Error   <br/> `STOP`      | Error <br/> `STOP`      | Read goal and start <br/> `EXECUTING`    | Keep  <br/> `EXECUTING` | Keep             <br/> `RESULT_TX` |
| **WAITING_RESULT** | Error   <br/> `STOP`      | Error <br/> `STOP`      | No goal, start      <br/> `EXECUTING`    | Keep  <br/> `EXECUTING` | Keep             <br/> `RESULT_TX` |

Notice that there is no way to move to `WAITING_GOAL` or `RESULT_TX`.
This happens because these states are not reached by a command from
the RSBB but by an internal event.

| Current Robot State | Event                | New Robot State           |
|:-------------------:|:--------------------:|:-------------------------:|
| **PREPARING**       | Preparation complete | `WAITING_GOAL`            |
| **EXECUTING**       | Goal complete        | `RESULT_TX`               |

Therefore, when the robot state is `PREPARING`, the robot should move
to the right position and prepare as needed to receive the goal. When
the robot state is `EXECUTING`, the robot should execute the goal.

For reference, the RSBB can be expected to update according to the
following transition table on its end.

|                  | **STOP**                                | **PREPARE**                                              | **GOAL_TX**                               | **WAITING_RESULT**                        |
|-----------------:|:---------------------------------------:|:--------------------------------------------------------:|:-----------------------------------------:|:-----------------------------------------:|
|         **STOP** | Keep or start <br/> `STOP` or `PREPARE` | Keep                 <br/> `PREPARE`                     | Retry           <br/> `PREPARE`           | Retry           <br/> `PREPARE`           |
|    **PREPARING** | Error         <br/> `STOP`              | Keep                 <br/> `PREPARE`                     | Retry           <br/> `PREPARE`           | Retry           <br/> `PREPARE`           |
| **WAITING_GOAL** | Error         <br/> `STOP`              | Send goal or no goal <br/> `GOAL_TX` or `WAITING_RESULT` | Keep            <br/> `GOAL_TX`           | Retry           <br/> `PREPARE`           |
|    **EXECUTING** | Error         <br/> `STOP`              | Retry                <br/> `PREPARE`                     | Wait for result <br/> `WAITING_RESULT`    | Keep            <br/> `WAITING_RESULT`    |
|    **RESULT_TX** | Error         <br/> `STOP`              | Retry                <br/> `PREPARE`                     | New goal or end <br/> `PREPARE` or `STOP` | New goal or end <br/> `PREPARE` or `STOP` |
