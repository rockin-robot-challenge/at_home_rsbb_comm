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


## Dependencies

You need to have installed a C++11 compiler, CMake, Boost, Protobuf
and OpenSSL.

If you are using Ubuntu, install the dependencies with:
```
sudo apt-get install build-essential cmake libboost-all-dev libprotoc-dev protobuf-compiler libssl-dev
```

This was tested with Ubuntu 12.04.5 LTS (Precise Pangolin) and
14.04.1 LTS (Trusty Tahr).

## Compiling

To compile, use:
```
cmake .
make
```

Two libraries will be compiled, that you can use on your projects:
```
lib/libroah_rsbb_msgs.a
lib/libprotobuf_comm.a
```

## Using roah_rsbb.h

This C++ header file provides a small decorator on top of
protobuf_comm specifically for RoAH. The classes `PublicChannel` and
`PrivateChannel` adapt the use of each channel specifically. They
provide a pooling interface for the last messages of the allowed
types received on each channel as well as a specific signal for each
one. Be careful with synchronization, the callbacks will be called
from different threads.
