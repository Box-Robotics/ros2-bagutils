ros2-bagutils
=============
This repository contains packages for working with bag files in ROS 2.

baggie
------
The `baggie` package provides a Python wrapper around the [C++ rosbag2
API](https://github.com/ros2/rosbag2/tree/master/rosbag2_cpp) as well as some
pure Python convenience interfaces for making working with bags in ROS 2
easier. Inspired, in part, by the (forthcoming?)
[rosbag2_py](https://github.com/ros2/rosbag2/pull/308/files) Python API, this
package exposes the necessary C++ API via Pybind11 but puts a Python interface
in front of it to make the easy things easy and the hard things possible.

In the simplest of use cases, you'll want to read from an existing bag. You can
do this with a context manager like:

```python
import baggie

[ ... ]

with baggie.BagReader("/path/to/file.bag") as bag:
    for topic, msg, stamp in bag.read_messages():
      # do something with the data
```

Writing a bag is similarly easy:

```python
import baggie
from example_interfaces.msg import Int32
from datetime import datetime as DT

[ ... ]

with baggie.BagWriter("/path/to/file.bag") as bag:
    msg = Int32()
    msg.data = 1
    bag.write("/int_topic", msg, DT.now())
```

The above examples are intentionally simplistic and accept many default
arguments. However, since `baggie` exposes a significant portion of the C++
API, much more complex use cases are supported. The context manager examples
above, are a front-end to the central fixture of this library, the
`baggie.Baggie` class. The `Baggie` class provides an interface for reading or
writing ROS 2 bag files directly. A given instance of a Baggie can be
instantiated as either a reader or a writer. One `Baggie` maps to exactly one
on-disk ROS 2 bag file (which may be made of up several files). Extensive
example code is available in the [test directory](./baggie/test/) to including
manually driving the lower-level C++ interface via the Python projections.

### Utility scripts
Beyond the Python library code, the `baggie` package provides several
command-line utilities for performing common operations on bagfiles that
roboticists will typically need to do. Here is a sampling of what is available
and their usage.

#### filter

#### split

#### join

LICENSE
=======
Please see the file called [LICENSE](LICENSE)

<p align="center">
  <br/>
  <img src="baggie/doc/figures/box-logo.png"/>
  <br/>
  Copyright &copy; 2020 Box Robotics, Inc.
</p>
