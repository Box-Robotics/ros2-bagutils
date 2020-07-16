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
**NOTE:** In the above example we pass a `datetime.datetime` as the timestamp
when we write the message. We do this for simplicity and convenience in this
example. However, we recommend passing an `rclpy.time.Time` instance instead
(the method accepts either). The reason for this recommendation is that
`datetime.datetime` in Python only support usec precision whereas the ROS 2
time libraries support nanosecond precision.

The above examples are intentionally simplistic and accept many default
arguments. However, since `baggie` [exposes](./baggie/src/py/_baggie.cpp) a
significant portion of the C++ API, much more complex use cases are
supported. The context manager examples above, are a front-end to the central
fixture of this library, the `baggie.Baggie` class. [The Baggie
class](./baggie/baggie/_Baggie.py) provides an interface for reading or writing
ROS 2 bag files directly. A given instance of a Baggie can be instantiated as
either a reader or a writer. One `Baggie` maps to exactly one on-disk ROS 2 bag
file (which may be made of up several files). Extensive example code is
available in the [test directory](./baggie/test/) to including manually driving
the lower-level C++ interface via the Python projections. Additionally,
real-world examples of using the `baggie` API can be gleaned from reading the
code for the [utility scripts](./baggie/baggie/cmd/) provided with the `baggie`
package.

### Utility scripts
Beyond the Python library code, the `baggie` package provides several
command-line utilities for performing common operations on bagfiles. Here is a
high-level overview of what is available including some simple examples.

#### filter
The `filter` script is used to copy bag files while also applying some
filtering during the copying process. Here is its help string:

```
$ ros2 run baggie filter --help
usage: filter [-h] -i INFILE [-o OUTFILE] [--start_time START_TIME] [--end_time END_TIME] [--topics T [T ...]] [--map M [M ...]]
              [--compress | --uncompress]

Copy a bag file, optionally filtered by topic and time

optional arguments:
  -h, --help            show this help message and exit
  -i INFILE, --infile INFILE
                        Path to input bag file (default: None)
  -o OUTFILE, --outfile OUTFILE
                        Output bag file, default: <infile>-filtered.bag (default: None)
  --start_time START_TIME
                        Earliest message stamp in output bag (nanos) (default: None)
  --end_time END_TIME   Latest message stamp in output bag (nanos) (default: None)
  --topics T [T ...]    List of topics to include in output bag (default: None)
  --map M [M ...]       Topic name remappings: --map from:to (default: None)
  --compress            Compress output file (default: False)
  --uncompress          Do not compress output file (default: False)
```

The only required option is `-i` (or `--infile`) to specify which input bag
file to operate on. When run in this way, `filter` acts like the Unix `cp`
command (albeit, inefficient). The output file will be named the same as the
input file except the suffix `-filtered.bag` will be appended to the end. Here
is an example:

```
$ ls
lidar.bag

$ ros2 bag info lidar.bag

Files:             lidar.bag_0.db3
Bag size:          403.6 MiB
Storage id:        sqlite3
Duration:          474.726s
Start:             Jun 18 2020 15:26:17.900 (1592508377.900)
End:               Jun 18 2020 15:34:12.626 (1592508852.626)
Messages:          2829
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1414 | Serialization Format: cdr
                   Topic: /os1_cloud_node/points_tf | Type: sensor_msgs/msg/PointCloud2 | Count: 1414 | Serialization Format: cdr


$ ros2 run baggie filter -i lidar.bag
[INFO] [1594828793.442763739] [rosbag2_storage]: Opened database 'lidar.bag/lidar.bag_0.db3' for READ_ONLY.
[INFO] [1594828793.464352982] [rosbag2_storage]: Opened database 'lidar-filtered.bag/lidar-filtered.bag_0.db3' for READ_WRITE.

$ ros2 bag info lidar-filtered.bag

Files:             lidar-filtered.bag_0.db3
Bag size:          403.6 MiB
Storage id:        sqlite3
Duration:          474.726s
Start:             Jun 18 2020 15:26:17.900 (1592508377.900)
End:               Jun 18 2020 15:34:12.626 (1592508852.626)
Messages:          2829
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1414 | Serialization Format: cdr
                   Topic: /os1_cloud_node/points_tf | Type: sensor_msgs/msg/PointCloud2 | Count: 1414 | Serialization Format: cdr
```

For exemplary purposes, suppose we wanted a new bag that only contained the
data from the topic `/os1_cloud_node/points_tf` but in the output bag, we
wanted the topic to be called `/points`. Also, suppose we want to compress the
output file. This looks like:

```
$ ros2 run baggie filter -i lidar.bag --compress --topics /os1_cloud_node/points_tf --map /os1_cloud_node/points_tf:/points
[INFO] [1594829379.985118419] [rosbag2_storage]: Opened database 'lidar.bag/lidar.bag_0.db3' for READ_ONLY.
[INFO] [1594829380.007895566] [rosbag2_storage]: Opened database 'lidar-filtered.bag/lidar-filtered.bag_0.db3' for READ_WRITE.

$ ros2 bag info lidar-filtered.bag

Files:             lidar-filtered.bag/lidar-filtered.bag_0.db3.zstd
Bag size:          372.3 MiB
Storage id:        sqlite3
Duration:          143.497s
Start:             Jun 18 2020 15:26:17.900 (1592508377.900)
End:               Jun 18 2020 15:28:41.397 (1592508521.397)
Messages:          1414
Topic information: Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 1414 | Serialization Format: cdr

$ cat lidar-filtered.bag/metadata.yaml
rosbag2_bagfile_information:
  version: 4
  storage_identifier: sqlite3
  relative_file_paths:
    - lidar-filtered.bag/lidar-filtered.bag_0.db3.zstd
  duration:
    nanoseconds: 143497058611
  starting_time:
    nanoseconds_since_epoch: 1592508377900096491
  message_count: 1414
  topics_with_message_count:
    - topic_metadata:
        name: /points
        type: sensor_msgs/msg/PointCloud2
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 1414
  compression_format: zstd
  compression_mode: FILE
```

**TODO:** Add a filter, similar to `--topics`, based on message type.

#### split
The `split` script is used to split a single bag into multiple (smaller) bags
on a **time basis**. Here is its help string:

```
$ ros2 run baggie split --help
usage: split [-h] -i INFILE [-o OUTDIR] [-f FRACTION]

Splits a bag file into time-based partitions

optional arguments:
  -h, --help            show this help message and exit
  -i INFILE, --infile INFILE
                        Path to input bag file (default: None)
  -o OUTDIR, --outdir OUTDIR
                        Path to output directory for split bag files (default: None)
  -f FRACTION, --fraction FRACTION
                        Time fraction (0., 1.]; .5 = split in half (default: 0.5)
```

The critical argument to this script is `-f` (or `--fraction`). It controls how
the bag is split based on the message time stamps. To split a bag into two
(smaller) bags on a time basis, you would pass `-f .5` denoting that each bag
should contain half (`.5`) of the wall clock time associated with this input
bag. This does **not** mean that the bags will each contain half of the
messages. The splitting is based on time. Let's look at a concrete example.

Suppose we have a bag full of pointclouds captured from a LiDAR that looks like
this:

```
$ ros2 bag info pointcloud.bag

Files:             pointcloud.bag_0.db3
Bag size:          403.5 MiB
Storage id:        sqlite3
Duration:          143.497s
Start:             Jun 18 2020 15:26:17.900 (1592508377.900)
End:               Jun 18 2020 15:28:41.397 (1592508521.397)
Messages:          1414
Topic information: Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 1414 | Serialization Format: cdr
```

Let's split it into 4 smaller bags on a time basis:

```
$ ros2 run baggie split -i pointcloud.bag -f .25
[INFO] [1594912449.657473501] [rosbag2_storage]: Opened database 'pointcloud.bag/pointcloud.bag_0.db3' for READ_ONLY.
[INFO] [1594912449.680794886] [rosbag2_storage]: Opened database './00_pointcloud.bag/00_pointcloud.bag_0.db3' for READ_WRITE.
[INFO] [1594912451.548708732] [rosbag2_storage]: Opened database './01_pointcloud.bag/01_pointcloud.bag_0.db3' for READ_WRITE.
[INFO] [1594912453.278216856] [rosbag2_storage]: Opened database './02_pointcloud.bag/02_pointcloud.bag_0.db3' for READ_WRITE.
[INFO] [1594912455.048369887] [rosbag2_storage]: Opened database './03_pointcloud.bag/03_pointcloud.bag_0.db3' for READ_WRITE.
```

We note that, by default, the output bags are created in the same directory as
the input source bag. Additionally, the output bags will inherit their storage,
converter, and compression options from the input bag from which they were
derived. Here is what `info` reports for the 4 generated output bags:

```
$ for f in $(ls -1 | grep -i "^0.*\.bag$"); do ros2 bag info ${f}; done

Files:             00_pointcloud.bag_0.db3
Bag size:          99.6 MiB
Storage id:        sqlite3
Duration:          35.802s
Start:             Jun 18 2020 15:26:17.900 (1592508377.900)
End:               Jun 18 2020 15:26:53.702 (1592508413.702)
Messages:          353
Topic information: Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 353 | Serialization Format: cdr


Files:             01_pointcloud.bag_0.db3
Bag size:          103.0 MiB
Storage id:        sqlite3
Duration:          35.827s
Start:             Jun 18 2020 15:26:53.796 (1592508413.796)
End:               Jun 18 2020 15:27:29.623 (1592508449.623)
Messages:          356
Topic information: Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 356 | Serialization Format: cdr


Files:             02_pointcloud.bag_0.db3
Bag size:          102.2 MiB
Storage id:        sqlite3
Duration:          35.805s
Start:             Jun 18 2020 15:27:29.695 (1592508449.695)
End:               Jun 18 2020 15:28:05.501 (1592508485.501)
Messages:          352
Topic information: Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 352 | Serialization Format: cdr


Files:             03_pointcloud.bag_0.db3
Bag size:          98.6 MiB
Storage id:        sqlite3
Duration:          35.797s
Start:             Jun 18 2020 15:28:05.599 (1592508485.599)
End:               Jun 18 2020 15:28:41.397 (1592508521.397)
Messages:          353
Topic information: Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 353 | Serialization Format: cdr
```

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
