# Copyright 2020 Box Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import importlib

from datetime import datetime as DT
from rclpy.time import Time

def stamp(dt=None):
    """
    Generates a timestamp compatiable for writing to rosbag2

      Parameters:
        dt (datetime.datetime or rclpy.time.Time):
          A `datetime` or `Time` object representing the timestamp. If this
          parameter is omitted, the current system time is used.

      Returns:
        An int encoding of the timestamp as nanoseconds past the epoch (of a
        particular clock; usually the system clock)

    """
    if dt is None:
        dt = DT.now()

    if isinstance(dt, DT):
        return int(dt.timestamp() * 1e9)
    elif isinstance(dt, Time):
        return dt.nanoseconds
    elif isinstance(dt, int):
        return dt
    else:
        # NOTE: let's not encourage passing in an `int` (see docs above -- we
        # don't document it -- and the error message below -- also,
        # undocumented).
        raise(TypeError(
            "stamp: 'dt' must be an instance of " +
            "'datetime.datetime' or 'rclpy.time.Time', " +
            " not '%s'" % type(dt)))

def msg2typestr(msg):
    """
    Introspects the message type from the passed in `msg` and encodes it as a
    string in the format required by rosbag2.

    Parameters
    ----------
    msg : Message
      A ROS 2 message (deserialized) whose type we need to introspect

    Returns
    -------
    A string encoding of the message type suitable for serialization to a ROS 2
    bag.

    """
    mod_components = msg.__module__.split(".")
    mod_components[-1] = msg.__class__.__name__
    return "/".join(mod_components)

def typestr2msgtype(type_str):
    """
    Given a ROS 2 bag message type encoding string, this function will return a
    Type instance of the actual Python message type that can then be used for
    creating instances of the particular message class. If the loader for the
    class type is not on your `sys.path` an exception will be thrown.

    Parameters
    ----------
    type_str : str
      A string encoding of a message type compatible with rosbag2

    Returns
    -------
    The `Type` of the Python message.

    """
    module_components = type_str.split("/")
    type_name = module_components[-1]

    module_str = ".".join(module_components[0:-1])
    module = importlib.import_module(module_str)

    return type(getattr(module, type_name)())
