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
import sys
from contextlib import contextmanager

import baggie.util
from baggie._baggie import BagMetadata as _BagMetadata
from baggie._baggie import CompressionMode as _CompressionMode
from baggie._baggie import CompressionOptions as _CompressionOptions
from baggie._baggie import ConverterOptions as _ConverterOptions
from baggie._baggie import BagInfo as _BagInfo
from baggie._baggie import Reader as _Reader
from baggie._baggie import StorageFilter as _StorageFilter
from baggie._baggie import StorageOptions as _StorageOptions
from baggie._baggie import TopicMetadata as _TopicMetadata
from baggie._baggie import TopicInformation as _TopicInformation
from baggie._baggie import Writer as _Writer

from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time

class BaggieException(Exception):
    """
    Exception wrapper around trapped errors from the rosbag2 C++ API
    """
    pass

class Baggie(object):
    """
    The Baggie class provides an interface for reading or writing ROS 2
    bag files.

    A given instance of a Baggie can be instantiated as either a reader or a
    writer. One Baggie maps to exactly one on-disk ROS 2 bag file.

    Attributes
    ----------
    DEFAULT_SERIALIZATION_FORMAT : str
      Class variable indicating the default serialization format used and
      assumed by the library. The default is `cdr`.

    DEFAULT_COMPRESSION_MODE : baggie._baggie._CompressionMode
      Class variable indicating the default compression mode applied when
      writing a compressed bag. The default is `FILE`.

    DEFAULT_COMPRESSION_FORMAT : str
      Class variable indicating the default compression format applied when
      writing a compressed bag. The default is `zstd`.

    DEFAULT_STORAGE_ID : str
      Class variable indicating the underlying storage format of the bag
      file. The default is `sqlite3`.

    All other attributes are "private" to the implementation.

    """
    DEFAULT_SERIALIZATION_FORMAT = "cdr"
    DEFAULT_COMPRESSION_MODE = _CompressionMode.FILE
    DEFAULT_COMPRESSION_FORMAT = "zstd"
    DEFAULT_STORAGE_ID = "sqlite3"

    def __init__(self, filename, mode="r",
                 storage_id=DEFAULT_STORAGE_ID,
                 compress=False,
                 storage_opts=None,
                 converter_opts=None,
                 compression_opts=None):
        """
        Instantiates a new Baggie instance in either reader ("r") or writer
        ("w") mode.

        Parameters
        ----------
          filename : str
            Path to the bag file to open. In "r" mode, this file must already
            exist. In "w" mode, this file must not exist. For clarity, bag
            files in ROS 2 are actually directories. The expected `filename` is
            the path to the bag directory.

          mode : str
            One of "r" or "w". A mode of "r" instantiates the Baggie as a
            reader. A mode of "w" instantiates the Baggie as a writer.

          compress : bool
            This parameter only applies when opened in "w" mode. As a reader,
            the compression options are introspected by the bag file
            metadata. If set to `True` as a writer, default compression options
            will be applied to the bag file as it is written out. When set to
            `False` as a writer, no compression options are applied. This
            parameter is overridden by the passed in `compression_opts`
            (optional).

          storage_opts : baggie._baggie.StorageOptions
            An instance of `StorageOptions` giving fine grained control over
            how the underlying bag file is opened. The `uri` filed of the
            passed in `storage_opts` will always be overridden by the required
            `filename` parameter passed into this ctor.

          converter_opts: baggie._baggie.ConverterOptions
            An instance of `ConverterOptions` giving fine grained control over
            how the underlying messages are serialized. Only applies when run
            in "w" mode.

          compression_opts: baggie._baggie.CompressionOptions
            An instance of `CompressionOptions` giving fine grained control
            over how the underlying bag file compression is treated. Only
            applies in "w" mode.

        """
        self.reader_ = None
        self.writer_ = None

        self.s_opt_ = storage_opts
        self.c_opt_ = converter_opts
        self.comp_opt_ = compression_opts

        self.meta_ = None

        if mode == "r":
            info = _BagInfo()

            try:
                if ((self.s_opt_ is None) or
                    (not isinstance(self.s_opt_, _StorageOptions))):
                    self.meta_ = info.read_metadata(filename, storage_id)
                else:
                    self.meta_ = \
                      info.read_metadata(filename, self.s_opt_.storage_id)

            except RuntimeError as rte:
                raise(BaggieException(rte))

            self.s_opt_ = _StorageOptions()
            self.s_opt_.uri = filename
            self.s_opt_.storage_id = self.meta_.storage_identifier

            self.c_opt_ = _ConverterOptions()
            self.c_opt_.input_serialization_format = \
              self.DEFAULT_SERIALIZATION_FORMAT
            self.c_opt_.output_serialization_format = \
              self.DEFAULT_SERIALIZATION_FORMAT

            self.comp_opt_ = _CompressionOptions()
            self.comp_opt_.compression_format = self.meta_.compression_format
            if self.meta_.compression_mode == "":
                self.comp_opt_.compression_mode = _CompressionMode.NONE
            else:
                # NOTE: as of now, only FILE compression is supported. We will
                # have to do some better introspection once other schemes are
                # available.
                self.comp_opt_.compression_mode = _CompressionMode.FILE

            try:
                self.reader_ = _Reader(self.comp_opt_)
                self.reader_.open(self.s_opt_, self.c_opt_)

            except Exception as ex:
                raise(BaggieException(ex))

        elif mode == "w":
            if self.s_opt_ is None:
                self.s_opt_ = _StorageOptions()
                self.s_opt_.storage_id = storage_id
            elif not isinstance(self.s_opt_, _StorageOptions):
                raise(TypeError(
                    "'storage_opts' must be an instance of " +
                    "'baggie._baggie.StorageOptions' or 'None', " +
                    " not '%s'" % type(self.s_opt_)))
            self.s_opt_.uri = filename

            if self.c_opt_ is None:
                self.c_opt_ = _ConverterOptions()
                self.c_opt_.input_serialization_format = \
                  self.DEFAULT_SERIALIZATION_FORMAT
                self.c_opt_.output_serialization_format = \
                  self.DEFAULT_SERIALIZATION_FORMAT
            elif not isinstance(self.c_opt_, _ConverterOptions):
                raise(TypeError(
                    "'converter_opts' must be an instance of " +
                    "'baggie._baggie.ConverterOptions' or 'None', " +
                    " not '%s'" % type(self.c_opt_)))

            if self.comp_opt_ is None:
                self.comp_opt_ = _CompressionOptions()
                self.comp_opt_.compression_format = \
                  self.DEFAULT_COMPRESSION_FORMAT
                if compress:
                    self.comp_opt_.compression_mode = \
                      self.DEFAULT_COMPRESSION_MODE
                else:
                    self.comp_opt_.compression_mode = _CompressionMode.NONE
            elif not isinstance(self.comp_opt_, _CompressionOptions):
                raise(TypeError(
                    "'compression_opts' must be an instance of " +
                    "'baggie._baggie.CompressionOptions' or 'None', " +
                    " not '%s'" % type(self.comp_opt_)))

            try:
                self.writer_ = _Writer(self.comp_opt_)
                self.writer_.open(self.s_opt_, self.c_opt_)

            except Exception as ex:
                raise(BaggieException(ex))

        else:
            raise ValueError("Unsupported mode: %s" % mode)

    def meta(self):
        """
        If the Baggie has been instantiated as a reader ("r" mode), it caches
        the bag metadata. This method provides an accessor to this cached
        information. If the Baggie as been instantiated as a writer ("w" mode),
        this method returns `None`.

        Returns
        -------
        baggie._baggie.BagMetadata or None

        """
        return self.meta_

    def write(self, topic, msg, t=None):
        """
        Write a message to Baggie opened in "w" mode.

        Parameters
        ----------
        topic : str
          The topic name to write to

        msg : Message
          The unserialzed message instance (of any message type) to be written
          on the topic.

        t : rclpy.time.Time or datetime.datetime or None
          The timestamp for the message. If its value is `None` the current
          system time is used.

        Exceptions
        ----------
        baggie.BaggieException if an attempt to write is made on a Baggie
        opened as a reader ("r" mode). Exceptions encountered by the underlying
        C++ library are propogated up should they be encountered while writing.

        """
        try:
            self.writer_.write(
                topic, serialize_message(msg), baggie.util.stamp(t))

        except IndexError as idx_err:
            t_meta = _TopicMetadata()
            t_meta.name = topic
            t_meta.serialization_format = self.c_opt_.input_serialization_format
            t_meta.type = baggie.util.msg2typestr(msg)

            self.writer_.create_topic(t_meta)
            self.writer_.write(
                topic, serialize_message(msg), baggie.util.stamp(t))

        except Exception as ex:
            if self.writer_ is None:
                raise(BaggieException("Cannot write in read-only mode"))

            raise(ex)

    def read_messages(self, topics=None, start_time=None, end_time=None):
        """
        Generator function used to produce each message in sequence from a
        Baggie opened as a reader ("r" mode).

        Parameters
        ----------
        topics : list of str
          A list of fully qualified topic names to include in the generated
          results. If this parameter is specified, topics not listed will be
          omitted from the generated output. Works together with `start_time`
          and `end_time`.

        start_time : rclpy.time.Time
          Only messages stamped with this start time or later will be included
          in the generated output. Works together with `topics` and
          `end_time`.

        end_time : rclpy.time.Time
          Only messages stamped with this end time or earlier will be included
          in the generated output. Works together with the `topics` and
          `start_time`.

        Returns
        -------
        3-tuple : str, msg, rclpy.time.Time
          [0] is the topic name
          [1] is the deserialized message
          [2] is the timestamp on the message

        If the Baggie is opened as a writer ("w" mode), a
        baggie.BaggieException is raised.

        """
        if self.reader_ is None:
            raise(BaggieException("Cannot read in write-only mode"))

        start_time_filter = Time(nanoseconds=0)
        if ((start_time is not None) and (isinstance(start_time, Time))):
            start_time_filter = start_time

        end_time_filter = Time(nanoseconds=sys.maxsize)
        if ((end_time is not None) and (isinstance(end_time, Time))):
            end_time_filter = end_time

        self.reader_.reset_filter()
        if topics is not None:
            filt = _StorageFilter()
            filt.topics = topics
            self.reader_.set_filter(filt)

        type_lut = {}
        t_meta_list = self.reader_.get_all_topics_and_types()
        for t_meta in t_meta_list:
            if ((topics is not None) and (t_meta.name not in topics)):
                continue
            type_lut[t_meta.name] = baggie.util.typestr2msgtype(t_meta.type)

        while self.reader_.has_next():
            topic, ser_msg, ts = self.reader_.read_next()
            msg_time = Time(nanoseconds=ts)
            if ((msg_time < start_time_filter) or
                (msg_time > end_time_filter)):
                continue

            msg = deserialize_message(ser_msg, type_lut[topic])
            yield topic, msg, msg_time

@contextmanager
def BagReader(*args, **kwargs):
    """
    Context Manager wrapper around a baggie.Baggie in `read` mode.
    """
    kwargs["mode"] = "r"
    bag = Baggie(*args, **kwargs)
    try:
        yield bag
    finally:
        pass

@contextmanager
def BagWriter(*args, **kwargs):
    """
    Context Manager wrapper around a baggie.Baggie in `write` mode.
    """
    kwargs["mode"] = "w"
    bag = Baggie(*args, **kwargs)
    try:
        yield bag
    finally:
        pass
