import unittest

import os
import tempfile
import time

import baggie
from example_interfaces.msg import Int32
from rclpy.serialization import deserialize_message, serialize_message

N_MSGS = 100
TOPIC_NAME = "/counter"
BAGNAME = "ziplock.bag"
MSGTYPE = "example_interfaces/msg/Int32"
SERFMT = "cdr"
COMPFMT = "zstd"

class TestSequentialCompressed(unittest.TestCase):
    """
    Test fixture for sequential read/write access, using compression, to a ROS2
    bag using the lower-level C++ interface.
    """

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.s_opt = baggie._StorageOptions()
        self.s_opt.uri = "{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)
        self.s_opt.storage_id = "sqlite3"
        os.makedirs(self.s_opt.uri)

        self.c_opt = baggie._ConverterOptions()
        self.c_opt.input_serialization_format = SERFMT
        self.c_opt.output_serialization_format = SERFMT

        self.comp_opt = baggie._CompressionOptions()
        self.comp_opt.compression_format = COMPFMT
        self.comp_opt.compression_mode = baggie._CompressionMode.FILE

        writer = baggie._Writer(self.comp_opt)
        writer.open(self.s_opt, self.c_opt)

        t_meta = baggie._TopicMetadata()
        t_meta.name = TOPIC_NAME
        t_meta.serialization_format = SERFMT
        t_meta.type = MSGTYPE
        writer.create_topic(t_meta)

        self.stamps = []
        for i in range(N_MSGS):
            msg = Int32()
            msg.data = i
            ts = baggie._stamp()
            self.stamps.append(ts)
            writer.write(t_meta.name, serialize_message(msg), ts)
            time.sleep(1./N_MSGS)

    def tearDown(self):
        self.tmp_dir.cleanup()

    def test_sequential_reader(self):
        info = baggie._BagInfo()
        meta = info.read_metadata(self.s_opt.uri, self.s_opt.storage_id)
        self.assertEquals(meta.storage_identifier, self.s_opt.storage_id)
        self.assertEquals(meta.message_count, N_MSGS)
        self.assertEquals(
          meta.compression_format, self.comp_opt.compression_format)
        self.assertEquals(meta.compression_mode, self.comp_opt.mode_to_string())

        reader = baggie._Reader(self.comp_opt)
        reader.open(self.s_opt, self.c_opt)

        i = 0
        while reader.has_next():
            topic, ser_msg, ts = reader.read_next()
            msg = deserialize_message(ser_msg, Int32)

            self.assertIsInstance(msg, Int32)
            self.assertEquals(i, msg.data)
            self.assertEquals(self.stamps[i], ts)

            i += 1

        self.assertEquals(i, N_MSGS)

if __name__ == '__main__':
    unittest.main()
