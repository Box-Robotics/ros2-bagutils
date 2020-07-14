import unittest

import os
import tempfile
import time

import baggie
from example_interfaces.msg import Int32
from example_interfaces.msg import String
from rclpy.time import Time

N_MSGS = 100
TOPIC_INT = "/counter"
TOPIC_STR = "/chatter"
BAGNAME = "ziplock.bag"

class TestBaggieWriter(unittest.TestCase):
    """
    Test fixture for the baggie context manager interfaces
    """

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        os.makedirs("{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME))

        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)
        with baggie.BagWriter(bag_file_name) as bag:
            for i in range(N_MSGS):
                int_msg = Int32()
                int_msg.data = i

                str_msg = String()
                str_msg.data = "The count is: %s" % i

                bag.write(TOPIC_INT, int_msg)
                bag.write(TOPIC_STR, str_msg)

                time.sleep(1./N_MSGS)

    def tearDown(self):
        self.tmp_dir.cleanup()

    def test_reader(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)

        with baggie.BagReader(bag_file_name) as bag:
            i = 0
            last_time = Time(nanoseconds=0)
            for topic, msg, ts in bag.read_messages():
                if topic == TOPIC_INT:
                    self.assertIsInstance(msg, Int32)
                elif topic == TOPIC_STR:
                    self.assertIsInstance(msg, String)

                self.assertIsInstance(ts, Time)
                self.assertTrue(last_time <= ts)
                last_time = ts
                i += 1

            self.assertEqual(i, N_MSGS*2)
