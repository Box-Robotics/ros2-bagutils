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
BAGNAME_COMPRESSED = "ziplock-c.bag"

class TestBaggieReader(unittest.TestCase):
    """
    Test fixture for the baggie.Baggie Python interface for reading bag files
    """

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()

        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)
        bag = baggie.Baggie(bag_file_name, mode="w", compress=False)

        bag_comp_file_name="{}{}{}".format(self.tmp_dir.name, os.sep,
                                           BAGNAME_COMPRESSED)
        bag_comp = baggie.Baggie(bag_comp_file_name, mode="w", compress=True)

        for i in range(N_MSGS):
            int_msg = Int32()
            int_msg.data = i

            str_msg = String()
            str_msg.data = "The count is: %s" % i

            bag.write(TOPIC_INT, int_msg)
            bag.write(TOPIC_STR, str_msg)

            bag_comp.write(TOPIC_INT, int_msg)
            bag_comp.write(TOPIC_STR, str_msg)

            time.sleep(1./N_MSGS)

    def tearDown(self):
        self.tmp_dir.cleanup()

    def test_file_does_not_exist(self):
        bag_file_name="{}{}{}".format(
          self.tmp_dir.name, os.sep, BAGNAME + "_foo")

        with self.assertRaises(baggie.BaggieException):
            bag = baggie.Baggie(bag_file_name, mode="r")

    def test_reader(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)

        bag = baggie.Baggie(bag_file_name, mode="r")
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

    def test_reader_topic_filtered(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)

        bag = baggie.Baggie(bag_file_name, mode="r")
        i = 0
        last_time = Time(nanoseconds=0)
        for topic, msg, ts in bag.read_messages(topics=[TOPIC_INT]):
            self.assertIsInstance(msg, Int32)
            self.assertIsInstance(ts, Time)
            self.assertTrue(last_time <= ts)
            last_time = ts
            i += 1

        self.assertEqual(i, N_MSGS)

    def test_compressed_reader(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep,
                                      BAGNAME_COMPRESSED)

        bag = baggie.Baggie(bag_file_name, mode="r")

        bag = baggie.Baggie(bag_file_name, mode="r")
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

    def test_readonly_write(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep,
                                      BAGNAME_COMPRESSED)

        bag = baggie.Baggie(bag_file_name, mode="r")
        int_msg = Int32()
        int_msg.data = 1000

        with self.assertRaises(baggie.BaggieException):
            bag.write(TOPIC_INT, int_msg)

    def test_reader_time_filtered(self):
        """ @todo """
        pass
