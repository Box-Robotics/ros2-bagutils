import unittest

import datetime
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
    """

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        os.makedirs("{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME))

        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)
        bag = baggie.Baggie(bag_file_name, mode="w", compress=False)

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

    def test_meta(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)

        bag = baggie.Baggie(bag_file_name, mode="r")
        meta = bag.meta()

        self.assertIsInstance(meta, baggie._BagMetadata)
        self.assertIsInstance(meta.duration, datetime.timedelta)
        self.assertIsInstance(meta.starting_time, datetime.datetime)
        self.assertEqual(meta.message_count, N_MSGS*2)
