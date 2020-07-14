import unittest

import baggie
from rclpy.time import Time
from datetime import datetime as DT

class TestStamps(unittest.TestCase):

    def test_stamps(self):
        now = DT.now()
        nanos_in = baggie.util.stamp(now)
        nanos_passthru = baggie.util.stamp(nanos_in)
        nanos_out = baggie.util.stamp(Time(nanoseconds=nanos_in))

        self.assertEqual(nanos_in, nanos_passthru)
        self.assertEqual(nanos_in, nanos_out)
