import unittest

import os
import tempfile
import time

import baggie
from example_interfaces.msg import Int32

N_MSGS = 100
TOPIC_NAME = "/counter"
BAGNAME = "ziplock.bag"

class TestBaggieWriter(unittest.TestCase):
    """
    Test fixture for the baggie.Baggie Python interface to writing bag files
    """

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp_dir.cleanup()

    def test_defaults(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)
        bag = baggie.Baggie(bag_file_name, mode="w")

        for i in range(N_MSGS):
            msg = Int32()
            msg.data = i
            bag.write(TOPIC_NAME, msg)
            time.sleep(1./N_MSGS)

    def test_compressed(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)
        bag = baggie.Baggie(bag_file_name, mode="w", compress=True)

        for i in range(N_MSGS):
            msg = Int32()
            msg.data = i
            bag.write(TOPIC_NAME, msg)
            time.sleep(1./N_MSGS)

    def test_legal_override_types(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)

        s_opt = baggie._StorageOptions()
        s_opt.storage_id = baggie.Baggie.DEFAULT_STORAGE_ID

        c_opt = baggie._ConverterOptions()
        c_opt.input_serialization_format = \
          baggie.Baggie.DEFAULT_SERIALIZATION_FORMAT
        c_opt.output_serialization_format = \
          baggie.Baggie.DEFAULT_SERIALIZATION_FORMAT

        comp_opt = baggie._CompressionOptions()
        comp_opt.compression_format = baggie.Baggie.DEFAULT_COMPRESSION_FORMAT
        comp_opt.compression_mode = baggie.Baggie.DEFAULT_COMPRESSION_MODE

        bag = baggie.Baggie(bag_file_name, mode="w",
                            storage_opts=s_opt,
                            converter_opts=c_opt,
                            compression_opts=comp_opt)

    def test_illegal_override_types(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)

        with self.assertRaises(TypeError):
            bag = baggie.Baggie(bag_file_name, mode="w", storage_opts="foo")

        with self.assertRaises(TypeError):
            bag = baggie.Baggie(bag_file_name, mode="w", converter_opts=100)

        with self.assertRaises(TypeError):
            bag = baggie.Baggie(bag_file_name, mode="w", compression_opts=1.0)


    def test_file_already_exits(self):
        bag_file_name="{}{}{}".format(self.tmp_dir.name, os.sep, BAGNAME)

        bag1 = baggie.Baggie(bag_file_name, mode="w")

        with self.assertRaises(baggie.BaggieException):
            bag2 = baggie.Baggie(bag_file_name, mode="w")
