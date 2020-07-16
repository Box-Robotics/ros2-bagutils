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

"""Joins multiple bags into a single bag file"""

import argparse
import glob
import os
import sys
from baggie import BagReader, BagWriter

def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Joins several ROS 2 bag files into a single combined bag")

    parser.add_argument("-o", "--outfile", type=str, required=True,
                        help="The output bag file name to create")
    parser.add_argument("infiles", metavar="INFILE", type=str, nargs="+",
                         help="The input bag files to join")
    compression_group = parser.add_mutually_exclusive_group()
    compression_group.add_argument("--compress", required=False,
                                   action="store_true", default=False,
                                   help="Compress the output file")
    compression_group.add_argument("--uncompress", required=False,
                                   action="store_true", default=False,
                                   help="Do not compress the output file")

    args = parser.parse_args(sys.argv[1:])
    return args

def main():
    args = get_args()

    infiles_ = []
    for infile in args.infiles:
        paths = glob.glob(infile)
        for path in paths:
            infiles_.append(path)

    infiles = sorted(set(infiles_))

    with BagWriter(args.outfile, compress=args.compress) as outbag:
        for infile in infiles:
            with BagReader(infile) as inbag:
                for topic, msg, t in inbag.read_messages():
                    outbag.write(topic, msg, t)

    return 0

if __name__ == '__main__':
    sys.exit(main())
