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

"""Script to copy a bag file, optionally filtered by topic and a time window"""

import argparse
import os
import sys
from rclpy.time import Time
from baggie import BagReader, BagWriter

def get_args():
    parser = argparse.ArgumentParser(
        description="Copy a bag file, optionally filtered by topic and time",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("-i", "--infile", required=True, type=str,
                        help="Path to input bag file")
    parser.add_argument("-o", "--outfile", required=False, type=str,
                        help="Output bag file, default: <infile>-filtered.bag")
    parser.add_argument("--start_time", required=False, type=int,
                        help="Earliest message stamp in output bag (nanos)")
    parser.add_argument("--end_time", required=False, type=int,
                        help="Latest message stamp in output bag (nanos)")
    parser.add_argument("--topics", required=False, type=str,
                        metavar="T", nargs="+",
                        help="List of topics to include in output bag")
    parser.add_argument("--map", required=False, type=str,
                        metavar="M", nargs="+",
                        help="Topic name remappings: --map from:to")
    compression_group = parser.add_mutually_exclusive_group()
    compression_group.add_argument("--compress", required=False,
                                   action="store_true", default=False,
                                   help="Compress output file")
    compression_group.add_argument("--uncompress", required=False,
                                   action="store_true", default=False,
                                   help="Do no compress output file")

    args = parser.parse_args(sys.argv[1:])
    return args

def main() -> int:
    args = get_args()

    infile = args.infile
    if not os.path.exists(infile):
        raise OSError("The input file does not exist: {}".format(infile))

    outfile = args.outfile
    if outfile is None:
        outfile = infile.rstrip(".bag")
        outfile += "-filtered.bag"

    os.makedirs(outfile)

    t0 = None
    t1 = None

    if args.start_time is not None:
        t0 = Time(nanoseconds=args.start_time)

    if args.end_time is not None:
        t1 = Time(nanoseconds=args.end_time)

    topic_list = None
    if args.topics is not None:
        topic_list = args.topics

    topic_lut = {}
    if args.map is not None:
        for remapping in args.map:
            from_to = remapping.split(":")
            if len(from_to) == 2:
                topic_lut[str(from_to[0])] = str(from_to[1])

    with BagReader(infile) as inbag:
        compress_opts = None
        if ((not args.compress) and (not args.uncompress)):
            compress_opts = inbag.comp_opt_

        with BagWriter(outfile,
                       compress=args.compress,
                       storage_opts=inbag.s_opt_,
                       converter_opts=inbag.c_opt_,
                       compression_opts=compress_opts) as outbag:

            for topic, msg, t, in inbag.read_messages(
              topics=topic_list, start_time=t0, end_time=t1):
                out_topic = topic
                if ((args.map is not None) and (topic in topic_lut)):
                    out_topic = topic_lut[topic]
                outbag.write(out_topic, msg, t)

    return 0

if __name__ == '__main__':
    sys.exit(main())
