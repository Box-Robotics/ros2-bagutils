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

"""Script to split a bag file into time-based partitions"""

import argparse
import os
import sys
from rclpy.time import Time, Duration
from baggie import BagReader, Baggie, _stamp

def get_args():
    parser = argparse.ArgumentParser(
        description="Splits a bag file into time-based partitions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    def fraction_type(x):
        x = float(x)
        if ((x <= 0.) or (x > 1.)):
            raise argparse.ArgumentTypeError("Fraction must be in: (0., 1.]")
        return x

    parser.add_argument("-i", "--infile", required=True, type=str,
                        help="Path to input bag file")
    parser.add_argument("-o", "--outdir", required=False,
                        help="Path to output directory for split bag files")
    parser.add_argument("-f", "--fraction", type=fraction_type, default=.5,
                        help="Time fraction (0., 1.]; .5 = split in half")

    args = parser.parse_args(sys.argv[1:])
    return args

def main() -> int:
    args = get_args()

    if args.outdir is None:
        args.outdir = os.path.dirname(args.infile)
        if args.outdir == "":
            args.outdir = "."

    with BagReader(args.infile) as inbag:
        meta = inbag.meta()
        t_start = Time(nanoseconds=_stamp(meta.starting_time_as_nanos()))
        t_end = Time(nanoseconds=_stamp(
            t_start + Duration(nanoseconds=meta.duration_as_nanos())))
        n_partitions = int(1./args.fraction)
        partitions = []
        for i in range(n_partitions):
            part_no = i + 1
            part_len = Duration(
                nanoseconds=int((t_end - t_start).nanoseconds * args.fraction))
            partitions.append(
              t_start + Duration(
                  nanoseconds=int(part_no * part_len.nanoseconds)))

        part_no = 0
        outfile = "{}{}{:02d}_{}".format(
            args.outdir, os.sep, part_no, os.path.basename(args.infile))
        outbag = Baggie(outfile, mode="w",
                        storage_opts=inbag.s_opt_,
                        converter_opts=inbag.c_opt_,
                        compression_opts=inbag.comp_opt_)

        for topic, msg, t in inbag.read_messages():
            if part_no == 0:
                if t <= partitions[part_no]:
                    outbag.write(topic, msg, t)
                else:
                    part_no += 1
                    outfile = "{}{}{:02d}_{}".format(
                        args.outdir, os.sep,
                        part_no, os.path.basename(args.infile))
                    outbag = Baggie(outfile, mode="w",
                                    storage_opts=inbag.s_opt_,
                                    converter_opts=inbag.c_opt_,
                                    compression_opts=inbag.comp_opt_)
                    outbag.write(topic, msg, t)

            elif (part_no == (len(partitions) - 1)):
                outbag.write(topic, msg, t)

            else:
                if ((t > partitions[part_no - 1]) and
                    (t <= partitions[part_no])):
                    outbag.write(topic, msg, t)
                else:
                    part_no += 1
                    outfile = "{}{}{:02d}_{}".format(
                        args.outdir, os.sep,
                        part_no, os.path.basename(args.infile))
                    outbag = Baggie(outfile, mode="w",
                                    storage_opts=inbag.s_opt_,
                                    converter_opts=inbag.c_opt_,
                                    compression_opts=inbag.comp_opt_)
                    outbag.write(topic, msg, t)

    return 0

if __name__ == '__main__':
    sys.exit(main())
