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

"""Output a JSON timeline of messages contained in a set of bags"""

import argparse
import glob
import json
import os
import sys
from datetime import datetime
from baggie import BagReader

def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="A JSON timeline of messages contained in a set of bags")

    parser.add_argument("infiles", metavar="INFILE", type=str, nargs="+",
                         help="The input bag files to add to the timeline")
    parser.add_argument("--pretty", dest="pretty", action="store_true",
                        default=False,
                        help="Pretty print the output JSON")
    ds_group = parser.add_mutually_exclusive_group()
    ds_group.add_argument("--aos", required=False,
                          action="store_true", default=False,
                          help="Output the data as an array-of-structs")
    ds_group.add_argument("--soa", required=False,
                          action="store_true", default=False,
                          help="Output the data as a struct-of-arrays")

    args = parser.parse_args(sys.argv[1:])
    return args

def get_time_str(ts):
    dt = datetime.fromtimestamp(ts // 1e9)
    s = dt.strftime('%Y-%m-%d %H:%M:%S')
    s += '.' + str(int(ts % 1e9)).zfill(9)
    return s

def main():
    args = get_args()

    infiles_ = []
    for infile in args.infiles:
        paths = glob.glob(infile)
        for path in paths:
            infiles_.append(path)

    aos = []
    soa = {"topic": [], "type": [], "stamp": []}

    infiles = sorted(set(infiles_))
    for infile in infiles:
        with BagReader(infile) as inbag:
            meta = inbag.meta()
            t_meta = {}
            for t_info in meta.topics_with_message_count:
                    t_name = t_info.topic_metadata.name
                    t_type = t_info.topic_metadata.type
                    t_meta[t_name] = t_type

            for topic, msg, t in inbag.read_messages():
                nanos = t.nanoseconds
                d = {'topic_name': topic,
                     'type': t_meta[topic],
                     'stamp': nanos,
                     'time': get_time_str(nanos)}
                aos.append(d)

                soa["topic"].append(topic)
                soa["type"].append(t_meta[topic])
                soa["stamp"].append(nanos)

    if args.pretty:
        if (((not args.aos) and (not args.soa)) or args.aos):
            print(json.dumps(aos, sort_keys=True, indent=4))
        else:
            print(json.dumps(soa, sort_keys=True, indent=4))
    else:
        if (((not args.aos) and (not args.soa)) or args.aos):
            print(json.dumps(aos, sort_keys=True))
        else:
            print(json.dumps(soa, sort_keys=True))

    return 0

if __name__ == '__main__':
    sys.exit(main())
