import argparse
import sys, os
p = os.path.abspath(__file__)
sys.path.append(os.path.join(os.path.dirname(p), "../"))

import fileMethods

def region2json(region_file, json_file):
    data = fileMethods.readFromFile(region_file)

    with open(json_file,'w+') as json_file_obj:
        json_file_obj.write("\n".join(data["Regions"]))
    json_file_obj.closed

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert region file to json file.")
    parser.add_argument('region_file', type=str, help='Path to region file')
    parser.add_argument('json_file', type=str, help='Path to save json file')

    args, unknown = parser.parse_known_args()
    print args

    region2json(args.region_file, args.json_file)
