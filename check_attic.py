#!/bin/python3
import filecmp
import glob
import os
from pprint import pprint as pp


def get_base_name(string_list):
    string_list = string_list \
        if isinstance(string_list, list) else [string_list]

    basenames = []

    for s in string_list:
        basenames.append(os.path.basename(s))
    return basenames


def get_globbed_files(glob_string):
    returned_files = []
    for f in glob.iglob(glob_string, recursive=True):
        returned_files.append(f)
    return returned_files


def main():
    attic_headers = get_globbed_files("attic/**/*.h")
    install_headers = get_globbed_files("install/**/*.h")
    install_headers_base = get_base_name(install_headers)
    header_map = {}

    for x, b_x in zip(attic_headers, get_base_name(attic_headers)):
        ind = [i for i, x in enumerate(install_headers_base) if x == b_x]
        # print("i = {} {}".format(len(ind), b_x))
        for i in ind:
            if filecmp.cmp(x, install_headers[i], shallow=False):
                header_map[x] = install_headers[i]

    for v in header_map.values():
        prefix = "install/include/"
        if v.startswith(prefix):
            print("\"{}\",".format(v[len(prefix):]))


if __name__ == "__main__":
    main()
