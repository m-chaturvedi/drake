import sys
if sys.version_info <= (3, 0):
    sys.stdout.write("Regex requires Python 3.x\n")
    sys.exit(1)

# KNOWN ASSUMPTIONS:
# 1. Lint tests pass (for the regex to work).
# 2. No shadowing for line-wise variable replacement.
# 3. The doc variables are always defined like so: 
# `constexpr auto& doc = ...`

import numpy as np
import collections
import re
from pprint import pprint as pp
import pdb
import unittest
import inspect

# \w = [a-zA-Z0-9_] (approximate since the variable names can't start with a
# number.
value_regex = 'constexpr auto& {} = ([\w.]+);'
replacement_regex = ' ({})(\.[\w.]+)'

replace_variables = ["doc", "cls_doc"]

def lookup_regexes(s, r):
    prog = re.compile(r)
    found_terms = prog.findall(s)
    return found_terms

def lookup_regexes_linewise(f_s, var_name):
    f_line_array = f_s.split('\n')
    var_value_array = None
    var_value = ""
    buf = []
    for l in f_line_array:
        var_value_array = lookup_regexes(l, value_regex.format(var_name))
        if (not var_value_array):
            line = re.sub(replacement_regex.format(var_name),
                    r' ' + var_value + r'\2', l)
        else:
            assert(len(var_value_array) == 1)
            var_value = var_value_array[0]
            line = l

        buf.append(line)
    return "\n".join(buf)

def parse_file(f_s):
    mod_f_s = [f_s]
    for x in replace_variables:
        mod_f_s.append(lookup_regexes_linewise(mod_f_s[-1], x))
    return mod_f_s[-1]

def check_if_value_regex(s):
    res = []
    for r in replace_variables:
        prog = re.compile(value_regex.format(r))
        res = res + prog.findall(s)

    return True if res else False

def get_all_replaced_strings(f_s):
    modified_buffer = parse_file(f_s)
    prog = re.compile(replacement_regex.format("pydrake_doc"))
    res = []

    for l in modified_buffer.split('\n'):
        if not check_if_value_regex(l):
            f = prog.findall(l)
            assert(len(f) <= 1)
            res = res + f
    return [x[0] + x[1] for x in res]


def compare_symbols_from_mkdoc(f1):
    with open("pybind_symbols.txt", "r") as f:
        f_s = f.read()
        f_arr = f_s.split("\n")
        set1 = set(f_arr)
        set2 = set(f1)
        assert(set1.issuperset(set2))


def main():
    FILE = "/home/chaturvedi/workspace/drake-distro/bindings/pydrake/math_py.cc"
    with open(FILE, "r") as f:
        f_s = f.read()
        final_array = get_all_replaced_strings(f_s)
        print("\n".join(final_array))
        compare_symbols_from_mkdoc(final_array)

if __name__ == "__main__":
    main()

