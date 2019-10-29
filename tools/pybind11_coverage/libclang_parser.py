# This file parses the python bindings files and extracts the doc variables
# using libclang.

# KNOWN ASSUMPTIONS:
# 1. No shadowing for line-wise variable replacement.
# 2. The `doc` variables are always defined like so:
#    `constexpr auto& doc = ...`
# 3. Anything that doesn't observe the line-wise replacement technique. For
#    example: There's a function to which the documentation variable is passed,
#    and the function is defined _before_ the call.  There can be other cases
#    well.
# 4. The replaced variables are from the list `replace_variables`.
# 5. The doc variables used like so (ignore the whitespaces):
#    `, <doc_variable> )`

from clang import cindex
import re
import logging

# Tested with Python 3
assert(__import__('sys').version_info[0] == 3)
cindex.Config.set_library_path("/usr/lib/llvm-6.0/lib/")
logging.basicConfig(level=logging.DEBUG)

replace_variables = ["doc", "cls_doc", "var_doc", "enum_doc"]


def var_value_search(var_name):
    """Defines the regex needed for extracting the values with which `var_name`
       is initialized.

    Args:
        var_name: The name of the variable which follows the convention
            (whitespace is ignored)
            `constexpr auto& <var_name> = ...`

    Returns:
        The regex hash needed for search.
    """
    re_for_search = {
        "start_tokens": ["constexpr", "auto", "&", var_name, "="],
        "value_regex": r'[\w.]+',
        "end_token": r';'
    }
    return re_for_search


def pydrake_doc_search():
    """Get the regex needed for finding locations where doc variables are used
       in pybind functions.  Locations such as:
       (whitespace is ignored)
       `..., pydrake_doc.<var_name> )`


    Args:

    Returns:
        The regex needed for subitution.
    """
    re_for_search = {
        "start_tokens": [","],
        "value_regex": r'pydrake_doc[\w.]+',
        "end_token": ")"
    }
    return re_for_search


def var_substitution_search(var_name):
    re_for_search = {
        "start_token": r'[=,]',
        "value_regex": var_name,
        "end_token": r'[\.]'
    }
    return re_for_search


def get_var_value(re_search, tokens, i):
    li = []
    i = i + len(re_search["start_tokens"])
    while(i < len(tokens) and tokens[i] != re_search["end_token"]):
        li.append(tokens[i])
        i = i + 1
    return li


def check_substitution_regex(tokens, i, value_li, var_name):
    re_for_search = var_substitution_search(var_name)
    st = re_for_search["start_token"]
    en = re_for_search["end_token"]

    tokens_match = False

    if re.match(st, tokens[i]) and len(tokens) > i + 2 and \
            re.match(en, tokens[i+2]):
        tokens_match = True

    if tokens_match and tokens[i+1] == var_name:
        tokens[i+1:i+2] = value_li


def replace_tokens(tokens, var_name):
    re_search = var_value_search(var_name)
    st_tokens = re_search["start_tokens"]
    value_li = None

    for i, t in enumerate(tokens):
        num_tokens = len(st_tokens)
        if tokens[i:(i + num_tokens)] == st_tokens:
            value_li = get_var_value(re_search, tokens, i)
            st = "".join(value_li)
            assert(re.match(re_search["value_regex"], st))

        elif value_li:
            check_substitution_regex(tokens, i, value_li, var_name)


def get_pydoc_strings(tokens):
    re_search = pydrake_doc_search()
    st_tokens = re_search["start_tokens"]
    rex = re_search["value_regex"]
    value_li = None
    pydoc_strings = []
    for i, t in enumerate(tokens):
        num_tokens = len(st_tokens)
        if tokens[i:(i + num_tokens)] == st_tokens:
            value_li = get_var_value(re_search, tokens, i)
            joined_li = "".join(value_li)
            if re.match(rex, joined_li):
                pydoc_strings.append(joined_li)
    return pydoc_strings


def write_to_file(arr, file_name):
    with open(file_name, "w") as f:
        [f.write(s + "\n") for s in arr]


def get_tokens(filename):
    # We don't particulary need to set the std, but it looks like a bug in
    # cindex used by us.  (Not in llvm-mirror's version of `cindex.py`.
    # `args_array` can be uninitialized.)
    # // NOLINTNEXTLINE(whitespace/line_length).
    # https://github.com/wjakob/clang-cindex-python3/blob/6a00cbc4a9b8e68b71caf7f774b3f9c753ae84d5/cindex.py#L2514-L2533
    tu = cindex.TranslationUnit.from_source(filename, ["-std=c++11"])
    FILE = tu.get_file(bytes(filename, 'utf8'))

    with open(filename, "r") as f:
        readlines = f.readlines()

    lines, cols = len(readlines), len(readlines[-1])
    st = cindex.SourceLocation.from_position(tu, FILE, 1, 1)
    en = cindex.SourceLocation.from_position(tu, FILE, lines, cols)
    extent = cindex.SourceRange.from_locations(st, en)
    tokens = tu.get_tokens(extent=extent)
    token_spellings = [t.spelling for t in tokens if t.kind is
                       not cindex.TokenKind.COMMENT]

    return token_spellings


def replace_tokens_in_file(filename):
    token_spellings_original = get_tokens(filename)
    token_spellings = token_spellings_original[:]

    for x in replace_variables:
        replace_tokens(token_spellings, x)
    pydoc_strings = get_pydoc_strings(token_spellings)

    write_to_file(token_spellings_original, "A.txt")
    write_to_file(token_spellings, "B.txt")
    return token_spellings, pydoc_strings


def get_docstring_for_bindings(filenames):
    """Given a list of pybind filenames, get thes docstrings used in them

    Args:
        filenames: Names of the files

    Returns:
        list: List of the docstrings used
    """

    array_for_all_files = []

    for f in filenames:
        logging.info("On file: {}".format(f))
        _, final_array = replace_tokens_in_file(f)
        array_for_all_files = array_for_all_files + final_array
    return [x for x in array_for_all_files if x]
