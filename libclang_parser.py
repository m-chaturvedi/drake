from clang import cindex
import pdb
import glob
import re
cindex.Config.set_library_path("/usr/lib/llvm-6.0/lib/")
import pandas
import pickle

replace_variables = ["doc", "cls_doc", "var_doc", "enum_doc"]

def var_value_search(var_name):
    re_for_search = {
        "start_tokens": ["constexpr", "auto", "&", var_name, "="],
        "value_regex": r'[\w.]+',
        "end_token": r';'
    }
    return re_for_search

def pydrake_doc_search():
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
    tu = cindex.TranslationUnit.from_source(filename)
    FILE = tu.get_file(filename)
    readlines = open(filename, "r").readlines()
    lines, cols = len(readlines), len(readlines[-1])
    st = cindex.SourceLocation.from_position(tu, FILE, 1, 1)
    en = cindex.SourceLocation.from_position(tu, FILE, lines, cols)
    extent = cindex.SourceRange.from_locations(st, en)
    tokens = tu.get_tokens(extent=extent)
    token_spellings = [t.spelling for t in tokens if t.kind is not cindex.TokenKind.COMMENT]
    return token_spellings


def replace_tokens_in_file(filename):
    token_spellings_original = get_tokens(filename)
    token_spellings = token_spellings_original[:]

    for x in replace_variables:
        replace_tokens(token_spellings, x)
    
    pydoc_strings = get_pydoc_strings(token_spellings)
    # [print(x) for x in pydoc_strings]

    write_to_file(token_spellings_original, "A.txt")
    write_to_file(token_spellings, "B.txt")
    return token_spellings, pydoc_strings


def compare_symbols_from_mkdoc(symbols_to_compare, prefix=""):
    pybind_file = "pybind11_without_attic.txt"
    with open(pybind_file, "r") as f:
        set_doc = set([line.rstrip('\n') for line in f])

        set_to_compare = set(symbols_to_compare)
        filtered_set = set(filter(lambda x: x.startswith(prefix), set_doc))

        set_to_compare = set_to_compare - \
            set(['pydrake_doc.drake.solvers.MosekSolver',
                'pydrake_doc.drake.solvers.GurobiSolver'])

        if not set_doc.issuperset(set_to_compare):
            print(set_to_compare - set_doc)
        assert(filtered_set.issuperset(set_to_compare))

        t_f_array = []
        all_symbols_list = list(set_doc)
        all_symbols_list.sort()
        for x in all_symbols_list:
            t_f_array.append(True if x in symbols_to_compare else False)
        print("Percentage: {}".format(100.0 * len(symbols_to_compare)/len(all_symbols_list)))
        print("percentage: {}/{}".format(len(set_to_compare), len(set_doc)))
        d = {
                "all_symbols": all_symbols_list,
                "is_used": t_f_array
            }
        df = pandas.DataFrame(data=d)
        df.to_csv("test.csv")


def main():
    bindings_path = "./bindings/pydrake/**/*.cc"
    ignore_path = ("./bindings/pydrake/common/test",
                   "./bindings/pydrake/attic")
    # bindings_path = "./bindings/pydrake/attic"
    # ignore_path = ("")

    array_for_all_files = []

    for f in glob.iglob(bindings_path, recursive=True):
        if f.startswith(ignore_path):
            continue

        print("On file: {}".format(f))
        _, final_array = replace_tokens_in_file(f)
        array_for_all_files = array_for_all_files + final_array

    # A = pickle.load(open("parse_pybind_strings.pickle", "rb"))
    # assert(set(A) == set(array_for_all_files))
    compare_symbols_from_mkdoc(array_for_all_files)


if __name__ == "__main__":
    main()
