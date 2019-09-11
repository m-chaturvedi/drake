from lxml import etree as ET
import unittest
import filecmp

XPATHS = {
    "all_doc_vars": ".//Node[@doc_var]",
    "doc_var_with_ignore": ".//Node[@doc_var][@ignore='0']",
    "class_decl": ".//Node[@kind='CursorKind.CLASS_DECL' or @kind='CursorKind.CLASS_TEMPLATE'][@ignore='0']",
}


def write_to_file(arr, file_name):
    with open(file_name, "w") as f:
        [f.write(s + "\n") for s in arr]


def compare_files(filename_1, filename_2):
    return filecmp.cmp(filename_1, filename_2, shallow=False)


def get_docstrings_from_xpath(root, xpath):
    """Gets docstrings for an xpath.  Namespaces have an empty doc_var.

    :root: Root of the XML Tree
    :xpath: Xpath to search
    :returns: Gets the doc vars (may include blanks)

    """
    all_found = [x.attrib["doc_var"] for x in root.xpath(xpath)]
    return all_found


class TestXML(unittest.TestCase):
    def test_previous_docstrings(root):
        """Tests the docstrings obtained with previous approaches

        :root: Root for the XML tree
        :returns: boolean

        """

        tree = ET.parse('bazel-bin/bindings/pydrake/documentation_pybind_xml.h')
        root = tree.getroot()

        all_doc_strings = get_docstrings_from_xpath(root, XPATHS["all_doc_vars"])
        write_to_file([x for x in all_doc_strings if x], "xml_docstrings.txt")
        assert(compare_files("xml_docstring.txt", "pybind_symbols.txt"))

        doc_strings_after_ignored = get_docstrings_from_xpath(
                root, XPATHS["doc_var_with_ignore"])
        write_to_file(
                [x for x in doc_strings_after_ignored if x],
                "xml_docstrings_after_ignore.txt")

        assert(compare_files("xml_docstrings_after_ignore.txt",
        "pybind11_without_attic_and_automotive_and_internal_and_common_and_examples.txt"))


if __name__ == "__main__":
    unittest.main()
