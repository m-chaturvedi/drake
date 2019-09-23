import pickle
import parse_pybind11_doc_xml
import os
from lxml import etree as ET
import pandas


def main():
    df = parse_pybind11_doc_xml.main()
    filenames, coverage = df.loc[:, "FileName"], df.loc[:, "Coverage"]
    file_coverage = dict(zip(filenames, coverage))
    root = parse_pybind11_doc_xml.make_tree(file_coverage)
    ids, labels, parents, nums, dens = [], [], [], [], []

    def BFS(root):
        if len(root.getchildren()) == 0:
            return
        name = root.attrib["name"]
        path_arr = name.split(os.sep)
        num_total = sum([int(n) for n in root.xpath('.//*/@num')])
        den_total = sum([int(n) for n in root.xpath('.//*/@den')])
        if num_total > 0.0:
            nums.append(num_total)
            dens.append(den_total)
            ids.append(name)
            labels.append(path_arr[-1])
            parents.append((os.sep).join(path_arr[:-1]))
        for r in root.getchildren():
            BFS(r)

    drake_node = root.getchildren()[0]
    assert(drake_node.tag == "drake")
    BFS(drake_node)

    data = {
        "ids":  ids,
        "labels": labels,
        "parents": parents,
        "nums": nums,
        "dens": dens
    }
    sunburst_df = pandas.DataFrame.from_dict(data)
    sunburst_df.to_csv(path_or_buf="sunburst_data1.csv", index=False)


if __name__ == "__main__":
    main()
