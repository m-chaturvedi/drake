from parse_pybind11 import *
import unittest

class TestRegexMethods(unittest.TestCase):

    def test_lookup_regex(self):
        A = """
.def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
constexpr auto& doc = pydrake_doc.drake.math;
using Class = RigidTransform<T>;

constexpr auto& cls_doc = doc.RigidTransform;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), cls_doc.ctor.doc_2args_R_p)

constexpr auto& doc = pydrake_doc.drake1.math1;
using Class = RigidTransform1<T>;



constexpr auto& cls_doc = doc.RigidTransform1;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), cls_doc.ctor.doc_2args_R_p)

"""

        expected_res = """
.def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
constexpr auto& doc = pydrake_doc.drake.math;
using Class = RigidTransform<T>;

constexpr auto& cls_doc = pydrake_doc.drake.math.RigidTransform;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), pydrake_doc.drake.math.RigidTransform.ctor.doc_2args_R_p)

constexpr auto& doc = pydrake_doc.drake1.math1;
using Class = RigidTransform1<T>;



constexpr auto& cls_doc = pydrake_doc.drake1.math1.RigidTransform1;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), pydrake_doc.drake1.math1.RigidTransform1.ctor.doc_2args_R_p)

"""
        res1 = lookup_regexes_linewise(A, "doc")
        res = lookup_regexes_linewise(res1, "cls_doc")
        assert(res == expected_res)
        replaced_strings = [
            "pydrake_doc.drake.math.RigidTransform.ctor.doc_2args_R_p",
            "pydrake_doc.drake1.math1.RigidTransform1.ctor.doc_2args_R_p",
        ]
        assert(replaced_strings == get_all_replaced_strings(A))


if __name__ == "__main__":
    unittest.main()
