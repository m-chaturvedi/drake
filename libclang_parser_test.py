import libclang_parser
import unittest
import pdb
import tempfile


class TestRegexMethods(unittest.TestCase):

    def test_lookup_regex(self):
        A = b"""
.def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
constexpr auto& doc = pydrake_doc.drake.math;
using Class = RigidTransform<T>;

constexpr auto& cls_doc = doc.RigidTransform;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), cls_doc.
            ctor.doc_2args_R_p)

constexpr auto& doc = pydrake_doc.drake1.math1;
using Class = RigidTransform1<T>;



constexpr auto& cls_doc = doc.RigidTransform1;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), cls_doc.ctor.
            doc_2args_R_p)

"""

        expected_res = b"""
.def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
constexpr auto& doc = pydrake_doc.drake.math;
using Class = RigidTransform<T>;

constexpr auto& cls_doc = pydrake_doc.drake.math.RigidTransform;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), pydrake_doc.drake.math.
            RigidTransform.ctor.doc_2args_R_p)

constexpr auto& doc = pydrake_doc.drake1.math1;
using Class = RigidTransform1<T>;



constexpr auto& cls_doc = pydrake_doc.drake1.math1.RigidTransform1;

.def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), pydrake_doc.drake1.math1.RigidTransform1
            .ctor.doc_2args_R_p)

"""

        with tempfile.NamedTemporaryFile(suffix=".cc", delete=False) as f1:
            f1.write(A)

        with tempfile.NamedTemporaryFile(suffix=".cc", delete=False) as f2:
            f2.write(expected_res)
        expected_tokens = libclang_parser.get_tokens(f2.name)

        t1_processed, pydoc_strings = libclang_parser.replace_tokens_in_file(f1.name)

        replaced_strings = [
            "pydrake_doc.drake.math.RigidTransform.ctor.doc_2args_R_p",
            "pydrake_doc.drake1.math1.RigidTransform1.ctor.doc_2args_R_p",
        ]
        assert(t1_processed == expected_tokens)
        assert(replaced_strings == pydoc_strings)


if __name__ == "__main__":
    unittest.main()
