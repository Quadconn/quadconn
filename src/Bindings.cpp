#include <pybind11/pybind11.h>
#include <pybind11/native_enum.h>

#include "FastMath.hpp"
#include "Manipulator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(QuadconnBindings, m, py::mod_gil_not_used()) {
    // ------------- FastMath ------------------
   
    m.def("sin", &fmath::sin);
    m.def("cos", &fmath::cos);

    // ------------- Manipulator ---------------
    
    auto manipulator = py::class_<Manipulator>(m, "Manipulator")
        .def(py::init<double, double, double, double, double, double>(),
            py::arg("l1"), py::arg("l2"),
            py::arg("l1_max_angle"), py::arg("l1_min_angle"),
            py::arg("l2_max_angle"), py::arg("l2_min_angle"))
        .def(py::init<double, double>(), py::arg("l1"), py::arg("l2"))
        .def("ik", &Manipulator::ik, py::arg("x"), py::arg("y"), py::arg("j"));

    py::class_<Manipulator::JointAngles>(manipulator, "JointAngles")
        .def(py::init<>())  // default constructor
        .def_readwrite("theta1_p", &Manipulator::JointAngles::theta1_p)
        .def_readwrite("theta2_p", &Manipulator::JointAngles::theta2_p)
        .def_readwrite("theta1_n", &Manipulator::JointAngles::theta1_n)
        .def_readwrite("theta2_n", &Manipulator::JointAngles::theta2_n);

    py::native_enum<Manipulator::Status>(manipulator, "Status", "enum.Enum")
        .value("UNREACHABLE", Manipulator::Status::UNREACHABLE)
        .value("OUT_OF_JOINT_LIMITS", Manipulator::Status::OUT_OF_JOINT_LIMITS)
        .value("SUCCESS", Manipulator::Status::SUCCESS)
        .finalize();
}
