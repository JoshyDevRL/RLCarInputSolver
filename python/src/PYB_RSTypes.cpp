#include "PYB.h"

PYB_INIT_F(RSTypes) {
#define PYB_CUR_CLASS Vec
	PYB_CLASS(Vec)
		PYB_DEFAULT_INITS()
		.def(pyb::init<float, float, float>(), PYBA("x"), PYBA("y"), PYBA("z"))
		PYBP(x)
		PYBP(y)
		PYBP(z)

		.def("__getitem__", [](const Vec& v, size_t i) { return v[i]; })
		.def("__setitem__", [](Vec& v, size_t i, float f) { v[i] = f; })

		.def("__str__", [](const Vec& v) { return RS_STR(v); })
		;
#define PYB_CUR_CLASS RotMat
	PYB_CLASS(RotMat)
		PYB_DEFAULT_INITS()
		.def(pyb::init<Vec, Vec, Vec>(), PYBA("forward"), PYBA("right"), PYBA("up"))
		.def("__getitem__", [](const RotMat& mat, std::pair <size_t, size_t> i) { return mat[i.first][i.second]; })
		.def("__setitem__", [](const RotMat& mat, std::pair <size_t, size_t> i, float val) { mat[i.first][i.second] = val; })

		PYBP(forward)
		PYBP(right)
		PYBP(up)

		.def("__str__", [](const RotMat& mat) { return RS_STR(mat);  })
		;

#define PYB_CUR_CLASS CarControls
	PYB_CLASS(CarControls)
		PYB_DEFAULT_INITS()
		
		PYBP(throttle)
		PYBP(steer)

		PYBP(pitch)
		PYBP(yaw)
		PYBP(roll)

		PYBP(boost)
		PYBP(jump)
		PYBP(handbrake)
		;
}