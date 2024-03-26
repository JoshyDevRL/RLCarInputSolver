#pragma once
#include "../../src/Solver.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
namespace pyb = pybind11;

// Set this to the current class we are working with
#define PYB_CUR_CLASS null

#define PYB_CLASS(className) pyb::class_<className>(m, #className)
#define PYB_CLASS_SH(className) pyb::class_<className, std::shared_ptr<className>>(m, #className)
#define PYB_INIT_F(name) void PYB_INIT_##name(pyb::module& m)
#define PYB_DEFAULT_INITS() \
	.def(pyb::init<>()) \
	.def(pyb::init<const PYB_CUR_CLASS&>())
#define PYBA pyb::arg

// Pybind string
#define PYBS(s) PYB_MakePythonString(s)

// Read-write property
#define PYBP(memberName) .def_readwrite(PYBS(#memberName), &PYB_CUR_CLASS::memberName)

// Read-write property with different name in bindings
#define PYBP_RN(memberName, pythonName) .def_readwrite(PYBS(#pythonName), &PYB_CUR_CLASS::memberName)

// NOTE: Literally leaks memory, but should only be called once for each string, so its fine
inline const char* PYB_MakePythonString(const char* name) {
	std::string* result = new std::string();
	char last = NULL;
	bool isInAcronym = false;
	for (const char* pc = name; *pc; pc++) {
		char c = *pc;
		if (isupper(c)) {
			if (last && isupper(last)) {
				isInAcronym = true;
			} else if (last) {
				*result += "_";
			}
		} else {
			if (isInAcronym) {
				*result += "_";
				isInAcronym = false;
			}
		}
		*result += tolower(c);

		last = c;
	}
	return result->c_str();
}

PYB_INIT_F(RSTypes);
PYB_INIT_F(SolverTypes);