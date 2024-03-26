#include "PYB.h"

PYBIND11_MODULE(rlcis_py, m) {
	m.def("solve", RLCIS::Solve, 
		PYBA("from_state"), PYBA("to_state"),
		PYBA("delta_time"), PYBA("config")
	);

	try {
		PYB_INIT_RSTypes(m);
		PYB_INIT_SolverTypes(m);
	} catch (std::exception e) {
		RS_ERR_CLOSE("Failed to initialize pybind11 module, exception: " << e.what());
	}
}