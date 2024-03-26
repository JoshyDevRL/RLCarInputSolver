cmake_minimum_required (VERSION 3.8)

project("RLCarInputSolver")

option(MAKE_PYBIND "Generate Python bindings using pybind11" ON)
option(TESTING_MODE "Enable test executable" OFF)

# Add all headers and code files
file(GLOB_RECURSE FILES_SRC "src/*.cpp" "src/*.h")

add_library(RLCarInputSolver STATIC ${FILES_SRC})

target_include_directories(RLCarInputSolver PUBLIC "src/")

# Set C++ version to 20
target_compile_features(RLCarInputSolver PUBLIC cxx_std_20)

# Include RocketSim
add_subdirectory("RocketSim")
target_link_libraries(RLCarInputSolver PUBLIC RocketSim)

#############################################

if (MAKE_PYBIND)
	message("Python bindings enabled")

	# Add pybind11
	add_subdirectory("python/pybind11")

	# Get all binding source files
	file(GLOB_RECURSE FILES_PY_SRC "python/src/*.cpp" "python/src/*.h")

	# Create pybind11 module
	pybind11_add_module(rlcis_py ${FILES_SRC} ${FILES_PY_SRC})
	target_link_libraries(rlcis_py PUBLIC RocketSim)

	# Set C++ version to 20
	target_compile_features(rlcis_py PUBLIC cxx_std_20)
	
	# Set output dir
	set_target_properties(rlcis_py PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/python/out")
	set_target_properties(rlcis_py PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASES "${PROJECT_SOURCE_DIR}/python/out")
endif()

#############################################

if (TESTING_MODE)
	message("Testing mode enabled")

	# Testing binary
	add_executable(AccuracyTest "test/AccuracyTest.cpp")
	target_link_libraries(AccuracyTest PRIVATE RLCarInputSolver)

	# Set C++ version to 20
	target_compile_features(AccuracyTest PUBLIC cxx_std_20)

	enable_testing()

	add_test(
		NAME AccuracyTest
		COMMAND $<TARGET_FILE:AccuracyTest>
	)
endif()