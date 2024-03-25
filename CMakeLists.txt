cmake_minimum_required (VERSION 3.8)

project("RLCarInputSolver")

# Add all headers and code files
file(GLOB_RECURSE FILES_SRC "src/*.cpp" "src/*.h")
add_library(RLCarInputSolver STATIC ${FILES_SRC})
target_include_directories(RLCarInputSolver PUBLIC "src/")

# Set C++ version to 20
target_compile_features(RLCarInputSolver PUBLIC cxx_std_20)

# Include RocketSim
add_subdirectory("${PROJECT_SOURCE_DIR}/RocketSim")
target_link_libraries(RLCarInputSolver PUBLIC RocketSim)

#############################################

option(TESTING_MODE "Enable test executable" OFF)

if (TESTING_MODE)
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