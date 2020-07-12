Project(SimpleAppTest)

link_directories(${LIB_DIR})

link_libraries(	core
				tgcreator
				util
				sensors
				terrain
				tgOpenGLSupport
				yaml-cpp
				TensegrityModel)

add_executable(	AppLinkedObjectTest
				buoyancySimulator.cpp
				AppLinkedObjectTest.cpp
				simpleBCUController.cpp
				StepwiseController.cpp
				)