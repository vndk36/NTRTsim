Project(linkedAppTest)

link_directories(${LIB_DIR})

link_libraries(	core
				tgcreator
				util
				sensors
				terrain
				tgOpenGLSupport
				yaml-cpp
				TensegrityModel
				)

add_executable(	AppLinkedObjectTest
				AppLinkedObjectTest.cpp
				simpleBCUController.cpp
				buoyancySimulator.cpp
				StepwiseController.cpp
				)
