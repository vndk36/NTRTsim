Project(simpleRodAppTest)

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

add_executable(	AppRodTest
				AppRodTest.cpp
				simpleBCUController.cpp
				rotBCUController.cpp
				buoyancySimulator.cpp
				StepwiseController.cpp
				)
