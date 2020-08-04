Project(rotTensegrity)

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

add_executable(	AppRotTensegrity
				AppRotTensegrity.cpp
				rotBCUController.cpp
				buoyancySimulator.cpp
				)
