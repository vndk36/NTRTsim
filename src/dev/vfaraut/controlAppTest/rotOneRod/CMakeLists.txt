Project(rotRodSphere)

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

add_executable(	AppRotRod
				AppRotRod.cpp
				rotRodBCUController.cpp
				buoyancySimulator.cpp
				)
