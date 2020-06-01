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

add_executable(	AppSimpleTest
				SimpleController.cpp
				rodBuoyancyCtrl.cpp
				AppSimpleTest.cpp
				)