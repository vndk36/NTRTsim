link_libraries(GoalSpine
                obstacles
                learningSpines
                sensors
                controllers
                tgcreator             
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport
                boost_program_options)
                
                
add_executable(AppGoalOnline
    SpineOnlineControl.cpp
    AppGoalOnline.cpp
)
