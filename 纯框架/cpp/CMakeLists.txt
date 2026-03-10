add_library(r2_decision_core
    src/state_machine.cpp
    src/strategy.cpp
    src/hardware_manager.cpp
)

target_include_directories(r2_decision_core PUBLIC include)

add_executable(r2_decision_demo src/main.cpp)
target_link_libraries(r2_decision_demo PRIVATE r2_decision_core)

add_executable(r2_hardware_example src/hardware_example.cpp)
target_link_libraries(r2_hardware_example PRIVATE r2_decision_core)
