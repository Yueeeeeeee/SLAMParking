# use modern target-based cmake features
cmake_minimum_required (VERSION 3.8)

# projectname
project (af3pihal)

# executable to create
add_library (${PROJECT_NAME} SHARED
    ./src/test_control_center.c
    ./src/camera_client.c
    ./src/temp_actuator.c
    ./src/gamepad.c
    ./src/control_center_downstream.c
    ./src/cansocket.c
    ./src/protocol_factory.c
    ./src/protocol_worker.c
    ./src/timeutil.c
    ./src/v2v_udp_basic.c
    ./src/af3_component_remote_proxy.c
    ./src/af3_v2v.c
    ./src/v2v_message_handlers.c
    ./src/protocol_coordinator.c
    ./src/af3_component.c
    ./src/protocol_control_center.c
    ./src/canthread.c
    ./src/listutil.c
    ./src/af3.c
    ./src/rumblepad.c

)

# directories where to search for header files
target_include_directories(${PROJECT_NAME}
    PUBLIC
        inc # the headerfiles in source are the includes
)


target_compile_features(${PROJECT_NAME} PRIVATE c_std_11) # cmake >= 3.8