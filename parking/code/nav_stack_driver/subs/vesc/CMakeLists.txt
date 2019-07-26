# use modern target-based cmake features
cmake_minimum_required (VERSION 3.7.2)

# projectname
project(vesc)

# executable to create
add_library(${PROJECT_NAME} SHARED
    ./src/buffer.c
    ./src/commands.c
    ./src/crc.c
    ./src/datagram.c
    ./src/hal.c
)

# directories where to search for header files
# PUBLIC means that parent projects should add the directory too
target_include_directories(${PROJECT_NAME}
    PUBLIC
        inc
    PRIVATE
        src
)

target_compile_features(${PROJECT_NAME}
    PRIVATE
        c_std_11 # cmake >= 3.8
)
