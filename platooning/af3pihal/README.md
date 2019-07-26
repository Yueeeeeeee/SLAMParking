make cross:
mkdir build/ && cd build/
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../../../ff1_dev_tools/toolchain_raspberry.cmake ../
