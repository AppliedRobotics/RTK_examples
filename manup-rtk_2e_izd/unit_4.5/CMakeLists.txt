cmake_minimum_required(VERSION 3.14) 
project(example_1)

file(GLOB SRC_DYNAMIXEL
"../src/dynamixel_sdk/*.cpp"
)

add_executable(example_1) 
target_sources(example_1 PRIVATE
example_1.cpp
${SRC_DYNAMIXEL}
)

target_compile_definitions(example_1 PRIVATE -DFMT_HEADER_ONLY=1)
target_compile_options(example_1 PRIVATE -Wno-psabi) 
target_include_directories(example_1 PRIVATE
../src
../src/dynamixel_sdk
)
