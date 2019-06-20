cmake_minimum_required(VERSION 3.5)
project(sample)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -stdlib=libc++")

add_executable(example example.cpp)
target_include_directories(example PUBLIC ${.})
target_include_directories(example PUBLIC ${/Library/Developer/CommandLineTools/usr/include/c++/v1/})

