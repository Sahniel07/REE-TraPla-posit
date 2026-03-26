cmake_minimum_required(VERSION 3.19.0)
project(dynamic_game_trajectory_planner VERSION 0.1.0 LANGUAGES C CXX ISPC)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

enable_language(ISPC)

# Choose precision at configure time: 16, 32, or 64
set(REAL_BITS "64" CACHE STRING "Precision in bits for real_t (16|32|64)")
set_property(CACHE REAL_BITS PROPERTY STRINGS 16 32 64)


option(ENABLE_AARCH64 "Enable aarch64-specific includes" OFF)
option(ENABLE_QUANTIZATION "Enable quantization feature" TRUE) 
option(ENABLE_POSIT "Enable Posit arithmetic" OFF) #Implementation of Posit 

if(ENABLE_AARCH64)
    add_compile_definitions(ARCH_AARCH64=1)
    if(REAL_BITS STREQUAL "32")
        set(ISPC_TARGET "neon-i32x4")
    elseif(REAL_BITS STREQUAL "16")
        set(ISPC_TARGET "neon-i16x8")
    else()
	set(ISPC_TARGET "neon-i32x4")
    endif()
else()
    add_compile_definitions(ARCH_AARCH64=0)
    if(REAL_BITS STREQUAL "64")
        set(ISPC_TARGET "avx2-i64x4")
    elseif(REAL_BITS STREQUAL "32")
        set(ISPC_TARGET "avx2-i32x8")
    elseif(REAL_BITS STREQUAL "16")
        message(FATAL_ERROR "16-bit SIMD not supported on x86_64")
    endif()
endif()

if(ENABLE_QUANTIZATION)
    message(STATUS "Quantization is ENABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=1) # Define macro for quantization
else()
    message(STATUS "Quantization is DISABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=0) # Define macro to disable quantization
endif()

# Posit macro definition
if(ENABLE_POSIT)
    message(STATUS "Posit Arithmetic is ENABLED")
    add_compile_definitions(ENABLE_POSIT=1)
else()
    message(STATUS "Posit Arithmetic is DISABLED")
    add_compile_definitions(ENABLE_POSIT=0)
endif()

message(STATUS "REAL_BITS = ${REAL_BITS}")
message(STATUS "Target = ${ISPC_TARGET}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(ispc_filescmake_minimum_required(VERSION 3.19.0)
project(dynamic_game_trajectory_planner VERSION 0.1.0 LANGUAGES C CXX ISPC)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

enable_language(ISPC)

# Choose precision at configure time: 16, 32, or 64
set(REAL_BITS "64" CACHE STRING "Precision in bits for real_t (16|32|64)")
set_property(CACHE REAL_BITS PROPERTY STRINGS 16 32 64)


option(ENABLE_AARCH64 "Enable aarch64-specific includes" OFF)
option(ENABLE_QUANTIZATION "Enable quantization feature" TRUE) 

if(ENABLE_AARCH64)
    add_compile_definitions(ARCH_AARCH64=1)
    if(REAL_BITS STREQUAL "32")
        set(ISPC_TARGET "neon-i32x4")
    elseif(REAL_BITS STREQUAL "16")
        set(ISPC_TARGET "neon-i16x8")
    else()
	set(ISPC_TARGET "neon-i32x4")
    endif()
else()
    add_compile_definitions(ARCH_AARCH64=0)
    if(REAL_BITS STREQUAL "64")
        set(ISPC_TARGET "avx2-i64x4")
    elseif(REAL_BITS STREQUAL "32")
        set(ISPC_TARGET "avx2-i32x8")
    elseif(REAL_BITS STREQUAL "16")
        message(FATAL_ERROR "16-bit SIMD not supported on x86_64")
    endif()
endif()

if(ENABLE_QUANTIZATION)
    message(STATUS "Quantization is ENABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=1) # Define macro for quantization
else()
    message(STATUS "Quantization is DISABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=0) # Define macro to disable quantization
endif()

message(STATUS "REAL_BITS = ${REAL_BITS}")
message(STATUS "Target = ${ISPC_TARGET}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(ispc_files
#     src/integrate.ispc
# )
if(ENABLE_QUANTIZATION)
    set(ISPC_PATH "${CMAKE_SOURCE_DIR}/src/integrate.ispc")
    set(ISPC_OUT_O "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.o")
    set(ISPC_OUT_H "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.h")
    set(ISPC_OUT_S "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.s")
    message("Using integrate.ispc with quantization")
else()
    set(ISPC_PATH "${CMAKE_SOURCE_DIR}/src/integrate_withoutQuantization.ispc")
    set(ISPC_OUT_O "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.o")
    set(ISPC_OUT_H "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.h")
    set(ISPC_OUT_S "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.s")
    message("Using integrate_withoutQuantization.ispc without quantization")
endif()

add_compile_definitions(REAL_BITS=${REAL_BITS})

add_custom_command(
    OUTPUT ${ISPC_OUT_O} ${ISPC_OUT_H}
    COMMAND ispc ${ISPC_PATH} -o ${ISPC_OUT_O} -I${CMAKE_CURRENT_SOURCE_DIR}/include 
    -O3 
    --target=${ISPC_TARGET} 
    --header-outfile=${ISPC_OUT_H} 
    --math-lib=default
    -DREAL_BITS=${REAL_BITS}
    DEPENDS ${ISPC_PATH}
    COMMENT "Compiling ISPC: ${ISPC_PATH}"
)
add_custom_command(
    OUTPUT  ${ISPC_OUT_S} ${ISPC_OUT_H}
    COMMAND ispc ${ISPC_PATH}
            -o ${ISPC_OUT_S}
            --emit-asm -g
            -O3
            -I${CMAKE_CURRENT_SOURCE_DIR}/include
            --math-lib=default
            --target=${ISPC_TARGET}
            --header-outfile=${ISPC_OUT_H}
            -DREAL_BITS=${REAL_BITS}
    DEPENDS ${ISPC_PATH}
    COMMENT "Compiling ISPC (with assembly): ${ISPC_PATH}"
)
add_custom_target(generate_ispc_asm
    DEPENDS ${ISPC_OUT_S}
    COMMENT "Generating annotated assembly for ISPC file"
)

include_directories(include Recorder4Cpp ISPC_OUT_H 
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/universal/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/universal/include/sw
)

option(USE_RECORDER "Use recorder" OFF)
set(recorder_file ${recorder_file})

if (USE_RECORDER)
  MESSAGE(STATUS "USE_RECORDER=ON")
  add_definitions(-DUSE_RECORDER)
  set(recorder_file ${recorder_file} Recorder4Cpp/recorder.h
  	Recorder4Cpp/recorder.cpp)
endif()
add_library(integrate_ispc STATIC ${ISPC_OUT_O})
set_target_properties(integrate_ispc PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(integrate_ispc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set(source_files
    src/main.cpp
    src/dynamic_game_planner.cpp
    src/utils.cpp
    src/vehicle_state.cpp
)

add_executable(dynamic_game_trajectory_planner ${source_files} ${recorder_file})

set_target_properties(dynamic_game_trajectory_planner PROPERTIES
    ISPC_INSTRUCTION_SETS ${ISPC_TARGET}
)

target_link_libraries(dynamic_game_trajectory_planner PRIVATE integrate_ispc)
target_include_directories(dynamic_game_trajectory_planner PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_compile_options(dynamic_game_trajectory_planner PRIVATE 
    -O3 
    -fno-fast-math -fno-associative-math -fno-reciprocal-math -fno-finite-math-only
    -ffp-contract=off)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
    COMMAND ${CMAKE_CXX_COMPILER}
            -S -g -fverbose-asm -O3
            -I${CMAKE_CURRENT_SOURCE_DIR}/include
            -I${CMAKE_CURRENT_SOURCE_DIR}/Recorder4Cpp
            -I${CMAKE_CURRENT_SOURCE_DIR}/build
            -o ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
            ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamic_game_planner.cpp
    DEPENDS src/dynamic_game_planner.cpp
    COMMENT "Generating annotated assembly for dynamic_game_planner.cpp"
)

add_custom_target(generate_cpp_asm
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
    COMMENT "Target to generate assembly code for dynamic_game_planner.cpp"
)

