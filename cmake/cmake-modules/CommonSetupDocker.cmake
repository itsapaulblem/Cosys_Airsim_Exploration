# Common setup instructions for Docker/ROS2 environment
# This version doesn't require AirSim.sln file

macro(CommonTargetLink)
    target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
endmacro(CommonTargetLink)

macro(IncludeEigen)
    include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)
endmacro(IncludeEigen)

macro(AddExecutableSource)
    set(PROJECT_CPP ${PROJECT_NAME}_sources)
    file(GLOB_RECURSE PROJECT_CPP "${AIRSIM_ROOT}/${PROJECT_NAME}/*.cpp")
    add_executable(${PROJECT_NAME} ${PROJECT_CPP})
endmacro(AddExecutableSource)

macro(SetupConsoleBuild)
    IF(UNIX)
    ELSE()
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
    ENDIF()
endmacro(SetupConsoleBuild)

macro(CommonSetup)
    find_package(Threads REQUIRED)
    
    # For Docker environment, check if AIRSIM_ROOT is already set
    if(NOT DEFINED AIRSIM_ROOT)
        # Try to find AirLib directory as a marker instead of AirSim.sln
        find_path(AIRSIM_ROOT NAMES AirLib PATHS 
            "/airsim_ros2_ws"
            "${CMAKE_CURRENT_SOURCE_DIR}/.." 
            "${CMAKE_CURRENT_SOURCE_DIR}/../.." 
            "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
            "${CMAKE_CURRENT_SOURCE_DIR}/../../../.."
            "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.."
            "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../.."
        )
        
        if(NOT AIRSIM_ROOT)
            message(FATAL_ERROR "Could not find AIRSIM_ROOT. Please set it manually or ensure AirLib directory exists.")
        endif()
    endif()
    
    message(STATUS "AIRSIM_ROOT set to: ${AIRSIM_ROOT}")

    #setup output paths
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
    SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)
    SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    #setup include and lib for rpclib which will be referenced by other projects
    set(RPCLIB_VERSION_FOLDER rpclib-2.3.1)
    set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/${RPCLIB_VERSION_FOLDER}/include")
    #name of .a file with lib prefix
    set(RPC_LIB rpc)

    #what is our build type debug or release?
    string( TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

    IF(UNIX)
        set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")
        set(BUILD_TYPE "linux")
        set(CMAKE_CXX_STANDARD 17)

        if (APPLE)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wstrict-aliasing -D__CLANG__")
        else ()
            if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wstrict-aliasing -Wno-unused-parameter -Wno-unused-variable -Wno-gnu-anonymous-struct -Wno-nested-anon-types -Wno-gnu-zero-variadic-macro-arguments -D__CLANG__")
            else()
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wstrict-aliasing -Wno-unused-parameter -Wno-unused-variable -D__GCC__")
            endif()
        endif ()

        # set these flags only on Ubuntu
        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-variadic-macros -Wno-gcc-compat -Wno-unused-function -Wno-unused -pthread")

            # make sure to match the compiler flags with which the Unreal
            # Engine is built with, in case of runtime issues, check these
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-pie")
        endif()

        set(BUILD_PLATFORM "x64")

        #setup lib names
        if (CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -DDEBUG=1 -DASSERTS")
            set(RPC_LIB_NAME librpc.a)
            set(BUILD_CONFIG "Debug")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
            set(RPC_LIB_NAME librpc.a)
            set(BUILD_CONFIG "Release")
        endif()

        #setup compiler flags
        set(CXX_EXP_LIB "-stdlib=libc++ -lc++ -lc++abi")

    ELSE()
        #windows cmake build is experimental
        set(CMAKE_CXX_STANDARD 17)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RPC_LIB_DEFINES} /Zc:__cplusplus")

        #setup lib names
        # Don't use any additional includes to the specific compiler include directories, this is Linux specific for now
        set(BUILD_PLATFORM "Win64")
        if (CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(RPC_LIB_NAME rpcd.lib)
            set(BUILD_CONFIG Debug)
        else()
            set(RPC_LIB_NAME rpc.lib)
            set(BUILD_CONFIG Release)
        endif()
    ENDIF()

    ## Hint for find_package
    set(RPC_LIB ${AIRSIM_ROOT}/build_debug/output/lib/${BUILD_PLATFORM}/${BUILD_CONFIG}/${RPC_LIB_NAME})
    set(RPC_INCLUDE_DIRS
        ${RPC_LIB_INCLUDES})
endmacro(CommonSetup)