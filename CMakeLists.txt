cmake_minimum_required(VERSION 3.16)
project(HoloScanner)

set(CMAKE_CXX_STANDARD 17)

include_directories(HL2UnityPlugin)
include_directories(CPP)

# Find Boost (optional)
find_package(Boost COMPONENTS system)
if (Boost_FOUND)
    message("Boost ${Boost_VERSION} found: ${Boost_LIBRARIES_STRING}")
    string(REPLACE ";" ", " Boost_LIBRARIES_STRING "${Boost_LIBRARIES}")
else ()
    message(WARNING "Boost not found so some targets may not be available")
endif ()

# DirectXMath
add_subdirectory(CPP/DirectXMath)

# Open3D
find_package(Open3D REQUIRED)

# libigl
option(LIBIGL_OPENGL "Use OpenGL" ON)
option(LIBIGL_GLFW "Use GLFW" ON)
option(LIBIGL_IMGUI "Use ImGui" ON)
add_subdirectory(CPP/libigl)
message("LIBIGL_INCLUDE_DIR: ${LIBIGL_INCLUDE_DIR}")
set(LIGIGL_LIBERIES igl::core igl::glfw igl::imgui)

# libTCPServer
if (Boost_FOUND)
    add_library(libTCPServer CPP/TerminalSocket.cpp CPP/TCPDataSource.cpp)
    target_link_libraries(libTCPServer PUBLIC
            ${Boost_SYSTEM_LIBRARY}
            DirectXMath
            ${LIGIGL_LIBERIES}
            )
else()
    message("=> Target libTCPServer is not available to build. Depends: Boost")
endif ()

# Pipeline: DepthToPCD
add_executable(DepthToPCD
#        CPP/FileDataSource.cpp
        CPP/DepthProcessor.cpp
        CPP/Pipelines/DepthToPCD.cpp
        CPP/Registrator.cpp
        )
target_link_libraries(DepthToPCD
        DirectXMath
        ${LIGIGL_LIBERIES}
        Open3D::Open3D
        )
if (TARGET libTCPServer)
    target_link_libraries(DepthToPCD libTCPServer)
    target_compile_definitions(DepthToPCD PUBLIC BOOST_AVAILABLE)
endif ()

# Pipeline: PCD
add_executable(PCD
        CPP/TerminalSocket.cpp
        CPP/TCPDataSource.cpp
        CPP/Pipelines/PCD.cpp
        )
target_link_libraries(PCD
        DirectXMath
        ${LIGIGL_LIBERIES})
if (TARGET libTCPServer)
    target_link_libraries(PCD libTCPServer)
    target_compile_definitions(PCD PUBLIC BOOST_AVAILABLE)
endif ()
