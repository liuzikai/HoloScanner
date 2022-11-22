cmake_minimum_required(VERSION 3.16)
project(HoloScanner)

set(CMAKE_CXX_STANDARD 17)

include_directories(HL2UnityPlugin)
include_directories(CPP)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)
string(REPLACE ";" ", " Boost_LIBRARIES_STRING "${Boost_LIBRARIES}")
message("Boost ${Boost_VERSION} found: ${Boost_LIBRARIES_STRING}")

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

add_executable(PCD
        CPP/TerminalSocket.cpp
        CPP/TCPDataSource.cpp
        CPP/Pipelines/PCD.cpp
        )
target_link_libraries(PCD
        ${Boost_SYSTEM_LIBRARY}
        DirectXMath
        igl::core igl::glfw igl::imgui)

add_executable(DepthToPCD
        CPP/TerminalSocket.cpp
        CPP/TCPDataSource.cpp
        CPP/FileDataSource.cpp
        CPP/DepthProcessor.cpp
        CPP/Pipelines/DepthToPCD.cpp
        CPP/Registrator.cpp
        )
target_link_libraries(DepthToPCD
        ${Boost_SYSTEM_LIBRARY}
        DirectXMath
        igl::core igl::glfw igl::imgui
        Open3D::Open3D
        )