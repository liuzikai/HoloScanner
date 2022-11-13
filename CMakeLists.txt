cmake_minimum_required(VERSION 3.18)
project(HoloScanner)

set(CMAKE_CXX_STANDARD 17)

include_directories(HL2UnityPlugin)
include_directories(TCPServer)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system filesystem)
string(REPLACE ";" ", " Boost_LIBRARIES_STRING "${Boost_LIBRARIES}")
message("Boost ${Boost_VERSION} found: ${Boost_LIBRARIES_STRING}")

# DirectXMath
add_subdirectory(CPP/DirectXMath)

# libigl
option(LIBIGL_OPENGL    "Use OpenGL"   ON)
option(LIBIGL_GLFW      "Use GLFW"     ON)
option(LIBIGL_IMGUI     "Use ImGui"    ON)
add_subdirectory(CPP/libigl)
message("LIBIGL_INCLUDE_DIR: ${LIBIGL_INCLUDE_DIR}")

add_executable(HoloScannerPCServer
        CPP/Test.cpp
        CPP/TerminalSocket.cpp
        CPP/TCPStreamingSource.cpp)
target_link_libraries(HoloScannerPCServer
        ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}
        DirectXMath
        igl::core igl::glfw igl::imgui)