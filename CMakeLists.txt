# =============================================================================
# CMakeLists.txt - UL MAC Manager (eNB侧上行MAC用户管理)
#
# 构建目标:
#   ul_mac_manager        - 演示主程序 (src/main.cpp)
#   ul_mac_manager_tests  - 单元测试 (tests/test_main.cpp)
#
# 用法 (Windows + MinGW g++):
#   cd <project_root>
#   cmake -S . -B build -G "MinGW Makefiles"
#   cmake --build build
#   cd build && ctest --output-on-failure
#
# 或直接使用 g++ 编译 (见文件末尾注释)。
# =============================================================================

cmake_minimum_required(VERSION 3.10)
project(ul_mac_manager CXX)

# ---- 标准与编译选项 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 严格编译告警 (与代码审查要求一致: -Wall -Wextra)
add_compile_options(-Wall -Wextra -Wpedantic)

# Windows 下 MinGW 需要线程库; 标准库线程需要 -pthread (MinGW 默认链接 winpthreads)
if(MINGW)
    set(CMAKE_THREAD_PREFER_PTHREAD ON)
endif()

find_package(Threads REQUIRED)

# 头文件包含根目录 (include/ 下以 ul_mac/ 为前缀组织)
include_directories(${CMAKE_SOURCE_DIR}/include)

# ---- 源文件清单 (lcg_buffer / ue_context 为 header-only, 无需单独 .cpp) ----
set(UL_MAC_SOURCES
    src/ue_sr_manager.cpp
    src/ue_bsr_manager.cpp
    src/ue_ul_harq_manager.cpp
    src/enb_ul_scheduler.cpp
    src/enb_bsr_manager.cpp
    src/enb_ul_harq_manager.cpp
    src/mac_pdu.cpp
)

# ---- 演示主程序 ----
add_executable(ul_mac_manager
    src/main.cpp
    ${UL_MAC_SOURCES}
)
target_link_libraries(ul_mac_manager PRIVATE Threads::Threads)

# ---- 单元测试 ----
enable_testing()
add_executable(ul_mac_manager_tests
    tests/test_main.cpp
    ${UL_MAC_SOURCES}
)
target_link_libraries(ul_mac_manager_tests PRIVATE Threads::Threads)
add_test(NAME unit_tests COMMAND ul_mac_manager_tests)

# ---- MinGW 静态链接 ----
# 避免 exe 运行时从 PATH 加载到其他 MinGW 发行版(如 Git for Windows)的旧版
# libstdc++-6.dll, C++ ABI 不匹配会在静态初始化阶段段错误 (详见 PROTOCOL_NOTES.md §10.3 P2-4)
if(MINGW)
    set_target_properties(ul_mac_manager ul_mac_manager_tests PROPERTIES LINK_FLAGS "-static")
endif()

# =============================================================================
# 备选: 纯 g++ 命令行编译 (无需 CMake)
#   g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 -static -Iinclude src/main.cpp src/*.cpp -o ul_mac_manager -pthread
#   g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 -static -Iinclude tests/test_main.cpp src/*.cpp -o ul_mac_manager_tests -pthread
# =============================================================================
