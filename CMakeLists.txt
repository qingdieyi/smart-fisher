# ═══════════════════════════════════════════════════════════════════
# 顶层 CMake 构建文件
#
# 这是 CMake 构建系统的入口文件。
# CMake 会自动从这个目录开始，递归处理所有子目录的 CMakeLists.txt。
#
# ── ESP-IDF 项目必须的四行 ───────────────────────────────────────
#
#   1. cmake_minimum_required(VERSION 3.22)
#      声明项目要求的最低 CMake 版本
#
#   2. include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#      引入 ESP-IDF 的 CMake 基础设施（定义了 idf.py build 等命令）
#
#   3. idf_build_set_property(MINIMAL_BUILD ON)
#      最小化构建：只编译用到的 IDF 组件（类似 Maven 的 scope=provided，
#      不把整个操作系统都编译进去）。加快编译速度，减小固件体积。
#
#   4. project(smart-fisher)
#      定义项目名称（类似 Maven 的 <artifactId>）
#
# ── Java 对比 ─────────────────────────────────────────────────────
#
#   这四行相当于：
#     <?xml version="1.0" encoding="UTF-8"?>
#     <project>
#         <modelVersion>4.0.0</modelVersion>     ← cmake_minimum_required
#         <parent>                                ← include(project.cmake)
#             <groupId>espressif</groupId>
#             <artifactId>esp-idf</artifactId>
#         </parent>
#         <artifactId>smart-fisher</artifactId>   ← project(...)
#     </project>
#
# ═══════════════════════════════════════════════════════════════════

# 要求 CMake 3.22 或更高版本
cmake_minimum_required(VERSION 3.22)

# 引入 ESP-IDF 的构建系统
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 最小化构建：只编译 main 组件及其传递依赖
# 关闭后会把 IDF 的所有组件都编译进去，固件会变大很多
idf_build_set_property(MINIMAL_BUILD ON)

# 项目名称
project(smart-fisher)
