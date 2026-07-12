# ═══════════════════════════════════════════════════════════════════
# main 组件的 CMake 构建文件
#
# ── 给 Java 开发者 ───────────────────────────────────────────────
#
# CMake 之于 C/C++，相当于 Maven/Gradle 之于 Java。
#
#   概念               Java (Maven)            C (CMake + ESP-IDF)
#   ─────────────────────────────────────────────────────────────────
#   构建文件           pom.xml                 CMakeLists.txt
#   项目/模块          <artifactId>            project() / component
#   依赖声明           <dependency>             PRIV_REQUIRES / REQUIRES
#   源文件             src/main/java/*.java    SRCS "a.c" "b.c"
#   构建命令           mvn compile             idf.py build
#   包发布             mvn deploy              （无标准中央仓库，有组件注册表）
#
# ── ESP-IDF 项目的 CMake 结构 ────────────────────────────────────
#
#   每个 ESP-IDF 组件（项目目录下每个文件夹）都可以有自己的 CMakeLists.txt。
#   main/ 目录是 ESP-IDF 强制要求的"主组件"，包含 app_main() 入口。
#   其他目录可以作为独立组件（类似 Java 的多模块项目）。
#
# ═══════════════════════════════════════════════════════════════════

# idf_component_register() — 注册当前目录为一个 ESP-IDF 组件
idf_component_register(
    # SRCS: 源文件列表（类似 Maven 的 <sourceDirectory>）
    #       列出所有要编译的 .c 文件
    SRCS "main.c" "ds18b20.c" "camera_handler.c"

    # INCLUDE_DIRS: 头文件搜索路径
    #               "." 表示当前目录，这样 #include "ds18b20.h" 就能找到
    #               类似 Java 的 classpath
    INCLUDE_DIRS "."

    # PRIV_REQUIRES: 私有依赖（只在当前组件内部使用）
    #   类似 Maven 的 <scope>runtime</scope>
    #
    #   我们依赖的 ESP-IDF 组件：
    #     - esp_wifi:       WiFi 驱动和相关 API
    #     - nvs_flash:      非易失性存储（存 WiFi 凭证）
    #     - esp_driver_gpio: GPIO 驱动（ESP-IDF v6.0 拆分出的独立组件）
    #     - esp32-camera:   OV5640 摄像头驱动（来自 Component Registry）
    #     - esp_timer:      高精度定时器（获取拍照时间戳）
    #
    #   后续阶段会添加：esp_mqtt 等
    PRIV_REQUIRES esp_wifi nvs_flash esp_driver_gpio esp32-camera esp_timer
)
