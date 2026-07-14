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

# ── 模型文件嵌入（仅在启用 YOLO 时） ───────────────────────────────
#
# 将 .espdl 模型文件作为二进制数据嵌入固件。
# 编译后可通过 extern 符号访问模型数据：
#
#   extern const uint8_t _binary_models_espdet_pico_fish_espdl_start[];
#   extern const uint8_t _binary_models_espdet_pico_fish_espdl_end[];
#
# 如果模型文件不存在，编译会报 "file not found"。
# 此时注释掉下面这行，或先完成模型训练（见 docs/yolo-practical-guide.md）。
if(CONFIG_SMART_FISHER_DETECTION_YOLO)
    set(YOLO_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models/espdet_pico_fish.espdl")
    if(EXISTS "${YOLO_MODEL_PATH}")
        target_add_binary_data(${COMPONENT_LIB} "${YOLO_MODEL_PATH}" BINARY)
        message(STATUS "YOLO model embedded: ${YOLO_MODEL_PATH}")
    else()
        message(WARNING "YOLO model file not found at: ${YOLO_MODEL_PATH}")
        message(WARNING "Place the trained .espdl model there, or switch to Frame Differencing.")
        message(WARNING "See: docs/yolo-practical-guide.md")
    endif()
endif()

# ── 条件依赖：YOLO 模式需要 ESP-DL ────────────────────────────────
set(EXTRA_PRIV_REQUIRES "")
if(CONFIG_SMART_FISHER_DETECTION_YOLO)
    list(APPEND EXTRA_PRIV_REQUIRES esp_dl)
    message(STATUS "Fish Detection: YOLO mode (ESP-DL required)")
else()
    message(STATUS "Fish Detection: Frame Differencing mode")
endif()

# idf_component_register() — 注册当前目录为一个 ESP-IDF 组件
idf_component_register(
    SRCS "main.c" "ds18b20.c" "camera_handler.c" "mqtt_handler.c" "fish_detector.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi nvs_flash esp_driver_gpio esp32-camera esp_timer esp_mqtt esp_jpeg ${EXTRA_PRIV_REQUIRES}
)
