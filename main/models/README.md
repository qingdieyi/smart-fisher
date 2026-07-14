# 模型文件目录

这个目录存放训练好的 ESP-DL 模型文件 (`.espdl` 格式)。

## 使用方法

1. 按照 `docs/yolo-practical-guide.md` 完成模型训练
2. 将生成的 `.espdl` 文件复制到此目录：

```
main/models/
└── espdet_pico_fish.espdl    ← 训练好的鱼群检测模型 (~500KB)
```

3. 启用 YOLO 模式：

```bash
idf.py menuconfig
# → Smart Fisher Configuration
#   → Fish Detection Method
#     → YOLO Object Detection (ESP-DL neural network)
```

4. 重新编译：

```bash
idf.py fullclean
idf.py build
idf.py -p COM8 flash monitor
```

## 当前状态

模型文件尚未生成。当前使用 **帧差法**（Frame Differencing）作为默认检测引擎。

切换到 YOLO 需要先完成 PC 端的模型训练流程。
