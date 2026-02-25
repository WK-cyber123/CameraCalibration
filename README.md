# CameraCalibration

## 项目结构
为便于后续扩展，代码已经按职责拆分为 `include/` 与 `src/`：

- `include/CalibrationCamera.h`：相机标定核心接口。
- `include/CannyDevernay.h`：亚像素边缘检测接口。
- `src/CalibrationCamera.cpp`：标定流程实现。
- `src/CannyDevernay.cpp`：Devernay 算法实现。
- `src/Main.cpp`：程序入口。

## 命名与风格约定
- **文件名**：CamelCase。
- **类名**：PascalCase。
- **函数名**：camelCase。
- **语言标准**：C++11 及以上。

## 构建
```bash
cmake -S . -B build
cmake --build build
```
