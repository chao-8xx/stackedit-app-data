[TOC]

# YOLOv5转NCNN在龙芯久久派下部署推理

> 参考视频：[【走马观碑-wuwu库】yolov5转ncnn在龙芯久久派下部署推理](https://b23.tv/seXKq4S)

## 一、项目概述

### 1.1 技术栈介绍

本项目实现了在龙芯（Loongson）架构的久久派（JiuJiuPai）开发板上部署YOLOv5目标检测模型，通过NCNN框架进行高效推理。

**核心技术**：
- **YOLOv5**：轻量级目标检测算法，平衡了精度与速度
- **NCNN**：腾讯开源的高性能神经网络前向计算框架，专为移动端和嵌入式设备优化
- **龙芯久久派**：基于LoongArch架构的国产开发板，适合边缘计算场景

### 1.2 项目优势

- ✅ **国产化**：完全基于国产芯片与开源框架
- ✅ **高性能**：NCNN针对ARM/LoongArch优化，推理速度快
- ✅ **低功耗**：适合嵌入式与边缘计算场景
- ✅ **易部署**：模型转换流程清晰，工具链完善

---

## 二、环境准备

### 2.1 硬件要求

| 组件 | 规格 |
|------|------|
| 开发板 | 龙芯久久派（LoongArch架构） |
| 内存 | ≥2GB RAM |
| 存储 | ≥8GB（用于存储模型与依赖） |
| 摄像头 | USB摄像头或CSI摄像头（可选） |

### 2.2 软件环境

#### 2.2.1 PC端环境（模型转换）

```bash
# 操作系统：Ubuntu 20.04 或更高版本
# Python版本：3.8+

# 安装依赖
pip install torch torchvision
pip install onnx
pip install onnx-simplifier
```

#### 2.2.2 龙芯久久派环境（推理部署）

```bash
# 操作系统：Loongnix或适配的Linux发行版
# 编译工具链：GCC 8.0+

# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础依赖
sudo apt install -y build-essential cmake git
sudo apt install -y libopencv-dev
```

---

## 三、YOLOv5模型准备

### 3.1 下载预训练模型

```bash
# 克隆YOLOv5仓库
git clone https://github.com/ultralytics/yolov5.git
cd yolov5

# 下载预训练权重（以yolov5s为例）
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
```

### 3.2 导出ONNX模型

```bash
# 安装YOLOv5依赖
pip install -r requirements.txt

# 导出ONNX格式（动态batch）
python export.py --weights yolov5s.pt --include onnx --simplify

# 导出ONNX格式（固定输入尺寸，推荐）
python export.py --weights yolov5s.pt --include onnx --simplify --imgsz 640 640
```

**导出参数说明**：
- `--weights`：预训练模型路径
- `--include onnx`：导出为ONNX格式
- `--simplify`：简化ONNX图，减少冗余节点
- `--imgsz`：输入图像尺寸（宽×高）

**输出文件**：`yolov5s.onnx`

---

## 四、ONNX转NCNN模型

### 4.1 安装NCNN工具

```bash
# 克隆NCNN仓库
git clone https://github.com/Tencent/ncnn.git
cd ncnn

# 编译NCNN工具（PC端）
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# 工具路径：build/tools/onnx/onnx2ncnn
```

### 4.2 转换模型

```bash
# 转换ONNX到NCNN
./build/tools/onnx/onnx2ncnn yolov5s.onnx yolov5s.param yolov5s.bin

# 优化NCNN模型（可选，减少模型大小）
./build/tools/ncnnoptimize yolov5s.param yolov5s.bin yolov5s-opt.param yolov5s-opt.bin 0
```

**生成文件**：
- `yolov5s.param`：网络结构描述文件（文本格式）
- `yolov5s.bin`：权重文件（二进制格式）

### 4.3 验证模型

```bash
# 检查param文件是否正常
cat yolov5s.param | head -20

# 检查bin文件大小（应与原模型相近）
ls -lh yolov5s.bin
```

---

## 五、龙芯久久派部署

### 5.1 交叉编译NCNN（针对LoongArch）

#### 5.1.1 下载LoongArch工具链

```bash
# 下载龙芯交叉编译工具链
wget http://ftp.loongnix.cn/toolchain/gcc/release/loongarch/gcc-8.3-loongarch64-linux-gnu-2021-09-30.tar.xz
tar -xvf gcc-8.3-loongarch64-linux-gnu-2021-09-30.tar.xz

# 设置环境变量
export PATH=$PATH:$(pwd)/gcc-8.3-loongarch64-linux-gnu/bin
```

#### 5.1.2 交叉编译NCNN

```bash
cd ncnn
mkdir build-loongarch && cd build-loongarch

# 配置交叉编译
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchains/loongarch64-linux-gnu.toolchain.cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DNCNN_VULKAN=OFF \
      -DNCNN_BUILD_EXAMPLES=ON \
      ..

make -j$(nproc)
make install
```

**注意**：如果官方未提供LoongArch工具链文件，需自行创建`loongarch64-linux-gnu.toolchain.cmake`：

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

set(CMAKE_C_COMPILER loongarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER loongarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH /path/to/loongarch64-sysroot)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

### 5.2 本地编译NCNN（直接在久久派上）

**推荐方式**：直接在久久派上编译，避免交叉编译问题。

```bash
# 在久久派上克隆NCNN
git clone https://github.com/Tencent/ncnn.git
cd ncnn

# 创建构建目录
mkdir build && cd build

# 配置编译选项
cmake -DCMAKE_BUILD_TYPE=Release \
      -DNCNN_VULKAN=OFF \
      -DNCNN_BUILD_EXAMPLES=ON \
      -DNCNN_OPENMP=ON \
      ..

# 编译（根据CPU核心数调整-j参数）
make -j4

# 安装
sudo make install
```

### 5.3 准备推理代码

创建`yolov5_ncnn.cpp`：

```cpp
#include <opencv2/opencv.hpp>
#include <net.h>
#include <iostream>
#include <vector>

// YOLO类别数（COCO数据集）
const int NUM_CLASSES = 80;
const float CONF_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.45;

struct Object {
    cv::Rect rect;
    int label;
    float prob;
};

// 后处理：解析YOLO输出
std::vector<Object> decode_outputs(const ncnn::Mat& out, int img_w, int img_h) {
    std::vector<Object> objects;
    
    // YOLOv5输出格式：[batch, num_anchors, 85]
    // 85 = x, y, w, h, confidence, 80 classes
    for (int i = 0; i < out.h; i++) {
        const float* values = out.row(i);
        
        float confidence = values[4];
        if (confidence < CONF_THRESHOLD) continue;
        
        // 找到最大类别概率
        int class_id = 0;
        float max_class_prob = 0;
        for (int j = 0; j < NUM_CLASSES; j++) {
            float class_prob = values[5 + j];
            if (class_prob > max_class_prob) {
                max_class_prob = class_prob;
                class_id = j;
            }
        }
        
        float prob = confidence * max_class_prob;
        if (prob < CONF_THRESHOLD) continue;
        
        // 解析边界框（中心坐标格式转换为左上角坐标）
        float cx = values[0] * img_w;
        float cy = values[1] * img_h;
        float w = values[2] * img_w;
        float h = values[3] * img_h;
        
        Object obj;
        obj.rect.x = cx - w / 2;
        obj.rect.y = cy - h / 2;
        obj.rect.width = w;
        obj.rect.height = h;
        obj.label = class_id;
        obj.prob = prob;
        
        objects.push_back(obj);
    }
    
    return objects;
}

// NMS非极大值抑制
void nms(std::vector<Object>& objects) {
    std::sort(objects.begin(), objects.end(), 
              [](const Object& a, const Object& b) { return a.prob > b.prob; });
    
    std::vector<bool> keep(objects.size(), true);
    
    for (size_t i = 0; i < objects.size(); i++) {
        if (!keep[i]) continue;
        
        for (size_t j = i + 1; j < objects.size(); j++) {
            if (!keep[j]) continue;
            
            // 计算IoU
            float inter_area = (objects[i].rect & objects[j].rect).area();
            float union_area = objects[i].rect.area() + objects[j].rect.area() - inter_area;
            float iou = inter_area / union_area;
            
            if (iou > NMS_THRESHOLD) {
                keep[j] = false;
            }
        }
    }
    
    std::vector<Object> filtered;
    for (size_t i = 0; i < objects.size(); i++) {
        if (keep[i]) filtered.push_back(objects[i]);
    }
    objects = filtered;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <image_path>\n", argv[0]);
        return -1;
    }
    
    // 加载图像
    cv::Mat img = cv::imread(argv[1]);
    if (img.empty()) {
        fprintf(stderr, "Failed to load image: %s\n", argv[1]);
        return -1;
    }
    
    int img_w = img.cols;
    int img_h = img.rows;
    
    // 初始化NCNN网络
    ncnn::Net yolov5;
    yolov5.opt.use_vulkan_compute = false;  // 龙芯不支持Vulkan
    yolov5.opt.num_threads = 4;             // 根据CPU核心数调整
    
    // 加载模型
    if (yolov5.load_param("yolov5s-opt.param") != 0 ||
        yolov5.load_model("yolov5s-opt.bin") != 0) {
        fprintf(stderr, "Failed to load model\n");
        return -1;
    }
    
    // 预处理：调整大小并归一化
    int target_size = 640;
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(
        img.data, ncnn::Mat::PIXEL_BGR, img.cols, img.rows, target_size, target_size);
    
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    in.substract_mean_normalize(0, norm_vals);
    
    // 推理
    ncnn::Extractor ex = yolov5.create_extractor();
    ex.input("images", in);
    
    ncnn::Mat out;
    ex.extract("output", out);
    
    // 后处理
    std::vector<Object> objects = decode_outputs(out, img_w, img_h);
    nms(objects);
    
    // 绘制结果
    for (const auto& obj : objects) {
        cv::rectangle(img, obj.rect, cv::Scalar(0, 255, 0), 2);
        
        char text[256];
        sprintf(text, "class %d: %.2f", obj.label, obj.prob);
        
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        
        cv::putText(img, text, cv::Point(obj.rect.x, obj.rect.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    
    // 保存结果
    cv::imwrite("result.jpg", img);
    printf("Detected %zu objects, result saved to result.jpg\n", objects.size());
    
    return 0;
}
```

### 5.4 编译推理程序

创建`CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.10)
project(yolov5_ncnn)

set(CMAKE_CXX_STANDARD 11)

# 查找OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 查找NCNN（如果已安装到系统）
find_package(ncnn REQUIRED)

# 或手动指定NCNN路径
# include_directories(/path/to/ncnn/include)
# link_directories(/path/to/ncnn/lib)

add_executable(yolov5_ncnn yolov5_ncnn.cpp)
target_link_libraries(yolov5_ncnn ncnn ${OpenCV_LIBS} pthread)
```

编译：

```bash
mkdir build && cd build
cmake ..
make -j4
```

### 5.5 运行推理

```bash
# 复制模型文件到执行目录
cp yolov5s-opt.param yolov5s-opt.bin ./build/

# 运行推理
./build/yolov5_ncnn test.jpg

# 查看结果
display result.jpg  # 或使用其他图像查看器
```

---

## 六、性能优化

### 6.1 模型量化

使用NCNN的量化工具减少模型大小并提升推理速度：

```bash
# 在PC端进行量化（需要准备校准数据集）
./build/tools/quantize/ncnn2table yolov5s-opt.param yolov5s-opt.bin \
    calibration_images.txt yolov5s.table mean=[0,0,0] norm=[0.00392,0.00392,0.00392] \
    shape=[640,640,3] pixel=BGR thread=4 method=kl

# 生成量化模型
./build/tools/quantize/ncnn2int8 yolov5s-opt.param yolov5s-opt.bin \
    yolov5s-int8.param yolov5s-int8.bin yolov5s.table
```

### 6.2 多线程加速

在推理代码中启用OpenMP多线程：

```cpp
yolov5.opt.num_threads = 4;  // 根据CPU核心数调整
yolov5.opt.use_fp16_arithmetic = false;  // 龙芯暂不支持FP16
```

### 6.3 输入尺寸优化

- 较小输入尺寸（如416×416）：速度快，精度略降
- 较大输入尺寸（如640×640）：精度高，速度慢

根据实际场景选择合适的输入尺寸。

---

## 七、常见问题与解决方案

### 7.1 编译错误

**问题1**：找不到`ncnn/net.h`

```bash
# 解决方案：检查NCNN是否正确安装
sudo make install  # 在ncnn/build目录执行

# 或手动指定头文件路径
export CPLUS_INCLUDE_PATH=/path/to/ncnn/include:$CPLUS_INCLUDE_PATH
```

**问题2**：链接错误`undefined reference to ncnn::Net::Net()`

```bash
# 解决方案：检查库文件路径
export LD_LIBRARY_PATH=/path/to/ncnn/lib:$LD_LIBRARY_PATH

# 或在CMakeLists.txt中指定
link_directories(/path/to/ncnn/lib)
```

### 7.2 运行时错误

**问题1**：推理结果异常（无检测或误检）

```bash
# 检查模型输入/输出节点名称
# 打开yolov5s.param，查看Input/Output层名称
cat yolov5s.param | grep "Input\|Output"

# 修改代码中的输入/输出名称
ex.input("images", in);      // 输入节点名称
ex.extract("output", out);   // 输出节点名称
```

**问题2**：推理速度慢

```bash
# 优化方案：
# 1. 使用量化模型（INT8）
# 2. 减小输入尺寸（640→416）
# 3. 增加线程数（根据CPU核心数）
# 4. 使用轻量模型（yolov5n或yolov5s）
```

### 7.3 LoongArch特定问题

**问题**：NCNN在龙芯上编译失败

```bash
# 解决方案1：使用较旧版本的NCNN（稳定性更好）
git clone -b 20230223 https://github.com/Tencent/ncnn.git

# 解决方案2：禁用不支持的功能
cmake -DNCNN_VULKAN=OFF \
      -DNCNN_AVX2=OFF \
      -DNCNN_SSE2=OFF \
      ..
```

---

## 八、实战案例：USB摄像头实时检测

### 8.1 代码实现

修改`yolov5_ncnn.cpp`支持视频流：

```cpp
int main(int argc, char** argv) {
    // 打开摄像头
    cv::VideoCapture cap(0);  // 0表示默认摄像头
    if (!cap.isOpened()) {
        fprintf(stderr, "Failed to open camera\n");
        return -1;
    }
    
    // 加载模型（代码同前）
    ncnn::Net yolov5;
    yolov5.opt.use_vulkan_compute = false;
    yolov5.opt.num_threads = 4;
    yolov5.load_param("yolov5s-opt.param");
    yolov5.load_model("yolov5s-opt.bin");
    
    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        
        // 推理（代码同前）
        int target_size = 640;
        ncnn::Mat in = ncnn::Mat::from_pixels_resize(
            frame.data, ncnn::Mat::PIXEL_BGR, frame.cols, frame.rows, target_size, target_size);
        
        const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
        in.substract_mean_normalize(0, norm_vals);
        
        ncnn::Extractor ex = yolov5.create_extractor();
        ex.input("images", in);
        
        ncnn::Mat out;
        ex.extract("output", out);
        
        std::vector<Object> objects = decode_outputs(out, frame.cols, frame.rows);
        nms(objects);
        
        // 绘制结果
        for (const auto& obj : objects) {
            cv::rectangle(frame, obj.rect, cv::Scalar(0, 255, 0), 2);
            char text[256];
            sprintf(text, "class %d: %.2f", obj.label, obj.prob);
            cv::putText(frame, text, cv::Point(obj.rect.x, obj.rect.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        
        cv::imshow("YOLOv5 Detection", frame);
        if (cv::waitKey(1) == 27) break;  // ESC退出
    }
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
```

### 8.2 性能测试

在龙芯久久派上测试帧率：

```bash
# 640×640输入：约5-8 FPS（yolov5s）
# 416×416输入：约10-15 FPS（yolov5s）
# 使用INT8量化：性能提升20-30%
```

---

## 九、项目总结

### 9.1 技术要点

1. **模型转换链路**：PyTorch → ONNX → NCNN
2. **交叉编译**：针对LoongArch架构优化NCNN
3. **推理优化**：量化、多线程、输入尺寸调整
4. **工程化部署**：CMake构建、OpenCV集成

### 9.2 应用场景

- 🚗 **智能监控**：行人/车辆检测
- 🏭 **工业质检**：缺陷检测
- 🤖 **机器人视觉**：目标识别与跟踪
- 🌾 **农业物联网**：作物病虫害识别

### 9.3 未来展望

- 支持更多YOLO版本（YOLOv8/v10）
- 集成TensorRT加速（如龙芯支持GPU）
- 开发ROS节点，用于机器人系统
- 探索模型剪枝与知识蒸馏

---

## 十、参考资源

### 10.1 官方仓库

- YOLOv5：https://github.com/ultralytics/yolov5
- NCNN：https://github.com/Tencent/ncnn
- 龙芯开源社区：https://github.com/loongson

### 10.2 技术文档

- NCNN Wiki：https://github.com/Tencent/ncnn/wiki
- YOLOv5 Docs：https://docs.ultralytics.com
- LoongArch手册：http://www.loongson.cn/

### 10.3 相关项目

- ncnn-android-yolov5：https://github.com/nihui/ncnn-android-yolov5
- yolov5-ncnn-raspberry-pi：https://github.com/shicai/yolov5-ncnn

---

## 附录：COCO类别名称映射

```cpp
const char* class_names[80] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
};
```

在绘制时使用：

```cpp
std::string label = std::string(class_names[obj.label]) + ": " + std::to_string(obj.prob);
cv::putText(img, label, cv::Point(obj.rect.x, obj.rect.y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
```

---

> **致谢**：本文档参考了NCNN官方Wiki、YOLOv5社区教程及龙芯开发者资料，感谢开源社区的贡献！

> Written with [StackEdit](https://stackedit.cn/).
