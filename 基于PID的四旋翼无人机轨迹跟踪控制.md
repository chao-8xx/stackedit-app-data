

# 基于PID的四旋翼无人机轨迹跟踪控制

## 二维笔记说明

 ### 1. 展示二维平面四旋翼圆形轨迹跟踪（以其为例）
 ![基于PID的二维平面四旋翼圆形轨迹跟踪](/imgs/2025-04-29/2zgJqCdRNayZgQQu.png)

### 2.状态方程解读


### 3. 轨迹生成

1. 一般的轨迹都是以正弦波的形式来生成的，即为simulink里的**Sine Wave**模块生成，只需要**Sine Wave正弦波模块**、**Outpart输出模块**、**Mux模块**与**To Workspace模块**，较为简单。

如下图展示：

![轨迹生成 （1）](/imgs/2025-04-29/pFWBZW8TqOlMpRLl.png)

2. 也可以通过**Fcn模块**自己写函数式来定义轨迹的形状，较为推荐此方式来进行轨迹的生成。

#### ​讲解： 
 
#### **核心模块解析**

#### ​**1. 正弦信号源模块**

-   ​**模块1**：`5*sin(u)`
    -   ​**功能**：生成振幅为5、频率与输入`u`相关的正弦波信号。
    -   ​**输出**：通过连线连接到输出端口`Y`（对应右侧的`y_fcn`信号线）。
-   ​**模块2**：`5*sin(0.5*u)`
    -   ​**功能**：生成振幅为5、频率为`0.5*u`的正弦波（频率是模块1的一半）。
    -   ​**输出**：通过连线连接到输出端口`Z`（对应右侧的`z_fcn`信号线）。

#### ​**2. 输出端口**

-   ​**Y和Z**：将两个正弦信号输出到工作空间或外部设备，便于后续分析（可用Scope模块观测波形）。

如下图展示：![轨迹生成 （2）](/imgs/2025-04-29/z0ECQc1znVrJvRlf.png)

### 4. PID控制器

#### **具体图展示**


![PID控制器](/imgs/2025-04-29/mieRgVuCIm2K7Tda.png)

#### **讲解**


![状态变量state](/imgs/2025-04-29/7dZkCcUQ3FJglUzR.png)


<!--stackedit_data:
eyJoaXN0b3J5IjpbLTQ3MzI2NzAwNiwtMTU2MjUzNzU5NiwxMD
M1MDgxNTk3LC01MjI3NjkyMTAsLTYzMTc1MjczNSw0NDA5MDU2
MTldfQ==
-->