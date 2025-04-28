

# 基于PID的四旋翼无人机轨迹跟踪控制

## 二维笔记说明

 ### 1. 展示二维平面四旋翼圆形轨迹跟踪（以其为例）
 ![基于PID的二维平面四旋翼圆形轨迹跟踪](/imgs/2025-04-29/2zgJqCdRNayZgQQu.png)

### 2.状态方程解读


### 3. 轨迹生成

- 一般的轨迹都是以正弦波的形式来生成的，即为simulink里的Sine Wave模块生成，只需要Sine Wave正弦波模块、输出模块、mux模块与To 为下图展示：

![轨迹生成 （1）](/imgs/2025-04-29/pFWBZW8TqOlMpRLl.png)

- 也可以通过Fcn模块自己写函数式来定义轨迹的形状，较为推荐此方式来进行轨迹的生成，如下图展示：![轨迹生成 （2）](/imgs/2025-04-29/z0ECQc1znVrJvRlf.png)
<!--stackedit_data:
eyJoaXN0b3J5IjpbNzU5Mjg3Nzc3LC02MzE3NTI3MzUsNDQwOT
A1NjE5XX0=
-->