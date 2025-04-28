

# 基于PID的四旋翼无人机轨迹跟踪控制

## 二维笔记说明

 ### 1. 展示二维平面四旋翼圆形轨迹跟踪（以其为例）
 ![基于PID的二维平面四旋翼圆形轨迹跟踪](/imgs/2025-04-29/2zgJqCdRNayZgQQu.png)

### 2.状态方程解读


### 3. 轨迹生成

- 一般的轨迹都是以正弦波的形式来生成的，即为simulink里的Sine Wave模块生成，只需要Sine Wave正弦波模块、Outpart输出模块、Mux模块与To Workspace模块，较为简单

如下图展示：

![轨迹生成 （1）](/imgs/2025-04-29/pFWBZW8TqOlMpRLl.png)

- 也可以通过Fcn模块自己写函数式来定义轨迹的形状，较为推荐此方式来进行轨迹的生成，将时钟Colck模块定义为u，在之后的Fcn模块中输入相应的函数关系式5*sin(u)，再又

如下图展示：![轨迹生成 （2）](/imgs/2025-04-29/z0ECQc1znVrJvRlf.png)
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTYxNDE2MDYyNSwtNjMxNzUyNzM1LDQ0MD
kwNTYxOV19
-->