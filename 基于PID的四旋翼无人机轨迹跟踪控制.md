

# 基于PID的四旋翼无人机轨迹跟踪控制

## 二维笔记说明

 ### 1. 展示二维平面四旋翼圆形轨迹跟踪（以其为例）
 ![基于PID的二维平面四旋翼圆形轨迹跟踪](/imgs/2025-04-29/2zgJqCdRNayZgQQu.png "基于PID的二维平面四旋翼圆形轨迹跟踪")

### 2.状态方程解读



### 3. 轨迹生成

1. 一般的轨迹都是以正弦波的形式来生成的，即为simulink里的**Sine Wave**模块生成，只需要**Sine Wave正弦波模块**、**Outpart输出模块**、**Mux模块**与**To Workspace模块**，较为简单。

如下图展示：

![轨迹生成 （1）](/imgs/2025-04-29/pFWBZW8TqOlMpRLl.png "轨迹生成 （1）")

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

如下图展示：![轨迹生成 （2）](/imgs/2025-04-29/z0ECQc1znVrJvRlf.png "轨迹生成 （2）")

### 4. PID控制器与二维四旋翼动态系统

### **讲解**

### 1.讲解**二维四旋翼动态系统**

#### **1. 整体架构**

-   ​**模型名称**：`quadrotor2_circle/二维四旋翼动态系统`  
    专注于二维平面内四旋翼飞行器的动力学行为仿真
-   ​**核心模块**：
    -   ​`quadrotor2_system`：初始化定义飞行器物理特性（质量m、转动惯量J），将输入的推力`F`和力矩`M`通过动力学方程转换为状态变量`state` 
    -   ​`state`：存储和传递状态变量（位置环中的y坐标、z坐标、姿态环的phi角度），形成闭环反馈回路。
    -   ​**输入输出**：通过输入模块`力F`和`力矩M`驱动系统，输出飞行器的实时状态（如位置、姿态）。

#### ​**2. 关键参数与设计**

-   ​**信号流逻辑**：
  ```mermaid
graph TB
    A[输入力F/力矩M] -->|动力学方程计算| B(quadrotor2_system)
    B -->|输出状态变量state| C[反馈至PID控制器]
    C -.->|形成闭环| A
```

#### ​**3. 技术含义**

-   ​**状态反馈**：`state`模块为控制器提供实时位置和姿态数据，是闭环控制的基础。


![状态变量state](/imgs/2025-04-29/7dZkCcUQ3FJglUzR.png "状态变量state")

### 2.解析**PID控制器设计**

#### **1. 整体架构**

-   ​**模型名称**：`quadrotor2_circle/PID控制器`  
    实现高度（z/y方向）和横滚角（phi）的双闭环控制。
-   ​**核心模块**：
    -   ​**PID模块**：包含两组PID控制器，分别为处理高度误差（`z_d - z`）的位置环与横向位置误差（`y_d - y`）及角度（`phi`）共同组成的姿态环
    -   ​**限幅模块**：对输出推力`F`和力矩`M`进行约束（如电机最大推力）。
    -   ​**加减法模块**：计算期望值与实际状态的误差（`error`信号）。

#### ​**2. 关键参数与设计**

-   ​**控制逻辑**：
    1.  ​**输入**：期望高度`z_d`、期望横向位置`y_d`
    2.  ​**误差计算**：通过减法模块生成`error`信号。
    3.  ​**PID调节**：将误差信号转换为控制量（`F`和`M`），并通过限幅模块防止超调。
    4.  ​**输出**：控制量传递给动态系统模型驱动飞行器。
-   ​**抗饱和设计**：限幅模块避免控制器输出超出电机能力范围。

#### ​**3. 技术含义**

-   ​**双闭环控制**：
    -   ​**外环**：位置控制（如高度z/y）生成角度期望值。
    -   ​**内环**：姿态控制（如横滚角phi）快速响应角度变化进而调整位置。
-   ​**解耦设计**：独立PID控制器处理不同自由度，简化多变量系统控制。

![PID控制器](/imgs/2025-04-29/mieRgVuCIm2K7Tda.png "PID控制器")

### **3.二者协同工作原理**

1.  ​**信号闭环流程**：
  ```mermaid
graph TD
    A(开始)
    B(测量状态 state)
    C(计算误差 error)
    D(生成控制信号)
    E(动态系统响应)
    F(更新状态 state)
    G(回到计算误差 error)
    
    A --> B
    B --> C
    C --> D
    D --> E
    E --> F
    F --> G
    G --> C
```

2.  ​**工程实现目标**：
    -   使四旋翼飞行器在二维空间内以圆形轨迹运动。
    -   通过PID参数整定（如比例系数K、积分时间time）优化动态响应。




```
function draw_quadrotor_2D(t, position, angle, traj)

% 闲鱼：一只小鲤鱼鸭

% B站： 芜胡起Fly

% 输入参数：

% t: 时间[n*1]

% position: (y, z)位置[n*2]

% angle: 角度[n*1]

% traj: (y, z)参考轨迹[n*2]

%% 坐标轴属性设置

fig=figure('Name','Quadrotor_2D','NumberTitle','off','Position',[500 250 400 410]);

ax=axes;

axis equal;

if nargin < 3

fprintf(" 输入参数：\n\t t: 时间[n*1]\n\t position: (y, z)位置[n*2]\n\t angle: 角度[n*1]\n\t traj: (y, z)参考轨迹[n*2]\n");

return;

end

if nargin > 3

% 跟踪轨迹

plot(ax,traj(:,1),traj(:,2),'--k','LineWidth',2);

end

y_lim_max = ceil(max(position(:,1))) + 2;

y_lim_min = floor(min(position(:,1))) - 2;

z_lim_max = ceil(max(position(:,2))) + 2;

z_lim_min = floor(min(position(:,2))) - 2;

set(ax,'looseinset',get(ax,'tightinset'),'nextplot','add','XGrid','on','YGrid','on',...

'Xlim',[y_lim_min y_lim_max],'Ylim',[z_lim_min z_lim_max],...

'XTick',y_lim_min:1:y_lim_max,'YTick',z_lim_min:1:z_lim_max);

title(ax,'2D\_Quadrotor','Fontname', 'Times New Roman','FontSize',12);

xlabel(ax,'y(m)','interpreter','latex','Fontname', 'Times New Roman','FontSize',12);

ylabel(ax,'z(m)','interpreter','latex','Fontname', 'Times New Roman','FontSize',12);

Quadrotor.L = 0.5; % 机臂长度

Quadrotor.H = 0.2; % 电机高度

Quadrotor.W = 0.15; % 螺旋桨半径

%% 定义构建四旋翼的关键坐标点

Quadrotor_Body = [Quadrotor.L 0 1;

-Quadrotor.L 0 1;

Quadrotor.L Quadrotor.H 1;

-Quadrotor.L Quadrotor.H 1;

Quadrotor.L+Quadrotor.W Quadrotor.H 1;

Quadrotor.L-Quadrotor.W Quadrotor.H 1;

-Quadrotor.L+Quadrotor.W Quadrotor.H 1;

-Quadrotor.L-Quadrotor.W Quadrotor.H 1 ]'; % 注意这里是转置

line = plot(ax,0,0,'-r','LineWidth',2); % 四旋翼实际轨迹

h1 = plot(ax,0,0,'-b.','LineWidth',2,'MarkerSize',2); % 机臂

h2 = plot(ax,0,0,'-b','LineWidth',2); % 电机1

h3 = plot(ax,0,0,'-b','LineWidth',2); % 电机2

h4 = plot(ax,0,0,'-b','LineWidth',2); % 螺旋桨1

h5 = plot(ax,0,0,'-b','LineWidth',2); % 螺旋桨2

legend("参考轨迹","实际轨迹");

for i = 1:1:size(t)

% 获取四旋翼位置和姿态

quadrotor_pos = position(i,:)';

phi=angle(i);

% 2D旋转矩阵

R = [cos(phi) sin(phi);

-sin(phi) cos(phi)];

% 通过把四旋翼在机体坐标系下的关键点变换到地球坐标系下

% 用于画四旋翼在地球坐标系下的真实姿态

wHb = [R quadrotor_pos;

0 0 1];

quadrotor_world = wHb * Quadrotor_Body; % [3x3][3x8]

quadrotor_atti = quadrotor_world(1:2, :);

% 四旋翼画图

set(h1,'Xdata',quadrotor_atti(1,[1 2]), 'Ydata',quadrotor_atti(2,[1 2])); % 机臂

set(h2,'Xdata',quadrotor_atti(1,[1 3]), 'Ydata',quadrotor_atti(2,[1 3])); % 电机1

set(h3,'Xdata',quadrotor_atti(1,[4 2]), 'Ydata',quadrotor_atti(2,[4 2])); % 电机2

set(h4,'Xdata',quadrotor_atti(1,[5 6]), 'Ydata',quadrotor_atti(2,[5 6])); % 螺旋桨1

set(h5,'Xdata',quadrotor_atti(1,[7 8]), 'Ydata',quadrotor_atti(2,[7 8])); % 螺旋桨2

set(line,'Xdata',position(1:i,1),'Ydata',position(1:i,2)); % 四旋翼轨迹

drawnow;

end
```
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTM0NzQxMTQ0NiwxMjg2MDI1NTI4LDE3Mz
AyMjY0OTAsLTExMzgyOTU5OTUsMTQ1MDc0NjgwMiwxMTc2NzI0
NTI1LDE3MjU2NDc5MTQsLTEwMzc4NDY1MTIsMTUxNTU5OTk3Ny
wxMzUwOTQyNzc5LDQ5MjAwMTQ0NywtMTI1ODIxNzQ0NSwzMzEx
MTg3ODYsLTE1NjI1Mzc1OTYsMTAzNTA4MTU5NywtNTIyNzY5Mj
EwLC02MzE3NTI3MzUsNDQwOTA1NjE5XX0=
-->