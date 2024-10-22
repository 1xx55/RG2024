# RoboGame2022 官方底盘代码食用指南

[toc]

## 1 说在前面的话

###　1.1 环境要求

-   Keil, 以及相关驱动, 相关固件库pack文件
-   STM32CubeMX, 以及相关的HAL库下载安装

### 1.2 相关知识要求

-   C语言
-   少量的C++, 比如模板函数, 类与类的继承
-   可有可无的微机原理与嵌入式系统内容

### 1.3 代码规范

-   代码规范很重要!

-   变量每个单词大写, 单词间下划线隔开, 要有实际意义

-   自定义类型名要写出类型的名称, 比如:

    ```cpp
    enum Enum_Control_Method
    {
        OPENLOOP = 0,
        OMEGA,
        ANGLE
    };
    ```

    表示一个枚举类型, 看过代码会发现这是确定电机控制方式的代码

-   注释不怕少, 一定要勤写

## 2 架构相关

### 2.1 本地文件架构

-   所有自己写的文件都在USER里面
-   要注意CubeMX这款软件特点, 它生成需修改的文件, 要把代码放在BEGIN与END之间

### 2.2 Cube文件架构

-   采用STM32F103RCT6单片机, 也是电设的单片机型号

-   系统内核
    -   GPIO
        -   电机转动方向控制需要推挽输出, 共8个
        -   电机霍尔编码器上升沿外部中断需要下拉输入, 共4个
        -   电机霍尔编码器另一相检测电平确定方向需要下拉输入, 共4个
        -   电机驱动依赖定时器PWM波形信号输出, 共4个
    -   NVIC
        -   电机霍尔编码器上升沿外部中断使能
    -   RCC
        -   系统实时时钟配置外部晶振, 用72MHZ, 算是发挥最大性能
    -   SYS
        -   debug选择SerialWire, 防止单片机变成一次性板砖, 虽然砸人也不咋疼
-   模拟信号交互
    -   暂无, 后续巡线模块可以用, 也可以不用
-   定时器
    -   TIM4, 用来PWM波生成的定时器通道
    -   TIM5, 用来进行速度解算的定时器中断

### 2.2 工程文件架构

-   工程文件中, c / cpp文件需要手动添加, h / hpp文件要在Keil中设置包含

-   工程文件和本地文件的拓扑结构一定程度上可以认为毫无关系
-   工程文件建议分层实现功能, 当然本仓库代码也是这样做的, 具体如下:
    -   应用层
        -   main.c
            -   显而易见是主函数
    -   模块层
        -   Chassis.cpp
            -   底盘类相关代码
    -   设备层
        -   Motor.cpp
            -   电机类相关代码
        -   Steer.cpp
            -   舵机类相关代码
    -   算法层
        -   PID.cpp
            -   PID控制反馈算法类相关代码
    -   固件层与汇编层
        -   系统自动生成, 除非专业人士, 否则切勿修改

## 3 代码使用相关

-   以Motor的代码为例
    -   Motor.hpp是头文件, 有一些和电机具体参数相关的常量声明 ( 比如减速比, 满电转速等 ) , 类的成员变量与成员函数的声明
    -   Motor.cpp是函数定义文件, 主要是全局变量以及函数定义, 比如初始化绑定GPIO的函数, 一些编码器中断处理函数, 速度解算中断处理函数等等
-   本实验的代码基于驰海CHP-36GP-555 ( 235rpm-i51 ) 电机, 搭配WheelBoard-D50A-12A大电流MOS双路直流电机驱动. 舵机采用普通的非总线舵机. 遥控器型号为大疆DT7-DR16