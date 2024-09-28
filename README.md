# Robogame2024 SleepNo.1 电控代码
## 前言
本代码为2024中国科学技术大学 Robo Game 机器人比赛SleepNO.1队机器人控制程序。本程序的开发和使用旨在学习机器人控制系统、电机控制理论和相关传感器的使用，仅供学习和娱乐用途使用。
## 目录：

##### I.代码架构介绍
1. STM32CubeMX配置
2. Keil工程代码结构介绍
##### II.各模块详细介绍
1. 任务分配
2. 发射机构
3. 夹取机构
4. 底盘
5. 通信

## 正文：
### I.代码架构介绍
#### I.1 STM32CubeMX配置
CubeMX工程配置文件位于 SleepNo1/SleepNO1.ioc

本车使用的单片机型号为 __STM32F407ZGT6__  

1. 定时器分配
- tim3 : 底盘电机控制信号pwm输出
- tim4 : 丝杆步进电机控制信号pwm输出
- tim5 : pid计算时钟
- tim8 : 摩擦轮电机控制pwm输出
- tim11 : 摄像头调整舵机pwm输出
- tim13 : 方块调整舵机pwm输出
- tim14 : 任务系统计时定时器
  
2. UART分配
- uart2 : 与机械臂总线舵机通信。通信协议见[幻尔总线舵机通信协议](readmeref/02+总线舵机通信协议.pdf)
- uart3 : 与上位机通信。通信协议见[单片机和树莓派的通信协议](单片机与树莓派通信协议.md)
  
3. GPIO分配
- 太多了，不一一列举。大部分GPIO口都有命名，这里仅解释一些命名的含义。具体分配请在[CubeMX工程文件](SleepNo1/SleepNO1.ioc)中查看。
- 命名以MsDriver开头的是丝杆步进电机驱动会用到的GPIO口，共6个。
- 命名以MCL开头的是控制摩擦轮用到的GPIO口，共4个。
- 命名以GDM开头的是控制光电门用到的GPIO口，配置为Exti, 共2个。
- 命名以B开头的是控制阀门和泵用到的GPIO口，共6个。
- 命名含HallEncoder的是底盘电机霍尔编码器用到的GPIO口，共8个。
- 命名含MotoDirection的是底盘电机用到的GPIO口，共8个。
- PA13和PA14是st-link烧写使用的swdio和swclk。
- 剩下的基本上是默认命名（定时器，uart）
- 备注：不属于以上情况的引脚可能未使用。

#### I.2 Keil工程代码结构介绍
SleepNO1使用CubeMX生成Keil工程代码结构，自己添加的文件位于SleepNO1/User , 工程主文件位于SleepNO1/Core/Src

1. 主文件编写内容简介
   
只修改了[main.c](SleepNO1/Core/Src/main.c)和[stm32f4xx_it.c](SleepNO1/Core/Src/stm32f4xx_it.c)。其中main.c中仅包含一堆初始化函数和一堆注释掉的调试代码( ^ 阿 _ 门 ^ )。while(1)内仅有 *Task_Schedule()* 一个任务轮询分配函数，具体任务分配逻辑实现在各综合模块代码中。stm32f4xx_it.c的末尾添加了一些中断回调函数(好像main.c也加了一个,在user code begin 4)。注意，具体实现逻辑都在各模块的.c文件中。

2. 添加文件(User)编写内容简介

打开[SleepNO1/User](SleepNO1/User)你将看到

- Camera
  - camera_pos_dj.c : 控制舵机实现摄像头旋转 
  - camera_pos_dj.h : 对应头文件 (由于每个.c文件实现都有对应.h头文件，之后就不再介绍头文件。)
- Chassis
  - 注：SleepNO1底盘代码来自[ROBOGAME2022官方底盘代码](https://gitee.com/yssickjgd/robogame2022_official_chassis)。在此感谢官方提供的代码为我们节省了不少的工作量awa!
  - Chassis.cpp : 底盘运动实现
  - Motor.cpp : 底盘电机实例实现
  - PID.cpp : pid算法实现
- JiXieBi
  - air_pump.c : 泵和阀门的控制实现
  - my_servo.c : 机械臂总线舵机的动作控制信息发送的实现
  - JiXieBi.c : 综合，机械臂夹取动作实现
- Shoot
  - fix_pos_dj.c : 顶部舵机旋转控制实现
  - MoCaLun.c : 摩擦轮控制实现
  - MSDriver.c : 丝杆步进电机运动控制实现
  - Shoot.c : 综合，发射功能实现
- Task
  - Task.c : 任务轮询综合实现（分任务在各模块中实现）
- com_to_raspi.c : 和上位机串口通信实现

### II.各模块详细介绍
#### II.1 任务分配
任务系统大致工作流程如下：

首先定义任务。任务由一些按一定时间和顺序触发的函数组构成（也可以只有一个函数）。在代码里只需要设置好每一个函数在任务开始后多少秒触发即可。任务中函数触发的顺序由当前任务的**task_id**确定，任务中函数触发的时机则由任务的**time_counter**确定。time_counter在任务启动时每10ms会加一，由tim14控制，需要写一个xx_TIM14_IT函数在tim14溢出中断时触发更新time_counter。

任务函数在各模块中定义为**xxx_schedule**,并被放在main文件的while(1)里不停轮询。

若要启动一个任务，只需要设置task_id为0(或其他你想要的值，如果你有多个任务). 之后，time_counter会不断增加，main函数会不停询问task_id为0的函数是否到达执行的时间。若到达，则执行task_id=0的函数，然后task_id修改为1，main函数继续询问是否到达下一函数执行时间。若要终止一个任务，则只需要将task_id修改为-1即可。

以[Shoot.c](SleepNO1/User/Shoot/Shoot.c)的SHOOT_TASK_Schedule函数为例，
他描述了这样一个发射任务：

1. 任务开始时，执行FIX_POS_DJ_CLOSE函数。即调整一下方块位置。
2. 任务开始后200ms ,执行MS_GO_UP函数。即丝杆向上运动。
3. 任务开始后1.2s,执行MOCALUN_start函数，即摩擦轮开始旋转。
4. 任务开始后2.2s,执行FIX_POS_DJ_OPEN函数，即方块位置调整舵机复位。
5. 任务开始后3.7s,执行MOCALUN_stop函数，即摩擦轮停转。
6. 任务开始后4s,执行MS_GO_DOWN函数，即丝杆向下运动。
7. 任务开始后满6s且确认丝杆运动到底部时,执行send_message函数，告诉上位机已完成发射，整个任务流程结束。
   
可以灵活调整task_id以及time_counter实现多样的任务。在一个任务中也可以启动另一个任务，只要task_id和time_counter不冲突(所以跨模块衔接会很方便)

你可能会注意到任务判断逻辑中time_counter的判断写的都是 time_counter >= time ,这是因为在time时刻main函数不一定在处理该任务，while(1)轮询完一轮之后time_counter可能会略大于time, 写成==可能会错过一些任务。

#### II.2 发射机构
当方块被夹取放在丝杆滑轨上时，可执行发射动作。发射动作流程在[Shoot.c](SleepNO1/User/Shoot/Shoot.c)的SHOOT_TASK_Schedule函数中被定义(上一部分也详细介绍了一下)。接下来讲讲各模块

调整方块位置所用的舵机用pwm波控制，简单解算角度与pwm波对应关系即可。

丝杆所用步进电机为四相步进电机，中间转接一个驱动，控制也是pwm波，但是是基于频率控制的。若要给步进电机加速，则要动态修改auto_reload和compare的值。由于步进电机不能直接给高频pwm到达速度最大值，我们采用低频到高频分段加速的方案，没采用pid。丝杆的运动还受到上下两个光电门的限位控制，一旦抵达限位位置则取消步进电机使能信号。相关逻辑实现见代码文件。光电门使用PNP型，配置时配置为下拉电阻exti。

摩擦轮使用C615电调控制，请注意这款电调已停产，无法使用上位机调试软件调试。控制时请仔细阅读手册，设置好pwm波的频率不超过500Hz,使用不同占空比进行控制。可以把10%-90%占空比pwm都给到电调看看反应，还有一点：电调开机时pwm需稳定至少3s。

#### II.3 夹取机构
我们采用总线舵机机械臂+吸盘的组合完成夹取任务。夹取任务基本上是机械臂做固定姿态动作，所以任务里面写了很多机械臂位置参数，调参就行了（可以用上位机调参软件，方便且迅速），实现起来没那么难。

总线舵机采用的是HTD-45H，用uart2通信，通信协议见[幻尔总线舵机通信协议](readmeref/02+总线舵机通信协议.pdf)。我们的舵机只用到了设置舵机角度这一种命令，在[my_servo.c](SleepNO1/User/JiXieBi/my_servo.c)里面对照通信协议简单实现了一下。（备注：这个文件编码是gbk，用utf-8打开可能会看到很多乱码，但是现在不想改了，就是懒awa）

关于吸盘的控制，就控制泵和阀门就行。泵和阀门就像两个开关一样控制就ok，开开关关一共也就4种组合，选两个能用的就行。

#### II.4 底盘
底盘代码源自[robogame2022官方底盘代码](https://gitee.com/yssickjgd/robogame2022_official_chassis)。再次感谢伟大的组委会awa！

我们根据小车实际情况在头文件修改了一些配置参数。实际小车使用半径75的麦轮，带霍尔编码器。切记麦轮要装成X形！（装错了可能移动会很不稳定qwq）

我们选用了位置pid，实测效果还行(虽然也是半开环)。由于外环用到了I参数，停车的时候要手动清空I值。

#### II.5 通信
UART3用于和上位机通信，通信协议见[单片机和树莓派的通信协议](单片机与树莓派通信协议.md)

通信逻辑就不再赘述，直接用Hal库即可(详情见[com_to_raspi.c](SleepNO1/User/com_to_raspi.c))。使用HAL_UART_Receive_IT开启中断式接收，然后写一个HAL_UART_RxCpltCallback处理就行。

接收上位机指令时，写了一个循环队列缓存指令。指令会按顺序先后触发,两个指令间隔100ms。发射指令和夹取指令有互斥关系，也在任务中进行了一定处理。

通信时注意usb转ttl可以不接5v线（有时候接了会通信不上，也不知道为啥）

### others
[bug日志](readmeref/bugs.md)
