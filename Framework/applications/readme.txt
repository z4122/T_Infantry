经常调整的参数：
摩擦轮的转速：drivers_uartrc_low.h 36行FRICTION_WHEEL_MAX_DUTY  一般在1200到1600的范围，全国赛用的1350 
两轴云台的零点位置：tasks_motor.c 中的yaw_zero，pitch_zero（不同的车不一样）
两轴云台的PID，tasks_motor.c 中，位置环PID：pitchPositionPID，yawPositionPID；速度环PID：pitchSpeedPID，yawSpeedPID
底盘速度PID，tasks_motor.c 中CM1SpeedPID和CMRotatePID，即对应到tasks_timed.h中的CHASSIS_MOTOR_ROTATE_PID_DEFAULT和CHASSIS_MOTOR_SPEED_PID_DEFAULT
步兵车号（不同的车参数不一样）：tasks_motor.h中的 #define INFANTRY_1
云台位置的目标值，一般只在遥控中修改，若用于其他地方的控制则修改这个值：tasks_motor.c 中78行yawAngleTarget，82行pitchAngleTarget
                                                                    （记得不是改它的初始值，是修改这个变量，同时注意其他还有什么地方修改了这个变量，一般范围是-10到10）
底盘速度的目标值（注意点同上）：drivers_uartrc.c：中65行结构体ChassisSpeedRef，前后ChassisSpeedRef.forward_back_ref，左右ChassisSpeedRef.left_right_ref，
                               旋转ChassisSpeedRef.rotate_ref，一般范围两到三万
遥控的设置，tasks_remotecontrol.c 213行的RemoteControlProcess(Remote *rc) 四通道摇杆rc->ch0到rc->ch3，数值为0到2047，中值为1024，
                                       摇杆控制前后左右的灵敏度STICK_TO_CHASSIS_SPEED_REF_FACT，控制云台运动的灵敏度STICK_TO_PITCH_ANGLE_INC_FACT，STICK_TO_YAW_ANGLE_INC_FACT
鼠标键盘的设置 243行的 MouseKeyControlProcess(Mouse *mouse, Key *key) 键位是key->v，键位表如下：
//Bit0-----W
//Bit1-----S
//Bit2-----A
//Bit3-----D
//Bit4-----Shift
//Bit5-----Ctrl
//Bit6-----Q
//Bit7-----E
//Bit8-----R
//Bit9-----F
//Bit10-----G
//Bit11-----Z
//Bit12-----X
//Bit13-----C
//Bit14-----V
//Bit15-----B
鼠标键盘下云台控制的灵敏度MOUSE_TO_PITCH_ANGLE_INC_FACT，MOUSE_TO_YAW_ANGLE_INC_FACT
          底盘移动的速度LOW_FORWARD_BACK_SPEED，MIDDLE_FORWARD_BACK_SPEED，NORMAL_FORWARD_BACK_SPEED
		                LOW_LEFT_RIGHT_SPEED，MIDDLE_LEFT_RIGHT_SPEED，NORMAL_LEFT_RIGHT_SPEED
执行一次发射 调用一次tasks_platemotor.c中的ShootOneBullet()

Drivers/CMSIS,Application/MDK-ARM,Drivers/STM32F4xx_HAL_Driver:
这些是ST官方提供的固件库，用以包装芯片底层驱动的库函数，不同系列的芯片库函数不一样，Cube配置时会自动生成。
不知道相应功能的库函数是什么时可以百度，可以查说明书，也可以在相应外设的库函数文件里面找

Middlewares/FreeRTOS:
FreeRTOS操作系统的程序文件，Cube配置时自动生成
其中的main函数

Middlewares/USB_Device_Library:
USB的配置库函数文件，主控板上有一个miniUSB接口，可以用这种方式通信，但我们目前没有开发

Application/User：
外设的配置，Cube生成

Framework/RTOS:
rtos_init.c:系统的初始化函数，对板子上的外设模块进行初始化。要区别芯片上的外设初始化，那个在这个之前，是Cube生成的。
这个要自己写，以后添加什么新硬件需要初始化的话在这里面加。
rtos_semaphore.c:系统进程信号量的初始化，添加信号量参照这个格式添加
rtos_task.c:系统进程初始化，添加任务参照这个格式添加

Framework/Peripheral:
peripheral_define.h: 定时器，串口，CAN的重命名，提高可读性
peripheral_gpio.c：外部中断读取MPU6050数据
peripheral_laser.h：激光瞄准器的开关函数

Framework/Utilities:
utilities_debug.c：重定向C标准库函数printf到串口DEBUG_UART,可以直接用printf输出信息到串口，方便调试
utilities_iopool.c：IOPOOL的定义，一种相比于全局变量，在线程间安全通信的方式，具体使用看C文件注释
utilities_minmax.h：求最大最小值的函数和角度规范化的函数
utilities_tim.c：使用1ms定时器获得上电后ms单位的时间
peripheral_tim.c：摩擦轮、舵机PWM所需定时器初始化函数

Framework/Drives:
drivers_led.c：红灯和绿灯的开关函数，以及两个亮灯的任务
drivers_imu.c：IMU传感器数据的读取函数，以及IMU数据的刷新任务
drivers_buzzer.c：蜂鸣器的执行函数，包括开机音乐
drivers_canmotor.c：CAN的底层驱动，CAN的接收是通过中断的方式，用以接收反馈的编码器位置、速度信息，以及单轴陀螺仪的数据。
                    CAN的发送用来驱动电机，以及设置单轴陀螺仪，需要调用
drivers_uart.c：所有串口的中断函数，包括遥控器串口，妙算串口，裁判系统串口
drivers_uartrc.c：遥控器串口接收函数，主要包括拨杆的模式设置，以及发射摩擦轮的启动逻辑
drivers_upper.c：妙算的通信函数，原来只回传一个大神符的击打数字
drivers_flash.c：flash写入和读取的函数，用于掉电保存，但我们目前没用到
drivers_sonar.c：超声波测距的驱动，基地上用，步兵没有用到
drivers_ramp.h：斜坡函数
pid_regulator.c：PID函数的实现，最基本的PID函数，更高级的算法要改这里
pwm_server_motor：不同PWM控制不懂的舵机角度，原先用来控制弹仓开关的，现在没有用到
drivers_uartjudge.c：裁判系统串口的中断函数，将裁判系统的数据帧解析成信息
drivers_platemotor.c：拨盘电机速度/位置控制
drivers_cmpower.c：根据能量槽剩余做动态上限

Framework/Applications：主体的任务都在这边，比较重要
tasks_motor.c：底盘和云台的控制任务，主要要调的底盘PID和云台PID都在这边
application_motorcontrol.c：执行电机CAN信号控制的函数
tasks_upper.c：妙算通信任务
application_quaternion：更新四元数的函数，但我们目前没有用到
tasks_remotecontrol.c：遥控器控制的任务函数，比较重要，遥控器拨杆、摇杆，键盘鼠标操作方式都在这边改
tasks_timed.c 2ms周期的任务，状态机切换
application_waveform.c 上位机观察电机信号波形，之前没有用到
tasks_platemotor.c 拨盘电机的任务，用来控制发射，调用一次ShootOneBullet()执行一次发射