Smart_Dog 0601版
以下为文件说明与版本更新记录

工程文件位于MDK-ARM文件夹中
用户库文件存放于Core文件夹中，请将头文件与库文件分别保存
工程使用了FreeRTOS操作系统用于协调各个任务运行

2021/06/08 hhq123更新内容如下
	使用LobotServoController与舵机通讯（USART6）
	使用Quadrudep_huaner进一步封装，仅需ik_Move(x,y,z)立即执行腿部运动
	使用test_workspace做了部分运动测试，如trot步态
	不清楚为什么trot步态会和FreeRTOS产生冲突-》因为任务堆栈开小了

	阶段计划
		更新蓝牙通讯、IIC通讯，并初步对OLED测试
		更新ROS通讯
		更新步态程序
		
2021/06/09 下雨
串口中断DMA参考资料
FreeRTOS操作量https://blog.csdn.net/hbsyaaa/article/details/107328274
https://blog.csdn.net/weixin_40729877/article/details/100025268
https://blog.csdn.net/u014470361/article/details/79206352
DMA设置方法https://blog.csdn.net/jasper_JA/article/details/103988271?utm_medium=distribute.pc_relevant.none-task-blog-baidujs_title-4&spm=1001.2101.3001.4242
使用邮箱发送https://bbs.21ic.com/icview-3039134-1-1.html
消息队列处理讯息https://blog.csdn.net/u010058695/article/details/112519110推荐
使用DMA发送https://blog.csdn.net/qq_42800231/article/details/109315411
USART6每次发送小段讯息，似乎没有必要设置DMA

IIC控制https://blog.csdn.net/as480133937/article/details/105259075

JY901九轴https://blog.csdn.net/Fred_1986/article/details/108350958

然后板子烧了

2021/06/12 多云，明天组长来调狗，我觉得要下雨
今天计划写完 蓝牙 JY901等，并搭建好环境供他人继续编写

为什么不直接用usb进行通讯呢
USB虚拟串口https://blog.csdn.net/qq_36561846/article/details/109427606

今日工作
Dog_interface 包含控制程序，可以直接在此处枚举增加功能
	bluetoothController为蓝牙控制器
	Gait_Controller为运动控制器

test_workspace 包含步态解算程序，可以直接移植，并写到Dog_interface中

此外具有多个外设操作
IIC同时控制OLED（如OLED_ShowStr(6, 2, str, 1);）与JY901（JY901_RDDat(&IMU);）
USART1与蓝牙通讯，使用printf函数信息反馈到蓝牙上
USART2与舵机控制板通讯
USB可以使用usb_printf函数打印信息

下次更新usb与ros通讯程序

2021/06/16 小雨，数据结构考炸了
今日计划测试单
	F4控制运行测试
	蓝牙通讯保护帧
	
	加减速测试  重点
		分析，考虑到单纯的降速，舵机移动速度同样会很快，因此这个方案不好
		最好在通讯协议中增加时间参数
			上个说法是有问题的，很难保证同步
		考虑增加步数的方式
		或者delay函数同步的方法
	
运行速度控制器
	开一个定时器控制底层驱动，同时控制信号量
	姿态控制器将会根据信号量
	
	传输完毕释放信号量
	
	控制系统优化
	
2021/06/16
MOD 0 /n
VAR




