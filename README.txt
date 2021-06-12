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

