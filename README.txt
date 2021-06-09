Smart_Dog 0601版
以下为文件说明与版本更新记录

工程文件位于MDK-ARM文件夹中
用户库文件存放于Core文件夹中，请将头文件与库文件分别保存
工程使用了FreeRTOS操作系统用于协调各个任务运行

2021/06/08 hhq123更新内容如下
	使用LobotServoController与舵机通讯（USART6）
	使用Quadrudep_huaner进一步封装，仅需ik_Move(x,y,z)立即执行腿部运动
	使用test_workspace做了部分运动测试，如trot步态
	不清楚为什么trot步态会和FreeRTOS产生冲突

	阶段计划
		更新蓝牙通讯、IIC通讯，并初步对OLED测试
		更新ROS通讯
		更新步态程序
		
2021/06/09 
