Smart_Dog 0601��
����Ϊ�ļ�˵����汾���¼�¼

�����ļ�λ��MDK-ARM�ļ�����
�û����ļ������Core�ļ����У��뽫ͷ�ļ�����ļ��ֱ𱣴�
����ʹ����FreeRTOS����ϵͳ����Э��������������

2021/06/08 hhq123������������
	ʹ��LobotServoController����ͨѶ��USART6��
	ʹ��Quadrudep_huaner��һ����װ������ik_Move(x,y,z)����ִ���Ȳ��˶�
	ʹ��test_workspace���˲����˶����ԣ���trot��̬
	�����Ϊʲôtrot��̬���FreeRTOS������ͻ-����Ϊ�����ջ��С��

	�׶μƻ�
		��������ͨѶ��IICͨѶ����������OLED����
		����ROSͨѶ
		���²�̬����
		
2021/06/09 ����
�����ж�DMA�ο�����
FreeRTOS������https://blog.csdn.net/hbsyaaa/article/details/107328274
https://blog.csdn.net/weixin_40729877/article/details/100025268
https://blog.csdn.net/u014470361/article/details/79206352
DMA���÷���https://blog.csdn.net/jasper_JA/article/details/103988271?utm_medium=distribute.pc_relevant.none-task-blog-baidujs_title-4&spm=1001.2101.3001.4242
ʹ�����䷢��https://bbs.21ic.com/icview-3039134-1-1.html
��Ϣ���д���ѶϢhttps://blog.csdn.net/u010058695/article/details/112519110�Ƽ�
ʹ��DMA����https://blog.csdn.net/qq_42800231/article/details/109315411
USART6ÿ�η���С��ѶϢ���ƺ�û�б�Ҫ����DMA

IIC����https://blog.csdn.net/as480133937/article/details/105259075

JY901����https://blog.csdn.net/Fred_1986/article/details/108350958

Ȼ���������

2021/06/12 ���ƣ������鳤���������Ҿ���Ҫ����
����ƻ�д�� ���� JY901�ȣ�����û��������˼�����д

Ϊʲô��ֱ����usb����ͨѶ��
USB���⴮��https://blog.csdn.net/qq_36561846/article/details/109427606

���չ���
Dog_interface �������Ƴ��򣬿���ֱ���ڴ˴�ö�����ӹ���
	bluetoothControllerΪ����������
	Gait_ControllerΪ�˶�������

test_workspace ������̬������򣬿���ֱ����ֲ����д��Dog_interface��

������ж���������
IICͬʱ����OLED����OLED_ShowStr(6, 2, str, 1);����JY901��JY901_RDDat(&IMU);��
USART1������ͨѶ��ʹ��printf������Ϣ������������
USART2�������ư�ͨѶ
USB����ʹ��usb_printf������ӡ��Ϣ

�´θ���usb��rosͨѶ����

2021/06/16 С�꣬���ݽṹ��ը��
���ռƻ����Ե�
	F4�������в���
	����ͨѶ����֡
	
	�Ӽ��ٲ���  �ص�
		���������ǵ������Ľ��٣�����ƶ��ٶ�ͬ����ܿ죬��������������
		�����ͨѶЭ��������ʱ�����
			�ϸ�˵����������ģ����ѱ�֤ͬ��
		�������Ӳ����ķ�ʽ
		����delay����ͬ���ķ���
	
�����ٶȿ�����
	��һ����ʱ�����Ƶײ�������ͬʱ�����ź���
	��̬��������������ź���
	
	��������ͷ��ź���
	
	����ϵͳ�Ż�
	
2021/06/16
MOD 0 /n
VAR

��̬�л�http://gxbwk.njournal.sdu.edu.cn/fileup/1672-3961/HTML/2018-4-42.htm

https://blog.csdn.net/Electrical_IT/article/details/107221567
https://blog.csdn.net/linmh0130/article/details/84494428?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162397794516780269842920%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=162397794516780269842920&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~rank_v29-3-84494428.pc_search_result_before_js&utm_term=py%E4%B8%B2%E5%8F%A3%E6%97%A0%E5%93%8D%E5%BA%94&spm=1018.2226.3001.4187


2021/06/18
rosͨѶhttps://blog.csdn.net/xiaoyuanwuhui/article/details/97895473?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162410192816780265480074%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=162410192816780265480074&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~rank_v29-2-97895473.pc_search_result_before_js&utm_term=ros+usb+F4&spm=1018.2226.3001.4187
https://blog.csdn.net/qq_36349536/article/details/82773064?ops_request_misc=&request_id=&biz_id=102&utm_term=ros%20usb%20F4&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-5-.nonecase&spm=1018.2226.3001.4187
https://blog.csdn.net/leida_wt/article/details/109690562?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162410679616780262524666%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=162410679616780262524666&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-7-109690562.pc_search_result_before_js&utm_term=stm32+ros+&spm=1018.2226.3001.4187

