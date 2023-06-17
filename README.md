#### 文件所有

本文适用于KF32A156的 USART 配置的示例例程，例程文件名：KF32A156_USART_ASYNC_INTERRUPT

* * *





#### 版权说明

目前的固件只是为了给使用者提供指导，目的是向客户提供有关产品的代码信息，

与使用者的产品信息和代码无关。因此，对于因此类固件内容和（或）客户使用此

处包含的与其产品相关的编码信息而引起的任何索赔，上海芯旺微电子技术有限公

司不承担任何直接、间接或后果性损害赔偿责任

* * *



#### 使用说明

* 本例展示了如何配置串口中断异步发送，使用USART0和LED1/LED2/LED3/LED4进行配合演示

* 上电后LED1/LED2/LED3/LED4点亮，同时串口助手接收到“ChipOn\r\n”字符串，该效果代表代码开始运行

* 使用KF32A156 MINI开发板的用户通过串口助手发送数据，串口会同时输出所接受到的数据

* 串口每接收到一次数据，LED1/LED2/LED3/LED4 翻转一次，并将接收到的数据和一串“ChipOn\r\n”字符发送至串口助手


* * *

