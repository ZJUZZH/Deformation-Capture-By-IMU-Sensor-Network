使用方法：

1、使用arduino IDE 进行程序烧录

2、主节点（master）使用官方arduino nano 33 ble（其他包含硬件IIC接口的Arduino均可），烧录Master程序；

3、子节点（slave）使用arduino atmega328p芯片，使用USBtinyISP下载器 + 3P双排1.25间距测试烧录夹烧录程序，Arduino IDE中开发板选项请安装MiniCore库，安装之后Clock选择external 8MHz，第一次使用先选择烧录Bootloader，正常烧录程序时选择“使用编程器上传”；

4、先烧录校准代码[Calibration_mpu6050toEEPROM.ino]至子节点中，将IMU放置在水平面进行校准，当子节点的LED进入快闪状态时表明校准完成，校准数据将自动保存至芯片EEPROM中。然后烧录[Slave.ino]程序即可；