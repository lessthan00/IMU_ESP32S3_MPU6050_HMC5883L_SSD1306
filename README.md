# IMU_ESP32S3_MPU6050_HMC5883L_SSD1306

## 基于ESP32的IMU系统与OLED显示屏

## 简介

本项目实现了一个基于ESP32微控制器的惯性测量单元（IMU）系统，集成了MPU6050（6轴加速度计和陀螺仪）、HMC5883L（3轴磁力计）和SSD1306 OLED显示屏（128x64像素）。系统通过卡尔曼滤波融合传感器数据，实时计算并显示设备的俯仰（pitch）、滚转（roll）和偏航（yaw）角度，适用于机器人、无人机或可穿戴设备等需要精确姿态测量的场景。

## 硬件要求

### 元件清单

| 元件                     | 描述                                   | 备注                              |
|--------------------------|----------------------------------------|-----------------------------------|
| ESP32微控制器            | ESP32开发板（如DevKitC）或ESP32S3芯片 | ESP32S3需USB转串口烧录           |
| MPU6050传感器模块        | 6轴加速度计和陀螺仪                   | I2C通信，地址0x68或0x69          |
| HMC5883L传感器模块       | 3轴数字磁力计                         | I2C通信，地址0x1E                 |
| SSD1306 OLED显示屏       | 128x64像素                            | I2C通信                           |
| 触控按钮（4个）          | 用于电源、模式、冻结和预留功能         | 连接到GPIO 15、16、17、18         |
| 跳线                     | 用于连接元件                           |                                   |
| 面包板或PCB              | 用于安装元件                           |                                   |
| 电源                     | 若不使用带USB的开发板需单独电源        | 3.3V                              |
| USB转串口转换器          | 如CP2102或CH340，仅ESP32S3芯片需要     | 用于程序烧录                      |

### 引脚接线图

所有传感器和显示屏通过I2C接口连接到ESP32的GPIO 21（SDA）和GPIO 22（SCL）。按钮连接到指定GPIO引脚。以下是详细接线表：

| 元件         | 引脚 | ESP32连接 | 备注                     |
|--------------|------|-----------|--------------------------|
| MPU6050      | SDA  | GPIO 11   | I2C数据线                |
| MPU6050      | SCL  | GPIO 12   | I2C时钟线                |
| MPU6050      | VCC  | 3.3V      | 电源                     |
| MPU6050      | GND  | GND       | 接地                     |
| HMC5883L     | SDA  | GPIO 11   | I2C数据线                |
| HMC5883L     | SCL  | GPIO 12   | I2C时钟线                |
| HMC5883L     | VCC  | 3.3V      | 电源                     |
| HMC5883L     | GND  | GND       | 接地                     |
| SSD1306 OLED | SDA  | GPIO 11   | I2C数据线                |
| SSD1306 OLED | SCL  | GPIO 12   | I2C时钟线                |
| SSD1306 OLED | VCC  | 3.3V      | 电源                     |
| SSD1306 OLED | GND  | GND       | 接地                     |
| 电源按钮     | -    | GPIO 15   | 控制电源开关             |
| 模式按钮     | -    | GPIO 16   | 切换绝对/相对模式        |
| 冻结按钮     | -    | GPIO 17   | 冻结/解冻显示            |
| 预留按钮     | -    | GPIO 18   | 预留未来功能             |

**ESP32S3芯片额外接线**（若使用芯片而非开发板）：
- TX（ESP32S3）→ RX（USB转串口）
- RX（ESP32S3）→ TX（USB转串口）
- CH_PD（ESP32S3）→ 3.3V（USB转串口）
- EN（ESP32S3）→ 3.3V（USB转串口）
- GND（ESP32S3）→ GND（USB转串口）
- VCC（ESP32S3）→ 3.3V（USB转串口）

**注意**：所有I2C设备（MPU6050、HMC5883L、SSD1306）共享相同的SDA和SCL线。按钮连接需使用内部上拉电阻或外部上拉电阻。

## 软件结构

### 程序结构框图

软件基于FreeRTOS实现多任务处理，包含以下并发任务：
- **传感器任务**：以20 Hz频率从MPU6050和HMC5883L读取数据，发送到传感器队列。
- **处理任务**：从传感器队列接收数据，使用卡尔曼滤波计算姿态（俯仰、滚转、偏航），以5 Hz频率发送到姿态队列。
- **显示任务**：从姿态队列接收数据，以5 Hz频率更新OLED显示屏。
- **按钮任务**：监控按钮状态，处理电源开关、模式切换（绝对/相对）和显示冻结。
- **初始化与校准任务**：启动时初始化I2C接口、传感器，并执行校准。

**框图描述**：
- **输入**：MPU6050（加速度、角速度）、HMC5883L（磁场）
- **任务流程**：
  - 传感器任务 → 传感器队列 → 处理任务 → 姿态队列 → 显示任务
  - 按钮任务 → 处理用户输入
- **输出**：SSD1306 OLED显示屏
- **同步机制**：使用FreeRTOS队列（`sensor_queue`、`attitude_queue`）和信号量（`i2c_mutex`、`ssd1306_mutex`、`calibration_done_sem`）进行任务间通信和资源保护。

### 流程图

1. **启动**：
   - 初始化ESP32S3外设（I2C、GPIO）
   - 创建FreeRTOS任务和队列
2. **校准**：
   - 校准MPU6050（加速度计和陀螺仪，设备需静止）
   - 校准HMC5883L（磁力计，需缓慢旋转设备）
3. **主循环**：
   - 传感器任务：持续读取传感器数据并发送到队列
   - 处理任务：从队列接收数据，处理并发送到显示队列
   - 显示任务：从队列接收数据，更新显示屏
   - 按钮任务：持续监控按钮并处理事件
4. **关机**：
   - 长按电源按钮，暂停任务并关闭系统

## 传感器与显示屏简介

### MPU6050
MPU6050是一个6轴惯性测量单元，包含3轴加速度计（量程±4g）和3轴陀螺仪（量程±500 dps）。它通过I2C通信（地址0x68或0x69），用于测量设备的俯仰和滚转角度。更多信息可参考 [MPU6050教程](https://www.instructables.com/Connecting-MPU6050-With-ESP32/)。

### HMC5883L
HMC5883L是一个3轴数字磁力计，用于测量磁场以确定偏航（罗盘方向）。它通过I2C通信（地址0x1E），支持倾斜补偿以提高精度。更多信息可参考 [HMC5883L与ESP32接口](https://circuitdigest.com/microcontroller-projects/diy-smartwatch-using-esp32-part3-interfacing-magnetometer-and-gyroscope-with-esp32)。

### SSD1306 OLED显示屏
SSD1306是一个128x64像素的OLED显示屏，通过I2C通信，显示实时传感器数据，包括俯仰、滚转、偏航、加速度、陀螺仪和磁力计读数。支持绝对和相对模式，并可根据Z轴加速度自动翻转显示。更多信息可参考 [ESP32与SSD1306教程](https://randomnerdtutorials.com/esp32-ssd1306-oled-display-arduino-ide/)。

## 使用说明

### 硬件设置
1. 按照引脚接线图连接MPU6050、HMC5883L、SSD1306和按钮到ESP32S3。
2. 若使用ESP32S3芯片，连接USB转串口模块（TX、RX、CH_PD、EN、GND）。
3. 使用面包板或PCB固定元件，确保3.3V电源稳定。

### 程序烧录
1. **使用ESP32S3开发板**：
   - 通过USB连接开发板到计算机。
   - 使用ESP-IDF或Arduino IDE（需安装ESP32支持）上传`main.c`程序。
2. **使用ESP32S3芯片**：
   - 连接USB转串口模块（如CP2102或CH340）。
   - 使用ESP-IDF或Arduino IDE烧录程序，确保正确配置串口引脚。

### 操作
- **开机**：短按电源按钮（GPIO 15）。
- **关机**：长按电源按钮（>2秒）。
- **模式切换**：按模式按钮（GPIO 16）切换绝对/相对测量模式。
- **冻结显示**：按冻结按钮（GPIO 17）冻结/解冻显示。
- **预留功能**：预留按钮（GPIO 18）为未来功能保留。

## 调试与日志
- 系统使用ESP_LOG框架记录日志，标签为“mpu6050 test”和“SSD1306”。
- 定期记录任务堆栈使用情况以监控内存使用。
- 校准期间需保持设备静止（MPU6050）或缓慢旋转（HMC5883L）。

## 当前存在的问题

1. **数据更新速度慢**：
   - 传感器任务以20 Hz频率生成数据，而处理任务和显示任务以5 Hz频率消费数据，导致生产速度快于消费速度，可能造成`sensor_queue`和`attitude_queue`队列满溢。日志中会记录“传感器队列已满”或“姿态队列已满”的警告。
   - **潜在解决方案**：优化处理任务的计算效率（如简化卡尔曼滤波算法）或增加队列大小，同时调整任务调度优先级以平衡生产和消费速度。

2. **关机后无法正常开机**：
   - 系统运行一段时间后，关机（长按电源按钮）并再次短按电源按钮可能无法正常开机，表现为任务未正确恢复或系统未完全初始化。
   - **潜在原因**：可能是任务挂起/恢复逻辑问题，或电源管理状态未正确重置。需要检查`button_task`中的任务恢复逻辑和硬件电源管理电路。

## 未来展望

1. **添加可视化更好的仪表界面**：
   - 当前SSD1306显示屏以文本形式显示数据，未来计划开发图形化仪表界面（如指针式仪表或3D姿态可视化），以提高用户体验。
   - 实现方式可能包括使用SSD1306的图形库绘制动态仪表盘，或升级到更高分辨率的显示屏（如SSD1327或彩色TFT屏）以支持更复杂的可视化。
   - 需优化显示任务以确保图形渲染不影响实时性能。

## 物料

GY-521模块,MPU6050,接口定义VCC,GND,SCL,SDA,XDA,SCL,ADO,INT
GY-271模块,HMC5883L,接口定义VCC,GND,SCL,SDA,DRDY
轻触开关
0.96寸OLED SSD1306模块,接口定义VCC,GND,SCL,SDA
因为I2C总线的通讯拥挤,可以选择使用SPI通讯.
0.96村OLED SSD1306模块,7针接口定义 GND,VCC,D0,D1,RES,DC,CS
GND 电源地, VCC 3.3V~5V, D0, SPI时钟线(SPI_CLK), D1,SPI数据线(SPI_MOSI), RES,复位, DC, 数据/命令选择脚,CS,片选信号,低电平有效,CS不可悬空.
CH340G
ESP32-S3FN8
ESP32-S3-WROOM-1-N16R8

## 数据手册

[ESP32-S3-WROOM-1 技术规格书](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_cn.pdf)
N16->  16MB(QuadSPI) Flash
R8-> 8MB PSRAM
[ESP32-S3-WROOM-1 参考设计](https://www.espressif.com/sites/default/files/documentation/ESP32-S3-WROOM-1U_V1.4_Reference_Design.zip)
[SPI2 硬件接口](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32s3/api-reference/peripherals/spi_master.html)
[ESP32-S3](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_cn.pdf#cd-pins-io-mux-gpio)
管脚名称

## SPI2 管脚对应表

| 管脚名称  | GPIO编号 (SPI2) |
|----------|----------------|
| CS0      | 10             |
| SCLK     | 12             |
| MISO     | 13             |
| MOSI     | 11             |
| QUADWP   | 14             |
| QUADHD   | 9              |