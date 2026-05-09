# 四路循迹机器人项目 Code Wiki

## 目录
1. [项目概述](#项目概述)
2. [项目架构](#项目架构)
3. [硬件平台](#硬件平台)
4. [模块说明](#模块说明)
5. [传感器系统详解](#传感器系统详解)
6. [关键函数说明](#关键函数说明)
7. [依赖关系](#依赖关系)
8. [项目运行方式](#项目运行方式)
9. [命令协议](#命令协议)

---

## 项目概述

本项目是一个基于STM32F103C8T6的四路循迹智能机器人控制系统，集成了多种传感器和智能功能模块，支持舵机控制、电机驱动、PS2手柄遥控、串口通信等功能。

### 主要功能特性
- **舵机控制**：支持6自由度机械臂控制（0-5号舵机）
- **电机驱动**：支持左右轮电机PWM控制（6-7号总线舵机/电机）
- **传感器系统**：四路循迹、超声波测距、颜色识别、声音检测
- **遥控方式**：PS2手柄、串口命令、蓝牙/WiFi
- **智能模式**：10种AI工作模式（循迹、避障、跟随、颜色识别等）
- **动作组存储**：支持W25Q64闪存存储动作组
- **逆运动学**：支持机械臂坐标控制

---

## 项目架构

### 目录结构
```
四路源代码/
├── CMSIS/                    # ARM Cortex-M3内核支持
│   ├── CM3/CoreSupport/      # 核心支持文件
│   └── CM3/DeviceSupport/    # 设备支持文件(STM32F10x)
├── Libraries/                # STM32标准库
│   ├── inc/                  # 库头文件
│   ├── src/                  # 库源文件
│   └── Startup/              # 启动文件
├── src/                      # 项目源代码
│   ├── z_main.c/h            # 主程序（新版）
│   ├── z_sensor.c/h          # 传感器模块
│   ├── z_color.c/h           # 颜色传感器驱动
│   ├── z_gpio.c/h            # GPIO配置
│   ├── z_timer.c/h           # 定时器配置
│   ├── z_usart.c/h           # 串口通信
│   ├── z_adc.c/h             # ADC采集
│   ├── z_global.c/h          # 全局变量
│   ├── z_delay.c/h           # 延时函数
│   ├── z_rcc.c/h             # 时钟配置
│   ├── z_ps2.c/h             # PS2手柄驱动
│   ├── z_w25q64.c/h          # W25Q64存储芯片
│   ├── z_kinematics.c/h      # 逆运动学算法
│   ├── z_oled_i2c.c/h        # OLED显示
│   └── z_type.h              # 类型定义
├── USER/                     # 用户代码（旧版）
│   └── main.c                # 旧版主程序
├── OUT/                      # 编译输出
├── LIST/                     # 链接文件
└── Project/                  # Keil工程文件
```

### 架构层次
```
┌─────────────────────────────────────────────────────────────┐
│                      应用层 (Application)                    │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │
│  │ 循迹模式 │ │ 避障模式 │ │ 颜色识别 │ │ 跟随模式 │        │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘        │
├─────────────────────────────────────────────────────────────┤
│                      控制层 (Control)                        │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │
│  │ 动作执行 │ │ 命令解析 │ │ 手柄处理 │ │ 运动学解算│        │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘        │
├─────────────────────────────────────────────────────────────┤
│                      驱动层 (Driver)                         │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │
│  │ 传感器驱动│ │ 舵机驱动 │ │ 电机驱动 │ │ 通信驱动 │        │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘        │
├─────────────────────────────────────────────────────────────┤
│                      硬件层 (Hardware)                       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │
│  │   GPIO   │ │  定时器  │ │   ADC    │ │   DMA    │        │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## 硬件平台

### 主控芯片
- **型号**：STM32F103C8T6
- **主频**：72MHz
- **Flash**：64KB
- **RAM**：20KB

### 引脚分配

#### 传感器引脚
| 传感器 | 引脚 | 说明 |
|--------|------|------|
| 循迹X1 | PA1 | 四路循迹传感器1 |
| 循迹X2 | PA0 | 四路循迹传感器2 |
| 循迹X3 | PA3 | 四路循迹传感器3 |
| 循迹X4 | PB1 | 四路循迹传感器4 |
| 超声波Trig | PB0 | 超声波触发信号 |
| 超声波Echo | PA2 | 超声波回波信号 |
| 声音传感器 | PA6 | 声音检测输入 |
| 颜色SCL | PA7 | TCS34725 I2C时钟 |
| 颜色SDA | PA5 | TCS34725 I2C数据 |

#### 舵机/电机引脚
| 设备 | 引脚 | 说明 |
|------|------|------|
| 舵机0 | PB3 | 机械臂底座 |
| 舵机1 | PB8 | 机械臂大臂 |
| 舵机2 | PB9 | 机械臂小臂 |
| 舵机3 | PB6 | 机械臂手腕 |
| 舵机4 | PB7 | 机械臂旋转 |
| 舵机5 | PB4 | 机械臂夹爪 |
| 电机左 | UART3总线 | 左轮电机 |
| 电机右 | UART3总线 | 右轮电机 |

#### 其他引脚
| 功能 | 引脚 | 说明 |
|------|------|------|
| 蜂鸣器 | PB5 | 蜂鸣器控制 |
| 工作LED | PB13 | 状态指示灯 |
| PS2_DAT | PA15 | PS2手柄数据 |
| PS2_CMD | PA14 | PS2手柄命令 |
| PS2_ATT | PA13 | PS2手柄片选 |
| PS2_CLK | PA12 | PS2手柄时钟 |
| 按键1 | PA8 | 功能按键 |
| 按键2 | PA11 | 功能按键 |

---

## 模块说明

### 1. 传感器模块 (z_sensor.c/h)

#### 文件位置
- [z_sensor.h](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.h)
- [z_sensor.c](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c)

#### 功能概述
传感器模块负责所有外部传感器的初始化和数据处理，包括四路循迹传感器、超声波传感器、颜色识别传感器和声音传感器。

#### 宏定义
```c
// 循迹传感器状态
#define XJ_ON  0   // 检测到黑线（低电平）
#define XJ_OFF 1   // 未检测到黑线（高电平）

// 循迹传感器读取宏
#define x1() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)  // 循迹传感器1
#define x2() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)  // 循迹传感器2
#define x3() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)  // 循迹传感器3
#define x4() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)  // 循迹传感器4

// 超声波传感器控制宏
#define Trig(x) gpioB_pin_set(0, x)                    // 超声波触发
#define Echo()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) // 超声波回波

// 声音传感器读取宏
#define sound() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) // 声音检测

// 颜色识别LED控制
#define YSSB_LED(x) TCS34725_LedON(x)
```

#### 关键函数

##### 初始化函数
| 函数名 | 功能 | 位置 |
|--------|------|------|
| `setup_sensor()` | 初始化所有传感器 | z_sensor.c:76 |
| `setup_xunji()` | 初始化四路循迹传感器IO | z_sensor.c:24 |
| `setup_csb()` | 初始化超声波传感器IO | z_sensor.c:39 |
| `setup_sound()` | 初始化声音传感器IO | z_sensor.c:58 |
| `setup_yssb()` | 初始化颜色识别传感器 | z_sensor.c:69 |

##### 数据采集函数
| 函数名 | 功能 | 位置 | 返回值 |
|--------|------|------|--------|
| `get_csb_value()` | 获取超声波测距值 | z_sensor.c:157 | u16 (距离，单位cm) |
| `get_adc_csb_middle()` | 获取超声波中值滤波数据 | z_sensor.c:184 | int (距离) |
| `get_adc_yssb_middle()` | 获取颜色传感器RGB数据 | z_sensor.c:133 | int |

##### AI模式函数
| 函数名 | 功能 | 位置 |
|--------|------|------|
| `loop_sensor()` | 传感器主循环，根据AI_mode分发 | z_sensor.c:86 |
| `AI_xunji_moshi()` | 循迹模式 | z_sensor.c:212 |
| `AI_xunji_bizhang()` | 循迹避障模式 | z_sensor.c:303 |
| `AI_gensui_moshi()` | 超声波跟随模式 | z_sensor.c:336 |
| `AI_ziyou_bizhang()` | 自由避障模式 | z_sensor.c:359 |
| `AI_dingju_jiaqu()` | 定距夹取模式 | z_sensor.c:402 |
| `AI_shengkong_jiaqu()` | 声控夹取模式 | z_sensor.c:422 |
| `AI_yanse_shibie()` | 颜色识别模式 | z_sensor.c:445 |
| `AI_xunji_dingju()` | 循迹定距模式 | z_sensor.c:483 |
| `AI_xunji_shibie()` | 循迹识别模式 | z_sensor.c:500 |
| `AI_shengkong_xunji()` | 声控循迹模式 | z_sensor.c:517 |

---

### 2. 颜色传感器模块 (z_color.c/h)

#### 文件位置
- [z_color.h](file:///c:/Users/Suma/Desktop/四路源代码/src/z_color.h)
- [z_color.c](file:///c:/Users/Suma/Desktop/四路源代码/src/z_color.c)

#### 功能概述
基于TCS34725颜色传感器的驱动模块，通过软件模拟I2C通信，支持RGBC数据采集和HSL颜色空间转换。

#### 数据结构
```c
// RGBC颜色数据结构
typedef struct{
    unsigned short c;  // 清色通道 [0-65536]
    unsigned short r;  // 红色通道
    unsigned short g;  // 绿色通道
    unsigned short b;  // 蓝色通道
} COLOR_RGBC;

// HSL颜色数据结构
typedef struct{
    unsigned short h;  // 色调 [0,360]
    unsigned char  s;  // 饱和度 [0,100]
    unsigned char  l;  // 亮度 [0,100]
} COLOR_HSL;
```

#### 关键函数
| 函数名 | 功能 | 位置 | 参数/返回值 |
|--------|------|------|-------------|
| `TCS34725_Init()` | 初始化颜色传感器 | z_color.c:291 | time: 积分时间, 返回: ID值 |
| `TCS34725_GetRawData()` | 获取RGBC原始数据 | z_color.c:327 | rgbc: 数据结构指针 |
| `TCS34725_GetR()` | 获取红色通道值 | z_color.c:344 | 返回: 红色值 |
| `TCS34725_GetG()` | 获取绿色通道值 | z_color.c:356 | 返回: 绿色值 |
| `TCS34725_GetB()` | 获取蓝色通道值 | z_color.c:368 | 返回: 蓝色值 |
| `TCS34725_GetC()` | 获取清色通道值 | z_color.c:380 | 返回: 清色值 |
| `TCS34725_LedON()` | 控制传感器LED | z_color.c:275 | enable: 0/1 |
| `RGBtoHSL()` | RGB转HSL颜色空间 | z_color.c:395 | Rgb: 输入, Hsl: 输出 |

#### I2C寄存器定义
```c
#define TCS34725_ADDRESS          (0x29)     // I2C地址
#define TCS34725_ID               (0x12)     // ID寄存器
#define TCS34725_RDATAL           (0x16)     // 红色数据低字节
#define TCS34725_GDATAL           (0x18)     // 绿色数据低字节
#define TCS34725_BDATAL           (0x1A)     // 蓝色数据低字节
#define TCS34725_CDATAL           (0x14)     // 清色数据低字节
```

---

### 3. 主程序模块 (z_main.c)

#### 文件位置
- [z_main.c](file:///c:/Users/Suma/Desktop/四路源代码/src/z_main.c)
- [z_main.h](file:///c:/Users/Suma/Desktop/四路源代码/src/z_main.h)

#### 功能概述
新版主程序，负责系统初始化、主循环调度和命令解析。

#### 主循环流程
```c
while(1) {
    loop_nled();        // 工作指示灯闪烁
    loop_uart();        // 串口数据处理
    loop_action();      // 动作组批量执行
    loop_ps2_data();    // PS2手柄数据读取
    loop_ps2_button();  // PS2按钮处理
    loop_ps2_car();     // PS2摇杆控制小车
    loop_monitor();     // 定时保存变量
    loop_sensor();      // 传感器AI模式处理
}
```

#### 关键函数
| 函数名 | 功能 | 位置 |
|--------|------|------|
| `main()` | 主函数入口 | z_main.c:144 |
| `parse_cmd()` | 命令解析函数 | z_main.c:679 |
| `parse_action()` | 动作解析执行 | z_main.c:935 |
| `save_action()` | 动作组保存 | z_main.c:796 |
| `do_group_once()` | 执行单次动作组 | z_main.c:916 |
| `car_set()` | 小车速度设置 | z_main.c:544 |
| `car_move()` | 麦轮小车控制 | z_main.c:560 |
| `kinematics_move()` | 逆运动学移动 | z_main.c:1064 |

---

### 4. 旧版主程序 (USER/main.c)

#### 文件位置
- [main.c](file:///c:/Users/Suma/Desktop/四路源代码/USER/main.c)

#### 功能概述
旧版主程序，功能与新版类似但结构不同，包含更多预定义的动作组。

#### 预定义动作组
- G0000: 偏差调节组
- G0001: 直立姿势
- G0002: 蜷缩姿势
- G0003-G0011: 大前抓右放动作序列
- G0012-G0018: 前爪前放动作序列
- G0019-G0027: 前爪左放动作序列
- G0028-G0036: 前爪右放动作序列

---

### 5. GPIO模块 (z_gpio.c/h)

#### 文件位置
- [z_gpio.h](file:///c:/Users/Suma/Desktop/四路源代码/src/z_gpio.h)

#### 位带操作宏
```c
// GPIO输出控制
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)

// GPIO输入读取
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)
```

#### 便捷操作宏
```c
#define beep_on()   {gpioB_pin_set(5, 1);}
#define beep_off()  {gpioB_pin_set(5, 0);}
#define nled_on()   {gpioB_pin_set(13, 0);}
#define nled_off()  {gpioB_pin_set(13, 1);}

// 传感器读取
#define sensor_pa0()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define sensor_pa1()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)
#define sensor_pb11() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)
```

---

### 6. 全局变量模块 (z_global.c/h)

#### 文件位置
- [z_global.h](file:///c:/Users/Suma/Desktop/四路源代码/src/z_global.h)

#### 关键数据结构
```c
// 舵机控制结构体
typedef struct {
    uint8_t  valid;    // 有效标志
    uint16_t aim;      // 目标位置
    uint16_t time;     // 执行时间
    float    cur;      // 当前位置
    float    inc;      // 每步增量
} duoji_t;

// EEPROM信息结构体
typedef struct {
    u32 version;              // 版本号
    u32 dj_record_num;        // 学习动作组数量
    u8  pre_cmd[PRE_CMD_SIZE + 1];  // 预存命令
    int dj_bias_pwm[DJ_NUM+1];      // 舵机偏差
    u8  color_base_flag;      // 颜色基准标志
    int color_red_base;       // 红色基准
    int color_grn_base;       // 绿色基准
    int color_blu_base;       // 蓝色基准
} eeprom_info_t;
```

#### 全局变量
| 变量名 | 类型 | 说明 |
|--------|------|------|
| `AI_mode` | u8 | 当前AI模式 (0-10) |
| `group_do_ok` | u8 | 动作组执行完成标志 |
| `duoji_doing[DJ_NUM]` | duoji_t | 舵机控制数组 |
| `eeprom_info` | eeprom_info_t | EEPROM配置信息 |
| `uart_receive_buf` | u8[] | 串口接收缓冲区 |
| `cmd_return` | u8[] | 命令返回缓冲区 |

---

## 传感器系统详解

### 四路循迹传感器

#### 工作原理
- 检测到黑线时，传感器指示灯亮，输出低电平(0)
- 未检测到黑线时，传感器指示灯灭，输出高电平(1)

#### 传感器布局
```
    小车前进方向
         ↑
    [X1] [X2] [X3] [X4]
     ↑    ↑    ↑    ↑
    左1  左2  右2  右1
```

#### 循迹逻辑 (AI_xunji_moshi)
```c
// 状态定义
#define TRACKING_STATUS_FORWARD    1  // 直行
#define TRACKING_STATUS_TURN_LEFT  2  // 左转
#define TRACKING_STATUS_TURN_RIGHT 3  // 右转
#define TRACKING_STATUS_STOP       4  // 停止

// 判断逻辑
if (x4 == 0 || (x2 == 1 && x3 == 0)) {
    // 右边出去，左转
    s_tracking_status = TRACKING_STATUS_TURN_LEFT;
} else if (x1 == 0 || (x2 == 0 && x3 == 1)) {
    // 左边出去，右转
    s_tracking_status = TRACKING_STATUS_TURN_RIGHT;
} else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1) {
    // 全白，停止
    s_tracking_status = TRACKING_STATUS_STOP;
} else if (x2 == 0 && x3 == 0) {
    // 中间检测到黑线，直行
    s_tracking_status = TRACKING_STATUS_FORWARD;
}
```

### 超声波传感器

#### 测距原理
1. Trig引脚发送20us高电平触发
2. 等待Echo引脚变高，启动定时器计数
3. 等待Echo引脚变低，停止定时器
4. 计算距离：距离 = 计数值 × 0.017 (cm)

#### 计算公式
```c
// 声速340m/s = 0.034cm/us
// 往返距离，所以除以2
// 距离(cm) = 时间(us) × 0.034 / 2 = 时间(us) × 0.017
csb_t = TIM_GetCounter(TIM3) * 0.017;
```

#### 关键代码
```c
u16 get_csb_value(void) {
    u16 csb_t;
    Trig(1);                    // 触发
    csb_Delay_Us(20);
    Trig(0);
    while(Echo() == 0);         // 等待高电平
    TIM_SetCounter(TIM3, 0);    // 清零计数器
    TIM_Cmd(TIM3, ENABLE);      // 启动定时器
    while(Echo() == 1);         // 等待低电平
    TIM_Cmd(TIM3, DISABLE);     // 停止定时器
    csb_t = TIM_GetCounter(TIM3) * 0.017;  // 计算距离
    return csb_t;
}
```

### 颜色识别传感器 (TCS34725)

#### 传感器特性
- **型号**：TCS34725
- **接口**：I2C (软件模拟)
- **数据格式**：16位RGBC
- **积分时间**：可配置 (2.4ms - 700ms)

#### 工作流程
1. 初始化I2C接口
2. 配置积分时间和增益
3. 使能传感器
4. 读取RGBC数据
5. 转换为HSL颜色空间进行识别

#### 颜色识别逻辑
```c
void AI_yanse_shibie() {
    TCS34725_GetRawData(&color_rgbc);  // 获取RGB数据
    YSSB_LED(0);  // 关闭LED
    
    if (color_rgbc.c < 1) {  // 检测到物体
        YSSB_LED(1);  // 打开LED
        tb_delay_ms(800);
        TCS34725_GetRawData(&color_rgbc);  // 再次读取
        
        if (color_rgbc.r > color_rgbc.g && color_rgbc.r > color_rgbc.b) {
            // 识别为红色
            parse_cmd("$DGT:12-18,1!");
        } else if (color_rgbc.g > color_rgbc.r && color_rgbc.g > color_rgbc.b) {
            // 识别为绿色
            parse_cmd("$DGT:19-27,1!");
        } else if (color_rgbc.b > color_rgbc.g && color_rgbc.b > color_rgbc.r) {
            // 识别为蓝色
            parse_cmd("$DGT:28-36,1!");
        }
    }
}
```

---

## 关键函数说明

### 传感器初始化函数

#### setup_sensor()
```c
void setup_sensor(void)
```
- **功能**：初始化所有传感器
- **位置**：[z_sensor.c:76](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L76)
- **调用**：依次调用setup_xunji()、setup_sound()、setup_csb()、setup_yssb()

#### setup_xunji()
```c
void setup_xunji(void)
```
- **功能**：初始化四路循迹传感器IO
- **位置**：[z_sensor.c:24](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L24)
- **配置**：PA0、PA1、PA3、PB1为上拉输入

#### setup_csb()
```c
void setup_csb()
```
- **功能**：初始化超声波传感器
- **位置**：[z_sensor.c:39](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L39)
- **配置**：PB0为推挽输出(Trig)，PA2为浮空输入(Echo)

### 数据采集函数

#### get_csb_value()
```c
u16 get_csb_value(void)
```
- **功能**：采集超声波数据
- **位置**：[z_sensor.c:157](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L157)
- **返回**：距离值，单位cm，范围0-250
- **说明**：使用TIM3计时，分辨率1us

#### get_adc_csb_middle()
```c
int get_adc_csb_middle()
```
- **功能**：获取超声波中值滤波数据
- **位置**：[z_sensor.c:184](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L184)
- **返回**：经过中值滤波的距离值
- **说明**：采集5次，排序后取中间值

### AI模式函数

#### AI_xunji_moshi()
```c
void AI_xunji_moshi(void)
```
- **功能**：四路循迹模式
- **位置**：[z_sensor.c:212](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L212)
- **逻辑**：根据四路传感器状态控制小车转向
- **速度**：直行11，转弯±15

#### AI_yanse_shibie()
```c
void AI_yanse_shibie()
```
- **功能**：颜色识别夹取
- **位置**：[z_sensor.c:445](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L445)
- **逻辑**：检测物体→识别颜色→执行对应动作组

#### AI_gensui_moshi()
```c
void AI_gensui_moshi(void)
```
- **功能**：超声波跟随
- **位置**：[z_sensor.c:336](file:///c:/Users/Suma/Desktop/四路源代码/src/z_sensor.c#L336)
- **逻辑**：距离30-50cm前进，<20cm后退，其他停止

---

## 依赖关系

### 头文件依赖图
```
z_main.h
├── z_rcc.h
├── z_gpio.h
│   └── stm32f10x_conf.h
├── z_global.h
│   └── stm32f10x_conf.h
├── z_delay.h
├── z_type.h
│   └── stm32f10x.h
├── z_usart.h
├── z_timer.h
│   └── stm32f10x.h
├── z_ps2.h
├── z_w25q64.h
├── z_sensor.h
│   ├── stm32f10x_conf.h
│   └── z_color.h
│       └── stm32f10x_conf.h
├── z_adc.h
└── z_kinematics.h
```

### 模块依赖关系
```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   z_main    │────→│  z_sensor   │────→│   z_color   │
└──────┬──────┘     └──────┬──────┘     └─────────────┘
       │                   │
       │            ┌──────┴──────┐
       │            │             │
       ↓            ↓             ↓
┌─────────────┐ ┌─────────┐ ┌─────────┐
│  z_kinematics│ │ z_timer │ │ z_gpio  │
└─────────────┘ └────┬────┘ └────┬────┘
                     │           │
                     ↓           ↓
              ┌─────────────┐ ┌─────────┐
              │   z_usart   │ │ z_delay │
              └──────┬──────┘ └─────────┘
                     │
                     ↓
              ┌─────────────┐
              │  Libraries  │
              │  (STM32 Std)│
              └─────────────┘
```

---

## 项目运行方式

### 开发环境
- **IDE**：Keil MDK-ARM
- **编译器**：ARMCC
- **调试器**：ST-Link/J-Link

### 编译步骤
1. 打开 `Project/Kibot1-32.uvprojx` 工程文件
2. 选择目标配置（Target 1）
3. 点击 Build 按钮编译
4. 生成的 hex 文件位于 `OUT/Kibot1-32.hex`

### 烧录方式
1. **USB一键下载**：使用FlyMcu.exe工具
2. **ST-Link**：通过Keil直接下载调试
3. **串口下载**：通过USART1使用Bootloader

### 启动流程
```
1. 上电复位
2. 执行startup_stm32f10x_hd.s启动文件
3. 调用SystemInit()配置时钟
4. 进入main()函数
5. 依次执行各模块初始化
6. 进入主循环
```

### 初始化顺序
```c
setup_rcc();        // 1. 初始化时钟
setup_global();     // 2. 初始化全局变量
setup_gpio();       // 3. 初始化GPIO
setup_nled();       // 4. 初始化工作指示灯
setup_beep();       // 5. 初始化蜂鸣器
setup_djio();       // 6. 初始化舵机IO
setup_w25q64();     // 7. 初始化存储器
setup_ps2();        // 8. 初始化PS2手柄
setup_systick();    // 9. 初始化滴答时钟
setup_uart1();      // 10. 初始化串口1
setup_uart3();      // 11. 初始化串口3
setup_others();     // 12. 初始化其他
setup_dj_timer();   // 13. 初始化舵机定时器
setup_start();      // 14. 启动信号
setup_interrupt();  // 15. 开启总中断
setup_sensor();     // 16. 初始化传感器
```

---

## 命令协议

### 命令格式
所有命令以 `$` 开头，以 `!` 结尾。

### 系统命令
| 命令 | 功能 |
|------|------|
| `$DRS!` | 测试应答 |
| `$DST!` | 所有舵机停止 |
| `$DST:x!` | 第x个舵机停止 |
| `$RST!` | 软件复位 |
| `$DJR!` | 所有舵机复位到1500 |
| `$GETA!` | 获取应答信号 |

### 动作组命令
| 命令 | 功能 | 示例 |
|------|------|------|
| `$DGS:x!` | 执行第x个动作 | `$DGS:1!` |
| `$DGT:x-y,z!` | 执行x到y动作组z次 | `$DGT:1-5,3!` |
| `$PTG:x-y!` | 打印x到y动作组 | `$PTG:0-10!` |

### 小车控制命令
| 命令 | 功能 | 示例 |
|------|------|------|
| `$DCR:x,y!` | 设置左右轮速度 | `$DCR:10,-10!` |
| `$CAR_STOP!` | 小车停止 | - |

### 智能模式命令
| 命令 | 功能 |
|------|------|
| `$SMODE0!` | 循迹模式 |
| `$SMODE1!` | 声控夹取 |
| `$SMODE2!` | 自由避障 |
| `$SMODE3!` | 颜色识别 |
| `$SMODE4!` | 定距夹取 |
| `$SMODE5!` | 跟随功能 |
| `$SMODE6!` | 循迹避障 |
| `$SMODE7!` | 循迹识别 |
| `$SMODE8!` | 循迹定距 |
| `$SMODE9!` | 声控循迹 |
| `$SMART_STOP!` | 停止智能模式 |

### 运动学命令
| 命令 | 功能 | 示例 |
|------|------|------|
| `$KMS:x,y,z,time!` | 坐标控制 | `$KMS:0,120,20,2000!` |

### 动作格式
```
#000P1500T1000!
│  │ │    │
│  │ │    └── 时间(毫秒)
│  │ └─────── 脉宽值(500-2500)
│  └───────── 舵机编号(000-255)
└──────────── 命令起始符
```

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 20180705 | 2018-07-05 | 基础版本 |
| 20200523 | 2022-05-23 | 新增四路循迹、运动学控制 |

---

## 作者信息

- **开发者**：XSJ / tacbo
- **公司**：杭州众灵科技有限公司 / 星甲智能
- **创建日期**：2017-11-08
- **最后更新**：2022-05-23
