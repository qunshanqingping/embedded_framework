# DBUS 遥控器协议解码模块说明文档

## 概述

DBUS 模块用于解码 DJI DT7/DR16 遥控器的数据协议，将原始数据流解析为可用的控制信息。

## 主要功能

### 1. 遥控器数据解码
- **函数**: [Remote_Ctrl_Dbus_Decode](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L113-L113)
- **功能**: 将 DR16 接收器的原始数据缓冲区解码为遥控器控制结构体

```c
void Remote_Ctrl_Dbus_Decode(volatile const uint8_t *dbus_buf, RemoteCtrlInfo_s *remote_ctrl_data)
```


#### 解码内容包括:
- 4个遥控器通道数据 (ch[0]-ch[3])
- 拨轮数据 (wheel)
- 2个开关位置 (s[0]-s[1])
- 鼠标坐标和按键状态
- 键盘状态数据

### 2. UART接收回调处理
- **函数**: [Dbus_RxCallback](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L131-L131)
- **功能**: UART DMA接收完成回调函数，实现双缓冲切换机制

#### 特性:
- 双缓冲机制确保连续数据接收
- 自动切换内存缓冲区 (Memory 0 ↔ Memory 1)
- 接收数据验证 (要求18个字节)
- 调用解码函数处理接收到的数据

### 3. DBUS实例注册
- **函数**: [Dbus_Register](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L121-L121)
- **功能**: 注册并初始化一个新的DBUS实例

```c
DbusInstance_s *Dbus_Register(DbusConfig_s *config)
```


#### 初始化流程:
1. 参数验证
2. 内存分配
3. 内存初始化
4. UART回调函数设置
5. UART驱动注册

## 数据结构

### RemoteCtrlInfo_s
包含所有遥控器控制信息的结构体:
- [rc](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L33-L38): 遥控器通道和开关数据
- [mouse](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L44-L51): 鼠标状态数据
- [key](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L57-L79): 键盘状态数据

### DbusInstance_s
DBUS实例结构体，包含:
- UART实例指针
- 遥控器数据结构体

## 技术细节

### 通道数据解码
- 使用位操作从字节流中提取11位通道数据
- 所有通道数据减去中位值([DT7_CH_MEDIAN](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L20-L21))进行校准

### 双缓冲机制
- 利用DMA的双缓冲功能实现无中断数据接收
- 通过检查和设置`DMA_SxCR_CT`位切换缓冲区
- 每次接收完成后重置DMA计数器为36

## 使用注意事项

1. 确保输入数据缓冲区大小为18个字节
2. 需要正确配置UART和DMA参数
3. 键盘和鼠标状态解析可能需要额外实现
4. 使用前需调用[Dbus_Register](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h#L121-L121)初始化实例

## 依赖项

- [dbus.h](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\modules\remote_control\dbus\dbus.h): 模块头文件
- [basic_math.h](file://C:\Users\29568\Documents\GitHub\MidFeed_OmniInfantry\gimbal\MidFeed_OmniInfantry_Gimbal\User\sys\basic\math\basic_math.h): 基础数学运算
- `string.h`: 内存操作函数
- UART驱动模块