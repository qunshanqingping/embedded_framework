# bsp_usb
## 配置说明
### 名词解释

以USB_OTG_HS为例子
 
USB是外设名称
 
OTG表示他既可以做USB HOST，也可以做USB DEVICE
USB Host是指驱动过后，板子就可以连接USB接口连接的器件，读取和转载数据了。

USB Device是指驱动过后，板子可以当作一个USB移动设备，通过USB连接到电脑上，类似U盘。

HS - High-Speed 高速模式 最大480Mbps
FS - Full-Speed 全速模式 最大12Mbps
LS - Low-Speed 低速模式 最大1.5Mbps

External HS Phy - 外置高速物理层
Internal FS Phy - 内置全速物理层
- Disable - 禁用PHY
- OTG/Dual_Role_Device - OTG/双角色设备
- Host_Only - 仅主机
- Device_Only - 仅设备
Activate_SOF - 激活SOF信号
Activate_VBUS - 激活VBUS感应