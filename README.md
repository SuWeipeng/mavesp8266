> 背景介绍：         
> WiFi 数传在飞控日常调试中使用非常方便，尤其是室内调试。       
> 今天 Sugar 解读一个非常棒的 WiFi 数传开源代码 mavesp8266。      
> WiFi 连接分为 TCP 和 UDP 两种，今天介绍的项目用 UDP。       
> 开发使用 platformio，使用 Arduino 库。         
> 在公众号后台回复 mavesp8266 获得源码地址。

与 mavlink 联系的地方
---
`/src/mavesp8266.h` 文件中有包含 mavlink 头文件
```c
#include <ardupilotmega/mavlink.h>
```

复位（恢复默认设置）
---
1、`main.cpp` 定义 GPIO02 为 2
```cpp
#define GPIO02  2
```
2、复位函数 `reset_interrupt()`
```cpp
//-- Reset all parameters whenever the reset gpio pin is active
void reset_interrupt(){
    Parameters.resetToDefaults();
    Parameters.saveAllToEeprom();
    ESP.reset();
}
```
3、启用 GPIO 中断
```cpp
#ifdef ENABLE_DEBUG
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    Serial1.begin(115200);
#else
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    pinMode(GPIO02, INPUT_PULLUP);
    attachInterrupt(GPIO02, reset_interrupt, FALLING);
#endif
```

默认 Wifi
---
> `mavesp8266_parameters.cpp` 中有默认 SSID 和密码。

```cpp
const char* kDEFAULT_SSID       = "PixRacer";
const char* kDEFAULT_PASSWORD   = "pixracer";
```

192.168.4.1
---
> 这个获取 AP 默认 IP 的函数是 `softAPIP()`

`main.cpp` 的 `setup()` 中这样写：
```cpp
    if(Parameters.getWifiMode() == WIFI_MODE_AP){
        //-- Start AP
        WiFi.mode(WIFI_AP);
        WiFi.encryptionType(AUTH_WPA2_PSK);
        WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
        localIP = WiFi.softAPIP();
        wait_for_client();
    }
```
这个 `softAPIP()` 函数来源于 Arduino 库，定义在 `ESP8266WiFiAP.h` 中。

捕获 mavlink 消息
---
> 文件：`mavesp8266_component.cpp`      
> 函数：handleMessage()

![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/mavesp_01.png)    

```cpp
bool
MavESP8266Component::handleMessage(MavESP8266Bridge* sender, mavlink_message_t* message) {

  //
  //   TODO: These response messages need to be queued up and sent as part of the main loop and not all
  //   at once from here.
  //
  //-----------------------------------------------

  //-- MAVLINK_MSG_ID_PARAM_SET
  if(message->msgid == MAVLINK_MSG_ID_PARAM_SET) {
      ...
      }
  //-----------------------------------------------
  //-- MAVLINK_MSG_ID_COMMAND_LONG
  } else if(message->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
      ...
          }
      }
  //-----------------------------------------------
  //-- MAVLINK_MSG_ID_PARAM_REQUEST_LIST
  } else if(message->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
      ...
      }
  //-----------------------------------------------
  //-- MAVLINK_MSG_ID_PARAM_REQUEST_READ
  } else if(message->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
      ...
  }

  //-- Couldn't handle the message, pass on
  return false;
}
```

world
---
1、`mavesp8266.h`       
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/world_01.png)        
2、`main.cpp`        
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/world_02.png)        

对象粗略分析如下：     
对象名 | 作用
--- | ---
`localIP` | ESP8266 的 IPv4 地址。        
`Component` |与 MavESP8266Bridge 和 mavlink 相关内容。      
`Parameters` |这个 ESP8266 软件项目本身的一些参数。     
`GCS` | UDP 通信和 mavlink 相关代码。      
`Vehicle` | 串口通信和 mavlink 相关代码。     
`updateServer` | webServer 相关代码。      
`updateStatus` | 似乎没有太多实现，粗看是与通过网络刷新 ESP8266 固件相关。     
`Logger` | 用于从串口 1 打印一些调试信息。

UDP与串口的数据交换
---
先看一下 bridge 的 UML 示图：       
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/bridge_01.png)       
从 UML 示图中可以看出 MavESP8266Bridge 的“桥接 mavlink 消息”通过两个子类来实现。        
`MavESP8266GCS` 处理 UDP 端的 mavlink 消息。        
`MavESP8266Vehicle` 处理串口端的 mavlink 消息。     
  
`mavesp8266.h` 文件中定义 MavESP8266Bridge 类，如下：
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/bridge_02.png)      
> 可以看出类内定义大多是虚函数和纯虚函数，因此能知道此类的作用是 **“统一接口，形成基本架构”。**

`main.cpp` 中的 loop() 函数如下：
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/mavesp_loop_01.png)       
> 通过 loop() 函数可知研究数据交换的入手点是 `GCS.readMessage()` 和 `Vehicle.readMessage()`       

下面看一下 `mavesp8266_gcs.cpp` 下的 readMessage() 函数：
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/gcs_read_01.png)       
分析上图可知：
(1) 这里的 _readMessage() 是 MavESP8266GCS 类的私有函数（通常类的私有成员命名时以下划线开头），作用是从 UDP 接收 mavlink 消息。      
(2) 通过 `_forwardTo` 指针将接收到的消息发出去，是否发向串口要看 `_forwardTo` 的指向。

下面追踪一下这个 `_forwardTo`：     
(1) 从上面 `MavESP8266Bridge` 父类中发现 `_forwardTo` 是父类里的一个指向父类的指针。       
(2) 在父类中有给 `_forwardTo` 赋值的函数，如下：      
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/begin_01.png)       
(3) 看一下两个子类如何调用这个父类的 begin() 函数，如下：      
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/begin_02.png)         
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/begin_03.png)        
(3) 最后看一下 `main.cpp` 的 `setup()` 函数，查一下两个子类的 `begin()` 函数的参数是谁，如下：         
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/begin_04.png)       
> 通过上面的追踪可确定：UDP 端的 _forwardTo 指向串口，串口端的 _forwardTo 指向 UDP，这是完成消息传递的“桥梁”。所有 UDP 和串口间的数据交换都从这个“桥”上走。

mavlink 阻断
---
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/gcs_read_01.png)         
> 上图中在 `_readMessage()` 通过后将 mavlink 消息通过 `_forwardTo` 传到串口，因此 `_readMessage()` 从功能上能够通过其返回值决定是否让 mavlink “过桥”。      

如下图，不论是从串口还是 UDP 接收到的 mavlink 都会由 `Component` 捕获，从设计角度考虑将 `_readMessage()` 的返回值与 `handleMessage()` 进行关联是合理的。     
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/mavesp_01.png)         
看一下 `mavesp8266_component.cpp` 文件里 `handleMessage()` 函数里有下面一段代码：      
![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/mavesp_eat_01.png)        
注释说：
```cpp
//-- If this was addressed to me only eat message
if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
  //-- Eat message (don't send it to FC)
  return true;
}
```
可以确定是否阻断某条 mavlink 依 handleMessage() 的返回值而定。

mavlink 通道
---
> `_recv_chan` 和 `_send_chan` 是定义在父类 `MavESP8266Bridge` 中的 `mavlink_channel_t` 类型的变量。       
>       
> 这里有个疑问：两个子类分别在构造函数里对相同的父类变量赋值，其结果应该是冲突的（依对象出现的先后顺序，以后出现的为准），目前未专门测试（只测试过数据传输没问题）。

子类 `MavESP8266GCS` 和子类 `MavESP8266Vehicle` 对 _recv_chan 和 _send_chan 的赋值相反。
子类 | _recv_chan | _send_chan
---| --- | ---
MavESP8266GCS | MAVLINK_COMM_1 | MAVLINK_COMM_0
MavESP8266Vehicle | MAVLINK_COMM_0 | MAVLINK_COMM_1

依 `mavesp8266_gcs.cpp` 中 `_sendRadioStatus()` 函数，UDP 通过 _recv_chan 发送 radio_status（队列状态、丢包率等）。      

依 `mavesp8266_vehicle.cpp` 中 `_sendRadioStatus()` 函数，串口通过 _send_chan 发送 radion_status（队列状态、丢包率等）。

> radio_status 消息的 COMP_ID 是 240（MAV_COMP_ID_UDP_BRIDGE)

AP 模式与 STA 模式
---
> AP 模式：其他设备通过 ESP 模块的 ssid 和 password 连接 ESP 模块（相当于用 ESP 模块开放热点让别的设备连）。       
> STA 模式：ESP 模块通过路由器或手机热点的 ssid 和 password 连接网络。

![](https://github.com/SuWeipeng/img/raw/master/24_ESP8266/mavesp_mode_01.png)        

从上面程序看，可知：      
1、如果通过参数配置成 STA 模式，那么在上电后 1 分钟内没有连接上 WiFi 之后，则会自动切换到 AP 模式。        
2、在 STA 模式下有在 ESP 里配置过 IP、网关和子网掩码。       

> 在 STA 模式下，最好能在路由器或手机上将 ESP 的 MAC 和 IP 进行绑定，给 ESP 始终分配固定 IP 更好。

PS
---
mavesp8266 最开始是给 pixracer 飞控上的 esp01 或 esp01-1m 模块使用的。ardupilot 官方给出的固件就是 mavesp8266 项目的。       
众所周知 esp8266 模块有几种不同的“长相”，不仅仅是 esp01 一种样子。1MByte Flash 的配置成 esp01-1m，4MByte Flash 的配置成 esp07，不管长啥样，我们只根据 Flash 大小来确定 mavesp8266 项目的配置。        
Sugar 实测 UDP 很好用，没有遇到丢包问题。如果非得要用 TCP，可以不写本文这么复杂的代码，直接用 Arduino 的 API 把串口和 TCP 的数据互传一下就行（Sugar 写过样板代码，没几行，很容易）。         
ArduPilot 官方把 mavlink 1.0 和 mavlink 2.0 做成了两个固件（对应两版不同的源码），Sugar 为了方便在自己的麦轮车和飞机上用同一套代码，将 mavesp8266 项目做了一点小改动，按宏定义选协议版本。github 地址在公众号后台回 mavesp8266 可得。

关注作者
---
欢迎扫码关注我的公众号`MultiMCU EDU`。
![](https://github.com/SuWeipeng/img/raw/master/gongzonghao.jpg)
### `提示：在公众号“关于我”页面可加作者微信好友。`
### `喜欢本文求点赞，有打赏我会更有动力。`





