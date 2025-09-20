# RODSEM Pix Driver 使用指南
---
需要用到一个以太网转CAN的盒子，配置需要在windows的上位机下进行。假设我们已经做好了can盒的配置，现在需要在ubuntu下进行配置。
**以太网转CAN：**
- `ip： 192.168.100.110 `（打开右上角设置，对有线网的IP v4进行配置）
![](/images/ethernet_config.png)
- CAN1：（Windows上位机进行的配置）
    - 监听端口： 4001
    - 发送端口： 8001
    - 模式：UDP Client
    

**主机：**
- ip： 192.168.100.10
- 监听端口： 8001 (和上面反过来)
- 发送端口： 4001
- 模式：UDP Client （一样的模式）


