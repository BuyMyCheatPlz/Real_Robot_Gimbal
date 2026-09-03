# 重载步兵云台 PID 整定资源

## 参考资料

- [TI AN-706: LM628/629 User Guide](https://www.ti.com/lit/pdf/snoa184)
  用于理解阶跃响应中的上升、振铃、稳定时间，以及先调 Kp/Kd、后加 Ki 的经验整定顺序。
- [TI SLAA503: Sensored 3-Phase BLDC Motor Control](https://edgeworker.ti.com/lit/pdf/slaa503)
  用于理解电机速度闭环中速度反馈、PI 输出与阶跃记录之间的关系。
- [RoboMaster 官方资源中心](https://www.robomaster.com/en-US/robo/rm?djifrom=nav)
  用于获取当前赛季的规则、制作规范和参赛资料。

## 社区经验

- RoboMaster 官方论坛与资源中心
  用于核对赛季规则、接口和工程实践；涉及安全或赛规的问题优先以官方最新公告为准。

## 当前缺口

- 当前工程缺少记录“目标角度、实际角度、速度环输出和前馈输出”的同一时间轴数据。完成首轮参数后，应补充该遥测再做定量复盘。
