智能服务小车
这是一个基于鸿蒙系统的应用程序。

项目简介
本项目是一个鸿蒙应用，主要内部功能包括语音输入指令、识别指令信息、大模型处理信息、与Atlas 200I DK A2通信将信息发送给Atlas 200I DK A2。

开发环境
开发工具: DevEco Studio
下载地址: https://developer.huawei.com/consumer/cn/download/
使用说明
前提条件
已安装 DevEco Studio。
已配置好鸿蒙开发环境（包括签名等）。
拥有一台已绑定并开启调试模式的鸿蒙手机。
运行步骤
在 DevEco Studio 中打开 MyApplication 文件夹。
使用 USB 数据线连接电脑和鸿蒙手机。
等待设备识别成功。
点击工具栏上的 运行 (Run) 按钮，将应用安装到手机并启动。
代码文件说明
核心文件功能
启动流程: Start.ets, Index.ets 和 Layout.ets 三个文件协同工作，实现软件APP启动时的小车实体的图片展示和页面跳转。
主功能页:
Control.ets: APP的“控制”页面的相关代码。
Mine.ets: APP的“我的”页面的相关代码。
网络服务: http 目录下的代码封装了与大语言模型（LLM）API 的交互。