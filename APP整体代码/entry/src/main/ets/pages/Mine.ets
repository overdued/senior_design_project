import socket from '@ohos.net.socket';
import wifiManager from '@ohos.wifiManager';
import util from '@ohos.util';
// ✅ 业务错误
import { BusinessError } from '@ohos.base';
// ✅ 导入 Control 页面的函数
import { getPendingMessage, setPendingMessage, getOnExecutionCompletedCallback } from './Control';

// 执行TCP通讯的对象
let tcpSocket = socket.constructTCPSocketInstance();

// 本地IP的数值形式
let ipNum = wifiManager.getIpInfo().ipAddress;
// 本地IP的字符串形式
let localIp = (ipNum >>> 24) + '.' + (ipNum >> 16 & 0xFF) + '.' + (ipNum >> 8 & 0xFF) + '.' + (ipNum & 0xFF);

@Entry
@Component
export struct Mine {
  // 连接、通讯历史记录
  @State msgHistory: string = '';
  // 要发送的信息
  @State sendMsg: string = '';
  // 本地端口
  @State localPort: number = 9990;

  // 服务端IP地址
  @State serverIp: string = '192.168.1.100';
  // 服务端端口
  @State serverPort: number = 5009;

  // 是否可以连接
  @State canConnect: boolean = false;
  // 是否可以发送消息
  @State canSend: boolean = false;
  scroller: Scroller = new Scroller();

  build() {
    Row() {
      Column() {
        Text('通讯')
          .fontSize(20)
          .fontWeight(FontWeight.Bold)
          .fontColor(Color.White)
          .width('100%')
          .textAlign(TextAlign.Center)
          .margin({ top: 40, bottom: 30 });

        Flex({ justifyContent: FlexAlign.SpaceBetween, alignItems: ItemAlign.Center }) {
          Text('本地端口：')
            .fontSize(16)
            .fontColor(Color.White)
            .width(100);

          TextInput({ text: this.localPort.toString() })
            .type(InputType.Number)
            .onChange((value) => {
              this.localPort = parseInt(value);
            })
            .width(150)
            .height(40)
            .fontSize(14)
            .fontColor(Color.White)
            .caretColor(Color.White)
            .backgroundColor(Color.Transparent)
            .border({ width: 1, color: Color.White, radius: 8 });

          Button('绑定')
            .onClick(() => {
              this.bind2Port();
            })
            .width(80)
            .height(40)
            .fontSize(14)
            .backgroundColor(Color.Blue)
            .fontColor(Color.White)
            .borderRadius(8);
        }
        .width('90%')
        .margin({ bottom: 20 });

        Flex({ justifyContent: FlexAlign.SpaceBetween, alignItems: ItemAlign.Center }) {
          Text('服务端地址：')
            .fontSize(16)
            .fontColor(Color.White)
            .width(100);

          TextInput({ text: this.serverIp })
            .onChange((value) => {
              this.serverIp = value;
            })
            .width(150)
            .height(40)
            .fontSize(14)
            .fontColor(Color.White)
            .caretColor(Color.White)
            .backgroundColor(Color.Transparent)
            .border({ width: 1, color: Color.White, radius: 8 });

          Text(':')
            .fontSize(16)
            .fontColor(Color.White)
            .width(20);

          TextInput({ text: this.serverPort.toString() })
            .type(InputType.Number)
            .onChange((value) => {
              this.serverPort = parseInt(value);
            })
            .width(80)
            .height(40)
            .fontSize(14)
            .fontColor(Color.White)
            .caretColor(Color.White)
            .backgroundColor(Color.Transparent)
            .border({ width: 1, color: Color.White, radius: 8 });

          Button('连接')
            .onClick(() => {
              this.connect2Server();
            })
            .enabled(this.canConnect)
            .width(80)
            .height(40)
            .fontSize(14)
            .backgroundColor(this.canConnect ? Color.Green : Color.Gray)
            .fontColor(Color.White)
            .borderRadius(8);
        }
        .width('90%')
        .margin({ bottom: 20 });

        Flex({ justifyContent: FlexAlign.SpaceBetween, alignItems: ItemAlign.Center }) {
          TextInput({ placeholder: '输入要发送的消息' })
            .onChange((value) => {
              this.sendMsg = value;
            })
            .placeholderColor(Color.Gray)
            .width('70%')
            .height(40)
            .fontSize(14)
            .fontColor(Color.White)
            .caretColor(Color.White)
            .backgroundColor(Color.Transparent)
            .border({ width: 1, color: Color.White, radius: 8 });

          Button('发送')
            .enabled(this.canSend)
            .width(80)
            .height(40)
            .fontSize(14)
            .backgroundColor(this.canSend ? Color.Orange : Color.Gray)
            .fontColor(Color.White)
            .borderRadius(8)
            .onClick(() => {
              this.sendMsg2Server();
            });
        }
        .width('90%')
        .margin({ bottom: 20 });

        Scroll(this.scroller) {
          Text(this.msgHistory)
            .textAlign(TextAlign.Start)
            .padding(10)
            .width('100%')
            .fontColor(Color.White)
            .backgroundColor(Color.Transparent);
        }
        .width('90%')
        .height(300)
        .border({ width: 1, color: Color.Gray, radius: 8 })
        .margin({ bottom: 20 });
      }
      .width('100%')
      .height('100%')
      .alignItems(HorizontalAlign.Center)
      .justifyContent(FlexAlign.Start);
    }
    .height('100%')
    .backgroundColor(Color.Black);
  }

  // ✅ 在组件中保存 interval 的引用
  private checkIntervalId: number | null = null;

  // ✅ 组件出现时订阅事件
  aboutToAppear() {
    // ✅ 监听全局变量 pendingMessage 的变化
    this.checkIntervalId = setInterval(() => {
      // ✅ 通过函数获取全局变量的值
      const message = getPendingMessage();
      if (message && this.canSend) {
        console.info('✅ 收到来自 Control 的待发送消息:', message);
        this.sendMsg = message;
        this.sendMsg2Server();
        // ✅ 通过函数清空全局变量
        setPendingMessage('');
      }
    }, 100); // 每 100ms 检查一次


    // ✅ 收到消息时的处理
    tcpSocket.on('message', (value: socket.SocketMessageInfo) => {
      try {
        let str = buf2String(value.message);
        let time = getCurrentTimeString();
        this.msgHistory += '服务端:' + str + time + '\r\n';
        this.scroller.scrollEdge(Edge.Bottom);

        // ✅ 关键：检查收到的消息是否是对你最后发送命令的确认
        if ( str.trim().toUpperCase() === 'OK') {
          this.msgHistory += '✅ 执行成功\r\n';

          // ✅ 通知 Control 页面，执行已完成
          // ✅ 通过函数获取回调函数
          const callback = getOnExecutionCompletedCallback();
          if (callback) {
            callback();
          }
        }
      } catch (e) {
        console.error('解析数据失败:', e);
        this.msgHistory += '解析数据失败\r\n';
      }
    });

    // 连接关闭
    tcpSocket.on('close', () => {
      console.info('TCP 连接已关闭');
      this.msgHistory += 'TCP 连接已关闭\r\n';
      this.canConnect = true;
      this.canSend = false;
    });

    // 错误处理
    tcpSocket.on('error', (err: BusinessError) => {
      console.error('Socket 错误:', JSON.stringify(err));
      this.msgHistory += `Socket错误: ${err.message}\r\n`;
      this.canConnect = true;
      this.canSend = false;
    });
  }

  // ✅ 组件消失时清理
  aboutToDisappear() {
    if (this.checkIntervalId !== null) {
      clearInterval(this.checkIntervalId);
      this.checkIntervalId = null;
    }
  }

  // 发送消息到服务端
  sendMsg2Server() {
    if (!this.sendMsg.trim()) {
      this.msgHistory += '❌ 发送失败: 消息为空\r\n';
      return;
    }

    // 1. 将字符串转换为 ArrayBuffer
    const textEncoder = new util.TextEncoder();
    const data = textEncoder.encode(this.sendMsg + '\r\n');

    // 2. 创建正确的 sendOptions
    const sendOptions: socket.TCPSendOptions = {
      data: data.buffer // 传递 ArrayBuffer
    };

    // 3. 发送数据
    tcpSocket.send(sendOptions)
      .then(async () => {
        this.msgHistory += '我:' + this.sendMsg + (await getCurrentTimeString()) + '\r\n';
      })
      .catch((e: BusinessError) => {
        this.msgHistory += '❌ 发送失败: ' + e.message + '\r\n';
      });
  }

  // 绑定本地端口
  async bind2Port() {
    // 本地地址
    let localAddress: socket.NetAddress = {
      address: '0.0.0.0',
      port: this.localPort,
      family: 1
    };

    try {
      await tcpSocket.bind(localAddress);
      this.msgHistory = '✅ 绑定成功\r\n';
      this.canConnect = true;
    } catch (err) {
      this.msgHistory = '❌ 绑定失败: ' + (err as BusinessError).message + '\r\n';
      this.canConnect = false;
    }
  }

  // 连接服务端
  connect2Server() {
    // 本地地址
    let serverAddress: socket.NetAddress = {
      address: this.serverIp,
      port: this.serverPort,
      family: 1
    };

    tcpSocket.connect({ address: serverAddress })
      .then(() => {
        this.msgHistory += '✅ 连接成功\r\n';
        this.canSend = true;
      })
      .catch((e: BusinessError) => {
        this.msgHistory += '❌ 连接失败: ' + e.message + '\r\n';
        this.canSend = false;
      });
  }
}

// ✅ 同步获取当前时间的字符串形式（使用标准 Date）
function getCurrentTimeString(): string {
  const now = new Date();
  const time = `${now.getHours()}:${now.getMinutes()}:${now.getSeconds()}`;
  return `[${time}]`;
}

// ✅ ArrayBuffer转utf8字符串
function buf2String(buf: ArrayBuffer): string {
  try {
    // ✅ 使用标准的 TextDecoder (new)
    let textDecoder = new util.TextDecoder('utf-8');
    let msgArray = new Uint8Array(buf);
    return textDecoder.decode(msgArray);
  } catch (e) {
    console.error('Failed to decode ArrayBuffer:', e);
    return '';
  }
}
