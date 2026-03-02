//成功版本
import promptAction from '@ohos.promptAction';
import { PermissionManager } from './PermissionManager';
import SpeechRecognizerManager from './SpeechRecognizerManager';
import { Permissions } from '@kit.AbilityKit';
import ALiYunHttpUtils from '../http/ALiYunHttpUtils';

// 全局变量
let pendingMessage: string = '';
let onExecutionCompletedCallback: (() => void) | null = null;

// 修改全局变量的函数
export function setPendingMessage(message: string): void {
  pendingMessage = message;
}

export function getPendingMessage(): string {
  return pendingMessage;
}

export function setOnExecutionCompletedCallback(callback: () => void): void {
  onExecutionCompletedCallback = callback;
}

export function getOnExecutionCompletedCallback(): (() => void) | null {
  return onExecutionCompletedCallback;
}

@Entry
@Component
export struct Control {
  @State status: string = 'idle'; // idle | recording | executing | completed
  @State recognizedText: string = '';
  @State isInitialized: boolean = false;

  build() {
    Column() {
      // 标题
      Text('语音控制小车')
        .fontSize(24)
        .fontWeight(FontWeight.Bold)
        .fontColor('#FFFFFF')
        .margin({ top: 40, bottom: 20 })

      // 语音识别状态
      if (this.status === 'recording') {
        Text('正在录音...')
          .fontSize(16)
          .fontColor('#FF9500')
          .margin({ bottom: 8 })

        if (this.recognizedText) {
          Text(`识别中: ${this.recognizedText}`)
            .fontSize(14)
            .fontColor('#CCCCCC')
            .margin({ top: 8 })
        }

        Text('指令控制小车')
          .fontSize(12)
          .fontColor('#888888')
          .margin({ top: 5 })
      }
      else if (this.status === 'executing') {
        Text('正在执行中...')
          .fontSize(16)
          .fontColor('#0A84FF')
          .padding({ top: 15, bottom: 15 })
          .backgroundColor('#1E1E1E')
          .width('80%')
          .height(50)
          .textAlign(TextAlign.Center)
          .borderRadius(12)
      }
      else if (this.status === 'completed') {
        Text('已完成')
          .fontSize(16)
          .fontColor('#34C759')
          .padding({ top: 15, bottom: 15 })
          .backgroundColor('#1E1E1E')
          .width('80%')
          .height(50)
          .textAlign(TextAlign.Center)
          .borderRadius(12)
      }

      // 控制按钮
      Button(this.getButtonText())
        .width('80%')
        .height(50)
        .backgroundColor(this.getButtonColor())
        .fontColor(Color.White)
        .onClick(() => {
          this.handleButtonClick();
        })
        .margin({ top: 30 })

      // 状态提示
      if (this.status === 'idle') {
        Text('点击上方按钮开始语音控制')
          .fontSize(14)
          .fontColor('#CCCCCC')
          .margin({ top: 10 })
      }

      // 确定按钮
      if (this.status === 'completed') {
        Button('确定')
          .width('50%')
          .height(40)
          .backgroundColor('#0A84FF')
          .fontColor(Color.White)
          .onClick(() => {
            this.resetToIdle();
          })
          .margin({ top: 20 })
      }

      //错误提示
      if (this.status === 'idle' && !this.isInitialized) {
        Text('语音识别初始化失败，请检查权限')
          .fontSize(12)
          .fontColor('#FF6B6B')
          .margin({ top: 10 })
      }
    }
    .width('100%')
    .height('100%')
    .backgroundColor('#000000')
    .alignItems(HorizontalAlign.Center)
    .justifyContent(FlexAlign.Center)
  }

  private getButtonText(): string {
    if (this.status === 'idle') return '点击开始录音';
    if (this.status === 'recording') return '停止录音';
    return '执行中...';
  }

  private getButtonColor(): Color {
    if (this.status === 'recording') return Color.Red;
    if (this.status === 'executing') return Color.Gray;
    return Color.Blue;
  }

  private async handleButtonClick(): Promise<void> {
    if (this.status === 'idle') {
      this.startRecording();
    }
    else if (this.status === 'recording') {
      this.stopRecording();
    }
  }

  // 开始录音
  private async startRecording(): Promise<void> {
    this.status = 'recording';
    this.recognizedText = '准备中...';

    try {
      // 检查麦克风权限
      const permissions: Permissions[] = ["ohos.permission.MICROPHONE"]
      const isPermission = await PermissionManager.checkPermission(permissions);

      if (!isPermission) {
        // 请求权限
        const granted = await PermissionManager.requestPermission(permissions);
        if (!granted) {
          this.status = 'idle';
          promptAction.showToast({
            message: '需要麦克风权限才能使用语音功能',
            duration: 3000
          });
          return;
        }
      }

      // 启动语音识别
      SpeechRecognizerManager.init((res) => {
        console.info('语音识别结果:', JSON.stringify(res));
        this.recognizedText = res.result;
      });

      this.isInitialized = true;

      promptAction.showToast({
        message: '请说出控制指令...',
        duration: 2000
      });

      console.info('✅ 开始语音识别');
    } catch (err) {
      console.error('语音识别启动失败:', JSON.stringify(err));
      this.status = 'idle';
      this.isInitialized = false;

      // 处理常见错误
      if (err.code === 1002200006) {
        promptAction.showToast({
          message: '语音引擎正忙，请稍后再试',
          duration: 3000
        });
      } else {
        promptAction.showToast({
          message: `语音识别初始化失败: ${err.message}`,
          duration: 3000
        });
      }
    }
  }

  // 停止录音
  private stopRecording(): void {
    try {
      // 1. 停止语音识别
      SpeechRecognizerManager.release();
      console.info('停止语音识别');

      // 2. 确保有识别内容
      if (!this.recognizedText || this.recognizedText === '准备中...') {
        this.recognizedText = '未识别到有效指令';
        promptAction.showToast({
          message: '未识别到有效指令',
          duration: 2000
        });
      }

      // 3. 立即进入执行阶段（关键修改）
      this.status = 'executing';
      // ✅ 关键：先让 AI 处理，而不是直接发送原始语音
      console.info('✅ 发送语音指令给 AI 进行语义理解:', this.recognizedText);
      // 调用 ALiYunHttpUtils，并在 AI 回复后发送
      ALiYunHttpUtils.request(this.recognizedText, (aiReply) => {
        console.info('✅ AI 理解结果:', aiReply);
        // ✅ 将 AI 理解后的指令存入全局变量
        setPendingMessage(aiReply);
        console.info('✅ 已将指令存入全局变量:', aiReply);
      });
    } catch (err) {
      console.error('停止录音失败:', err);
      this.status = 'idle';
      promptAction.showToast({
        message: '停止录音失败',
        duration: 2000
      });
    }
  }

  // 组件销毁时清理
  aboutToDisappear() {
    // 停止录音
    if (this.status === 'recording') {
      this.stopRecording();
    }

    console.info('Control 组件已销毁');
  }

  // 注册执行完成回调
  aboutToAppear() {
    setOnExecutionCompletedCallback(() => {
      this.status = 'completed';
      console.info('Control 收到执行完成通知');
    });
    console.info('Control 组件已加载');
  }


  private resetToIdle(): void {
    this.status = 'idle';
    this.recognizedText = '';
  }
}
