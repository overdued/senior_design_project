import http from '@ohos.net.http';
import hilog from '@ohos.hilog';

interface GeneratedTypeLiteralInterface_2 {
  total_tokens: number;
  input_tokens: number;
  output_tokens: number;
}

interface GeneratedTypeLiteralInterface_1 {
  text: string;
  finish_reason: string;
}

interface AliyunResponse {
  output: GeneratedTypeLiteralInterface_1;
  usage: GeneratedTypeLiteralInterface_2;
  request_id: string;
}

// 定义回调函数类型
type OnResultCallback = (reply: string) => void;

class ALiYunHttpUtils {
  request(question: string, onResult?: OnResultCallback): void {
    //构造新的提示词 (Prompt)
    const prompt = `
      你是一个管家，你需要听取用户指令，识别他们想要的物品。
      你可以选择提供的物品包括：蛋糕，盒子，网球，杯子。
      若用户想要食物，返回“蛋糕”；
      若用户想要药品，返回“盒子”；
      若用户想要运动，返回“网球”；
      若用户渴了，返回“杯子”；
      若现有物品不可以满足用户需求，你需要回答“我无法提供服务，请换一个要求”。
      用户说：${question}
    `;
    hilog.info(0x0000, 'testTag', 'ALiYunHttpUtils request invoke. question: %{public}s', question);

    // 1. 创建 HTTP 请求对象
    const httpRequest = http.createHttp();

    // 2. 发起请求
    httpRequest.request(
      "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation",
      {
        method: http.RequestMethod.POST,
        header: {
          "Content-Type": "application/json",
          "Authorization": "sk-835f0b6bb2314d4288e89fa135647942"
        },
        extraData: {
          model: "qwen-plus",
          input: {
            messages: [
              {
                role: "user",
                content: prompt
              }
            ]
          }
        }
      },
      (err, data) => {
        if (err) {
          hilog.error(0x0000, 'testTag', '❌ 请求失败: %{public}s', JSON.stringify(err));
          // 调用回调，返回错误信息
          onResult && onResult("网络错误，请检查 API Key 或网络连接");
        } else {
          try {
            const result : AliyunResponse = JSON.parse(data.result as string);
            const reply : string = result.output?.text || "抱歉，AI 没有返回有效内容。";
            hilog.info(0x0000, 'testTag', '✅ 请求成功: %{public}s', reply);
            onResult && onResult(reply); // 调用回调函数
          } catch (parseError) {
            hilog.error(0x0000, 'testTag', '❌ 解析失败: %{public}s', JSON.stringify(parseError));
            onResult && onResult("解析 AI 回复失败");
          }
        }
        httpRequest.destroy(); // 请求完成后销毁
      }
    );
  }
}

export default new ALiYunHttpUtils();