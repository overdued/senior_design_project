// 跳转页面入口函数
@Builder
export function StartBuilder() {
  Start();
}

@Component
struct Start {
  //控制跳转的对象
  pathStack: NavPathStack = new NavPathStack();
  //过3秒自动跳转到LAYOUT页面
  aboutToAppear(): void {//生命周期
    setTimeout(() => {
      this.pathStack.replacePathByName("Layout", null, false)
    }, 3000)//以ms为单位
  }

  build() {
    NavDestination() {
      Stack({alignContent: Alignment.TopEnd}) {
        Image($r('app.media.pic5'))//这里是广告页面，可换图片
          .width('100%')
          .height('100%')
          .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.TOP, SafeAreaEdge.BOTTOM])//安全区
        Button('跳过')
          .backgroundColor(Color.Grey)
          .margin(15)
          .onClick(() => {
            this.pathStack.replacePathByName("Layout", null, false)//点击跳过跳转且不能再返回到该页
          })
      }
    }
    .onReady((context: NavDestinationContext) => {
      this.pathStack = context.pathStack;
    })
  }
}