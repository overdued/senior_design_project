//导入
import { Control } from "./Control";
import { Mine } from "./Mine";

interface  TabClass {
  text: string
  icon: ResourceStr
}

// 跳转页面入口函数
@Builder
export function LayoutBuilder() {
  Layout();
}

@ComponentV2
struct Layout {
  @Local currentIndex: number = 0
  //控制跳转的对象
  pathStack: NavPathStack = new NavPathStack();
  tabData: TabClass[] = [
    {text: '控制', icon: $r('app.media.pic4')},//下方菜单的图片，可修改
    {text: '我的', icon: $r('app.media.pic3')}
  ]

  @Builder tabBuilder (item: TabClass, index: number) {
    Column({space: 5}) {
      Image(item.icon)
        .width(24)
        .fillColor(this.currentIndex === index ? '#E85A88' : '#63AAAA')
      Text(item.text)
        .fontSize(14)
        .fontColor(this.currentIndex === index ? '#E85A88' : '#63AAAA')
    }
  }

  build() {
    NavDestination() {
      Tabs({barPosition: BarPosition.End}) {
       ForEach(this.tabData, (item: TabClass, index: number) => {
         TabContent() {
           if (index === 0) {
             Control()
           } else if (index === 1) {
             Mine()
           }
         }
         .tabBar(this.tabBuilder(item, index))//自定义导航栏，使用@BUILDER
         .backgroundColor('#131215')
         .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.TOP, SafeAreaEdge.BOTTOM])
       })
      }
      .backgroundColor('#3B3F42')
      .onChange((index: number) => {
        this.currentIndex = index
      })
      .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.TOP, SafeAreaEdge.BOTTOM])
    }
    .onReady((context: NavDestinationContext) => {
      this.pathStack = context.pathStack;
    })
  }
}