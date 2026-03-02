//import ALiYunHttpUtils from '../http/ALiYunHttpUtils';
@Entry
@Component
struct Index {
  pathStack : NavPathStack = new NavPathStack();

  build() {
    Navigation(this.pathStack){
    }.onAppear(() => {
      this.pathStack.pushPathByName("Start", null, false);
    })
    .hideNavBar(true)//不会把自己放到控制跳转的对象里面，也就是点返回的时候不会返回到这个导航页
  }
}