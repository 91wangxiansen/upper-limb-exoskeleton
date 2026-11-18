# 上肢外骨骼控制系统

肩、肘双关节与四指手部外骨骼的本地控制程序，包含独立控制版本与统一上位机。

| 项目 | 内容 | 启动入口 |
| --- | --- | --- |
| [上肢控制](上肢控制/使用说明.md) | 肩肘与手部统一连接、组合模型、定姿态控制、手势与实时反馈 | `上肢控制/启动上位机.bat` |
| [臂部控制](臂部控制/README.md) | 整合前的肩肘双关节独立上位机 | `臂部控制/run.bat` |
| [手部控制](手部控制/README_v2.md) | 四指独立控制台、ESP32固件与SCServo驱动库 | `手部控制/debug_bridge.bat` |

## 本地运行

上位机使用Windows、Python 3和pyserial，依赖见各上位机目录的`requirements.txt`。双击启动脚本后打开`http://127.0.0.1:8765/`。独立臂部和统一上位机共享8765端口与单实例锁，选择其中一套运行。

手臂通过CANable/SLCAN连接CyberGear。手部通过Hand-Exo Wi-Fi连接ESP32。统一上位机中单击手部模型或顶部“手指控制”，在同一页面进入完整的四指控制界面。操作与参数范围详见[上肢控制使用说明](上肢控制/使用说明.md)。

独立手部网页由`debug_bridge.bat`提供，默认入口为`http://127.0.0.1:8767/`。

## 固件与网页资源

手部固件入口为`ServoDriver/ServoDriver.ino`，使用ESP32 Dev Module与Huge APP分区；也可使用项目中的PlatformIO配置。SCServo库、网页资源和现成的`WebAssets.h`均随源码提供。修改手部网页后，可在对应手部项目目录运行`python tools/build_assets.py --embed-only`，重新生成内嵌资源，再自行编译上传。编译与上传脚本位于`tools`。

## 验证与许可

各项目的`tests`保留控制逻辑、接口、页面和曲线处理的回归测试。Python测试使用`python -m unittest discover -s tests`。浏览器模拟测试另需Playwright及测试脚本指定的本机Edge。

第三方驱动与资源的版权信息见手部项目内的`LICENSE`、`THIRD_PARTY.md`及各依赖源文件声明。

演示视频链接https://www.bilibili.com/video/BV1oKsU6fE5k/?vd_source=2a57870915cf82f4fca6aeef582bccb8
