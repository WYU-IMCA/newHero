英雄的新自瞄代码 
功能全都还没测试 自瞄基本没问题   前哨站的没测过（有新旧两种方案）

## 文件目录结构 📂
```shell
.
├── src  根目录
|   ├── rm_auto_aim          自瞄模块
|   │    ├──armor_detector   装甲板检测模块
|   │    ├──armor_solver     装甲板追踪模块
|   │    └──rm_auto_aim
|   ├── rm_auto_outpost      自动前哨站模块
|   ├── rm_auto_record       录像模块
|   ├── rm_bringup           
|   ├── rm_harware_driver    相机驱动模块及串口模块
|   ├── rm_interfaces        自定义话题类型
|   ├── rm_robot_description 机器人建模文件
|   └── rm_utils             工具模块(包含pnp、弹道解算等)
|
└── launch.sh         一键启动的脚本
