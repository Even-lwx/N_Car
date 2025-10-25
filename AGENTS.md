# Repository Guidelines

## 项目结构与模块组织
- `code/` 存放核心控制逻辑，含 `imu.c`、`motor.c`、`pid.c` 及 `EKF/` 扩展卡尔曼滤波库；硬件抽象和算法调整均在此集中维护。
- `user/` 承载启动流程与中断服务，调度周期任务；`libraries/` 引入逐飞底层驱动；`Debug/` 和 `build/` 为 ADS 生成的目标产物，提交前请清理。

## 构建、测试与开发命令
- ADS 图形界面：在工程树右键 `Build Project` 完整构建；调试烧录使用 `Debug`。
- 命令行：在 `Debug/` 执行 `make clean && make` 触发 TASKING 编译链；如需快速验证菜单配置，可单独编译 `code/menu.c` 相关单元后链接集成。

## 编码风格与命名约定
- C 代码统一使用 4 空格缩进，保持 `snake_case` 函数与变量命名，枚举与宏采用全大写加下划线，如 `MENU_EVENT_ENTER`。
- 依赖 `clang-format` (配置见 `.settings/`) 处理格式；提交前运行 `make format` 或在 ADS 中执行 `Source -> Format` 保持一致。

## 测试指引
- 现有功能测试集中在 `code/acc_calibration_test.*` 与 `code/delayed_stop.*` 等硬件在环模块，按模块命名 `*_test.c`；运行前在 ADS 选择相应主函数入口。
- IMU、速度环调试需连接小车，串口输出位于 `user/cpu0_main.c` 中 `debug_print()`；记录 CSV 后用 `build/tools/imu_plot.py` 分析，目标覆盖关键场景（静止、加速、转弯）。

## 提交与 Pull Request 规范
- Git 历史遵循 `type: 描述` 前缀（如 `feat:`, `style:`），描述使用简练动词句；每次提交聚焦单一改动。
- PR 请求需附变更摘要、验证步骤、关联需求编号，并在涉及界面或波形调整时上传示意图；评审前确保通过构建与硬件自测。

## 校准与配置提示
- 首次接入新底盘请运行 `code/acc_calibration_test.c`，完成 IMU 与电机零点校准后更新 `param_save.c` 默认值。
- IPS114 菜单配置项对应 `menu_config.c`，新增页面时同步维护 `menu.h` 枚举，避免魔法数字硬编码。
