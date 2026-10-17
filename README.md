# MEIC 智能寻迹小车 · 招新题核心控制代码与开发指南

本项目针对 MEIC 智能寻迹小车控制类招新考核开发，已完整实现**基础任务（1m×1m正方形循迹+4直角弯+定点停车）**、**任务二（倒车入库/侧方停车）**以及**现场标定与自检系统**。

---

## 一、硬件接线与引脚映射

主控芯片：**STM32F103C8T6**（C03C 核心板）

| 部件 / 信号 | 引脚 | 定时器 / 通道 | 说明 |
|---|---|---|---|
| 左轮 PWMA | **PA0** | TIM2_CH1 | 10kHz PWM，速度调节 |
| 左轮 AIN1 / AIN2 | **PA1 / PA2** | GPIO_Out | 左轮正反转方向控制 |
| 右轮 PWMB | **PA3** | TIM2_CH4 | 10kHz PWM，速度调节 |
| **右轮 BIN1 / BIN2** | **PB1 / PB8** | GPIO_Out | **★ 关键实测适配引脚**（非 PA4/PA5） |
| TB6612 STBY | **3V3** | - | 待机使能，必须接 3.3V |
| 左轮编码器 A/B 相 | **PA6 / PA7** | TIM3 | 4 倍频正交编码器模式 |
| 右轮编码器 A/B 相 | **PB6 / PB7** | TIM4 | 4 倍频正交编码器模式 |
| 灰度传感器 AD0 / AD1 / AD2 | **PB12 / PB13 / PB14** | GPIO_Out | 74HC4051 模拟分时选择通道 |
| 灰度传感器模拟输出 OUT | **PB0** | ADC1_IN8 | 模拟电压采样（< 阈值判断为压黑） |
| 板载任务切换按键 KEY1 | **PA11** | GPIO_IPU | 运行时切换基础巡线 / 泊车模式 |

---

## 二、七大避坑设计与修复（针对交接单痛点）

1. **解决 Speed_Loop 短路死区**：移除了硬编码的 `return 3000;`，重构为带有积分限幅与抗饱和的真正 PI 闭环速度环。
2. **解除 12 秒与 100cm 定距限制**：`TEST_RUN0_STOP_CM` 设为 `0.0f`，移除计时强行切断，保证小车完整跑完正方形赛道并返回停止线。
3. **双轮脉冲当量与机械不对称配平**：内建 `RIGHT_TRIM`（右轮修正倍率）与 `RUN_MODE=6` 自动配平算法，消解更换电机带来的转速偏差。
4. **严格锁定 10ms 控制周期**：控制循环使用 `CTRL_PERIOD_MS = 10`，保证增益与积分项稳态收敛，避免机身抖动。
5. **起跑线误触发防御机制（s_armed）**：起步阶段当车身横跨黑线时，先识别持续居中的单线后才激活直角弯检测，彻底根除刚起步就打方向的误判。
6. **模拟灰度反向极性判定**：针对 8 路红光模拟分时版特性，严格按照 `ADC 读数 < TRACK_THRESHOLD` 判定黑线。
7. **免烧录现场任务切换**：通过 PA11 按键或 Keil Watch 在线修改全局变量 `g_task`，完全符合学长“验收中途不可重新烧录程序”的要求。

---

## 三、现场标定快速三步法

在 Keil 调试模式（Ctrl + F5）打开 **Watch 1** 窗口，勾选 `View -> Periodic Window Update`：

1. **灰度阈值标定（RUN_MODE = 3）**：
   - 将小车置于白底，观察 `g_dbg[0..7]` 读数（通常在 2800~3500 左右）；
   - 将小车置于黑线上，观察读数（通常在 600~1200 左右）；
   - 取两者的折中平均值（默认 1800），在线修改 `g_track_threshold`。
2. **两轮自动配平（RUN_MODE = 6）**：
   - 将小车架空或置于平地，小车运行 2 秒后自动停止；
   - 查看生成的 `g_trim` 变量值，填入 `g_trim` 维持左右轮真实线速度一致。
3. **脉冲当量推车测量**：
   - 推车直行 100cm，查看 `g_sumL` 与 `g_sumR`，除以 100 得到每厘米脉冲数并更新 `g_pulse_per_cm`（默认约 71.3）。

---

## 四、GitHub 上传指南

因 AI 沙箱虚拟机与外部网络隔离，代码已在本地生成，请按照以下任一方式上传到您的 GitHub 仓库：

### 方式 1：GitHub 网页端直接上传（最简便）
1. 打开您的仓库页面：[shenmonkey7-beep/C-language-study](https://github.com/shenmonkey7-beep/C-language-study)
2. 点击右上角的 **`Add file` -> `Upload files`**；
3. 将本次生成的 `User/`、`Hardware/` 目录及 `README.md` 拖拽上传；
4. 填写 Commit message（例如：`feat: complete basic line tracking, parking and calibration modules`），点击 **Commit changes**。

### 方式 2：Git 命令行推送
在您的本地代码目录中执行：
```bash
git add Hardware/ User/ README.md
git commit -m "feat: complete basic line tracking, parking and calibration modules"
git push origin main
```
