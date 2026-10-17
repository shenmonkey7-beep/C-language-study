# STM32 构建依赖来源

`Start/` 和 `Library/` 补齐了原 Keil 工程缺失的 CMSIS、启动代码及五个必需的标准外设库模块，未引入旧模板的 OLED、TRACE、TIM3 PWM 等无关驱动，以免与编码器 TIM3 冲突。

来源：STM32F10x Standard Peripherals Library V3.5.0 的公开镜像 [wajatimur/stm32f10x-stdperiph-lib](https://github.com/wajatimur/stm32f10x-stdperiph-lib)，固定提交 `afa743577f2784e95be2d5003380fdb84a702519`。文件中的 STMicroelectronics / ARM 版权及许可声明保留原样，CMSIS 随包许可见 `Start/CMSIS_License.doc`。

映射：

- `Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/{stm32f10x.h,system_stm32f10x.h,system_stm32f10x.c}` → `Start/`。
- 同目录 `startup/arm/startup_stm32f10x_md.s` → `Start/`，用于 ARMCC5 / Keil。
- `Libraries/CMSIS/CM3/CoreSupport/core_cm3.h.old` → `Start/core_cm3.h`，仅恢复标准头文件名，内容未改动。
- `Libraries/STM32F10x_StdPeriph_Driver/{inc,src}/` 下 `misc`、`stm32f10x_adc`、`stm32f10x_gpio`、`stm32f10x_rcc`、`stm32f10x_tim` 的 `.c/.h` → `Library/`。

`tests/stm32f10x.h` 是本仓库的测试替身，只模拟寄存器和外设调用，不属于 STM32 标准库，也不能加入 Keil 固件头文件路径。验证用 TinyCC 保留在工作目录，不随源码分发。
