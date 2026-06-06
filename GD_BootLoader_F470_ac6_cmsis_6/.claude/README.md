# GD32F470 V2 BootLoader Project

这是 V2 硬件的 BootLoader 工程，使用 AC6 + CMSIS6。它和同级 App 工程配套，用于实现 APP2 暂存区到 APP1 运行区的可靠升级。

## 工程信息

| 项 | 内容 |
| --- | --- |
| 入口工程 | `MDK/Project.uvprojx` |
| Scatter | `MDK/BootLoader_F470.sct` |
| IROM | `0x08000000 0x0000C000` |
| 共用分区 | `common/bl_partition.h` |
| 参数页 | `common/bl_param.h` |
| 核心逻辑 | `BOOTLOADER/Src/bl_core.c` |

## 目录说明

| 目录 | 内容 |
| --- | --- |
| `BOOTLOADER/` | BootLoader 状态机、Flash 擦写接口和头文件。 |
| `common/` | BootLoader / App 共用分区和参数页定义。 |
| `Components/bsp/` | V2 BootLoader 板级初始化，主要用于 USART0 debug。 |
| `APP/` | 保留必要的串口、调度等支持代码。 |
| `Driver/`、`Libraries/` | CMSIS6、GD32F4xx 标准外设库和启动文件。 |
| `PACK/` | 工程内置扩展包，例如 perf_counter 2.5.4。 |
| `USER/` | BootLoader 入口 `main.c` 和中断文件。 |
| `MDK/` | Keil 工程、scatter 和输出目录。 |

## 启动流程

1. 初始化 SysTick 和 debug 串口。
2. 读取参数页 main copy 和 backup copy。
3. 校验 magic、版本、App 地址、App 大小、CRC 和 tail magic。
4. 参数页异常时，用有效副本或默认参数修复，并写入日志。
5. 如果 `update_flag == BL_UPDATE_FLAG_PENDING`，校验 APP2 向量表和 CRC。
6. 校验通过后按 `app_size` 擦写 APP1，并再次校验 APP1 CRC。
7. 成功后清除升级标志、增加 `update_counter`、写成功日志并复位。
8. 无待升级任务时，校验 APP1 向量表并跳转。
9. APP1 无效时记录 `BL_ERR_APP1_INVALID` 并停留在 BootLoader。

## Flash 分区

| 区域 | 地址范围 | 大小 |
| --- | --- | --- |
| BootLoader | `0x08000000` - `0x0800BFFF` | 48 KB |
| 参数页 | `0x0800C000` - `0x0800CFFF` | 4 KB |
| APP1 | `0x0800D000` - `0x08044FFF` | 224 KB |
| APP2 | `0x08045000` - `0x0807CFFF` | 224 KB |
| DATA | `0x0807D000` - `0x0807FFFF` | 12 KB |

## 编译和烧录

1. 打开 `MDK/Project.uvprojx`。
2. 确认 Linker 使用 `.\BootLoader_F470.sct`。
3. 编译后将 hex 下载到 `0x08000000`。
4. 再编译并烧录配套 App 工程到 APP1，或通过 OTA 写入。

烧录 App 时不要整片擦除，避免清掉 BootLoader 和参数页。
