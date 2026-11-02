# 烧录与固化

> 两个模式差别很大：**Program SRAM 掉电即失，Program Flash 上电自启动**。
> 展会展出必须用后者。

---

## 1. 两种模式对比

| | Program SRAM | Program Flash |
|---|---|---|
| 写入目标 | FPGA 内部配置 RAM | 板载 W25Q16 Flash |
| 掉电后 | **失效**，恢复为 Flash 中的旧版本 | **保留**，上电自动加载 |
| 耗时 | 数秒 | 数十秒 |
| 适用场景 | 调试、临时验证 | 展出、交付、演示 |

---

## 2. 接线与识别

1. USB 线接板上 **USB-JTAG** 口（此时板子已上电）
2. 打开 TD → 工具栏 **BitWriter**（或 `Tools > BitWriter`，也可直接运行 `bin\bw.exe`）
3. 点 **Refresh**：
   - `Debug Server` 应显示 **AL-LINK**
   - 器件应显示 **EG4S20BG256**

> 认不到器件多为 USB 线或驱动问题，对应报错 `PRG-9000` / `PRG-9511`。

---

## 3. 临时调试：Program SRAM

1. 模式选 **`Program SRAM`**
2. **Add** 选择 `bitstream/hdmi_media_player_C77.bit`
3. 点 **Run**，等进度 100%
4. 板子立即开始运行

> 断电或重新上电后会自动回到 Flash 中的旧版本，这是预期行为。

---

## 4. 固化：Program Flash

### 4.1 步骤

1. BitWriter 中 **Refresh**，确认识别到 `EG4S20BG256`
2. 下载模式选 **`Program Flash`**
   - ⚠️ 不要选 `Program SRAM`
   - ⚠️ **不要碰 `Merge DualBoot Bit`**——那会把两个位流合并成双镜像并改写 Flash 布局
3. **`MemorySize` 改成 `16 Mbits`**（选 Program Flash 后才出现这一栏，默认是 `128 Mbits`）
4. **Add** 选择 `bitstream/hdmi_media_player_C77.bit`
5. 速率保持默认 **5 MHz**
6. 点 **Run**，等进度到 **100%**
7. **完整断电，再上电**（拨板上的电源总开关）
8. 上电后数码管亮、HDMI 出图，即为固化成功

### 4.2 为什么 `MemorySize` 必须改成 16 Mbits

板载 Flash 是 **W25Q16 = 16 M-bit = 2 MB**（实物丝印 `25Q16` 已确认）。

若按默认 `128 Mbits` 声明，擦除/写入会按 16 MB 计算地址，典型表现：

- 擦除阶段报错
- `Verify Flash` 结果不一致
- **进度走到 100% 但断电重启不启动**（最常见，也最容易被误判成"固件有问题"）

### 4.3 为什么必须"断电重启"而不是按复位

FPGA 只在**重新上电**时走一遍配置加载流程。按复位键只复位用户逻辑，不会重新从 Flash 加载位流。
本项目在调试阶段多次因此误判。

---

## 5. 校验与维护

| 目的 | 操作 |
|---|---|
| 确认烧进去的与文件一致 | 同窗口选 **`Verify Flash`**，或 `Flash Read` 回读比对 |
| 换新版本前 | 先 **`Erase Flash`** 整片擦除，再按 4.1 重烧 |
| 只想临时验证不固化 | 用 `Program SRAM`，掉电自动恢复旧版 |
| 确认产品批次 | 读 Flash 中的 `# Date:` 头（见下） |

---

## 6. 关于 md5 与"同一份配置"的判定

`.bit` 文件前约 140 字节是**文本头**，含生成时刻 `# Date:`。
同一份配置重跑一次，**md5 必然变化**，但配置区逐位相同。

正确判据：

```bash
# 方法一：比对位流 CRC（文件内 "# Bitstream CRC" 字段）
grep "Bitstream CRC" *.bit

# 方法二：逐字节比较，确认差异只落在前 140 字节
cmp -l a.bit b.bit | awk '$1 > 140' | wc -l    # 期望 0
```

本仓库 C77 的 CRC：`10100010101001101100011101101011`

> 用 md5 判断"是否同一配置"会误判成不一致——`bitstream/MD5.txt` 只用于**传输完整性校验**，
> 不代表配置等价。

---

## 7. 恢复被写坏的 Flash

若 Flash 被误写导致无法启动，机器仍可通过 JTAG 救回（配置口不受 Flash 内容影响）：

1. 断电 → 按住板上的模式/复位不影响，直接上电
2. BitWriter → Refresh → 识别到器件后选 `Program Flash` → `MemorySize = 16 Mbits`
3. 重新烧录正确的 `.bit`

只要 JTAG 能识别到器件，Flash 内容永远是可覆盖的。
