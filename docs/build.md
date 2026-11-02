# 构建指南

> 目标：从零开始，把仓库编译成可烧录的 `.bit`。
> 全程只需 **Anlogic TD 6.2.1**，无需第三方库。

---

## 1. 环境依赖

| 项 | 要求 | 检查方式 |
|---|---|---|
| 操作系统 | Windows 10 / 11（TD 仅提供 Windows 版） | — |
| 工具链 | **Anlogic TD 6.2.1**（Engineer 6.2.168.116） | `bin\td_commands_prompt.exe` 存在 |
| 器件库 | `eagle_s20.db`（随 TD 安装） | TD 新建工程时能看到 `EG4S20BG256` |
| 磁盘空间 | ≥ 500 MB（一次完整构建的中间库约 65 MB） | — |
| 下载驱动 | AL-LINK / USB-JTAG 驱动 | BitWriter 能 Refresh 到器件 |

> **版本强绑定**：TD 的 run 数据库跨版本往往读不进去。用 6.2.1 以外的版本打开，
> 可能需要删除 `hdmi_media_player_Runs/*/` 下的中间文件重新构建。

---

## 2. 拿到代码

```bash
git clone https://github.com/fjh77/fpga-audio-visual-player.git
cd fpga-audio-visual-player
```

工程可整包搬到任意路径——`.al` / `.prj` / `settings.cfg` 里全部是相对路径。

---

## 3. 两种构建方式

### 3.1 TD GUI（推荐给初次接触者）

1. 启动 TD 6.2.1
2. `File > Open Project`
3. 选择 `hardware/hdmi_media_player/src/td_project/hdmi_media_player.al`
4. 仓库**不含 run 数据库**，`syn_1` / `phy_1` 会显示"未运行"，这是正常的
5. 右键 `syn_1` → **Run**，约 2 分钟（`read_design → opt_gate`）
6. 右键 `phy_1` → **Run**，约 4 分钟（`opt_place → bitgen`，复用 `syn_1/gate.db`）
7. 产物：`hdmi_media_player_Runs/phy_1/hdmi_media_player.bit`

### 3.2 命令行

```bash
TD=C:/Anlogic/TD_6.2.1_Engineer_6.2.168.116/bin/td_commands_prompt.exe
cd hardware/hdmi_media_player/src/td_project/hdmi_media_player_Runs

# ① 综合（约 2 分钟）→ syn_1/gate.db
(cd syn_1 && "$TD" ../../flow_holdfix.tcl > run.log 2>&1)

# ② 实现 + bitgen（约 4 分钟）
(cd phy_1 && "$TD" ../../flow_holdfix.tcl > run.log 2>&1)

# 收尾：进程会停在 td > 提示符
taskkill //F //IM td_commands_prompt.exe
```

**注意**：`td_commands_prompt.exe` **不能加任何 flag**。加 `-tcl` / `-source` 会变成
`source -tcl xxx` 并报 `wrong # args`。

---

## 4. 为什么必须用 `flow_holdfix.tcl`

TD 自带的 `DefaultFlow.tcl` 把"路由后单独跑 `fix_hold`"绑在 `arr_filter` 变量上，
本工程该变量为 `false`，于是**后置 hold 修复从不执行**。而路由阶段内置的 `route fix_hold`
在 slices 超过阈值时又会被跳过（日志 `RUN-1001 ... hold fixing is skipped`）。

本工程的 `flow_holdfix.tcl` 在布线后**无条件补跑一次 `fix_hold`**。实测效果：

| 阶段 | Hold WNS | 违例端点 |
|---|---|---|
| 布局后 | −11 ps | 17 |
| 布线后（未跑 fix_hold） | 负值 | 多个 |
| **`fix_hold` 之后** | **+3 ps** | **0** |

> 若你看到 hold 为负、且日志有 `hold fixing is skipped`，先检查是不是用了默认流程脚本。

---

## 5. 改完 RTL 必做三步

TD 每轮流程结束会执行 `export_flow_status`，**自动把 `start_step` 改回 `opt_place`**。
下一次点"重新构建"就只跑布局布线，开头 `import_db ../syn_1/...gate.db`——**源码改了也白改**。

所以每次改 RTL 后：

1. 把 `syn_1/settings.cfg` 的 `start_step` 改回 `read_design`
2. 删掉 `syn_1/` 下的三个进度标记：
   `.read_design.end.f`、`.opt_rtl.end.f`、`.opt_gate.end.f`
3. 跑完核对 `syn_1/gate.qor` 的 `#luts` 是否为预期值（改动引入新逻辑时数字应当变化）

**一条命令判据**：

```bash
grep -m1 -E "elaborate|import_db" syn_1/run.log
```

| 输出 | 含义 |
|---|---|
| `elaborate -top top` | ✅ 真正重新综合了 |
| `import_db ../syn_1/...gate.db` | ❌ 用了旧网表，改动未生效 |

---

## 6. 构建判据

| 检查 | 命令 | 期望 |
|---|---|---|
| 无 ERROR | `grep -c "^ERROR" run.log` | `0` |
| 悬空 net | `grep -c HDL-5314 run.log` | `2`（见 [FAQ](faq.md)） |
| 无死条目 | `grep -c PRJ-7001 run.log` | `0` |
| 无驱动冲突 | `grep -c HDL-8007 run.log` | `0` |
| hold 修复被跳过 | `grep -c "hold fixing is skipped" run.log` | `0` |
| 综合资源 | `grep "#luts" syn_1/gate.qor` | 9,400（基线值，改动后以新值为准） |
| 签核时序 | `grep -E "Setup WNS\|Hold WNS" phy_1/route.qor` | `+160` / `+3` |
| 面积 | 打开 `phy_1/*_phy.area` | LUT 15,670（79.95%） |

---

## 7. 常见构建错误

| 报错 | 原因 | 处理 |
|---|---|---|
| `HIER-1013` / `HDL-8001` | 在 tcl 里单独用 `open_project` + `elaborate -top top`（缺 `-single_run`） | 正常现象，走完整 flow 即可 |
| `PRJ-7001` | 工程里有已删除的源文件条目 | 打开 `.al` 移除失效条目 |
| `HDL-8007` | 同一 net 多驱动 | 检查 `grant`/`mux` 是否有重复赋值 |
| `RUN-1001 ... hold fixing is skipped` | slices 超阈值 | 先做面积清理，或确认用了 `flow_holdfix.tcl` |
| `PRG-9000` / `PRG-9511` | 下载器未识别 | 换 USB 线 / 重装 AL-LINK 驱动 |
| 综合很久不结束 | 误把 `include/curl_lut.vh` 当源文件加入 | 它是头文件，只需 `include` 路径 |

---

## 8. 构建耗时参考

| 阶段 | 耗时 | 产物 |
|---|---|---|
| 综合 `syn_1` | ≈ 2 min | `gate.db`、`gate.qor` |
| 实现 `phy_1` | ≈ 4 min | `pr.db`、`route.qor`、`.bit` |
| 合计 | **≈ 6 min** | — |

---

## 9. 还原到干净状态

若构建被污染（例如中途 kill、版本切换），最省事的做法：

```bash
cd hardware/hdmi_media_player/src/td_project/hdmi_media_player_Runs
find . -type f \( -name "*.db" -o -name "*.log" -o -name "*.f" -o -name "*.qor" \) -delete
# 保留 settings.cfg / .prj / .tcl / *.area
```

然后按第 3 节重新跑两段流程。仓库里的 `settings.cfg` 已预设为正确起点
（`syn_1: read_design`，`phy_1: opt_place`）。
