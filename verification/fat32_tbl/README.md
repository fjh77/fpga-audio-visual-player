# fat32_dir 文件表容量验证

本目录是 `fat32_dir` 文件表容量从 16 降到 10 时配套做的单元验证，可脱离工程独立运行，不需要 TD。

## 为什么需要这个验证

`fat32_dir` 把 FAT32 根目录解析成一个寄存器文件表，表深度原先写死为 16。为了给布局腾面积，把它改成跟随 `MAX_FILES` 参数（工程例化为 10）。

表深度是 `reg` 数组的范围参数。改小之后若某个下标越界，**综合器不会报错**，只会读到 X 或者别的槽位；上板表现为"文件个数对、内容错"，极难定位。所以必须在改动落地的同一轮里，用仿真把读回内容逐字节核一遍。

## 覆盖的三个场景

| 用例 | 输入 | 期望 |
|---|---|---|
| TC1 | 8 个文件（4 BMP + 4 WAV，工程实际用法） | `err_code=0`，`o_count=8`，索引 0~7 的名字/首簇/大小/属性全部正确 |
| TC2 | 11 个文件（超出容量） | `err_code=9`（`ERR_OVF`），`o_count` 饱和在 `MAX_FILES-1` |
| TC3 | 2 个文件（少量） | `err_code=0`，`o_count=2`，不误报 |

镜像按真实 FAT32 结构构造：MBR（LBA 0）+ DBR（LBA 32）+ 根目录扇区（LBA 97），几何与模块内部的 `fat_lba` / `data_lba` / `root_lba` 计算一一对应。

## 容量语义

沿用改动前的既有语义：**容量 `MAX_FILES` 个槽位，实际可收纳 `MAX_FILES-1` 个条目**。

- 改前 `MAX_FILES=16` → 最多 15 个文件
- 改后 `MAX_FILES=10` → 最多 9 个文件

工程实际用 4 张 BMP + 4 段配对 WAV = 8 个文件，9 个槽位留 1 个余量。

## 怎么自己跑

```bash
cd 03_verify/fat32_tbl

# 按工程实际参数（10）验证
iverilog -o t10.vvp -g2005 tb_fat32_dir_tbl.v fat32_dir_dut.v
vvp t10.vvp
# 期望：结果：PASS

# 对照：改前的参数（16），验证参数化没有改变原有行为
iverilog -o t16.vvp -g2005 -Ptb_fat32_dir_tbl.MAX_FILES=16 tb_fat32_dir_tbl.v fat32_dir_dut.v
vvp t16.vvp
# 期望：结果：PASS，且 11 个文件时 o_count=11、err_code=0（容量足够）
```

只需要 `iverilog`。`fat32_dir_dut.v` 与工程里的 `hdl_source/fat32_dir.v` 逐字节相同，只是换了文件名以便单独编译。

## 这个验证抓到的两个真实缺陷

改动第一版写完时，两个都通过了编译器语法检查，但仿真立刻暴露出来：

| 缺陷 | 现象 | 根因 |
|---|---|---|
| `rec_idx[4:0] == TBL_LAST` | 表满后 `ERR_OVF` 永不触发，`rec_idx` 继续增长到 10 并越界写表 | `rec_idx` 是 4 位，取 `[4:0]` 是越界位选，比较恒为假。改为 `rec_idx == TBL_LAST[3:0]` |
| 复位循环 `for (k = 0; k < 16; ...)` | TD 报 `HDL-8007: index 10 is out of range [0:9]`，整个模块被降级成黑盒 | 循环上界硬编码 16。改为 `< MAX_FILES` |

第二个尤其值得记：**TD 不会因为表深度缩小而自动收窄硬编码的下标范围**，而一旦越界，它把模块当黑盒处理（`remains a black box due to errors in its contents`），此后综合结果完全不可信。
