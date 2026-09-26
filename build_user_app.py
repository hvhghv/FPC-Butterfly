#!/usr/bin/env python3
"""
用户程序一键编译 + 打包脚本 (v6)

编译用户程序 → 合并分区表 B → 输出从 0x140000 开始的**单一烧录文件**

【v6 打包方案】
    输出文件从 0x140000 起，内容:
      偏移 0x0000 (0x140000)  分区表 B     (4KB)
      偏移 0x1000 (0x141000)  0xFF 填充    (nvs 位置, 60KB)
      偏移 0x10000 (0x150000) 应用镜像     (user_app)

    IAP 收到后**整段写入 0x140000**，无需解析分区表 —— 与 esptool 语义一致。

用法:
    # 在用户程序工程目录下执行
    python build_user_app.py --target esp32c6

    # 只打包（已编译过）
    python build_user_app.py --no-build

烧录方式:
    - 通过 IAP: 发送 build/<name>_flash.bin（XMODEM / HTTP / 浏览器）
    - 用 esptool: esptool.py write_flash 0x140000 build/<name>_flash.bin

依赖: 需要已设置 ESP-IDF 环境（idf.py 可用）
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# IAP 工程根目录（本脚本位于 examples/user_app_template/ 下）
HERE = Path(__file__).resolve().parent

# --- 可变区布局 (与 tools/gen_partitions.py / iap_common.h 一致) ---
VAR_REGION_BASE   = 0x140000      # 可变区起点 (分区表 B)
PARTITION_TABLE_B = 0x140000      # 分区表 B 地址
USER_APP_ADDR     = 0x150000      # 用户程序地址
NVS_APP_ADDR      = 0x141000      # nvs 地址 (不写入, 仅占位)
ERASE_BYTE        = 0xFF


def run(cmd, **kwargs):
    """执行命令，失败即退出"""
    print(f"\n$ {' '.join(str(c) for c in cmd)}")
    result = subprocess.run(cmd, **kwargs)
    if result.returncode != 0:
        print(f"\n[错误] 命令失败 (退出码 {result.returncode})")
        sys.exit(result.returncode)


def main():
    ap = argparse.ArgumentParser(
        description="编译用户程序并打包为从 0x140000 开始的单一烧录文件")
    ap.add_argument("--target", "-t", default=None,
                    help="目标芯片 (如 esp32c6)；不指定则沿用已有 sdkconfig")
    ap.add_argument("--name", "-n", default=None,
                    help="工程名；默认取 CMakeLists.txt 中的 PROJECT_NAME")
    ap.add_argument("--output", "-o", default=None,
                    help="输出文件名；默认 <工程名>_flash.bin")
    ap.add_argument("--no-build", action="store_true",
                    help="跳过编译，直接使用已有 build/*.bin")
    ap.add_argument("--no-merge", action="store_true",
                    help="只输出纯应用镜像 (不合并分区表, 兼容旧流程)")
    ap.add_argument("--build-dir", default="build",
                    help="构建目录，默认 build")
    args = ap.parse_args()

    build_dir = Path(args.build_dir)

    # --- 推断工程名 --------------------------------------------------------
    name = args.name
    if name is None:
        cmake = HERE / "CMakeLists.txt"
        if cmake.exists():
            for line in cmake.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if line.startswith("set(PROJECT_NAME"):
                    name = line.split('"')[1]
                    break
        name = name or "user_app"

    # --- 1. 编译 -----------------------------------------------------------
    if not args.no_build:
        if args.target:
            run(["idf.py", "set-target", args.target], cwd=HERE)
        run(["idf.py", "build"], cwd=HERE)

    # --- 2. 定位产物 -------------------------------------------------------
    app_bin = build_dir / f"{name}.bin"
    if not app_bin.exists():
        candidates = [p for p in build_dir.glob("*.bin")
                      if p.name not in ("bootloader.bin", "partition-table.bin")]
        if len(candidates) == 1:
            app_bin = candidates[0]
            print(f"[提示] 未找到 {build_dir / (name + '.bin')}，改用 {app_bin.name}")
        else:
            print(f"[错误] 找不到应用 bin: {app_bin}")
            print(f"       build 目录下的 bin: {[p.name for p in build_dir.glob('*.bin')]}")
            sys.exit(1)

    # --- 3. 校验镜像头 -----------------------------------------------------
    with open(app_bin, "rb") as f:
        magic = f.read(1)
    if magic != b"\xE9":
        print(f"[错误] {app_bin.name} 首字节为 0x{magic.hex().upper()}，"
              f"不是 ESP 镜像 magic (0xE9)")
        sys.exit(1)

    # --- 4. 输出 -----------------------------------------------------------
    out = Path(args.output) if args.output else Path(f"{name}_flash.bin")

    if args.no_merge:
        # 兼容旧流程: 纯应用镜像
        shutil.copy2(app_bin, out)
        merged = False
    else:
        # v6: 合并分区表 B + 应用镜像 -> 从 0x140000 开始
        pt_bin = build_dir / "partition_table" / "partition-table.bin"
        if not pt_bin.exists():
            print(f"[错误] 找不到分区表 B: {pt_bin}")
            print("       请先执行 idf.py build（会生成 partition_table/partition-table.bin）")
            sys.exit(1)

        pt_data = pt_bin.read_bytes()
        app_data = app_bin.read_bytes()

        # 分区表必须放得下 (到 user_app 之前)
        if len(pt_data) > (USER_APP_ADDR - PARTITION_TABLE_B):
            print(f"[错误] 分区表过大: {len(pt_data)} 字节")
            sys.exit(1)

        total = USER_APP_ADDR - VAR_REGION_BASE + len(app_data)
        buf = bytearray([ERASE_BYTE] * total)

        # 分区表 B @ 0x140000
        buf[0:len(pt_data)] = pt_data
        # 应用镜像 @ 0x150000
        app_off = USER_APP_ADDR - VAR_REGION_BASE
        buf[app_off:app_off + len(app_data)] = app_data

        with open(out, "wb") as f:
            f.write(buf)
        merged = True

    # --- 5. 汇总 -----------------------------------------------------------
    print("\n" + "=" * 60)
    print("  编译完成")
    print("=" * 60)
    print(f"  应用 bin : {app_bin}  ({app_bin.stat().st_size} 字节)")
    print(f"  输出文件 : {out}  ({out.stat().st_size} 字节)")
    if merged:
        print(f"  镜像头   : 0xE9 (ESP 应用镜像) @ 0x150000")
        print(f"  分区表 B : 已合并 @ 0x140000")
        print()
        print("  文件布局 (从 0x140000 开始):")
        print(f"    0x140000  分区表 B     ({len(pt_data)} 字节)")
        print(f"    0x141000  0xFF 填充    (nvs 位置, 不写入)")
        print(f"    0x150000  应用镜像     ({len(app_data)} 字节)")
        print(f"    总计      {out.stat().st_size} 字节")
    else:
        print(f"  镜像头   : 0xE9 (纯 ESP 应用镜像, 未合并分区表)")
    print()
    print("  烧录方式:")
    if merged:
        print(f"    IAP     : 发送 {out}（XMODEM / HTTP / 浏览器）")
        print(f"              IAP 会整段写入 0x140000")
        print(f"    esptool : esptool.py --chip <chip> write_flash 0x140000 {out}")
    else:
        print(f"    IAP     : 发送 {out}（XMODEM / HTTP / 浏览器）")
        print(f"    esptool : esptool.py --chip <chip> write_flash 0x150000 {out}")
    print()
    print("  说明: 镜像完整性由 bootloader 启动时校验，IAP 侧不再重复校验。")
    print("        用户程序**不访问配置区** (v5)，配置通过 RTC RAM 启动参数传递。")
    print()


if __name__ == "__main__":
    main()
