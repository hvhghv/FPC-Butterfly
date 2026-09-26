# 蝴蝶板灯珠 HTTP 控制应用

ESP32-C6 蝴蝶板 4 颗 RGB 灯珠的 HTTP 控制程序，基于 **ESP IAP 用户程序**架构，
由 IAP 程序切换启动分区后由 bootloader 加载，通过 WiFi SoftAP 提供 Web 控制界面与 REST API。

> **注意**：新版 IAP 使用**纯 ESP 应用镜像**（无自定义文件头），
> 因此 `idf.py build` 产物**无需打包**，可直接烧录。
> 镜像合法性由 bootloader 自校验（magic / 段表 / SHA256 / chip_id），
> 校验失败时自动回落 factory (IAP)，不会变砖。

## 硬件

4 颗共阴极 RGB 灯珠，每颗 3 路 PWM，共 12 路：

| 灯珠 | 位置坐标 | 红色 | 绿色 | 蓝色 | 公共阴极 |
|------|----------|------|------|------|----------|
| D1 左上翅膀 | (17, 22) mm | IO4 | IO5 | IO6 | GND |
| D2 左下翅膀 | (22, 39) mm | IO0 | IO1 | IO7 | GND |
| D3 右上翅膀 | (53, 22) mm | IO18 | IO19 | IO20 | GND |
| D4 右下翅膀 | (48, 39) mm | IO21 | IO22 | IO23 | GND |

**驱动方式**: 双硬件 PWM 外设（LEDC + MCPWM），12 路零 CPU 占用

**控制粒度**: 每颗灯珠的颜色、亮度、效果、效果周期、开关均**独立可调**，
可同时运行不同效果（如 D1 呼吸 + D2 闪烁 + D3 彩虹 + D4 静态）。
每颗灯珠还可编排**效果序列**（如 1s 静态 → 1s 闪烁 → 1s 呼吸 循环），
最多 8 步。全局接口保留为「一键同步到全部灯珠」的便捷封装。

ESP32-C6 的单个 PWM 外设都不够 12 路，但两个外设合计正好够：

| 外设 | 通道数 | 依据 |
|------|--------|------|
| LEDC | 6 | `soc_caps.h: SOC_LEDC_CHANNEL_NUM = 6` |
| MCPWM | 6 | `mcpwm_ll.h`: 3 操作器 × 2 比较器/生成器 |
| **合计** | **12** | ✅ |

通道分配：

| 通道 | 灯珠 | 驱动方式 | 分辨率 | 载波 |
|------|------|----------|--------|------|
| 0-5 | D1 全部 + D2 全部 | **LEDC 硬件 PWM** | 12 位 | 5 kHz（可调） |
| 6-11 | D3 全部 + D4 全部 | **MCPWM 硬件 PWM** | 12 位 | 5 kHz（可调） |

> 两者都是**纯硬件 PWM**，运行期 CPU 占用 **0%**，无抖动。
> 共阴极接法：占空比越大越亮，占空比 0 为熄灭。
> 若实际硬件为公共阳极接 VCC，调用 `led_ctrl_set_invert(true)` 即可反转输出逻辑。

### MCPWM 波形生成原理

每个生成器配置两条规则：

```
定时器事件 EMPTY (计数归零) -> 输出 HIGH   (周期开始)
比较器事件 (计数 == duty)   -> 输出 LOW    (占空比到达)
```

6 路共用 1 个定时器，因此载波相位完全同步。

### 演进历史

本项目曾用 `esp_timer` 中断软件 PWM 补足后 6 路，但发现 MCPWM 可提供
6 路独立硬件 PWM 后改为当前方案。对比：

| 方案 | 硬件 PWM | CPU 占用 | 抖动 |
|------|----------|----------|------|
| 软件 PWM | 6/12 | ~1.2% | 有 |
| **LEDC + MCPWM** | **12/12** | **0%** | **无** |

## 目录结构

```
APP/
├── CMakeLists.txt              # 顶层工程 (PROJECT_NAME=led_butterfly)
├── sdkconfig.defaults          # 关键配置（分区表、WiFi、BLE、HTTP）
├── partitions_user_app.csv     # 分区表（必须与 IAP 一致）
├── build_user_app.py           # 一键编译脚本（无需打包）
├── .github/workflows/
│   ├── build.yml               # 编译固件 + 打 tag 自动发布 Release
│   └── pages.yml               # 部署 Web 页面到 GitHub Pages
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                  # 应用入口、IAP 对接、上电加载配置、自检动画
│   ├── iap_user_api.c/.h       # IAP 对接接口（从 IAP 工程复制）
│   ├── led_ctrl.c/.h           # 12 路双硬件 PWM 驱动 (LEDC+MCPWM) + 逐颗效果引擎
│   │                           #   含 NVS 配置持久化 (save/load/clear_saved)
│   ├── app_wifi.c/.h           # WiFi AP/STA/APSTA（可配置，存 NVS）
│   ├── app_cmd.c/.h            # 命令层（传输无关，HTTP 与 BLE 共用）
│   ├── app_http.c/.h           # HTTP 服务（传输层）
│   ├── app_ble.c/.h            # BLE GATT 服务（传输层）
│   └── web/
│       └── index.html          # Web 控制界面（编译时自动嵌入固件）
└── README.md
```

## 自动构建 (GitHub Actions)

仓库内置两个 workflow，推送后自动运行。

### `build.yml` — 编译固件

| 触发条件 | 行为 |
|---------|------|
| push 到 `main` | 编译 + 上传 artifact |
| PR 到 `main` | 编译 + 上传 artifact |
| 手动触发 | 同上 |
| **打 tag (`v*`)** | 编译 + **自动创建 GitHub Release** |

产物（artifact 名 `led-butterfly-esp32c6`，保留 30 天）：

| 文件 | 用途 |
|------|------|
| `led_butterfly_flash.bin` | **烧录文件**。从 `0x140000` 起，含分区表 B + 应用镜像 |
| `index.html` | Web 控制页面 |
| `SHA256SUMS.txt` | 校验和 |

> **只有一个烧录文件** —— IAP 与 esptool 都是从 `0x140000` **整段写入**，
> 语义一致，因此无需单独提供纯应用镜像。
>
> - IAP：把 `led_butterfly_flash.bin` 发送给设备，IAP 整段写入 `0x140000`
> - esptool：`esptool.py --chip esp32c6 write_flash 0x140000 led_butterfly_flash.bin`

**IDF 版本锁定 `v6.0.3`**，与本地开发环境一致。

构建流程：`idf.py build` → `build_user_app.py --no-build` 打包 →
校验产物 → 上传。校验步骤会检查镜像头 `0xE9`、烧录文件在 `0x150000`
处的镜像头、文件大小，任一不符即失败。

### `pages.yml` — 部署 Web 页面

`main/web/` 变更时自动部署到 GitHub Pages。

> **为什么需要 Pages**：Web Bluetooth 只能在安全上下文（https / localhost）
> 中使用。设备自身的 `http://192.168.4.1/` 页面**无法**调用蓝牙。
> 部署到 Pages 后页面是 https，手机/电脑打开即可用蓝牙直连设备，
> 且无需与设备在同一网络。

**首次使用需在仓库设置里开启**：`Settings → Pages → Source` 选择
**GitHub Actions**（不是 `gh-pages` 分支）。

### 发布新版本

```bash
git tag v1.0.0
git push origin v1.0.0
```

推送 tag 后会自动编译并创建 Release，附上烧录文件与校验和。

## Web 控制界面

页面源码是独立的 `main/web/index.html`，**不内嵌在 C 代码中**。
编译时由 ESP-IDF 的 `EMBED_FILES` 自动转成二进制并链接进固件：

```
main/web/index.html  →  _binary_index_html_start / _binary_index_html_end
```

`app_http.c` 通过这两个符号直接把页面发给浏览器，因此**修改界面只需编辑
HTML 文件后重新编译**，无需再手工转义 C 字符串。

> 注意：嵌入数据不带结尾 `\0`，发送时必须显式传长度
> （`index_html_end - index_html_start`），不能用 `HTTPD_RESP_USE_STRLEN`。

若以后 `web/` 下文件变多，可在 `main/CMakeLists.txt` 中改用
`target_add_binary_data(${COMPONENT_LIB} "web/index.html" TEXT)` 一次性嵌入。

## 快速开始

### 1. 编译

```bash
idf.py set-target esp32c6
idf.py build
```

> ⚠️ **工程路径不能含中文**。ESP-IDF 的 CMake 在含非 ASCII 字符的路径下会
> 静默崩溃（退出码 `0xC0000409`，无任何输出）。若路径含中文，请先复制到
> 纯 ASCII 路径（如 `C:\work\APP`）再编译。

### 2. 输出可烧录镜像（无需打包）

```bash
python build_user_app.py --target esp32c6 --name led_butterfly
```

生成**一个**烧录文件：

| 产物 | 大小 | 内容 |
|------|------|------|
| `led_butterfly_flash.bin` | ~1.1 MB | 分区表 B + 应用镜像 |

> 新版 IAP 不再使用自定义文件头，**不需要** `iap_pack.py` 打包。
>
> **IAP 与 esptool 都是从 `0x140000` 整段写入**，语义一致，
> 因此只需这一个文件。IAP 收到后不做解析，直接整段写入。

### 3. 烧录

| 通道 | 命令 |
|------|------|
| **UART** | `iap> xmodem recv` 然后发送 `led_butterfly_flash.bin` |
| **WiFi** | `curl -X POST --data-binary @led_butterfly_flash.bin http://192.168.4.1/api/upload` |
| **浏览器** | 打开 `esp_iap_tool.html` → 「选择文件」→「烧录到设备」 |
| **直接写 Flash** | `esptool.py write_flash 0x140000 led_butterfly_flash.bin` |

> 所有方式都烧 **`0x140000`**（分区表 B 起始）。
> `led_butterfly_flash.bin` 的布局如下：
>
> | 偏移 | 内容 |
> |------|------|
> | `0x140000` | 分区表 B（4KB） |
> | `0x141000` | 0xFF 填充（nvs 位置，不写入） |
> | `0x150000` | 应用镜像（user_app） |
>
> 切勿对 `0x0` ~ `0x140000` 区域做任何写入——那是 bootloader、
> `iap_cfg`、分区表 A、IAP 程序的领地。
>
> 镜像合法性由 bootloader 启动时自校验，IAP 侧不重复校验。
> 版本号由用户程序调用 `iap_user_report_version()` 上报到配置区。

### 4. 启动并访问控制页面

```
iap> app boot
```

设备启动后会执行上电自检动画（依次点亮 D1-D4 的红/绿/蓝），然后启动 WiFi AP。

1. 连接 WiFi：SSID `ESP-LED`，密码 `12345678`（首次启动的默认值）
2. 浏览器打开：**http://192.168.4.1/**

> 连上后可在 Web 界面的「WiFi 热点设置」卡片中修改 SSID / 密码 / 信道 /
> 连接数 / 隐藏 / IP，配置会存入 NVS 并持久保存。

---

## Web 控制界面

页面提供：

- **单颗灯珠控制**：每颗灯珠独立的 R/G/B 滑条 + 取色器，实时生效
- **逐颗高级控制**：每颗灯珠独立的亮度 / 效果 / 周期 / 开关
- **效果序列**：每颗灯珠可编排最多 8 步的效果组合循环
  （如 1s 静态 → 1s 闪烁 → 1s 呼吸）
- **快捷预设**：全红 / 全绿 / 全蓝 / 全白 / 全部熄灭 / 全部同步为 D1 颜色
- **一键同步**：把亮度 / 效果 / 周期 / PWM 频率应用到全部灯珠
- **PWM 频率**：100-40000 Hz 可调（LEDC 与 MCPWM 同步改频）
- **WiFi 设置**：模式 (AP/STA/APSTA) / SSID / 密码 / 信道 / 连接数 / 隐藏 / IP，
  以及连接路由器（含扫描周边热点、静态 IP、手动重连）
- **连接方式**：WiFi (HTTP) / 蓝牙 (BLE) 通道切换，页面自动适配
- **蓝牙配对**：可选 6 位配对码（默认不启用），支持设置/修改/清除
- **设备状态**：WiFi 热点 / 路由器 / 蓝牙 / 系统四张状态卡，含信号强度与信道
- **配置导入 / 导出**：导出为 JSON 文件、显示为文本、从文件或文本框导入；
  可选「包含 WiFi 密码」用于完整迁移（含安全警告与二次确认）
- **配置持久化**：点「保存当前配置」把灯珠状态（颜色/亮度/效果/序列/频率）
  手动保存到 NVS，断电重启后自动恢复上次保存的配置；也可清除已保存配置
- **设备信息**：芯片型号、IDF 版本、程序版本、MAC、空闲堆、运行时间、
  启动计数、IAP 上报版本号、AP 信息、接入客户端数
- **操作按钮**：刷新状态 / 重启设备 / 进入 IAP 下载模式

---

## 蓝牙控制 (BLE)

除 WiFi 外，设备还提供 **BLE GATT** 控制通道，供浏览器
(Web Bluetooth API) 或手机 App 直接连接，无需切换 WiFi。

### 架构：双通道共用命令层

```
      传输层                    命令层              业务层
   ┌──────────┐            ┌─────────────┐     ┌──────────┐
   │ HTTP     │──JSON────▶ │             │───▶ │ led_ctrl │
   │ (app_http)│◀──JSON──── │ app_cmd     │ ◀── │ app_wifi │
   ├──────────┤            │             │     └──────────┘
   │ BLE      │──JSON────▶ │             │
   │ (app_ble)│◀──JSON──── │             │
   └──────────┘            └─────────────┘
```

两条通道**共用同一套命令实现** (`app_cmd.c`)，因此新增功能只需写一次。
BLE 侧只是把收到的 JSON 交给 `app_cmd_execute_src()`。

### 配对码

可为蓝牙启用 6 位数字配对码，启用后客户端连接需输入该码。

**默认不启用** —— 首次启动时配对码为空，客户端可直接连接。
用户可随时设置、修改或清除配对码。

| 方法 | 路径 | 请求体 | 说明 |
|------|------|--------|------|
| POST | `/api/ble/pin` | `{"get":1}` | 查询配对码与启用状态 |
| POST | `/api/ble/pin` | `{"pin":"123456"}` | 设置或修改配对码（6 位数字） |
| POST | `/api/ble/pin` | `{"pin":""}` 或 `{"enabled":0}` | 关闭配对 |

响应统一为 `{"ok":true,"enabled":bool,"pin":"..."}`。
未启用时 `enabled` 为 `false`、`pin` 为空串。

```bash
# 查询当前状态
curl -X POST -d '{"get":1}' http://192.168.4.1/api/ble/pin
# -> {"ok":true,"enabled":false,"pin":""}

# 设置配对码
curl -X POST -d '{"pin":"123456"}' http://192.168.4.1/api/ble/pin

# 修改配对码 (同样用 pin 字段)
curl -X POST -d '{"pin":"654321"}' http://192.168.4.1/api/ble/pin

# 关闭配对
curl -X POST -d '{"pin":""}' http://192.168.4.1/api/ble/pin
```

> 配对码存 NVS（命名空间 `blecfg`），掉电保存。
> 校验规则：必须是 **6 位纯数字**，否则返回 `400`。

#### 权限模型

配对码是敏感信息，读取与修改按**来源通道**鉴权：

| 来源 | 权限 |
|------|------|
| HTTP | 允许（已通过 WiFi 密码保护） |
| BLE **已配对**（链路加密） | 允许 |
| BLE **未配对** | 拒绝，返回「读取配对码需先完成配对」 |
| 本地（串口/内部） | 允许 |

这样「连接后获取配对码」的语义才成立：先输入配对码完成配对，
建立加密链路后即可读取。未配对的连接读不到，避免绕过配对机制。

> 未启用配对时不受此限制 —— 此时不存在配对码，也就没有"绕过"问题。

> 实现上用 `ble_gap_conn_find()` 读连接的 `sec_state.encrypted`
> 判断链路是否加密 —— 这是协议栈在配对完成后更新的真实状态。

配对码启用后，设备的安全参数为：`sm_io_cap = DISP_ONLY`（显示方）、
`mitm = 1`（要求中间人保护）、`bonding = 1`（绑定避免重复输入）、
`sm_sc = 1`（LE Secure Connections）。

### GATT 结构

| 特征 | UUID | 属性 | 说明 |
|------|------|------|------|
| Service | `6c6f6f70-0001-4c45-4400-000000000000` | — | 主服务 |
| RX | `6c6f6f70-0002-...` | Write | 客户端写入命令 JSON |
| TX | `6c6f6f70-0003-...` | Read / Notify | 设备回传响应 JSON |

设备名：`Butterfly-LED`

### 广播内容

| 位置 | 内容 | 说明 |
|------|------|------|
| 广播包 (ADV_IND) | flags + 设备名 + **16 位 UUID `0xFFE0`** | 22 字节，不超 31 字节上限 |
| scan response | 128 位服务 UUID | 手机扫描时可识别服务 |

> **为什么广播包里放 16 位 UUID？**
>
> Web Bluetooth 的 `requestDevice({filters:[{services:[...]}]})` 在
> Chrome/Edge 上**只解析广播包，不解析 scan response**。128 位 UUID
> 占 18 字节，与设备名一起会超出 31 字节上限，只能放 scan response，
> 于是浏览器按服务过滤时找不到设备（手机系统蓝牙会读 scan response，
> 所以能搜到）。
>
> 因此广播包里额外放一个 16 位 UUID `0xFFE0` 供浏览器过滤；实际
> GATT 服务仍是上面的 128 位 UUID。前端 `filters` 同时匹配
> `0xFFE0` 与设备名前缀 `Butterfly-LED`，双保险。

### 分片协议

BLE 单包受 MTU 限制（默认 20 字节，协商后最大 244），因此命令按
**长度前缀**分片：

```
首片: [总长度 2 字节小端][数据...]
后续: [数据...]
```

响应方向不带长度前缀，客户端按 JSON 结尾的 `}` 判断收齐。

### 命令格式

与 HTTP API 一一对应，只是把路径换成 `cmd` 字段：

| HTTP | BLE 命令 |
|------|---------|
| `GET /api/status` | `{"cmd":"status"}` |
| `POST /api/led` | `{"cmd":"led.set","id":0,"r":255}` |
| `POST /api/led/effect` | `{"cmd":"led.effect","id":0,"name":"breath"}` |
| `POST /api/led/sequence` | `{"cmd":"led.sequence","id":0,"steps":[...]}` |
| `POST /api/wifi` | `{"cmd":"wifi.set","sta_ssid":"HomeWiFi"}` |
| `GET /api/wifi` | `{"cmd":"wifi.get"}` |
| `POST /api/wifi/scan` | `{"cmd":"wifi.scan"}` |
| `POST /api/off` | `{"cmd":"off"}` |

完整命令名：`status` / `led.set` / `led.effect` / `led.brightness` /
`led.enable` / `led.sequence` / `all` / `brightness` / `effect` / `off` /
`freq` / `wifi.get` / `wifi.set` / `wifi.scan` / `wifi.reconnect` /
`wifi.reset` / `wifi.enable` / `ble.enable` / `ble.pin` /
`config.export` / `config.import` / `reboot` / `upgrade`

响应统一为 `{"ok":true,...}` 或 `{"ok":false,"error":"..."}`。

### 浏览器使用限制

> ⚠️ **Web Bluetooth 只能在安全上下文 (`https://` 或 `localhost`) 中使用。**
> 设备自身的 `http://192.168.4.1/` **不能**用蓝牙 —— 浏览器会禁用该 API。

| 页面来源 | 能否用蓝牙 |
|---------|-----------|
| `http://192.168.4.1/` (设备页面) | ❌ |
| `https://` (自签证书) | ⚠️ 需手动信任 |
| `http://localhost/` (本地服务) | ✅ |
| GitHub Pages 等 https 站点 | ✅ |

浏览器支持：**Chrome / Edge / Opera** (桌面 + Android)。
**Firefox、Safari、iOS 全部不支持**，这是规范层面的限制。

Web 界面已内置「连接方式」切换（WiFi / 蓝牙），在支持的浏览器中
直接点「连接蓝牙」即可。

### 内存与射频

BLE 与 WiFi 共用 2.4 GHz 射频，已开启软件共存 (`CONFIG_SW_COEXIST_ENABLE`)。

| 项目 | 无 BLE | 有 BLE |
|------|--------|--------|
| DRAM | 20.4% | 39.8% |
| IRAM | 62.9% | 67.1% |
| Flash | 726KB | 898KB |

> IRAM 通过关闭 WiFi/BLE 的 IRAM 优化换回约 25KB
> （本项目无 ISR 参与灯控，代码放 flash 执行无影响）。

### 配置导入 / 导出

把整机配置导出为一份 JSON，可在另一台设备导入，或作为备份。

| 方法 | 路径 | 请求体 | 说明 |
|------|------|--------|------|
| GET | `/api/config` | — | 导出配置（**不含密码**） |
| POST | `/api/config/export` | `{"secrets":1}` | 导出配置（**含密码明文**） |
| POST | `/api/config` | 导出的 JSON | 导入配置 |
| POST | `/api/config/save` | — | 保存当前配置到 NVS |
| POST | `/api/config/reset` | — | 清除已保存配置（下次上电用默认值） |

导出内容：

```json
{
  "v": 1,
  "freq": 5000,
  "invert": false,
  "secrets": false,
  "leds": [
    {"id":1,"r":255,"g":0,"b":0,"brightness":255,"effect":"none",
     "period":2000,"enabled":true,
     "steps":[{"effect":"none","duration":1000,"period":0},
              {"effect":"blink","duration":1000,"period":0},
              {"effect":"breath","duration":1000,"period":500}]},
    ...
  ],
  "wifi": {"mode":2,"ssid":"ESP-LED","channel":6,"maxconn":4,
           "hidden":false,"ip":"192.168.4.1",
           "secure":true,"passlen":8,
           "sta_ssid":"HomeWiFi","sta_secure":true,"sta_passlen":10}
}
```

- **`v`** 为格式版本，导入时若版本高于固件支持会拒绝
- **`secrets`** 标记该文件是否含密码明文
- 导入时逐字段校验并收敛到合法范围，未知效果名降级为 `none`
- 导入会**覆盖**当前灯珠与 WiFi 设置

#### 密码处理

| 场景 | 导出命令 | 密码行为 |
|------|---------|---------|
| 普通备份 / 分享 | `GET /api/config` | 只导出 `secure`/`passlen` 标记，**无明文** |
| 完整迁移 | `POST /api/config/export` + `{"secrets":1}` | 导出 `password` / `sta_password` 明文 |

导入时若配置**不含**密码字段，则**保留设备当前密码** ——
因此普通备份导入到新设备后，需手动设置一次密码。

> ⚠️ 含密码的配置文件请妥善保管，不要分享或上传到公开位置。
> Web 界面勾选「包含 WiFi 密码」时会显示警告并要求二次确认，
> 下载文件名也会带 `-with-secrets` 后缀以便识别。

```bash
# 普通导出（不含密码）
curl http://192.168.4.1/api/config > backup.json

# 完整迁移导出（含密码）
curl -X POST -d '{"secrets":1}' http://192.168.4.1/api/config/export > full.json

# 从文件导入
curl -X POST --data-binary @full.json http://192.168.4.1/api/config
```

> Web 界面「配置导入 / 导出」卡片支持三种方式：
> 下载文件、显示为文本（可复制）、从文件或文本框导入。

### 配置持久化（断电保存）

灯珠状态采用**手动保存**：调整后点「保存当前配置」才写入 NVS，
断电重启后自动恢复。不保存则重启后回到上次保存的状态。

| 项目 | 存储位置 | 说明 |
|------|---------|------|
| 灯珠状态 | NVS 命名空间 `ledcfg`，键 `state` | 颜色 / 亮度 / 效果 / 周期 / 序列 / 频率 / 反转 / 熄灭 |
| WiFi 配置 | NVS 命名空间 `wificfg` | 模式 / SSID / 密码 / 信道 / 静态 IP 等 |
| 蓝牙配对码 | NVS 命名空间 `blecfg` | 6 位配对码 |

**手动保存**：只有 `config.save` 命令（前端「保存当前配置」按钮）
会写入 NVS。其他命令（`led.set` / `led.effect` / `led.brightness` /
`led.enable` / `led.sequence` / `all` / `brightness` / `effect` /
`off` / `freq`）仅改变运行状态、**不落盘**，避免拖动滑条时频繁写 flash。

**上电加载**：`main.c` 启动流程中，`led_ctrl_init()` 之后立即调用
`led_ctrl_load()`；若有已保存记录，直接恢复上次状态并**跳过自检动画**，
否则播放自检动画并使用默认值。

**手动控制**：

```bash
# 保存当前配置（唯一保存入口）
curl -X POST http://192.168.4.1/api/config/save

# 清除已保存配置（下次上电恢复默认值，当前显示不变）
curl -X POST http://192.168.4.1/api/config/reset
```

> 存储格式为带版本号的 blob（当前 `LED_NVS_VERSION = 1`），
> 结构变更时递增版本号即可让旧数据自动失效，回退到默认值。

### 命令名对照 (BLE)

BLE 通道使用同一套命令名：

| HTTP | BLE 命令 |
|------|---------|
| `GET /api/config` | `{"cmd":"config.export"}` |
| `POST /api/config/export` | `{"cmd":"config.export","secrets":1}` |
| `POST /api/config` | `{"cmd":"config.import", ...}` |
| `POST /api/config/save` | `{"cmd":"config.save"}` |
| `POST /api/config/reset` | `{"cmd":"config.reset"}` |

---

## REST API

所有 POST 请求体为 JSON，响应也是 JSON。颜色值为 0-255。

### 逐颗控制

每颗灯珠都有**独立**的颜色、亮度、效果、效果周期与开关，
可以同时运行不同效果（例如 D1 呼吸、D2 闪烁、D3 彩虹、D4 静态）。

| 方法 | 路径 | 请求体 | 说明 |
|------|------|--------|------|
| POST | `/api/led` | `{"id":0,"r":255,"g":0,"b":0,"brightness":128,"effect":"breath","period":2000,"enabled":true}` | 设置单颗灯珠完整状态（除 `id` 外字段均可选，缺省表示保持当前值） |
| POST | `/api/led/effect` | `{"id":0,"name":"breath","period":2000}` | 只改某颗灯珠的效果 |
| POST | `/api/led/brightness` | `{"id":0,"value":128}` | 只改某颗灯珠的亮度 |
| POST | `/api/led/enable` | `{"id":0,"enabled":true}` | 启用/禁用某颗灯珠（禁用后输出全灭，设置保留） |
| POST | `/api/led/sequence` | `{"id":0,"steps":[{"effect":"none","duration":1000},{"effect":"blink","duration":1000},{"effect":"breath","duration":1000,"period":500}]}` | 设置效果序列（多效果组合循环），`steps:[]` 清空 |

### 效果序列（多效果组合）

把若干效果按时长依次播放，播完最后一步自动回到第一步，**无限循环**。
例如「1 秒静态 → 1 秒闪烁 → 1 秒呼吸」：

```json
{"id":0,"steps":[
  {"effect":"none",  "duration":1000},
  {"effect":"blink", "duration":1000},
  {"effect":"breath","duration":1000,"period":500}
]}
```

| 字段 | 必填 | 说明 |
|------|------|------|
| `effect` | 是 | 该步效果名称 |
| `duration` | 是 | 该步持续时长，10 - 600000 ms |
| `period` | 否 | 该步内效果的周期，0 或缺省表示与 `duration` 相同 |

- 每颗灯珠**独立**拥有自己的序列，最多 **8 步**
- 序列非空时**覆盖**该灯珠的单效果设置；清空后回退到单效果
- `period` 用于控制该步内效果的快慢，例如 3 秒的呼吸步骤设
  `period:3000` 就是一次完整呼吸，设 `period:750` 则呼吸 4 次

> Web 界面每颗灯珠卡片下方有可视化的序列编辑器，可增删步骤并一键应用。

### 全局控制（一键同步到所有灯珠）

| 方法 | 路径 | 请求体 | 说明 |
|------|------|--------|------|
| GET | `/` | — | Web 控制界面 |
| GET | `/api/status` | — | 灯珠状态 + 系统信息 |
| POST | `/api/all` | `{"r":255,"g":0,"b":0}` | 设置全部灯珠颜色 |
| POST | `/api/brightness` | `{"value":128}` | 全部灯珠亮度 (0-255) |
| POST | `/api/effect` | `{"name":"breath","period":2000}` | 全部灯珠效果与周期 |
| POST | `/api/off` | `{}` | 全部熄灭（同时关闭效果） |
| POST | `/api/freq` | `{"freq":5000}` | PWM 频率 (100-40000) |
| POST | `/api/reboot` | `{}` | 重启设备 |
| POST | `/api/upgrade` | `{}` | 重启进入 IAP 下载模式 |

### 设备状态查询

`GET /api/status` 返回灯珠状态、系统信息、WiFi 与蓝牙状态：

```json
{
  "chip": "ESP32-C6", "cores": 1, "idf": "v6.0.3",
  "app": "led_butterfly v1", "mac": "AA:BB:CC:DD:EE:FF",
  "heap": 123456, "uptime": 3725,
  "boottarget": 1, "iapver": "1.0.0", "cfgaddr": 0,

  "ssid": "ESP-LED", "ip": "192.168.4.1", "clients": 2,

  "ble": {
    "running": true, "connected": true, "name": "Butterfly-LED",
    "paired": true, "pairing": true
  },

  "wifi": {
    "mode": 2, "ap_clients": 2,
    "sta_enabled": true, "sta_connected": true, "sta_static": true,
    "sta_ssid": "HomeWiFi", "sta_ip": "192.168.1.100",
    "sta_gw": "192.168.1.1", "sta_mask": "255.255.255.0",
    "sta_dns": "8.8.8.8", "sta_rssi": -52, "sta_channel": 6
  },

  "count": 4, "leds": [...],
  "brightness": 255, "effect": "none", "period": 2000,
  "freq": 5000, "invert": false, "off": false
}
```

| 字段 | 说明 |
|------|------|
| `heap` | 空闲堆字节数 |
| `uptime` | 运行秒数 |
| `ble.running` | BLE 服务是否启动 |
| `ble.connected` | 是否有客户端已连接 |
| `ble.paired` | 当前连接是否已配对（链路加密） |
| `ble.pairing` | 是否启用了配对码 |
| `wifi.mode` | 0=AP 1=STA 2=APSTA |
| `wifi.sta_connected` | 是否已连上路由器 |
| `wifi.sta_static` | 是否使用静态 IP |
| `wifi.sta_rssi` | 信号强度 dBm（未连接为 0） |
| `wifi.sta_channel` | 当前信道（未连接为 0） |

> Web 界面「设备信息」卡片用四张状态卡直观展示这些数据：
> WiFi 热点、路由器连接、蓝牙、系统，各带状态指示灯。

### WiFi 配置

支持 **AP / STA / APSTA** 三种模式：

| 模式 | 值 | 说明 |
|------|-----|------|
| 仅热点 | `0` | 默认，设备自己开热点，最安全 |
| 仅连路由器 | `1` | ⚠️ 连不上路由器将无法访问页面，只能靠串口 |
| 热点 + 路由器 | `2` | **推荐**，保留热点兜底，同时接入局域网 |

| 方法 | 路径 | 请求体 | 说明 |
|------|------|--------|------|
| GET | `/api/wifi` | — | 读取配置与 STA 状态（密码不回传明文） |
| POST | `/api/wifi` | `{"mode":2,"ssid":"MyLED","password":"12345678","channel":6,"maxconn":4,"hidden":false,"ip":"192.168.4.1","sta_ssid":"HomeWiFi","sta_password":"mypass"}` | 修改配置并立即生效（字段均可选，缺省保持原值） |
| POST | `/api/wifi/scan` | `{}` | 扫描周边热点（阻塞 2-4 秒） |
| POST | `/api/wifi/reconnect` | `{}` | 手动触发 STA 重连 |
| POST | `/api/wifi/reset` | `{}` | 恢复默认配置并重启（清除 NVS 用户配置） |
| POST | `/api/wifi/enable` | `{"enabled":0,"confirm":1}` | 启用/禁用 WiFi（**需 `confirm:1`**） |
| POST | `/api/ble/enable` | `{"enabled":0,"confirm":1}` | 启用/禁用蓝牙（**需 `confirm:1`**） |

#### 服务开关

关闭服务是有风险的操作，因此**必须显式传 `confirm:1`**，否则拒绝执行：

```bash
# 关闭蓝牙
curl -X POST -d '{"enabled":0,"confirm":1}' http://192.168.4.1/api/ble/enable

# 重新启用
curl -X POST -d '{"enabled":1,"confirm":1}' http://192.168.4.1/api/ble/enable
```

> ⚠️ **关闭 WiFi 会立即断开当前连接**。若蓝牙也未启用，
> 设备将只能通过串口恢复。响应会带 `warning` 字段明确提示。
> Web 界面点击开关时会弹出二次确认。

参数校验：

| 字段 | 范围 | 说明 |
|------|------|------|
| `mode` | 0 / 1 / 2 | 工作模式 |
| `ssid` | 1-32 字符 | AP 热点名（仅 AP 模式校验） |
| `password` | 8-63 字符，或空串 | AP 密码；空串 = 开放网络；**不传 = 保持原密码** |
| `channel` | 1-13 | |
| `maxconn` | 1-10 | 最大客户端数 |
| `hidden` | 0 / 1 | 是否隐藏 SSID |
| `ip` | 合法 IPv4 | AP 自身地址，必须以 `.1` 结尾 |
| `sta_ssid` | 1-32 字符 | 路由器 SSID（仅 STA 模式校验） |
| `sta_password` | 8-63 字符，或空串 | 路由器密码；空串 = 开放网络 |
| `sta_static` | 0 / 1 | STA 是否用静态 IP（0 = DHCP，默认） |
| `sta_ip` | 合法 IPv4 | 静态 IP |
| `sta_mask` | 合法 IPv4 | 子网掩码 |
| `sta_gw` | 合法 IPv4 | 网关 |
| `sta_dns` | 合法 IPv4 或空 | DNS；留空则用网关 |

#### STA 静态 IP

`sta_static` 为 `1` 时使用静态 IP，校验规则：

- `sta_ip` / `sta_mask` / `sta_gw` **三项必填**
- IP **不能与网关相同**
- IP 与网关**必须在同一子网**（`ip & mask == gw & mask`）
- `sta_dns` 可留空，此时用网关作 DNS

```bash
# 启用静态 IP
curl -X POST -d '{"sta_static":1,"sta_ip":"192.168.1.100",
                  "sta_mask":"255.255.255.0","sta_gw":"192.168.1.1"}' \
  http://192.168.4.1/api/wifi

# 切回 DHCP
curl -X POST -d '{"sta_static":0}' http://192.168.4.1/api/wifi
```

> 实现上，静态 IP 需先 `esp_netif_dhcpc_stop()` 再
> `esp_netif_set_ip_info()` —— IDF 的 DHCP 客户端与静态 IP 互斥，
> 不停 DHCP 直接设 IP 会返回 `ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED`。

- 配置写入 NVS（命名空间 `wificfg`），**掉电保存**
- 修改 SSID / 密码 / 信道 / 模式会**断开当前连接**，需用新参数重新连接
- STA 断线会自动重连，最多 **8 次**（每次间隔 3 秒）；超过后可手动重连
- 参数优先级：**NVS 用户配置 > IAP 启动参数 > 内置默认值**

```bash
# 查看当前配置与 STA 状态
curl http://192.168.4.1/api/wifi

# 切到 APSTA 并连接家里的路由器
curl -X POST -d '{"mode":2,"sta_ssid":"HomeWiFi","sta_password":"mypass"}' \
  http://192.168.4.1/api/wifi

# 扫描周边热点
curl -X POST -d '{}' http://192.168.4.1/api/wifi/scan

# 路由器重连
curl -X POST -d '{}' http://192.168.4.1/api/wifi/reconnect

# 恢复默认配置并重启
curl -X POST -d '{}' http://192.168.4.1/api/wifi/reset
```

> **关于 APSTA**：强烈建议保留热点。纯 STA 模式下若路由器不可用
> （改密码、路由器关机、信号太差），设备将完全失联，只能通过串口救回。
> APSTA 下热点始终可用，是可靠的兜底通道。

效果名称：`none` / `breath` / `blink` / `rainbow` / `chase`

> **关于 PWM 频率**：`/api/freq` 是全局的，无法逐颗独立。
> 硬件上前 6 路共用 LEDC 定时器、后 6 路共用 MCPWM 定时器，
> 因此频率只能整组统一设置。

### 示例

```bash
# 点亮 D1 为红色
curl -X POST -d '{"id":0,"r":255,"g":0,"b":0}' http://192.168.4.1/api/led

# D1 呼吸 (2s)，D2 闪烁 (0.5s)，D3 彩虹 (3s) —— 三颗同时不同效果
curl -X POST -d '{"id":0,"effect":"breath","period":2000}' http://192.168.4.1/api/led
curl -X POST -d '{"id":1,"effect":"blink","period":500}'  http://192.168.4.1/api/led
curl -X POST -d '{"id":2,"effect":"rainbow","period":3000}' http://192.168.4.1/api/led

# 单独调暗 D2 并关闭 D4
curl -X POST -d '{"id":1,"value":64}' http://192.168.4.1/api/led/brightness
curl -X POST -d '{"id":3,"enabled":false}' http://192.168.4.1/api/led/enable

# D1 循环播放: 静态 1s -> 闪烁 1s -> 呼吸 1s
curl -X POST -d '{"id":0,"steps":[
  {"effect":"none","duration":1000},
  {"effect":"blink","duration":1000},
  {"effect":"breath","duration":1000,"period":500}]}' \
  http://192.168.4.1/api/led/sequence

# 清空 D1 的序列，回退到单效果
curl -X POST -d '{"id":0,"steps":[]}' http://192.168.4.1/api/led/sequence

# 全部灯珠设为紫色，50% 亮度（一键同步）
curl -X POST -d '{"r":255,"g":0,"b":255}' http://192.168.4.1/api/all
curl -X POST -d '{"value":128}' http://192.168.4.1/api/brightness

# 全部熄灭
curl -X POST -d '{}' http://192.168.4.1/api/off

# 查看状态（含每颗灯珠的独立参数）
curl http://192.168.4.1/api/status
```

`/api/status` 的 `leds[]` 中每颗灯珠都带自己的高级参数与效果序列：

```json
{"count":4,"leds":[
  {"id":1,"name":"D1 左上翅膀","pos":"(17, 22) mm",
   "gpio":{"r":4,"g":5,"b":6},"r":255,"g":0,"b":0,
   "brightness":255,"effect":"none","period":2000,"enabled":true,
   "steps":[{"effect":"none","duration":1000,"period":0},
             {"effect":"blink","duration":1000,"period":0},
             {"effect":"breath","duration":1000,"period":500}]},
  ...],
 "brightness":255,"effect":"none","period":2000,
 "freq":5000,"invert":false,"off":false}
```

> 顶层 `brightness`/`effect`/`period` 是**汇总值**（取首颗灯珠），
> 供「一键同步」控件回显使用；逐颗真实值在 `leds[]` 内。

---

## 启动流程

```
上电
  │
  ├─ bootloader 加载 IAP (factory)
  │
  ├─ IAP 读取 iap_cfg 配置
  │    ├─ 下载位被置位？ → 进入下载模式（等待烧录）
  │    ├─ 等待 N 秒，期间检查触发源（GPIO/I2C/UART/WiFi）
  │    └─ 无触发 → 校验 user_app → 写入启动参数 → 切换启动分区
  │
  └─ bootloader 加载本程序 (user_app)
       ├─ iap_user_report_boot_ok()   清除下载位、递增启动计数
       ├─ iap_user_report_version()   上报版本号到 IAP 配置区
       ├─ load_boot_params()          读取 IAP 传来的启动参数
       ├─ led_ctrl_init()             配置 12 路 PWM (LEDC + MCPWM) + 效果任务
       ├─ boot_animation()            上电自检动画
       ├─ app_wifi_start()            启动 SoftAP（读 NVS 配置，默认 ESP-LED / 12345678）
       ├─ app_http_start()            启动 HTTP 服务
       └─ 正常工作，等待 HTTP 控制请求
```

### 启动参数

IAP 在启动用户程序前，把 `iap_boot_param_t`（124 字节）写入 **RTC 保留内存**
(`rtc_retain_mem_t.custom[]`)，本程序启动后通过 `iap_user_get_boot_param()`
读取，再从其中的 `param[]` 字段解析出 `key1=value1;key2=value2` 字符串。

> 两侧的 `CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC_SIZE=128` 必须一致，
> 否则 `custom[]` 偏移与 CRC 计算不匹配，读到的数据无效。

| 键 | 含义 | 示例值 |
|----|------|--------|
| `mode` | 运行模式 | `normal` / `debug` / `factory` |
| `reason` | 进入 IAP 的原因 | `wait_timeout` / `user_request` |
| `boot` | 启动计数 | `7` |
| `baud` | 建议通信波特率 | `921600` |

`mode=debug` 会打开全模块 DEBUG 日志。

### WiFi 参数来源

优先级：**NVS 用户配置** → IAP 启动参数 → 内置默认值（`ESP-LED` / `12345678`）。

- 首次启动时 NVS 中无配置，使用内置默认值，模式为**仅热点**
- 通过 Web 界面或 `POST /api/wifi` 修改后，配置写入 NVS 命名空间
  `wificfg`，**掉电保存**，后续启动直接生效
- 当前 `iap_boot_param_t` 中**不含 WiFi 配置字段**，因此中间那层
  （IAP 启动参数）实际未使用；若需 IAP 与用户程序共用参数，
  需 IAP 端在启动参数中传递，或扩展 `iap_boot_param_t`

这样 IAP 下载页面与本程序控制页面共用同一个 AP，客户端无需切换网络即可
完成「下载 → 启动 → 控制」的完整流程。

> **后续规划**：蓝牙 (BLE) 控制通道。计划在保留 WiFi 的同时增加 BLE
> GATT 服务，与 HTTP 共用同一套 JSON 命令层。
> 需注意 Web Bluetooth 只能在安全上下文（https / localhost）下使用，
> 且仅 Chrome / Edge / Opera 支持。

---

## 关键约束

| 约束 | 说明 |
|------|------|
| **分区表必须一致** | 用户程序与 IAP 使用**两份独立分区表**（A @ `0xB000` / B @ `0x140000`），但 `iap_cfg` 地址必须一致 |
| **分区标签** | 用户程序运行在 `user_app`（`ota_0` 子类型）分区 |
| **入口地址** | 用户程序镜像位于 `0x150000`；合并镜像从 `0x140000` 起烧 |
| **分区表 B 地址** | `0x140000`，已合并在 `led_butterfly_flash.bin` 内 |
| **无需打包** | 新版 IAP 用纯 ESP 镜像，`idf.py build` 产物直接烧 |
| **镜像校验** | 由 bootloader 自校验，失败自动回落 factory (IAP) |
| **启动参数结构体** | `iap_boot_param_t` 必须与 IAP 端一致（124 字节），经 RTC RAM 传递 |
| **工程路径** | 不能含中文，否则 ESP-IDF CMake 会静默崩溃 |

---

## 固件升级

### 方式一：Web 页面

控制页面点击「进入 IAP 下载模式」，设备重启后停留在 IAP，
再用 IAP 页面（`http://192.168.4.1/`）上传新的 `led_butterfly_flash.bin`。

### 方式二：API

```bash
curl -X POST http://192.168.4.1/api/upgrade
```

### 方式三：代码内触发

```c
app_request_upgrade();   /* 线程安全，可从任意任务投递 */
```

---

## 镜像校验与版本上报

### 镜像校验（由 bootloader 完成）

新版 IAP 不再使用自定义文件头，`user_app` 分区内就是**纯 ESP 应用镜像**：

```
user_app 分区 (2752KB, 0x2B0000)
+---------------------------+ offset 0
|  ESP 应用镜像 (magic 0xE9)|  ← bootloader / OTA 直接可读
+---------------------------+ offset image_size
|  0xFF 填充 (未使用)       |
+---------------------------+ offset size
```

镜像合法性由 bootloader 自校验（magic / 段表 / SHA256 / chip_id），
校验失败时自动回落 factory (IAP)，**不会变砖**。

> 为什么不用自定义文件头：ESP 镜像的 segment 表用**相对偏移**串接，
> 镜像必须从 offset 0 连续存放；且 `esp_ota_set_boot_partition()` 内部
> 的 `esp_image_verify()` 要求分区 offset 0 就是 `0xE9`。
> 任何前置数据都会导致 `ESP_ERR_OTA_VALIDATE_FAILED`。

### 版本上报

由于没有文件头，版本号等元数据存于 **IAP 配置区**，由用户程序启动后上报：

```c
#define APP_VERSION   0x010000    /* 1.0.0 */

iap_user_report_version(APP_VERSION);
```

IAP 的 `app info` 命令会显示该版本号。本程序在 `app_main` 步骤 2 调用，
`/api/status` 也会返回 `iapver` 字段。

---

## 内存与资源占用

| 项目 | 数值 |
|------|------|
| 固件大小 | 927,488 字节（约 906 KB） |
| 应用分区 | 0x2B0000（2,818,048 字节，2752 KB） |
| 分区剩余 | 约 67% |
| 空闲堆 | 约 300 KB（实测见串口日志） |
| 效果任务栈 | 3072 字节 |
| 状态任务栈 | 3072 字节 |
| 升级任务栈 | 3072 字节 |
| HTTP 服务栈 | 8192 字节 |

---

## 常见问题

**Q: 编译报错找不到 `iap_cfg` 分区？**
A: 确认 `sdkconfig.defaults` 中 `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME`
指向 `partitions_user_app.csv`，且 `CONFIG_PARTITION_TABLE_OFFSET=0x140000`。
注意：`iap_cfg` (`0x8000`) **不在分区表内**，是 IAP 私有区域，
用户程序不得读写，只能通过 RTC RAM 启动参数获取信息。

**Q: cmake 配置阶段无输出直接失败（退出码 0xC0000409）？**
A: 工程路径含中文字符。把工程复制到纯 ASCII 路径（如 `C:\work\APP`）再编译。

**Q: IAP 拒绝加载用户程序？**
A: 新版 IAP 使用纯 ESP 镜像，直接烧 `idf.py build` 产物即可，无需打包。
若报 `ESP_ERR_OTA_VALIDATE_FAILED`，说明镜像损坏或不完整，重新编译烧录。

**Q: 用户程序读不到启动参数？**
A: 确认两侧 `CONFIG_BOOTLOADER_RESERVE_RTC_MEM=y`、
`CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC=y`、
`CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC_SIZE=128` 完全一致。
不一致会导致 `custom[]` 偏移/CRC 不匹配，`iap_user_get_boot_param()`
返回错误或读到无效数据。

**Q: 灯珠不亮或亮度相反？**
A: 共阴极接法下占空比越大越亮。若硬件为公共阳极接 VCC，调用
`led_ctrl_set_invert(true)` 反转输出逻辑。

**Q: 为什么用了两个 PWM 外设？**
A: ESP32-C6 的 LEDC 只有 6 个通道（`SOC_LEDC_CHANNEL_NUM = 6`），
而硬件需要 12 路。MCPWM 可再提供 6 路（3 操作器 × 2 比较器/生成器），
两者合计 12 路，且都是纯硬件 PWM。

**Q: 运行时出现 `Cache error`？**
A: 当前方案无中断参与调光，不应出现。若出现，检查是否有代码在
ISR 中访问了 flash 中的符号（函数或常量表）。

**Q: 编译报错找不到 `mcpwm_prelude.h`？**
A: 确认 `main/CMakeLists.txt` 的 REQUIRES 含 `esp_driver_mcpwm`。

**Q: 如何回到 IAP 下载模式？**
A: Web 页面点「进入 IAP 下载模式」，或 `POST /api/upgrade`，
或代码调用 `iap_user_request_download()`。

**Q: WiFi 连不上？**
A: 首次启动使用默认 SSID `ESP-LED`、密码 `12345678`。若之前改过配置，
用新参数连接；忘记时可 `POST /api/wifi/reset` 恢复默认（会重启），
或擦除 NVS 分区。密码需 8-63 字符，否则会自动降级为开放网络
（串口日志会打印警告）。

**Q: 改了 WiFi 密码后浏览器打不开了？**
A: 这是预期的 —— 修改 SSID / 密码 / 信道 / 模式会断开当前连接。
用新参数重新连接 WiFi，再访问 `http://<新 IP>/`。
若 IP 也改了，注意用新地址访问。

**Q: 选了「仅连路由器」后连不上了怎么办？**
A: 纯 STA 模式下路由器不可用就会失联。此时只能通过串口操作：
重新烧录，或擦除 NVS 分区（`esptool erase_region 0x141000 0x3000`）
恢复默认热点。**建议用 APSTA 模式**，热点始终可用作兜底。

**Q: STA 一直连不上路由器？**
A: 检查 SSID / 密码是否正确（密码需 8-63 字符）。
设备会自动重试 8 次（间隔 3 秒），超过后停止。
可在页面点「重连路由器」或 `POST /api/wifi/reconnect` 手动重试。
注意 ESP32-C6 只支持 2.4 GHz，无法连接 5 GHz 频段的路由器。

