/*
 * ============================================================
 *  LJ640U34 EL 显示器 · 双核驱动（v6 当前主版本，2026-08-27 定稿）
 *  ============================================================
 *  ⚠️ 本文件为当前主版本，后续改动基于它（v3/v4/v5 已归档勿改）。
 *
 *  v6 相对 v5 的改动（风扇开环调速，实测通过）：
 *   1. GP6 PWM 25kHz 驱动 5V 4线风扇（蓝线PWM），开环按工作档位查表输出占空比。
 *      档位=热度 视频>图片>图标>DD>时钟>Cyn>表情，默认 85/75/60/50/40/35/30%；
 *      本地页按 page 查表（图标0→60%、Cyn1→35%、DD2→50%），推流按内容类型
 *      ('T' 命令设 contentLevel)，息屏停(0%)；开机先 50% 5s 后平滑回落，
 *      转速每 300ms 最多 ±10 平滑。
 *   2. 每档占空比存 flash（UserSettings.fanPct[7]），设置页可读/写。
 *   3. 协议扩展：'V'+7字节写档位(回 FAN OK)、'R' 回传 SET行+FAN行、
 *      'T'+1字节设推流内容类型；上位机 sendContentType() 按当前内容源发 'T'。
 *      ⚠️ arduino-pico 频率用 analogWriteFreq(freq)(全局)，值用 analogWrite(pin,0-255)。
 *
 *  v5 相对 v4 的改动（差分 + toBitmap 性能优化）：
 *   1. 差分推流：同步头 AA 55 55 AB，数据=[行号2B+80B]×N，只发变化行；
 *      与整帧RLE比较大小取更小者，每 30 帧强制整帧重同步基准防残留/撕裂。
 *   2. toBitmap：灰度内联、Uint32Array 一次写一字、dither 类型移出循环。
 *
 *  v4 相对 v3 的改动（修复推流撕裂/重启，里程碑）：
 *   1. 推流接收竞态修复：新增 rxTmp 临时缓冲，收帧写 rxTmp、收满后 memcpy 到当前
 *      drawIdx 再 commitFrame()，修复"收帧中途 core1 swap 改 drawIdx → 一帧劈两半 →
 *      上半撕裂(越靠上越严重) + 高速大帧(眨眼)触发看门狗复位重启"。
 *   2. 时序校准：VD_SETUP 50→170µs(≥t_vw 160µs)、帧尾 H.D/V.D 立即拉低+VD_BLANK 800µs、
 *      BLANK_WORDS=3(行周期 40.9µs 合规)。
 *
 *  v3 相对 v2 的改动（Cyn 文字页字体/字形精修，实测通过）：
 *   1. 文字页字体：Codename Coder Free 4F 12×22 位图（fontCoder.h，
 *      uint16_t[95][22]，bit11=最左列），1:1 绘制无需放大；
 *      布局 charW=12、lineH=24、cynText 上下各 1 空行 → 边框完整落屏。
 *   2. 字形精修：'0' 切角方块+左竖线补全、'D' 去内缘杂点、
 *      'N' 抄反斜杠 3px 连续斜线、'[' ']' 用户版宽窄。
 *   3. 'x' 噪点改 2px×1px 细棋盘格（仿 v2 花屏；大方格反而眼花）。
 *   4. setRowBits 跨 32 位字边界修复（原 sh>26 会丢 12px 字符右半）。
 *
 *  v2 相对基础版的新增功能（全部实测通过）：
 *   1. 外壳电源控制：GP8 = 12V 高边开关（直流 SSR / MOS），软件保证
 *      "先 5V 后 12V" 上电时序：GP8 默认低 → 初始化 → 0.5s 后 12V 通
 *      → 0.2s 逆变器稳定 → 再启动 core1 刷新引擎
 *   2. 单键操作（GP7，内部上拉，轮询非中断）：亮屏单击切页
 *      （Cyn→图标→DD 循环，pageOrder 表）、长按 1s 息屏（断 12V）、
 *      息屏单击唤醒；40ms 消抖 + 边沿检测
 *   3. 网页设置页 + EEPROM 断电保存：默认开机页 / LED 颜色 /
 *      呼吸模式·速度·亮度 / 休眠亮度；协议见 handleSerial：
 *      命令帧 + 'R' 读（回 SET 行）/ 'S' + 8 字节写（回 SET OK）
 *   4. 命令帧 'B'：回默认开机页（网页"停止/清除帧"即时生效，不等超时）
 *   5. 推流超时 4s：兼容浏览器后台标签页 timer 节流（2-3s/次）
 *   6. WS2812 琥珀呼吸灯（⚠️ 微雪灯珠是 RGB 字节序，非标准 GRB）
 *   7. 看门狗 2s + PIO FIFO 忙等超时 + swap 超时（防卡死自动复位）
 *
 *  ⚠️ 本文件踩坑记录（避免重踩，详见各函数注释）：
 *   A. RP2040 双核写 flash 必崩（XIP 取指冲突）：
 *      写 flash 前 core1Halt → core1 WFE 睡眠（不取指，PIO 继续跑）
 *      → core0 禁中断 + EEPROM.commit() → SEV 唤醒。
 *      暂停等待必须带 500ms 超时，否则 core1 异常时 core0 死等 → watchdog。
 *   B. WS2812 三连坑：手写 PIO 十六进制编码不可靠（用 pio_encode_*）；
 *      jmp !x 跳转地址错一位（应跳 3 误写 2）→ 全白屏；
 *      时钟 40MHz 不匹配 → 8MHz（10 cycles/bit × 800kHz）。
 *   C. 设置协议字节数必须精确对齐：设置字段 8 个，固件曾傻等 9 字节
 *      → SET TIMEOUT got=8（网页监听打点定位）。
 *   D. 动画节拍计数器不得放在"偶数才重绘"条件内部（奇数永不命中
 *      → 动画定格，DD X 动画曾静止）。
 *
 *  ============================================================
 *  架构说明：由 LJ640U34_solver 定型的程序改造而来，针对
 *  "显示面积越大画面越不稳定/闪烁重" 的问题做架构级优化：
 *
 *  原版问题（单核串行）：
 *    loop = 画一帧 → 发一帧。大面积图像（图标页每帧 memcpy 32KB、
 *    DD X 页大范围斜线浮点扫描）绘制耗时长，直接挤占刷新时间 →
 *    帧间隔抖动、帧率下降 → EL 磷光余晖 + 帧率低 = 闪烁感加重。
 *
 *  优化方案（RP2040 双核，Cortex-M0+ × 2 @ 133MHz）：
 *    core0 = 专职绘制/动画/串口/WS2812（"画面内容引擎"）
 *    core1 = 专职帧发送（"显示刷新引擎"，PIO 忙等 + V.D/H.D 时序）
 *    刷新节奏恒定：core1 发完一帧若无新帧就重复发旧帧，绝不等绘制。
 *    绘制再慢也只影响内容更新速度，不影响刷新节奏 → 大面积图不闪。
 *
 *  双缓冲乒乓（硬件类比：双口 RAM 乒乓缓冲，core1 是唯一仲裁者）：
 *    drawIdx/dispIdx 两个缓冲，core0 画 drawIdx、core1 发 dispIdx；
 *    core0 画完置 swapReq=1，core1 发完当前帧后交换并清 0。
 *    core0 只有在 swapReq==0（即上一帧已发送、画布空闲）时才开画，
 *    二者永远不会碰同一块缓冲。RP2040 双核共享 SRAM 无缓存，
 *    单字节 volatile 标志天然原子，无需加锁。
 *
 *  配套优化：图标页只在页面切换时画一次（pageDirty 机制），不再每帧
 *  memcpy 32KB → 大面积图标显示时 core0 负载接近零，刷新 100% 恒定。
 *  Cyn 文字页与 DD X 动画页每帧重绘（core0 单帧绘制 <5ms，远小于刷新
 *  周期 ~17ms，不影响刷新节奏）：前者保持 'x' 噪点花屏闪烁，后者动画
 *  逐帧更新比原版"每 2 帧"更流畅。
 *
 *  联机推流（已实现）：USB CDC 推流模式，帧格式见 handleSerial()：
 *    4B 同步头 AA 55 55 AA + 4B 长度 + 数据；共三种：
 *      · 长度=32000  → 原样 1bit 像素（整帧）；
 *      · 长度<32000  → RLE 压缩流 [value][count]（解码后 32000B）；
 *      · 同步头 AA 55 55 AB → 差分帧 [行号2B+80B]×N（v5，只发变化行）。
 *    每帧回 'K' ack 流控；4000ms 无新帧自动回本地页。core0 收帧 →
 *    commitFrame()，core1 只管发屏，天然解耦。
 *    推流时上位机按内容发 'T'+档位索引 → 固件 contentLevel 驱动风扇档位。
 *  PC 端网页上位机（Web Serial，免 Python）：LJ640U34/usb_el_display.html
 *  ⚠️ 推流内容同样无法避开坏线 y=103/105（硬件缺陷，与本地页一致）。
 *
 *  硬件接线 / LJ245A / 电源 / 安全：
 *  （GP10→CK0, GP11→Din0, GP12→Din1, GP13→H.D, GP14→V.D，
 *   GP7→按键, GP8→12V 控制，GP16→WS2812，GP6→风扇 PWM(蓝线,25kHz,3.3V 直驱)，
 *   风扇红线→5V(建议 LC 滤波)、黑线→GND、白线(Tach)可选，
 *   外部 5V→pin3/4、12V→pin1/2，先 5V 后 12V，严禁 USB 供电，
 *   通电时背面 200V AC 脉冲严禁触碰）。
 *
 *  ⚠️ 大面积闪烁的硬件侧排查（若仍有闪烁）：
 *    1) EL 面板点亮面积越大电流越大（电容性高压负载），12V/5V 电源
 *       余量不足或走线压降大 → 高压逆变器输出抖动 → 亮度闪烁。
 *       对策：面板电源引脚附近并 470µF 电解 + 100nF 瓷片；
 *       外部电源电流余量 ≥ 300mA；LJ245A 与面板 GND 共地粗线。
 *    2) 面板空白检测省电：全亮/大部分暗画面可能触发关高压（电流
 *       突变 60mA 级）→ 用棋盘格 ~50% 亮均匀图对比测试确认。
 *
 *  ⚠️ 面板已知缺陷：行 y=103、y=105 常亮（无法掩盖）
 *  ⚠️ 推流撕裂/重启：已由 v4 修复（推流接收竞态），v5 差分 / v6 风扇未引入新撕裂。
 *  ⚠️ 复杂画面帧率偏低（动画视频 8-10fps、开 jjn 抖动更低）：内容信息量 + RP2040
 *      USB1.1-FS 吞吐 + 误差扩散串行逐像素三因素；要更高帧率可上 ordered(bayer) 或 Web Worker。
 *  ⚠️ 风扇为开环（不随温度）：高压变压器射频干扰导致测温不稳；按模式调速，高负载档转速留足余量。
 * ============================================================
 */

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <EEPROM.h>                // 用户设置 flash 持久化（模拟 EEPROM）
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"        // 双核：multicore_launch_core1
#include "hardware/watchdog.h"     // 看门狗：系统卡死 2s 自动复位兜底
#include "solver_bitmap.h"         // 页面0：Absolute Solver 图标（640×400 全帧）
#include "fontCoder.h"             // Codename Coder Free 4F 等宽 12×22 字体表 uint16_t[95][22]（0x20~0x7E，bit11=最左列）

// ---------- 引脚定义 ----------
#define PIN_CK0   10
#define PIN_DIN0  11
#define PIN_DIN1  12
#define PIN_HD    13
#define PIN_VD    14
#define PIN_LED   16
#define PIN_PWR12V 8    // 外壳版：12V 电源控制（GPIO 高=通、低=断，驱动 SSR/MOS 高边开关）
#define PIN_BTN    7    // 外壳版：单键（内部上拉，按下=低；单击切页 / 长按息屏）
#define PIN_FAN_PWM 6   // 风扇调速：5V 4线 PWM(蓝线)，3.3V 逻辑直接驱动，25kHz（远离 GP10-14 屏线束减串扰）
                        // ⚠️ 注：RP2040-Zero 未引出 GP17/18，改用 GP8/GP7（侧面引脚，远离时钟线束）；
                        //    GP26-29 保留作 ADC，GP0/GP1 保留作 UART 调试

// ---------- 屏参数 ----------
#define SCR_W       640
#define SCR_H       400
#define WORDS_ROW   20
#define FRAME_BYTES (SCR_W / 8 * SCR_H)   // 推流单帧字节数 = 32000（1bit 全帧）

// ---------- 时序参数 ----------
#define TARGET_PCLK  9000000.0f   // 数据时钟 9MHz（最终档，已定型）：9.5MHz 行同步失败
                                  // （上下重复），7.5MHz 拍频共振最重，8.5MHz 位置漂移 →
                                  // 9MHz 为可用范围内最优。撕裂根因模型：行频与 EL 高压逆变器
                                  // ~21kHz 拍频共振。2026-08-25 时序手册核对发现 TH（行周期）
                                  // 下限 40µs：9MHz+352out=39.1µs 偏低 → 试 BLANK_WORDS=3
                                  // （TH≈40.9µs 合规）烧录后撕裂依旧 → TH 假设排除，恢复为 2
#define PIO_CYCLES_PER_BIT 13
#define BLANK_WORDS  3            // 疑点3重测(帧尾已修复后): 行周期 39.1→40.9µs 合规(手册40~45µs)。
                                   // 残余撕裂若消失→坐实行周期是残留主因; 若无效→基本排除时序, 转向电源/EMI。
                                   // 原值2=39.1µs略低于手册下限(前提变化, 之前大问题掩盖了它)。
#define VD_SETUP_US  170   // 疑点1: V.D 上升沿→首行需 ≥ t_vw=160µs（手册 Page6），原 50µs 不足
#define VD_BLANK_US  800   // 疑点2: V.D=0 垂直消隐 T_vb；原 VD_TAIL 1150µs（V.D 高悬空+H.D 悬高）已删
                           //        800µs 使帧率≈60Hz（400×39.1 + 170 + 800 ≈ 16.66ms）
// 注: VD_TAIL_US 已弃用，原帧尾 V.D 高悬 1150µs 疑违反手册（见 elSendFrame 帧尾注释）

// ============================================================
//  双缓冲乒乓（双核共享，volatile）
//  drawIdx：core0 正在画的缓冲；dispIdx：core1 正在发的缓冲
//  swapReq：core0 画完置 1 请求交换；core1 发完交换后清 0
// ============================================================
static uint32_t framebuf[2][SCR_H][WORDS_ROW];
// 推流帧临时缓冲（32000B）：收帧数据先写这里，收满后一次性 memcpy 到 framebuf[drawIdx]。
// ⚠️ 修复竞态：原直接把推流数据写 framebuf[drawIdx]，而 core1 会在收帧中途 swap 改写 drawIdx，
//    导致一帧被劈成两半写入两块缓冲 → 上半撕裂/混写，高速大帧甚至越界→看门狗复位。
//    独立临时缓冲彻底规避：收帧期间 drawIdx 怎么变都不影响，收满后 memcpy 到“当前”drawIdx。
static uint32_t rxTmp[SCR_H][WORDS_ROW];
static volatile int drawIdx = 0;
static volatile int dispIdx = 1;
static volatile uint8_t swapReq = 0;
static volatile uint32_t sendCnt = 0;   // core1 已发送帧计数（core0 读作诊断）
// ⚠️ 双核写 flash 保护标志：写 flash 期间 core1 必须暂停（WFE 睡眠不取指）。
//    定义在文件顶部：core1Refresh（前部）与 settingsSave（后部）都要用
static volatile bool core1Halt = false;
static volatile bool core1Paused = false;

// ============================================================
//  PIO 程序（4 条自循环，H.D 由 CPU 控制）—— 与原版相同
// ============================================================
static uint16_t el_program[] = {
  (uint16_t)(pio_encode_out(pio_pins, 2) | (0u << 12u) | (1u << 8u)),
  (uint16_t)(pio_encode_nop()            | (0u << 12u) | (4u << 8u)),
  (uint16_t)(pio_encode_nop()            | (1u << 12u) | (2u << 8u)),
  (uint16_t)(pio_encode_jmp(0)           | (1u << 12u) | (2u << 8u)),
};
static pio_program_t el_prog = { el_program, 4, -1 };
#define EL_PIO pio0
#define EL_SM  0

void elPioInit() {
  pio_gpio_init(EL_PIO, PIN_CK0);
  pio_gpio_init(EL_PIO, PIN_DIN0);
  pio_gpio_init(EL_PIO, PIN_DIN1);
  pio_sm_set_consecutive_pindirs(EL_PIO, EL_SM, PIN_DIN0, 2, true);
  pio_sm_set_consecutive_pindirs(EL_PIO, EL_SM, PIN_CK0, 1, true);
  uint off = pio_add_program(EL_PIO, &el_prog);
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_sideset(&c, 1, false, false);
  sm_config_set_sideset_pins(&c, PIN_CK0);
  sm_config_set_out_pins(&c, PIN_DIN0, 2);
  sm_config_set_out_shift(&c, true, true, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  float div = (float)F_CPU / (TARGET_PCLK * PIO_CYCLES_PER_BIT);
  sm_config_set_clkdiv(&c, div);
  sm_config_set_wrap(&c, off, off + 3);
  pio_sm_init(EL_PIO, EL_SM, off, &c);
  EL_PIO->sm[EL_SM].clkdiv    = c.clkdiv;
  EL_PIO->sm[EL_SM].execctrl  = c.execctrl;
  EL_PIO->sm[EL_SM].shiftctrl = c.shiftctrl;
  EL_PIO->sm[EL_SM].pinctrl   = c.pinctrl;
  pio_sm_set_enabled(EL_PIO, EL_SM, true);
}

// ============================================================
//  帧发送（core1 专职调用，CPU 忙等 PIO）
//  防御：FIFO 忙等带超时——PIO 一旦挂起不再死等，清 FIFO 自愈、
//  跳帧重来；由看门狗兜底最终复位（2026-08-23 切页偶发卡死修复）
// ============================================================
static inline bool elWaitRowDone(uint32_t timeout_us) {
  uint32_t mask = 1u << (EL_SM + PIO_FDEBUG_TXSTALL_LSB);
  EL_PIO->fdebug = mask;
  uint32_t t0 = micros();
  while (!(EL_PIO->fdebug & mask)) {
    if (micros() - t0 >= timeout_us) return false;
  }
  return true;
}

// PIO FIFO 推字：10ms 超时保护（原来 while(满) 无超时 = 卡死点）
// 超时 → 清 FIFO 尝试恢复并返回 false → 调用方跳帧重来
static inline bool fifoPush(uint32_t v) {
  uint32_t t0 = micros();
  while (pio_sm_is_tx_fifo_full(EL_PIO, EL_SM)) {
    if (micros() - t0 >= 10000u) {
      pio_sm_clear_fifos(EL_PIO, EL_SM);   // 清 FIFO 自愈
      return false;
    }
  }
  EL_PIO->txf[EL_SM] = v;
  return true;
}

void elSendFrame() {
  int d = dispIdx;                 // 发送期间 dispIdx 只有 core1 自己改，快照一次
  gpio_put(PIN_VD, 1);
  delayMicroseconds(VD_SETUP_US);
  // 帧首强制行同步沿：上一帧行 399 结束时 H.D 保持高，若直接送行 0 数据，
  // 屏端看不到"新行开始"沿 → 行 0 被当上帧尾巴 → 顶部撕裂（推流视频可见）。
  // 机制与行间消隐相同（H.D 低 + BLANK_WORDS 消隐时钟 → H.D 高）。
  // ⚠️ 注意：消隐期（H.D 低）内 PIO 输出的任何字节都会被屏端当作消隐数据
  // 移位——因此消隐期只能填 0，绝不能预填下一行真实数据（会污染行寄存器
  // → 屏幕左边竖条异常 + 横线花屏，2026-08-23 踩坑已回退）。
  gpio_put(PIN_HD, 0);
  for (int i = 0; i < BLANK_WORDS; i++) if (!fifoPush(0)) return;
  elWaitRowDone(50000);
  gpio_put(PIN_HD, 1);
  for (int y = 0; y < SCR_H; y++) {
    for (int i = 0; i < WORDS_ROW; i++) if (!fifoPush(framebuf[d][y][i])) return;
    elWaitRowDone(50000);
    gpio_put(PIN_HD, 0);
    for (int i = 0; i < BLANK_WORDS; i++) if (!fifoPush(0)) return;
    elWaitRowDone(50000);
    gpio_put(PIN_HD, 1);
  }
  // 疑点2: 帧尾立即拉低 H.D（进入消隐）与 V.D（进入垂直消隐 T_vb），
  //        不再让 400 行结束后 V.D/H.D 在空转期悬空高 1150µs（原 VD_TAIL）
  gpio_put(PIN_HD, 0);
  gpio_put(PIN_VD, 0);
  delayMicroseconds(VD_BLANK_US);
}

// ============================================================
//  core1：显示刷新引擎（只发帧 + 乒乓交换，绝不做绘制/串口）
//  发完当前帧若没有新帧（swapReq==0）就重复发旧帧 → 刷新节奏恒定
// ============================================================
void core1Refresh() {
  while (true) {
    // flash 写入保护：core0 保存设置时暂停本核（WFE 睡眠不取指 flash）
    if (core1Halt) {
      core1Paused = true;
      while (core1Halt) { __asm volatile("wfe"); }
      core1Paused = false;
    }
    elSendFrame();
    sendCnt++;
    if (swapReq) {
      // 乒乓交换：core1 独占执行（唯一写 drawIdx/dispIdx 的地方，
      // 且只在两帧发送之间、core0 未绘制时发生）→ 天然原子
      int t = drawIdx;
      drawIdx = dispIdx;
      dispIdx = t;
      swapReq = 0;                 // 通知 core0：画布已空闲，可以开画
    }
  }
}

// core0 提交一帧：等上一帧发送完成交换 → 置位请求
// 100ms 超时兜底：core1 异常时强制继续（不再死等 = 卡死点修复）
void commitFrame() {
  uint32_t t0 = micros();
  while (swapReq) {
    if (micros() - t0 >= 100000u) { swapReq = 0; break; }
  }
  swapReq = 1;
}

// ============================================================
//  画图 API（core0 调用，操作 drawIdx，文字用）—— 与原版相同
// ============================================================
void clearScreen(bool on) {
  uint32_t v = on ? 0xFFFFFFFFu : 0;
  for (int y = 0; y < SCR_H; y++)
    for (int i = 0; i < WORDS_ROW; i++)
      framebuf[drawIdx][y][i] = v;
}

void drawPixel(int x, int y, bool on) {
  if (x < 0 || x >= SCR_W || y < 0 || y >= SCR_H) return;
  uint32_t &w = framebuf[drawIdx][y][x >> 5];
  uint32_t m  = 1u << (x & 31);
  if (on) w |= m; else w &= ~m;
}

// 5×7 ASCII 字体（0x20~0x7E，含小写字母）
static const uint8_t font5x7[95][7] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x04,0x04,0x04,0x04,0x04,0x00,0x04},
  {0x0A,0x0A,0x0A,0x00,0x00,0x00,0x00},
  {0x0A,0x0A,0x1F,0x0A,0x1F,0x0A,0x0A},
  {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04},
  {0x19,0x19,0x02,0x04,0x08,0x13,0x13},
  {0x0C,0x12,0x14,0x08,0x15,0x12,0x0D},
  {0x0C,0x04,0x08,0x00,0x00,0x00,0x00},
  {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
  {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
  {0x00,0x04,0x15,0x0E,0x15,0x04,0x00},
  {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
  {0x00,0x00,0x00,0x00,0x0C,0x04,0x08},
  {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},
  {0x01,0x02,0x04,0x08,0x10,0x00,0x00},
  {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
  {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
  {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
  {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E},
  {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
  {0x1F,0x10,0x1E,0x01,0x01,0x01,0x1E},
  {0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E},
  {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
  {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
  {0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E},
  {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00},
  {0x00,0x0C,0x0C,0x00,0x0C,0x04,0x08},
  {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
  {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
  {0x08,0x04,0x02,0x01,0x02,0x04,0x08},
  {0x0E,0x11,0x01,0x02,0x04,0x00,0x04},
  {0x0E,0x11,0x15,0x15,0x16,0x10,0x0F},
  {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
  {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
  {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
  {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C},
  {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
  {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
  {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},
  {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
  {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
  {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
  {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
  {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
  {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
  {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
  {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
  {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
  {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
  {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
  {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},
  {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
  {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
  {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
  {0x11,0x11,0x11,0x15,0x15,0x15,0x0A},
  {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
  {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
  {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
  {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E},
  {0x10,0x08,0x04,0x02,0x01,0x00,0x00},
  {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E},
  {0x04,0x0A,0x11,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
  // 0x60 '`'
  {0x04,0x02,0x00,0x00,0x00,0x00,0x00},
  // a-z (0x61-0x7A)
  {0x0E,0x11,0x01,0x0F,0x11,0x11,0x0F},
  {0x10,0x10,0x1E,0x11,0x11,0x11,0x1E},
  {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
  {0x01,0x01,0x0F,0x11,0x11,0x11,0x0F},
  {0x0E,0x11,0x1F,0x10,0x10,0x11,0x0E},
  {0x06,0x09,0x08,0x1C,0x08,0x08,0x08},
  {0x0F,0x11,0x11,0x0F,0x01,0x11,0x0E},
  {0x10,0x10,0x1E,0x11,0x11,0x11,0x11},
  {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E},
  {0x01,0x00,0x03,0x01,0x01,0x01,0x0E},
  {0x10,0x10,0x12,0x14,0x18,0x14,0x12},
  {0x0C,0x04,0x04,0x04,0x04,0x04,0x0E},
  {0x1B,0x15,0x15,0x15,0x15,0x15,0x15},
  {0x1E,0x11,0x11,0x11,0x11,0x11,0x11},
  {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
  {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
  {0x0F,0x11,0x11,0x0F,0x01,0x01,0x01},
  {0x16,0x19,0x11,0x10,0x10,0x10,0x10},
  {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},
  {0x08,0x08,0x1C,0x08,0x08,0x09,0x06},
  {0x11,0x11,0x11,0x11,0x11,0x13,0x0D},
  {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04},
  {0x11,0x11,0x15,0x15,0x15,0x15,0x0A},
  {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
  {0x11,0x11,0x11,0x0F,0x01,0x11,0x0E},
  {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
  // 0x7B-0x7E { | } ~
  {0x03,0x04,0x04,0x08,0x04,0x04,0x03},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x18,0x04,0x04,0x02,0x04,0x04,0x18},
  {0x08,0x15,0x02,0x00,0x00,0x00,0x00},
};

void drawChar(int x, int y, char c, bool on) {
  if (c < 0x20 || c > 0x7E) return;
  const uint8_t *g = font5x7[c - 0x20];
  for (int row = 0; row < 7; row++)
    for (int col = 0; col < 5; col++)
      if (g[row] & (0x10 >> col))
        drawPixel(x + col, y + row, on);
}

void drawString(int x, int y, const char *s, bool on = true) {
  while (*s) {
    drawChar(x, y, *s, on);
    x += 6;
    s++;
  }
}

// ============================================================
//  WS2812 工作指示灯（PIO1，core0 控制）
//  2026-08-23 重写：旧版为手写十六进制编码（从未验证，疑似错误
//  + 时钟 40MHz 不匹配）→ 改为与 EL PIO 同款 pio_encode_* 编码的
//  官方标准 4 指令程序 + 8MHz 时钟（10 周期/位 × 800kHz）
// ============================================================
#define LED_PIO  pio1
#define LED_SM   0
// 标准 WS2812 bitloop（side-set 0.25 周期近似），指令格式与 EL PIO 相同：
//   delay 在 bit8-11（4 位），sideset 在 bit12-15
//   index0: out x,1 side0 [1]     —— 移出 1 位到 X
//   index1: jmp !x,3  side1 [1]   —— X=0 → 跳 do_zero(3)；X=1 落到 do_one(2)
//   index2: jmp 0     side1 [2]   —— do_one（1 位波形）
//   index3: nop       side0 [2]   —— do_zero（0 位波形）⚠️ 曾误跳 2 → 全 1 白屏
// 周期数 = 2+2+3+3 = 10 cycles/bit（WS2812 800kHz → PIO 时钟 8MHz）
static uint16_t ws2812_program[] = {
  (uint16_t)(pio_encode_out(pio_x, 1)     | (0u << 12u) | (1u << 8u)),
  (uint16_t)(pio_encode_jmp_not_x(3)      | (1u << 12u) | (1u << 8u)),  // jmp !x, 3 (do_zero)
  (uint16_t)(pio_encode_jmp(0)            | (1u << 12u) | (2u << 8u)),
  (uint16_t)(pio_encode_nop()             | (0u << 12u) | (2u << 8u)),
};
static pio_program_t ws2812_prog = { ws2812_program, 4, -1 };

void ledInit() {
  pio_gpio_init(LED_PIO, PIN_LED);
  pio_sm_set_consecutive_pindirs(LED_PIO, LED_SM, PIN_LED, 1, true);
  uint off = pio_add_program(LED_PIO, &ws2812_prog);
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_sideset(&c, 1, false, false);
  sm_config_set_sideset_pins(&c, PIN_LED);
  sm_config_set_out_shift(&c, false, true, 32);   // MSB first（WS2812 要求），自动拉取
  float div = (float)F_CPU / 8000000.0f;          // 10 cycles/bit × 800kHz = 8MHz
  sm_config_set_clkdiv(&c, div);
  sm_config_set_wrap(&c, off, off + 3);
  pio_sm_init(LED_PIO, LED_SM, off, &c);
  LED_PIO->sm[LED_SM].clkdiv    = c.clkdiv;
  LED_PIO->sm[LED_SM].execctrl  = c.execctrl;
  LED_PIO->sm[LED_SM].shiftctrl = c.shiftctrl;
  LED_PIO->sm[LED_SM].pinctrl   = c.pinctrl;
  pio_sm_set_enabled(LED_PIO, LED_SM, true);
}

void ledSetColor(uint8_t r, uint8_t g, uint8_t b) {
  // ⚠️ 微雪 RP2040-Zero 板载灯珠为 RGB 字节顺序（非标准 GRB）：
  //    实测发 GRB 显示绿色 → 打包顺序改为 R,G,B
  uint32_t grb = ((uint32_t)r << 24) | ((uint32_t)g << 16) | ((uint32_t)b << 8);
  while (pio_sm_is_tx_fifo_full(LED_PIO, LED_SM));
  LED_PIO->txf[LED_SM] = grb;
}

// ============================================================
//  画面：纯位图显示 + 串口页面切换
//    串口 '0' → Absolute Solver 图标页
//    串口 '1' → Cyn 首次重启文字页（残缺效果）
//    串口 '2' → DD（Disassembly Drone）X 标志动画页
// ============================================================
static uint32_t animTick = 0;
static int page = 0;        // 0=图标页，1=Cyn 文字页，2=DD X 动画页
static int hudFont = 1;     // HUD 文字页字体：0=5×7 放大(10px) 1=Codename Coder 12×22（1:1）
static volatile bool pageDirty = true;   // 静态页只在切换时重绘一次

// 正弦查表（256 项，int8，范围 -31..31）
static int8_t sinTab[256];
void initSinTab() {
  for (int i = 0; i < 256; i++)
    sinTab[i] = (int8_t)(31.0 * sin(i * 6.2832 / 256.0));
}

// Bresenham 画线（单像素）
void drawLine(int x0, int y0, int x1, int y1, bool on = true) {
  int dx = abs(x1 - x0), dy = -abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    drawPixel(x0, y0, on);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// 行位填充：把 (xl, y)-(xr, y) 一行像素置 1（32 位字操作，快）
// 安全：clamp 后重新判空，杜绝整段在屏外时 w1 变负 → row[-1] 越界写
static inline void fillRowBits(int xl, int xr, int y) {
  if (y < 0 || y >= SCR_H) return;
  if (xr < 0 || xl >= SCR_W) return;
  if (xl < 0) xl = 0;
  if (xr > SCR_W - 1) xr = SCR_W - 1;
  if (xl > xr) return;
  int w0 = xl >> 5, w1 = xr >> 5;
  uint32_t m0 = 0xFFFFFFFFu << (xl & 31);
  uint32_t m1 = 0xFFFFFFFFu >> (31 - (xr & 31));
  uint32_t *row = framebuf[drawIdx][y];
  if (w0 == w1) {
    row[w0] |= (m0 & m1);
  } else {
    row[w0] |= m0;
    for (int w = w0 + 1; w < w1; w++) row[w] = 0xFFFFFFFFu;
    row[w1] |= m1;
  }
}

// 斜切矩形条带粗线：线身 = 法向宽度恒定的平行条带，端面垂直于线（与线身成直角）
// 全部浮点计算最后取整 → 端面边缘与条带边缘精确衔接，为平滑直线
// 扫描范围扩展 dY 覆盖端面尖角，四角完整不缺失；法向宽度恒定 = w
void drawThickLine(int x0, int y0, int x1, int y1, int w) {
  int half = w / 2;
  int dx = x1 - x0, dy = y1 - y0;
  if (dy == 0) {
    int xa = (x0 < x1) ? x0 : x1, xb = (x0 > x1) ? x0 : x1;
    for (int y = y0 - half; y <= y0 + half; y++)
      fillRowBits(xa, xb, y);
    return;
  }
  if (dx == 0) {
    int ya = (y0 < y1) ? y0 : y1, yb = (y0 > y1) ? y0 : y1;
    for (int y = ya; y <= yb; y++)
      fillRowBits(x0 - half, x0 + half, y);
    return;
  }
  // 斜线：端面垂直于线的矩形条带
  float len = sqrtf((float)(dx * dx + dy * dy));
  float hxf = (float)half * len / (float)((dy > 0) ? dy : -dy);  // 行半宽（浮点）
  int dY = (dx != 0) ? (int)((float)half * len / (float)((dx > 0) ? dx : -dx) + 0.5f) : 0;
  int ymin = ((y0 < y1) ? y0 : y1) - dY;
  int ymax = ((y0 > y1) ? y0 : y1) + dY;
  for (int y = ymin; y <= ymax; y++) {
    float xc  = (float)x0 + (float)dx * (float)(y - y0) / (float)dy;
    float xlo = (float)x0 - (float)dy * (float)(y - y0) / (float)dx;
    float xhi = (float)x1 - (float)dy * (float)(y - y1) / (float)dx;
    float l = xc - hxf; if (l < xlo) l = xlo;
    float r = xc + hxf; if (r > xhi) r = xhi;
    fillRowBits((int)(l + 0.5f), (int)(r + 0.5f), y);   // 四舍五入
  }
}

// DD X 标志动画页：静态 X 5 秒 → 眨眼一次 0.1 秒 → 回到静态 X（循环）
// 斜切矩形条带：端面垂直于线（直角边），参数 xL=10/xR=630/YOFF=100/W=36
void drawDDPage() {
  clearScreen(false);

  const int xL = 10, xR = 630;   // 左右端点 x
  const int cy = 200;            // 中心 y
  const int YOFF = 100;          // 静态 X 的垂直半高
  const int W = 36;              // 线宽
  const uint32_t BLINK = 100;    // 眨眼总时长 ms

  uint32_t t = millis() % (5000 + BLINK);   // 周期 5.1s
  int yOff = YOFF;               // 静态 X 的垂直偏移
  if (t >= 5000) {
    // 眨眼：dt=0 → yOff=YOFF(张开)；dt=BLINK/2 → yOff=0(收拢成线)；dt=BLINK → yOff=YOFF
    uint32_t dt = t - 5000;                              // 0..BLINK ms
    int p = (dt < BLINK / 2) ? (int)((BLINK / 2 - dt) * 2) : (int)((dt - BLINK / 2) * 2);
    yOff = (int)((long)YOFF * p / BLINK);                // 垂直偏移 100 → 0 → 100
  }

  int y1 = cy - yOff, y2 = cy + yOff;                    // 两臂端点 y
  drawThickLine(xL, y1, xR, y2, W);                      // 臂 A（左上→右下）
  drawThickLine(xL, y2, xR, y1, W);                      // 臂 B（左下→右上）
}

// Cyn 首次重启的终端文字（原始残缺版，17 行：顶部/底部各 1 空行 + 虚线边框，x=看不清 → 花屏噪点）
static const char *cynText[] = {
  "- - - - - - - - - - - - - - - - - - - - - - - - - - -",
  "- - - - - - - - - - - - - - - - - - - - - - - - - - -",
  "",
  "[][][]hello :]",
  " I see you are xxxxxxxxxxxxxxxxxxxxxxxxoful/sad]",
  " I see you are inxxexxxxxxxxxxxxxxxxend]]",
  "  ---  | fD--t] [selfD sxxxxxxxxxxxx[]-xxxxd.]end]]",
  " I see we could [trl] [scx]xxxxxxxxxxxedix] for you",
  " I will not discardxyou",
  " [][][][][][Absolute[sxxxxx]",
  " access? Y/N",
  " ",
  "                                       [ 0001%] '  sh",
  " I see you will be hexe for                 la wxxxx",
  "",
  "- - - - - - - - - - - - - - - - - - - - - - - - - - -",
  "- - - - - - - - - - - - - - - - - - - - - - - - - - -",
};

// 快速行位写入：在某行写入位掩码（处理跨字边界）——比逐像素 drawPixel 快 100 倍
static inline void setRowBits(int x, int y, uint32_t bits) {
  if (y < 0 || y >= SCR_H || x < 0 || x >= SCR_W) return;
  int sh = x & 31;
  int wi = x >> 5;
  framebuf[drawIdx][y][wi] |= bits << sh;
  // 跨 32 位字边界：只要 sh>0 就补写下一字；若未溢出则 bits>>(32-sh) 为 0，无副作用。
  // 注：字符宽度可达 12px(12×22 字库)，sh 在 21~26 时即跨边界，原 sh>26 会漏掉右半导致裁切。
  if (sh > 0 && wi < WORDS_ROW - 1) framebuf[drawIdx][y][wi + 1] |= bits >> (32 - sh);
}

// 10px 高字符渲染：5×7 字体纵向映射到 10 行（sy = row*7/10），水平加粗 2px
// 每行一次字操作（快），'x' 显示为棋盘格噪点（花屏残缺效果）
void drawCharH10(int x, int y, char c) {
  if (c == 'x') {   // 残缺 → 噪点块（相位交替 = 闪烁花屏）
    int ph = ((sendCnt >> 3) & 1);   // 用刷新帧计数翻转相位（core0 无帧计数依赖）
    for (int row = 0; row < 10; row++) {
      uint32_t bits = 0;
      for (int col = 0; col < 5; col++)
        if ((col + row + ph) & 1) bits |= 3u << col;
      setRowBits(x, y + row, bits);
    }
    return;
  }
  if (c == ' ') return;              // 空格
  if (c == '|') {                    // 竖线（字体表外）
    for (int row = 0; row < 10; row++) setRowBits(x, y + row, 0b11u);
    return;
  }
  if (c < 0x20 || c > 0x7E) return;   // 支持到 ~（0x7E），含小写字母
  const uint8_t *g = font5x7[c - 0x20];
  for (int row = 0; row < 10; row++) {
    int sy = row * 7 / 10;           // 目标行 → 源行（7 行扩展为 10 行）
    uint8_t pat = g[sy];
    if (!pat) continue;              // 空行跳过
    uint32_t bits = 0;
    for (int col = 0; col < 5; col++)
      if (pat & (0x10 >> col)) bits |= 3u << col;   // 加粗（col 和 col+1）
    setRowBits(x, y + row, bits);
  }
}

// Codename Coder 12×22 等宽字符渲染（fontCoder 为 uint16_t，bit11=最左列）
// 从 TTF 直接在 12×22 栅格化，1:1 绘制（避免旧 6×11 位图放大导致对角笔画糊团）
void drawCharCoder(int x, int y, char c) {
  if (c == 'x') {   // 残缺 → 噪点块（细 2px×1px 棋盘格，仿 V2 花屏；方格越大越像网格反而眼花）
    int ph = ((sendCnt >> 3) & 1);
    for (int row = 0; row < 22; row++) {
      uint32_t bits = 0;
      for (int col = 0; col < 12; col += 2)
        if (((col >> 1) + row + ph) & 1) bits |= 3u << col;
      setRowBits(x, y + row, bits);
    }
    return;
  }
  if (c == ' ') return;
  if (c < 0x20 || c > 0x7E) return;
  const uint16_t *g = fontCoder[c - 0x20];
  for (int row = 0; row < 22; row++) {
    uint16_t pat = g[row];
    if (!pat) continue;
    uint32_t bits = 0;
    for (int col = 0; col < 12; col++)
      if (pat & (0x800 >> col)) bits |= (1u << col);
    setRowBits(x, y + row, bits);
  }
}

// Cyn 文字页：分隔线 --- --- --- 循环满宽，文字块整体居中（布局不变，保留缩进）
void drawCynTextPage() {
  clearScreen(false);          // 关键：清屏，避免与上一页面残留叠加！

  const bool coder = (hudFont == 1);
  const int charW = coder ? 12 : 11;              // coder 12宽(12×22字库) / 5×7版 6宽加粗+5间距
  const int lineH = coder ? 24 : 21;              // 24高(加大行距，上下各删一空行后边框落在屏内) / 10高+11行距
  const int midOff = coder ? 10 : 4;              // 分隔线纵向偏移
  int rows = sizeof(cynText) / sizeof(cynText[0]);

  // 找最长文字行（分隔线除外），用于整体居中
  int maxLen = 0;
  for (int r = 0; r < rows; r++) {
    if (cynText[r][0] == '-') continue;
    int l = strlen(cynText[r]);
    if (l > maxLen) maxLen = l;
  }
  int x0 = (SCR_W - maxLen * charW) / 2;   // 文字块整体居中起点
  if (x0 < 0) x0 = 0;

  for (int r = 0; r < rows; r++) {
    const char *s = cynText[r];
    if (s[0] == '-') {
      // 分隔线：完整横线段循环（-----），整体左右居中，右缘不画半截段
      int yMid = r * lineH + midOff;
      const int segLen = 30, gapLen = 22, period = 52;
      int nSeg = (SCR_W + gapLen) / period;                    // (640+22)/52 = 12 段
      int total = nSeg * segLen + (nSeg - 1) * gapLen;         // 602px
      int x0d = (SCR_W - total) / 2;                           // (640-602)/2 = 19，居中
      for (int i = 0; i < nSeg; i++) {
        int sX = x0d + i * period;
        for (int x = sX; x <= sX + segLen - 1; x++) drawPixel(x, yMid, true);
      }
      continue;
    }
    // 文字行：从统一 x0 开始（保留各行原文缩进，布局不变）
    int px = x0;
    for (; *s; s++) {
      if (coder) drawCharCoder(px, r * lineH, *s);
      else drawCharH10(px, r * lineH, *s);
      px += charW;
    }
  }
}

void drawDemoFrame() {
  if (page == 1) {
    drawCynTextPage();
    return;
  }
  if (page == 2) {
    drawDDPage();
    return;
  }
  // 页面 0：图标（静态页只画一次，不重复 memcpy）
  for (int y = 0; y < SCR_H; y++)
    memcpy(framebuf[drawIdx][y], solverBitmap[y], WORDS_ROW * 4);
}

// ============================================================
//  串口接收：本地页命令 + USB 推流（二合一状态机，core0 调用）
//   命令（ASCII）：'0'/'1'/'2' → 本地页切换（仅非推流模式响应）
//   命令帧（网页端切页用，推流模式下也生效）：
//     同步头 AA 55 55 AA + 4B 长度=0 + 1B 命令('0'/'1'/'2')
//   推流帧：4B 同步头 AA 55 55 AA + 4B 长度 + 数据
//     长度 = 32000        → 原样 1bit 像素（兼容旧网页/工具）
//     长度 < 32000        → RLE 压缩流 [value][count] 交替，解码后必须 32000B
//     收完一帧回 'K' ack；推流数据不解释为命令；500ms 无新帧回本地页
//   RLE 编码（网页端）：连续相同字节 run（1-255）→ [value][count]
//   压缩收益不足时网页端自动改发原样（长度 32000）
//   写入格式与 framebuf 完全一致：每行 20 字 × 4B 小端，
//   word 内 bit0=LSB 对应 x 最小，偶x→Din0、奇x→Din1（同本地绘制）
// ============================================================
// ============================================================
//  用户设置（flash 持久化，EEPROM 模拟存储，断电保存）
//  网页设置页读写：默认开机页 / LED 颜色 / 呼吸模式与速度
// ============================================================
struct UserSettings {
  uint8_t bootPage = 0;      // 开机默认页 0=图标 1=Cyn 2=DD
  uint8_t ledR = 12;         // LED 颜色 R（默认琥珀）
  uint8_t ledG = 6;          // LED 颜色 G
  uint8_t ledB = 0;          // LED 颜色 B
  uint8_t breathMode = 1;    // 呼吸模式 0=常亮 1=呼吸 2=熄灭
  uint8_t breathSpeed = 3;   // 呼吸周期秒数 1-10
  uint8_t breathBright = 100;// 呼吸峰值亮度 0-100%
  uint8_t sleepBright = 10;  // 息屏待机亮度 0-100%
  uint8_t fanPct[7] = {85, 75, 60, 50, 40, 35, 30};  // 风扇占空比 0-100（档位：视频/图片/图标/DD/时钟/Cyn/表情，热→凉）
  uint8_t magic = 0xA5;      // 存储校验魔数（非法数据时回默认）
};
static UserSettings settings;

void settingsLoad() {
  EEPROM.begin(128);
  UserSettings tmp;
  EEPROM.get(0, tmp);
  settings = (tmp.magic == 0xA5) ? tmp : UserSettings();
  // 风扇档位兼容：旧版本 flash 里 fanPct 区是残余数据，若任一 >100 则整组回默认（防读到乱值导致全速/停机）
  bool bad = false;
  for (int i = 0; i < 7; i++) if (settings.fanPct[i] > 100) { bad = true; break; }
  if (bad) { UserSettings d; for (int i = 0; i < 7; i++) settings.fanPct[i] = d.fanPct[i]; }
}

void settingsSave() {
  settings.magic = 0xA5;
  EEPROM.put(0, settings);
  Serial.println("SAVE BEGIN");
  // ⚠️ RP2040 代码从 flash 取指（XIP）：写/擦 flash 期间，任何核取指都会
  //    崩 → 复位。core1 一直在跑 elSendFrame（flash 代码），必须暂停它：
  //    WFE 睡眠（不取指，PIO 硬件继续跑 → 屏幕不断），写完 SEV 唤醒。
  core1Halt = true;
  uint32_t t0 = millis();
  while (!core1Paused) {
    if (millis() - t0 > 500) {        // 500ms 超时：core1 未暂停 → 放弃保存（防死等+watchdog 复位）
      core1Halt = false;
      __asm volatile("sev");
      Serial.println("SAVE FAIL: core1 no response");
      return;
    }
  }
  noInterrupts();                   // 禁中断：中断 handler 也在 flash
  EEPROM.commit();
  interrupts();
  core1Halt = false;
  __asm volatile("sev");            // 唤醒 core1
  Serial.println("SAVE DONE");
}

enum { RX_SYNC, RX_LEN, RX_DATA, RX_DIFF, RX_CMD, RX_SET, RX_FAN } rxState = RX_SYNC;
static bool rxDiff = false;           // true=差分帧(同步头 AA 55 55 AB)；false=整帧原样/RLE

// 差分帧解码：把 [行号2B + 行数据80B]×N 应用到当前 drawIdx 对应行（其余行保留，不再整帧覆盖）
// ⚠️ 差分行数据 80B 与 framebuf 行布局一致（20×uint32，同 754 行注释的整帧格式）
void applyDiffToFramebuf(uint32_t total) {     // total = 差分数据总字节数（由调用处传入）
  const uint8_t *p = (const uint8_t *)rxTmp;   // 差分数据收在 rxTmp 字节流
  uint32_t off = 0;
  while (off + 2 <= total) {
    uint32_t row = (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8);
    off += 2;
    if (row >= SCR_H) break;                   // 行号越界防御 → 丢弃剩余
    if (off + 80 > total) break;               // 数据越界防御
    memcpy(&framebuf[drawIdx][row], p + off, 80);
    off += 80;
  }
}
static uint8_t rxSync[4]; static int rxSyncIdx = 0;
static uint8_t rxLen[4];  static int rxLenIdx = 0;
static uint32_t rxFrameLen = 0;      // 长度字段（压缩后字节数或 32000 原样）
static uint32_t compGot = 0;         // 已收压缩/原样字节
static uint32_t decGot = 0;          // 已解码写入 framebuf 的字节
static bool rleNeedVal = true;       // RLE 解码：下一个字节是 value
static uint8_t rleVal = 0;           // RLE 解码：当前 value
static bool streamMode = false;
static int  contentLevel = -1;   // 推流内容档位索引（-1=未设; 0视频 1图片 2图标 3DD 4时钟 5Cyn 6表情），上位机 'T' 命令设置
static uint32_t streamLastMs = 0;
static uint8_t setBuf[8];            // 设置数据缓冲（'S' 命令后 8 字节：boot r g b mode speed 呼吸亮度 休眠亮度）
static uint8_t fanBuf[7];            // 风扇档位缓冲（'V' 命令后 7 字节：视频/图片/图标/DD/时钟/Cyn/表情 0-100）
static int    fanIdx = 0;
static uint32_t fanLastMs = 0;       // 'V' 接收超时保护
static int setIdx = 0;
static uint32_t setLastMs = 0;       // 设置接收超时保护

void handleSerial() {
  while (Serial.available()) {
    // 数据期优先整块读（不逐字节预读，避免丢字节）
    if (rxState == RX_DATA) {
      int need = (int)(rxFrameLen - compGot);
      int n = Serial.available();
      if (n > need) n = need;
      if (n > 0) {
        uint8_t *dst = (uint8_t *)rxTmp;           // 写临时缓冲，不直接进 framebuf（避免与 core1 swap 竞态）
        if (rxFrameLen == FRAME_BYTES) {
          // 原样：直接块写
          int rd = Serial.readBytes((char *)dst + decGot, n);
          compGot += rd; decGot += rd;
        } else {
          // RLE：边收边解（块读进小缓冲）
          uint8_t tmp[64];
          if (n > 64) n = 64;   // ⚠️ 必须限制，否则 readBytes 写爆 tmp 栈缓冲 → 死机！
          int rd = Serial.readBytes((char *)tmp, n);
          for (int k = 0; k < rd; k++) {
            compGot++;
            if (rleNeedVal) {
              rleVal = tmp[k];
              rleNeedVal = false;
            } else {
              uint32_t cnt = tmp[k];
              if (decGot + cnt > FRAME_BYTES) {   // 越界防御 → 重同步
                rxState = RX_SYNC; rxSyncIdx = 0;
                rleNeedVal = true;
                break;
              }
              memset(dst + decGot, rleVal, cnt);
              decGot += cnt;
              rleNeedVal = true;
            }
          }
          if (rxState == RX_SYNC) continue;
        }
        // 帧完成判定：解码满 32000 提交；压缩流收完但解码不足 → 坏帧重同步
        if (decGot == FRAME_BYTES) {
          streamMode = true;
          streamLastMs = millis();
          // 整帧收进 rxTmp 后，一次性拷入当前 drawIdx 缓冲（此时 drawIdx 稳定、core1 读的是 dispIdx 另一块）
          memcpy(framebuf[drawIdx], rxTmp, FRAME_BYTES);
          commitFrame();
          Serial.write('K');
          rxState = RX_SYNC; rxSyncIdx = 0;
          rleNeedVal = true;
        } else if (compGot == rxFrameLen) {
          rxState = RX_SYNC; rxSyncIdx = 0;
          rleNeedVal = true;
        }
      }
      continue;
    }

    // ---- 差分帧数据接收（同步头 AA 55 55 AB）：[行号2B + 行数据80B]×N，收满后应用到当前 drawIdx ----
    if (rxState == RX_DIFF) {
      int need = (int)(rxFrameLen - compGot);
      int n = Serial.available();
      if (n > need) n = need;
      if (n > 0) {
        int rd = Serial.readBytes((char *)((uint8_t *)rxTmp) + compGot, n);
        compGot += rd;
        if (compGot == rxFrameLen) {
          streamMode = true;
          streamLastMs = millis();
          applyDiffToFramebuf(compGot);   // 差分数据总字节数（收满时 == rxFrameLen）
          commitFrame();
          Serial.write('K');
          rxState = RX_SYNC; rxSyncIdx = 0;
          rleNeedVal = true;
        }
      }
      continue;
    }

    // ---- 设置数据接收（'S' 命令后 8 字节：boot r g b mode speed 呼吸亮度 休眠亮度）----
    if (rxState == RX_SET) {
      setLastMs = millis();
      int n = Serial.available();
      if (n > 8 - setIdx) n = 8 - setIdx;
      if (n > 0) {
        setIdx += Serial.readBytes((char *)setBuf + setIdx, n);
        if (setIdx == 8) {
          // 应用并保存（LED 即时生效；bootPage 下次开机生效）
          settings.bootPage  = setBuf[0] & 0x03;
          settings.ledR = setBuf[1];
          settings.ledG = setBuf[2];
          settings.ledB = setBuf[3];
          settings.breathMode = setBuf[4] & 0x03;
          settings.breathSpeed = setBuf[5];
          if (settings.breathSpeed < 1) settings.breathSpeed = 1;
          if (settings.breathSpeed > 10) settings.breathSpeed = 10;
          settings.breathBright = setBuf[6];  if (settings.breathBright > 100) settings.breathBright = 100;
          settings.sleepBright  = setBuf[7];  if (settings.sleepBright  > 100) settings.sleepBright  = 100;
          settingsSave();
          Serial.println("SET OK");
          rxState = RX_SYNC; rxSyncIdx = 0;
        }
      }
      continue;
    }

    // ---- 风扇档位接收（'V' 命令后 7 字节：视频/图片/图标/DD/时钟/Cyn/表情占空比 0-100）----
    if (rxState == RX_FAN) {
      fanLastMs = millis();
      int n = Serial.available();
      if (n > 7 - fanIdx) n = 7 - fanIdx;
      if (n > 0) {
        fanIdx += Serial.readBytes((char *)fanBuf + fanIdx, n);
        if (fanIdx == 7) {
          for (int i = 0; i < 7; i++) { if (fanBuf[i] > 100) fanBuf[i] = 100; settings.fanPct[i] = fanBuf[i]; }
          settingsSave();
          Serial.println("FAN OK");
          rxState = RX_SYNC; rxSyncIdx = 0;
        }
      }
      continue;
    }

    uint8_t u = (uint8_t)Serial.read();

    // 本地页命令：仅非推流模式响应
    if (!streamMode) {
      if (u == '0' || u == '1' || u == '2') {
        page = u - '0';
        pageDirty = true;
        rxState = RX_SYNC; rxSyncIdx = 0;   // 重置推流状态机
        Serial.print("PAGE "); Serial.println(page);
        continue;
      }
      if (u == 'F') {   // 直接输 'F' 切换文字字体（0=5×7 放大 / 1=Codename Coder 12×22）
        hudFont ^= 1;
        pageDirty = true;
        rxState = RX_SYNC; rxSyncIdx = 0;
        Serial.print("FONT "); Serial.println(hudFont);
        continue;
      }
    }

    // 推流同步/长度状态机
    if (rxState == RX_SYNC) {
      rxSync[rxSyncIdx++] = u;
      if (rxSyncIdx == 4) {
        if (rxSync[0] == 0xAA && rxSync[1] == 0x55 && rxSync[2] == 0x55 && rxSync[3] == 0xAA) {
          rxDiff = false;                       // 整帧原样 / RLE
          rxState = RX_LEN; rxLenIdx = 0;
        } else if (rxSync[0] == 0xAA && rxSync[1] == 0x55 && rxSync[2] == 0x55 && rxSync[3] == 0xAB) {
          rxDiff = true;                        // 差分帧：[行号2B + 行数据80B]×N
          rxState = RX_LEN; rxLenIdx = 0;
        } else {
          // 移位保留后 3 字节继续匹配
          rxSync[0] = rxSync[1]; rxSync[1] = rxSync[2]; rxSync[2] = rxSync[3];
          rxSyncIdx = 3;
        }
      }
    } else if (rxState == RX_LEN) {
      rxLen[rxLenIdx++] = u;
      if (rxLenIdx == 4) {
        rxFrameLen = (uint32_t)rxLen[0] | ((uint32_t)rxLen[1] << 8)
                   | ((uint32_t)rxLen[2] << 16) | ((uint32_t)rxLen[3] << 24);
        if (rxFrameLen == 0) {
          rxState = RX_CMD;   // 命令帧：长度 0 → 后跟 1 字节命令
        } else if (rxFrameLen <= FRAME_BYTES) {
          compGot = 0; decGot = 0;
          rleNeedVal = true;
          rxState = rxDiff ? RX_DIFF : RX_DATA;
        } else {
          rxState = RX_SYNC; rxSyncIdx = 0;   // 非法长度，重新同步
        }
      }
    } else if (rxState == RX_CMD) {
      // 命令帧：'0'/'1'/'2' 切页（推流模式下也生效，网页端"切页按钮"用）
      //         'R' 读设置（回传 SET 行） / 'S' 写设置（后跟 7 字节）
      if (u == '0' || u == '1' || u == '2') {
        page = u - '0';
        pageDirty = true;
        streamMode = false;   // 命令帧强制退出推流模式
        Serial.print("PAGE "); Serial.println(page);
      } else if (u == 'B') {
        // 回固件默认开机页（网页端"清除帧/停止推流"用——立即生效，不等超时）
        page = settings.bootPage;
        pageDirty = true;
        streamMode = false;
        Serial.print("PAGE "); Serial.println(page);
      } else if (u == 'F') {
        // 切换 HUD 文字字体：0=5×7 放大(10px) / 1=Codename Coder 12×22（仅影响文字页 page=1）
        hudFont ^= 1;
        pageDirty = true;
        streamMode = false;
        Serial.print("FONT "); Serial.println(hudFont);
      } else if (u == 'R') {
        // 读设置回传：SET boot r g b mode speed breathBright sleepBright
        Serial.print("SET ");
        Serial.print(settings.bootPage); Serial.print(' ');
        Serial.print(settings.ledR); Serial.print(' ');
        Serial.print(settings.ledG); Serial.print(' ');
        Serial.print(settings.ledB); Serial.print(' ');
        Serial.print(settings.breathMode); Serial.print(' ');
        Serial.print(settings.breathSpeed); Serial.print(' ');
        Serial.print(settings.breathBright); Serial.print(' ');
        Serial.println(settings.sleepBright);
        Serial.print("FAN ");
        for (int i = 0; i < 7; i++) { Serial.print(settings.fanPct[i]); Serial.print(' '); }
        Serial.println();
      } else if (u == 'S') {
        Serial.println("SET CMD");   // 诊断：固件已收到写设置命令
        setIdx = 0;
        rxState = RX_SET;      // 转入设置数据接收
        continue;
      } else if (u == 'V') {
        Serial.println("FAN CMD");   // 诊断：已收到写风扇档位命令
        fanIdx = 0;
        rxState = RX_FAN;      // 转入风扇档位接收（后跟 7 字节占空比）
        continue;
      } else if (u == 'T') {
        // 内容类型：'T' + 1字节档位索引（0视频 1图片 2图标 3DD 4时钟 5Cyn 6表情）→ 推流档位
        if (Serial.available()) { int ct = Serial.read(); if (ct >= 0 && ct <= 6) contentLevel = ct; }
        Serial.print("CTYPE "); Serial.println(contentLevel);
      }
      rxState = RX_SYNC; rxSyncIdx = 0;
    }
  }

  // 设置接收超时：500ms 收不满 8 字节 → 放弃，回到同步（防网页半途发送卡死状态机）
  if (rxState == RX_SET && (millis() - setLastMs > 500)) {
    Serial.print("SET TIMEOUT got="); Serial.println(setIdx);   // 诊断
    rxState = RX_SYNC; rxSyncIdx = 0;
  }
  // 风扇档位接收超时：500ms 收不满 7 字节 → 放弃
  if (rxState == RX_FAN && (millis() - fanLastMs > 500)) {
    Serial.println("FAN TIMEOUT");
    rxState = RX_SYNC; rxSyncIdx = 0;
  }

  // 推流超时：4000ms 无新帧 → 回本地页（网页端停止发送即触发）
  // ⚠️ 浏览器后台标签页 setTimeout 节流实测 2-3s/次（2026-08-23），
  //    超时需留足余量，否则后台推流误触发回本地页 → 断续。
  //    原 500ms → 2000ms → 4000ms（用户实测后台刷新 2-3s）
  if (streamMode && (millis() - streamLastMs > 4000)) {
    streamMode = false;
    pageDirty = true;
    Serial.println("STREAM TIMEOUT -> LOCAL");
  }
}

// ============================================================
//  外壳版：单键 + 12V 电源控制（GP8=12V 高边开关，GP7=按键）
//  亮屏状态：单击 → 切内置页（0→1→2→0 循环）；长按(≥1s) → 断 12V 息屏
//  息屏状态：单击 → 恢复 12V 亮屏
//  轮询实现（不用中断：arduino-pico 3.1.1 中断链路不可靠，既定避坑策略）
// ============================================================
static bool screenOn = true;                 // 12V 输出状态（true=亮屏）
static const uint32_t BTN_DEBOUNCE_MS = 40;  // 消抖：按住 <40ms 视为毛刺
static const uint32_t BTN_LONG_MS    = 1000; // 长按阈值 1s
static uint32_t btnDownMs = 0;               // 按下时刻
static bool btnWasDown = false;              // 上一采样电平
static bool btnLongDone = false;             // 本次按下已触发过长按

// 12V 通断：on=true 开屏（先开 12V，等逆变器稳定后再由调用方恢复刷新）
static void setScreen(bool on) {
  if (on == screenOn) return;
  screenOn = on;
  gpio_put(PIN_PWR12V, on ? 1 : 0);
  if (on) {
    Serial.println("PWR 12V ON");
    delay(200);   // EL 逆变器启动稳定时间（core0 阻塞无妨，core1 继续发旧帧）
  } else {
    Serial.println("PWR 12V OFF");
  }
}

void handleButton() {
  bool down = (gpio_get(PIN_BTN) == 0);   // 按下=低
  uint32_t now = millis();

  if (down && !btnWasDown) {              // 刚按下（边沿）
    btnWasDown = true;
    btnDownMs = now;
    btnLongDone = false;
  } else if (down && btnWasDown) {        // 持续按住 → 长按检测
    if (!btnLongDone && (now - btnDownMs >= BTN_LONG_MS)) {
      btnLongDone = true;
      if (screenOn) setScreen(false);     // 长按：息屏（息屏状态长按无动作）
    }
  } else if (!down && btnWasDown) {       // 释放
    btnWasDown = false;
    uint32_t hold = now - btnDownMs;
    if (!btnLongDone && hold >= BTN_DEBOUNCE_MS) {   // 单击（排除毛刺与长按释放）
      if (screenOn) {
        // 循环切页顺序（用户指定）：Cyn(1) → 图标(0) → DD(2) → 循环
        static const uint8_t pageOrder[3] = { 1, 0, 2 };
        int cur = 0;
        for (int i = 0; i < 3; i++) if (pageOrder[i] == page) { cur = i; break; }
        page = pageOrder[(cur + 1) % 3];
        pageDirty = true;
        streamMode = false;               // 退出推流（与命令帧切页语义一致）
        Serial.print("BTN PAGE "); Serial.println(page);
      } else {
        setScreen(true);                  // 息屏中单击 → 亮屏
      }
    }
  }
}

// ============================================================
//  主程序：core0 = 画面内容引擎（绘制/串口/指示灯/诊断）
//  core1 = 显示刷新引擎（在 setup 末尾启动）
// ============================================================
// ============================================================
//  风扇开环调速（GP6 PWM 25kHz）：按当前工作档位查表输出占空比
//  档位：视频/图片/图标/DD/时钟/Cyn/表情；息屏停；开机先 50% 后回落
// ============================================================
enum { FAN_VID=0, FAN_IMG=1, FAN_ICON=2, FAN_DD=3, FAN_CLK=4, FAN_CYN=5, FAN_FACE=6 };
void fanSetup() {
  pinMode(PIN_FAN_PWM, OUTPUT);
  analogWriteFreq(25000);   // 全局 PWM 频率 25kHz（arduino-pico 为全局设置，不带 pin）
  analogWrite(PIN_FAN_PWM, 0);
}
int curFanLevel() {
  if (!screenOn) return -1;                                    // 息屏：停
  if (streamMode) return (contentLevel >= 0 && contentLevel <= 6) ? contentLevel : FAN_VID;
  if (page == 0) return FAN_ICON;
  if (page == 1) return FAN_CYN;
  if (page == 2) return FAN_DD;
  return FAN_ICON;
}
void fanUpdate() {
  static uint32_t fanLastMs = 0;
  static int fanCur = 0;
  uint32_t now = millis();
  int target;
  if (now < 5000) target = 50;                                 // 开机先 50%，5s 后回落
  else { int lvl = curFanLevel(); target = (lvl < 0) ? 0 : settings.fanPct[lvl]; }
  if (now - fanLastMs >= 300) {                                // 每 300ms 最多 ±10 平滑
    int d = target - fanCur;
    if (d > 10) d = 10; if (d < -10) d = -10;
    fanCur += d;
    analogWrite(PIN_FAN_PWM, (uint8_t)(fanCur * 255 / 100));
    fanLastMs = now;
  }
}

void setup() {
  // 外壳版电源控制：12V 默认断开 → 软件保证"先 5V 后 12V"上电时序
  pinMode(PIN_PWR12V, OUTPUT);
  gpio_put(PIN_PWR12V, 0);          // 12V 初始断开（5V/逻辑先稳定）
  pinMode(PIN_BTN, INPUT_PULLUP);   // 按键：内部上拉，按下=低
  screenOn = false;

  settingsLoad();                    // 读 flash 用户设置（默认页/LED 参数）
  fanSetup();                        // 风扇 PWM 初始化（GP6, 25kHz, 初始 0%）

  Serial.begin(115200);
  delay(200);
  Serial.println("LJ640U34 Solver DUALCORE starting...");

  // 看门狗：2s 兜底。任何死循环/HardFault → 自动复位（比手动复位强，
  // 复位后走完整上电时序，12V 0.5s 后自动恢复亮屏）
  watchdog_enable(2000, false);

  pinMode(PIN_HD, OUTPUT);
  pinMode(PIN_VD, OUTPUT);
  gpio_put(PIN_HD, 1);
  gpio_put(PIN_VD, 0);

  ledInit();
  ledSetColor(settings.ledR * settings.sleepBright / 100,
              settings.ledG * settings.sleepBright / 100,
              settings.ledB * settings.sleepBright / 100);   // 初始灯 = 休眠亮度

  initSinTab();
  elPioInit();

  // 首帧：画用户设置的默认开机页
  page = settings.bootPage;          // 开机默认显示页（网页设置）
  pageDirty = true;
  drawIdx = 0;
  drawDemoFrame();
  drawIdx = 1;
  dispIdx = 0;
  swapReq = 0;

  // 外壳版时序：通电跑起来后延迟 0.5s → 12V 亮屏
  delay(500);
  gpio_put(PIN_PWR12V, 1);          // 12V 通 → EL 高压上电
  screenOn = true;
  delay(200);                       // 逆变器稳定，避免首帧花屏

  // 启动 core1 刷新引擎
  multicore_launch_core1(core1Refresh);
  Serial.println("core1 refresh engine started (9MHz)");
}

void loop() {
  watchdog_update();    // 喂狗（loop 单圈耗时 << 2s，安全）
  handleButton();       // 外壳版：单键（单击切页 / 长按息屏 / 息屏单击亮屏）
  handleSerial();       // 推流接收 + 切页命令（内部处理 ack/超时）
  fanUpdate();          // 风扇调速（按当前档位查表平滑输出）

  // ⚠️ 推流模式下禁止本地页面绘制：否则本地页（DD/Cyn 每帧重绘）会与
  //    推流帧抢同一块 drawIdx 缓冲 + 抢交换权 → 画面互相覆盖打架，
  //    且 commit 排队导致 ack 延迟（网页端 400ms 超时 → 视频掉到 1-2fps）。
  //    推流模式 = 屏幕整帧交给推流，本地绘制完全停止（2026-08-23 修复）。
  //    息屏时（screenOn=false）也跳过：12V 已断，绘制纯属浪费。
  if (!streamMode && screenOn) {
    if (page == 0) {
      // 图标页：切换时绘制一次（pageDirty 机制）。
      // ⚠️ 不能每帧重绘：曾致 core0 高频 commit 与 core1 交换恶性竞争 → 宕机（2026-08-23 回退）。
      // 画前 while(swapReq) 同步：确保上一帧交换完成、memcpy 期间 drawIdx 稳定，
      // 否则画图标途中 core1 交换会把图标写进两个 buffer → 切页闪烁/覆盖。
      if (pageDirty) {
        pageDirty = false;
        uint32_t t0 = micros();
        while (swapReq) { if (micros() - t0 >= 100000u) break; }   // 超时兜底防死等
        drawDemoFrame();
        commitFrame();
      }
    } else {
      // Cyn 文字页（'x' 噪点需持续闪烁花屏）/ DD X 动画页：每帧重绘。
      // core0 单帧绘制 <5ms，远小于刷新周期 ~17ms → 不影响 core1 刷新节奏，
      // 动画逐帧更新（~57fps）比原版每 2 帧更流畅。
      animTick += 2;
      drawDemoFrame();
      commitFrame();
    }
  }

  // WS2812 呼吸灯（每 60ms 微调，参数来自用户设置：
  // 颜色 ledR/G/B、模式 breathMode 0=常亮 1=呼吸 2=熄灭、
  // 速度 breathSpeed、呼吸峰值亮度 breathBright%、休眠亮度 sleepBright%）
  static uint32_t lastLed = 0;
  uint32_t now = millis();
  if (now - lastLed >= 60) {
    lastLed = now;
    if (!screenOn) {
      // 息屏：休眠亮度（0=全灭）
      ledSetColor(settings.ledR * settings.sleepBright / 100,
                  settings.ledG * settings.sleepBright / 100,
                  settings.ledB * settings.sleepBright / 100);
    } else if (settings.breathMode == 2) {
      ledSetColor(0, 0, 0);                                  // 模式 2：LED 熄灭
    } else if (settings.breathMode == 0) {
      ledSetColor(settings.ledR, settings.ledG, settings.ledB);  // 模式 0：常亮
    } else {
      // 模式 1：呼吸（三角波，周期 = breathSpeed 秒，峰值 = breathBright%）
      uint32_t period = 500u * settings.breathSpeed;         // 半周期 ms
      uint32_t t = now % (2 * period);
      uint32_t b = (t < period) ? (t * 31u / period) : ((2 * period - t) * 31u / period);
      ledSetColor((uint32_t)settings.ledR * b * settings.breathBright / (31u * 100u),
                  (uint32_t)settings.ledG * b * settings.breathBright / (31u * 100u),
                  (uint32_t)settings.ledB * b * settings.breathBright / (31u * 100u));
    }
  }

  // 刷新 fps 诊断（每 5 秒，读 core1 的 sendCnt）
  static uint32_t lastDiag = 0;
  static uint32_t lastCnt = 0;
  if (now - lastDiag >= 5000) {
    lastDiag = now;
    uint32_t sc = sendCnt;
    float fps = (sc - lastCnt) * 1000.0f / 5000.0f;
    lastCnt = sc;
    Serial.printf("REFRESH fps=%.1f sendCnt=%lu\n", fps, (unsigned long)sc);
  }
}
