# LJ640U34 EL 显示器驱动

基于 **Sharp LJ64EU34**（640×400 单色琥珀 EL 显示屏）的显示驱动与上位机项目。
本项目为**个人非商业粉丝自制**，与 Glitch Productions 无关。

## 仓库内容

 `LJ640U34_solver_dualcore_v6/`  RP2040-Zero 下位机固件   （arduino-pico core 6.0.0） 
 `usb_el_display_single.html`    上位机   **单文件版**    （Web Serial 推流） 
 `Cyn_face_sim_single.html`      模拟器   **单文件版**     (验证绘制效果的仿真模拟器)
 `icon_editor_single.html`       绘制工具 **单文件版**     (图标绘制工具)
 `LICENSE`                       授权声明                 （代码 MIT、素材 CC BY-NC、第三方声明） 

## 硬件概览

- **面板**：Sharp LJ64EU34（640×400 单色 EL；5 路 TTL + 5V/12V 双电源）
- **主控**：RP2040-Zero（3.3V）+ LJ245A 电平转换（VCA3.3 / VCB5V / 共地 / OE=GND / DIR=5V）
- **信号接线**：GP10→CK0、GP11→Din0、GP12→Din1、GP13→H.D、GP14→V.D
- **外设**：GP7 单键、GP8 12V 高边开关、GP6 风扇 PWM（25kHz）、GP16 WS2812

## 使用

1. **固件**：用 arduino-pico core 6.0.0 打开 `LJ640U34_solver_dualcore_v6/` 并烧录。
2. **上位机**：浏览器打开 `usb_el_display_single.html`，连接串口后选择推流内容类型。

## 授权

- **代码**（固件 / 上位机逻辑）：MIT License —— 见 `LICENSE`。
- **素材**（自绘表情 / 图标）：CC BY-NC 4.0（署名-非商业性使用）。
- **角色形象**：版权归 **Glitch Productions**（《无机杀手 / Murder Drones》），本项目为粉丝自制、非官方。

## 安全注意（务必遵守）

- 面板背面存在 **200V AC 脉冲**，通电后严禁触碰。
- 上电顺序：**先 5V，后 12V**。
- **严禁用 USB 给面板供电**。
