# Term3Project

## full_testv1 / full_v1 使用说明（中文）

`full_v1/full_v1.ino` 是集成传感器调试代码，用于查看 9 路 QTR 巡线传感器、左右 HC-SR04 超声波距离传感器，并手动控制投放舵机。上传后请打开 Serial Monitor，波特率设为 `115200`。

启动后程序会先进行约 5 秒 QTR 校准。校准期间请把 QTR 传感器在白色地面和黑线之间移动，让每个传感器都看到白底和黑线。

### 串口按键

| 按键 | 功能 | 使用方法 |
| --- | --- | --- |
| `u` / `U` | 显示 QTR 巡线传感器数据 | 在 Serial Monitor 输入 `u`，程序会持续输出 9 路黑线检测结果和校准值。 |
| `i` / `I` | 显示左右 HC-SR04 距离数据 | 输入 `i`，程序会持续输出左、右超声波传感器距离。 |
| `s` / `S` | 停止数据显示 | 输入 `s`，停止连续打印 QTR 或距离数据。 |
| `a` / `A` | 舵机顺时针转 60 度 | 输入 `a`，投放/拨盘舵机按设定方向移动一步。 |
| `d` / `D` | 舵机逆时针转 60 度 | 输入 `d`，投放/拨盘舵机反方向移动一步。 |
| `x` / `X` | 舵机回到 0 度 | 输入 `x`，舵机回到初始 0 度位置。 |

### 实体按钮和 LED

| 引脚 | 功能 | 行为 |
| --- | --- | --- |
| `D32` | Revive button 1 | 使用 `INPUT_PULLUP`，按钮按下时读数为 `LOW`。 |
| `D33` | Revive button 2 | 使用 `INPUT_PULLUP`，按钮按下时读数为 `LOW`。 |
| `D34` | 红色 LED | 两个按钮都没按下时亮起；任意按钮按下时熄灭。 |
| `D35` | 绿色 LED | 任意按钮按下时亮起；两个按钮都没按下时熄灭。 |

注意：当前实体按钮只控制红/绿 LED 状态，用来测试 revive button 输入；它不会自动改变舵机、距离传感器或 QTR 显示模式。

## full_testv1 / full_v1 User Guide (English)

`full_v1/full_v1.ino` is an integrated sensor debug sketch. It is used to view the 9-channel QTR line sensor readings, the left/right HC-SR04 ultrasonic distance readings, and manually move the seed/dropper servo. After uploading, open the Serial Monitor at `115200` baud.

On startup, the sketch runs QTR calibration for about 5 seconds. During calibration, move the QTR sensor bar across both the white floor/background and the black line so every sensor sees both surfaces.

### Serial Keys

| Key | Function | How to use it |
| --- | --- | --- |
| `u` / `U` | Show QTR line sensor readings | Type `u` in the Serial Monitor to continuously print the 9-channel black-line detection and calibrated values. |
| `i` / `I` | Show left/right HC-SR04 distance readings | Type `i` to continuously print the left and right ultrasonic sensor distances. |
| `s` / `S` | Stop data display | Type `s` to stop the continuous QTR or distance output. |
| `a` / `A` | Rotate servo clockwise by 60 degrees | Type `a` to move the seed/dropper servo one step in the configured clockwise direction. |
| `d` / `D` | Rotate servo counter-clockwise by 60 degrees | Type `d` to move the seed/dropper servo one step in the opposite direction. |
| `x` / `X` | Reset servo to 0 degrees | Type `x` to return the servo to the initial 0-degree position. |

### Physical Buttons and LEDs

| Pin | Function | Behavior |
| --- | --- | --- |
| `D32` | Revive button 1 | Uses `INPUT_PULLUP`, so the pin reads `LOW` when pressed. |
| `D33` | Revive button 2 | Uses `INPUT_PULLUP`, so the pin reads `LOW` when pressed. |
| `D34` | Red LED | On when neither button is pressed; off when either button is pressed. |
| `D35` | Green LED | On when either button is pressed; off when neither button is pressed. |

Note: the physical buttons currently only drive the red/green LED state for revive button input testing. They do not automatically change the servo, distance sensor, or QTR display mode.
