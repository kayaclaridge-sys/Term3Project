// 定义引脚
const int trigPin = 35;  // 连接到 Giga 的 D2
const int echoPin = 37;  // 连接到 Giga 的 D3

// 定义变量
long duration;
float distance;

void setup() {
  // 初始化串口通信，Giga 的 USB 串口速度可以设得很高
  Serial.begin(115200);
  while (!Serial) {
    ; // 等待串口连接（仅 Giga/Leonardo 等 USB 芯片需要）
  }

  // 设置引脚模式
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("--- HC-SR04 超声波测距测试开始 ---");
}

void loop() {
  // 1. 清空 Trig 引脚（确保干净的高电平脉冲）
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 2. 触发传感器：发送一个 10 微秒的高电平脉冲
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 3. 读取 Echo 引脚返回的高电平时间（单位：微秒）
  // pulseIn 会等待引脚变高，并开始计时，变低时停止计时
  duration = pulseIn(echoPin, HIGH);

  // 4. 计算距离
  // 声速 = 340 m/s = 0.034 cm/μs
  // 距离 = (时间 * 声速) / 2 （因为是往返双程）
  distance = (duration * 0.0343) / 2;

  // 5. 打印结果到串口监视器
  if (duration == 0) {
    Serial.println("超出测量范围或未检测到回波");
  } else {
    Serial.print("当前距离: ");
    Serial.print(distance, 1); // 保留一位小数
    Serial.println(" cm");
  }

  // 延迟 200 毫秒再进行下一次测量，防止发射波干扰接收波
  delay(200);
}