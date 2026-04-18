// =========================================================
// Pins
// =========================================================
const int PIN_R1  = 5;    // Heater 1
const int PIN_R2  = 18;   // Heater 2
const int PIN_FAN = 19;   // Fan

// =========================================================
// PWM settings
// =========================================================
const int PWM_FREQ = 25000;     // 25 kHz (good for PC fan)
const int PWM_RES  = 8;         // 0–255

// =========================================================
// PWM values
// =========================================================
int pwm_r1  = 0;
int pwm_r2  = 0;
int pwm_fan = 0;

// =========================================================
// Serial input buffer
// =========================================================
String inputLine = "";

// =========================================================
// Apply PWM to hardware
// =========================================================
void applyPWM() {
  ledcWrite(PIN_R1, pwm_r1);
  ledcWrite(PIN_R2, pwm_r2);
  ledcWrite(PIN_FAN, pwm_fan);

  Serial.print("Applied -> R1=");
  Serial.print(pwm_r1);
  Serial.print(" | R2=");
  Serial.print(pwm_r2);
  Serial.print(" | FAN=");
  Serial.println(pwm_fan);
}

// =========================================================
// Parse incoming command
// Expected: R1=120,R2=80,FAN=200
// =========================================================
void parseCommand(String line) {
  line.trim();

  int r1_index  = line.indexOf("R1=");
  int r2_index  = line.indexOf("R2=");
  int fan_index = line.indexOf("FAN=");

  if (r1_index == -1 || r2_index == -1 || fan_index == -1) {
    Serial.println("Invalid command format");
    return;
  }

  int comma1 = line.indexOf(',', r1_index);
  int comma2 = line.indexOf(',', r2_index);

  if (comma1 == -1 || comma2 == -1) {
    Serial.println("Missing comma");
    return;
  }

  String r1_str  = line.substring(r1_index + 3, comma1);
  String r2_str  = line.substring(r2_index + 3, comma2);
  String fan_str = line.substring(fan_index + 4);

  pwm_r1  = constrain(r1_str.toInt(),  0, 255);
  pwm_r2  = constrain(r2_str.toInt(),  0, 255);
  pwm_fan = constrain(fan_str.toInt(), 0, 255);

  applyPWM();
}

// =========================================================
// Setup
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Attach PWM channels
  ledcAttach(PIN_R1,  PWM_FREQ, PWM_RES);
  ledcAttach(PIN_R2,  PWM_FREQ, PWM_RES);
  ledcAttach(PIN_FAN, PWM_FREQ, PWM_RES);

  // Start OFF
  ledcWrite(PIN_R1, 0);
  ledcWrite(PIN_R2, 0);
  ledcWrite(PIN_FAN, 0);

  Serial.println("ESP32 3-channel PWM ready");
  Serial.println("Format: R1=120,R2=80,FAN=200");
}

// =========================================================
// Loop
// =========================================================
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      parseCommand(inputLine);
      inputLine = "";
    }
    else if (c != '\r') {
      inputLine += c;
    }
  }
}