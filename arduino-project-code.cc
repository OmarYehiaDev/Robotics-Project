#define enA 10
#define in1 9
#define in2 7
#define in3 8
#define in4 6 
#define enB 5
#define L_S A1
#define R_S A0

// إعدادات السرعة
int baseSpeed = 150;       // السرعة الأساسية عند السير
int turnSpeed = 100;       // سرعة الدوران
int adjustSpeed = 80;      // سرعة التصحيح السريع
int fineAdjustSpeed = 40;  // سرعة التصحيح الدقيق

// متغيرات التحكم
unsigned long lastAdjustTime = 0;
const int adjustInterval = 50; // زمن التعديل بالمللي ثانية

void setup() {
  Serial.begin(9600);
  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
}

void loop() {
  // قراءة السنسورات (الآن: 1 = أسود، 0 = أبيض)
  bool rightBlack = digitalRead(R_S);
  bool leftBlack = digitalRead(L_S);

  if (millis() - lastAdjustTime > adjustInterval) {
    if (leftBlack && rightBlack) {
      // كلا السنسورين على الأسود - حركة أمامية
      smoothForward();
      Serial.println("Moving - Both on BLACK");
    } 
    else if (!leftBlack && !rightBlack) {
      // كلا السنسورين على الأبيض - توقف كامل
      fullStop();
      Serial.println("STOP - Both on WHITE");
    }
    else if (!leftBlack && rightBlack) {
      // اليسار أبيض (خارج المسار) - تصحيح لليمين
      quickAdjustRight();
      Serial.println("Adjust RIGHT (Left on white)");
    }
    else if (leftBlack && !rightBlack) {
      // اليمين أبيض (خارج المسار) - تصحيح لليسار
      quickAdjustLeft();
      Serial.println("Adjust LEFT (Right on white)");
    }
    lastAdjustTime = millis();
  }
}

// ========= دوال الحركة =========
void smoothForward() {
  analogWrite(enA, baseSpeed);
  analogWrite(enB, baseSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void quickAdjustRight() {
  // المحرك الأيمن يعمل (يسار العربة يرفع)
  analogWrite(enA, adjustSpeed);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void quickAdjustLeft() {
  // المحرك الأيسر يعمل (يمين العربة يرفع)
  analogWrite(enA, 0);
  analogWrite(enB, adjustSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void fineAdjustRight() {
  analogWrite(enA, fineAdjustSpeed);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void fineAdjustLeft() {
  analogWrite(enA, 0);
  analogWrite(enB, fineAdjustSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void fullStop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
