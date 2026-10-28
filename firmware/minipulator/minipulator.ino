/*
  minipulator — прошивка манипулятора для работы с ROS 2 по USB-порту.
  =================================================================
  Плата — «глупый исполнитель»: получает целевые углы серв и время
  перехода, плавно едет, отдаёт текущее положение. Вся кинематика
  живёт на стороне ПК.

  Порт: 115200, строки ASCII, перевод строки '\n'.

  ------------------------------------------------------------------
  ПРОТОКОЛ
  ------------------------------------------------------------------
  С ПК на плату:
      T<s1> <s2> <s3> <s4> <s5> <s6> <мс>     цели в градусах сервы + время
      P                                        пинг

  Пример: T90 100 10 90 0 0 800

  С платы на ПК:
      READY                 готова, в стартовой позе
      S<s1> <s2> <s3> <s4> <s5> <s6>     текущее положение, 20 раз в секунду
      PONG                  ответ на ping
      E <текст>             сообщение об ошибке

  ------------------------------------------------------------------
  ПОВЕДЕНИЕ
  ------------------------------------------------------------------
  - При включении: пауза 1,5 с, затем моторы включаются ПО ОЧЕРЕДИ (250 мс)
    и встают в стартовую позу 90/90/0/40/0/0.
  - Углы из команды T ограничиваются рабочими диапазонами серв.
  - Время перехода не может быть меньше, чем нужно для скорости MAX_SPEED —
    защита от рывка при «T ... 1».
  - Если команды не приходят дольше WATCHDOG_MS, плата один раз пишет
    "E timeout" и продолжает держать последнее положение. Специально не
    отключает сервы: рука без питания падает под собственным весом.
  - Углы — в градусах СЕРВЫ (0..180), а не в координатах модели. Пересчёт
    q <-> серва делает драйвер на ПК.
*/

#include <Servo.h>

const uint8_t AXES = 6;
const uint8_t PINS[AXES] = {4, 5, 6, 7, 8, 9};

// рабочие диапазоны серв, градусы
const float SERVO_MIN[AXES] = {  0,   0,   0,   0,   0,   0};
const float SERVO_MAX[AXES] = {180, 180,  90, 180, 180,  90};
const float START_ANGLE[AXES] = {90,  90,   0,  40,   0,   0};

const unsigned long BAUD           = 115200;
const float         MAX_SPEED      = 150.0;   // град/с, защита от рывков
const unsigned long WATCHDOG_MS    = 1000;
const unsigned long REPORT_MS      = 50;      // 20 Гц
const unsigned long TICK_MS        = 10;      // шаг интерполяции
const unsigned long MOVE_MAX_MS    = 10000;
const unsigned long MOVE_MIN_MS    = 50;

Servo sv[AXES];
bool  on[AXES];

float pos[AXES];        // текущее положение, градусы сервы
float fromAng[AXES];    // откуда едем
float toAng[AXES];      // куда едем
unsigned long moveStart = 0;
unsigned long moveDur   = 0;
bool  moving = false;

unsigned long lastCmd = 0;
unsigned long lastReport = 0;
unsigned long lastTick = 0;
bool  timeoutReported = false;

char    line[80];
uint8_t lineLen = 0;

// ---------------------------------------------------------------- сервы

int usOf(float deg) {
  int us = (int)(500.0f + deg * (2000.0f / 180.0f) + 0.5f);
  if (us < 500) us = 500;
  if (us > 2500) us = 2500;
  return us;
}

void writeServo(uint8_t i) {
  if (on[i]) sv[i].writeMicroseconds(usOf(pos[i]));
}

void enableAll() {
  for (uint8_t i = 0; i < AXES; i++) {
    if (on[i]) continue;
    sv[i].attach(PINS[i], 500, 2500);
    on[i] = true;
    writeServo(i);
    delay(250);   // по очереди — меньше бросок тока
  }
}

// ---------------------------------------------------------------- движение

void startMove(float target[AXES], unsigned long durMs) {
  float maxDelta = 0;
  for (uint8_t i = 0; i < AXES; i++) {
    float v = target[i];
    if (v < SERVO_MIN[i]) v = SERVO_MIN[i];
    if (v > SERVO_MAX[i]) v = SERVO_MAX[i];
    toAng[i] = v;
    fromAng[i] = pos[i];
    float d = toAng[i] - fromAng[i];
    if (d < 0) d = -d;
    if (d > maxDelta) maxDelta = d;
  }

  unsigned long minDur = (unsigned long)(maxDelta / MAX_SPEED * 1000.0f);
  if (minDur < MOVE_MIN_MS) minDur = MOVE_MIN_MS;
  if (durMs < minDur) durMs = minDur;
  if (durMs < MOVE_MIN_MS) durMs = MOVE_MIN_MS;
  if (durMs > MOVE_MAX_MS) durMs = MOVE_MAX_MS;

  moveStart = millis();
  moveDur   = durMs;
  moving    = true;
}

void tick() {
  if (!moving) return;
  if (millis() - lastTick < TICK_MS) return;
  lastTick = millis();

  float f = (float)(millis() - moveStart) / (float)moveDur;
  if (f >= 1.0f) { f = 1.0f; moving = false; }

  for (uint8_t i = 0; i < AXES; i++) {
    pos[i] = fromAng[i] + (toAng[i] - fromAng[i]) * f;
    writeServo(i);
  }
}

// ---------------------------------------------------------------- протокол

void report() {
  Serial.print('S');
  for (uint8_t i = 0; i < AXES; i++) {
    Serial.print(' ');
    Serial.print(pos[i], 1);
  }
  Serial.println();
}

void err(const __FlashStringHelper *msg) {
  Serial.print(F("E "));
  Serial.println(msg);
}

void handleT(char *s) {
  float v[AXES];
  char *tok = strtok(s, " ,\t");
  for (uint8_t i = 0; i < AXES; i++) {
    if (tok == NULL) { err(F("need 6 angles")); return; }
    v[i] = atof(tok);
    tok = strtok(NULL, " ,\t");
  }
  unsigned long dur = 500;
  if (tok != NULL) {
    float d = atof(tok);
    if (d > 0) dur = (unsigned long)d;
  }
  startMove(v, dur);
  lastCmd = millis();
  timeoutReported = false;
}

void handleLine(char *s) {
  while (*s == ' ') s++;
  if (*s == '\0') return;

  char c = *s;
  if (c == 'T' || c == 't') { handleT(s + 1); return; }
  if (c == 'P' || c == 'p') { Serial.println(F("PONG")); return; }

  err(F("unknown command"));
}

// ---------------------------------------------------------------- main

void setup() {
  Serial.begin(BAUD);
  for (uint8_t i = 0; i < AXES; i++) {
    pos[i] = fromAng[i] = toAng[i] = START_ANGLE[i];
    on[i] = false;
  }

  delay(1500);
  enableAll();

  lastCmd = millis();
  Serial.println(F("READY"));
}

void loop() {
  unsigned long now = millis();

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[lineLen] = '\0';
      handleLine(line);
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = ch;
    }
  }

  tick();

  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    report();
  }

  // millis(), а не now: команда могла обработаться уже после снятия now,
  // и тогда now - lastCmd беззнаково уходит в 4 миллиарда — ложный timeout
  if (!timeoutReported && (millis() - lastCmd > WATCHDOG_MS)) {
    timeoutReported = true;
    err(F("timeout — держу последнее положение"));
  }
}
