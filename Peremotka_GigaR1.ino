#include <Arduino_GigaDisplay_GFX.h>
#include <GigaDisplayRGB.h>
#include <avr/pgmspace.h>

GigaDisplay_GFX tft;
GigaDisplayRGB rgb;

// --- Моторы ---
struct MotorPins { uint8_t step, dir, en, ms; };
constexpr MotorPins motors[] = {
  {D2,  D3,  D4,  D5},
  {D6,  D7,  D8,  D9},
  {D10, D11, D12, D13}
};

// --- Энкодеры ---
struct EncoderPins { uint8_t clk, dt, sw; int lastClk, lastDt, lastSw; };
EncoderPins encoders[] = {
  {D48, D50, D52, HIGH, HIGH, HIGH}, // Speed encoder
  {D42, D44, D46, HIGH, HIGH, HIGH}  // Layer encoder
};

// --- Constants ---
constexpr uint8_t speedPins[]   = {D17, D18, D19, D20, D21};
constexpr int     speedValues[] = {0, 60, 120, 180, 240};
constexpr float   MAX_LAYERS    = 40.0f;
constexpr uint8_t SWITCH_PIN    = D40;
constexpr uint8_t ENDSTOP_L     = A2;
constexpr uint8_t ENDSTOP_R     = A3;

// --- Colors (RGB565) ---
constexpr uint16_t BLACK      = 0x0000;
constexpr uint16_t RED        = 0xF800;
constexpr uint16_t GREEN      = 0x07E0;
constexpr uint16_t YELLOW     = 0xFFE0;
constexpr uint16_t LIGHTGREY  = 0xC618;
constexpr uint16_t LIGHTGREEN = 0xAFE5;
constexpr uint16_t WHITE      = 0xFFFF;

// --- Modes (Flash) ---
const char mode0[] PROGMEM = "SETUP";
const char mode1[] PROGMEM = "WAITING";
const char mode2[] PROGMEM = "WINDING";
const char mode3[] PROGMEM = "AUTOSTOP";
const char* const modes[] PROGMEM = {mode0, mode1, mode2, mode3};

// --- State ---
uint8_t  currentMode      = 1;
int      currentSpeed     = 0;
int      targetSpeed      = 180;
float    currentLayer     = 0.0f;
float    targetLayers     = 25.0f;
bool     targetModeActive = true;
bool     lockControls     = false;
unsigned long lockStart   = 0;

// Filament parameters
float filamentWidth      = 1.75f;  // initial reference thickness
static int rawRef        = 0;      // ADC reference at startup
static float lastDelta   = 0.0f;   // last applied width delta;  // initial set to 1.70 mm
int acceleration   = 20; // delta RPM per second
float slowdownLayers = 3.0f;

constexpr int MIN_SPEED  = 0;
constexpr int MAX_SPEED  = 300;
constexpr int SPEED_STEP = 1;

// LED blinking
bool     ledState            = false;
unsigned long previousMillis = 0;
const long onInterval        = 0;
const long offInterval       = 2000;
unsigned long currentInterval= onInterval;
uint8_t  r = 0, g = 255, b = 0;

int activeButton = 0;

// Prototypes
void initializePins();
void initializeRGB();
void initializeDisplay();
void drawInterface();
void drawMainInfo();
void drawSpeedButtons();
void updateSpeedDisplay(int speed);
void checkControls();
void drawBlock(int x, int y, int w, int h, uint16_t bg, const __FlashStringHelper* txt, uint8_t sz, uint16_t tc);
void debugFilamentWidth();
void updateRGB();

void setup() {
  Serial.begin(9600);
  initializePins();
  initializeRGB();
  initializeDisplay();
}

void loop() {
  // Handle RGB LED (buttons override blinking)
  updateRGB();

  // Update filament width display when changed" display when changed
  debugFilamentWidth();

  // Controls
  if (!lockControls) {
    checkControls();
  } else if (millis() - lockStart >= 2000) {
    lockControls = false;
    drawInterface();
  }

  // Mode auto-switch
  uint8_t newMode = (currentSpeed > 0 ? 2 : (currentMode == 2 ? 1 : currentMode));
  if (newMode != currentMode && !lockControls) {
    currentMode = newMode;
    drawInterface();
  }
}

// Reads A0, computes delta from startup, updates if change ≥0.01
void debugFilamentWidth() {
  const int lm = 10;
  int raw = analogRead(A0);
  if (rawRef == 0) {
    // first call: capture reference
    rawRef = raw;
    // initial display
    tft.fillRect(lm + 20, 338, 200, 16, BLACK);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.setCursor(lm + 20, 340);
    tft.print(F("Filament: "));
    tft.setTextColor(0x07FF);
    tft.print(filamentWidth, 2);
    tft.setTextColor(WHITE);
    tft.print(F(" mm"));
    return;
  }
  int deltaRaw = raw - rawRef;
  float deltaWidth = -deltaRaw * 0.000195f;
  if (fabs(deltaWidth - lastDelta) >= 0.01f) {
    lastDelta = deltaWidth;
    filamentWidth = 1.75f + deltaWidth;
    // update display
    tft.fillRect(lm + 20, 338, 200, 16, BLACK);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.setCursor(lm + 20, 340);
    tft.print(F("Filament: "));
    tft.setTextColor(0x07FF);
    tft.print(filamentWidth, 2);
    tft.setTextColor(WHITE);
    tft.print(F(" mm"));
  }
}
void initializePins() {
  for (auto &m : motors) {
    pinMode(m.step, OUTPUT);
    pinMode(m.dir,  OUTPUT);
    pinMode(m.en,   OUTPUT);
    pinMode(m.ms,   OUTPUT);
  }
  for (auto &e : encoders) {
    pinMode(e.clk, INPUT_PULLUP);
    pinMode(e.dt,  INPUT_PULLUP);
    pinMode(e.sw,  INPUT_PULLUP);
    e.lastClk = digitalRead(e.clk);
    e.lastDt  = digitalRead(e.dt);
    e.lastSw  = digitalRead(e.sw);
  }
  for (auto p : speedPins) pinMode(p, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_L,  INPUT);
  pinMode(ENDSTOP_R,  INPUT);
}

void initializeRGB() {
  rgb.begin();
  rgb.on(r, g, b);
}

void initializeDisplay() {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  drawInterface();
}

void drawBlock(int x, int y, int w, int h, uint16_t bg, const __FlashStringHelper* txt, uint8_t sz, uint16_t tc) {
  tft.fillRoundRect(x, y, w, h, 8, bg);
  if (txt) {
    tft.setTextSize(sz);
    tft.setTextColor(tc);
    const char* p = reinterpret_cast<const char*>(txt);
    int len = strlen_P(p);
    int tw  = len * 6 * sz;
    int th  = sz * 8;
    tft.setCursor(x + (w - tw) / 2, y + (h - th) / 2);
    tft.print(txt);
  }
}

void drawInterface() {
  tft.fillScreen(BLACK);
  int lm = 10, rm = 10;
  int availW = tft.width() - lm - rm;
  int bw = availW / 4 - 2;
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t col = (i == currentMode) ? GREEN : LIGHTGREY;
    int x = lm + i * (bw + 2);
    tft.fillRoundRect(x, 5, bw, 60, 8, col);
    tft.setTextColor(BLACK);
    tft.setTextSize(3);
    char buf[10]; strcpy_P(buf, (char*)pgm_read_ptr(&(modes[i])));
    int tw = strlen(buf) * 18;
    tft.setCursor(x + (bw - tw) / 2, 25);
    tft.print(buf);
  }
  drawMainInfo();
  drawSpeedButtons();
}

void drawMainInfo() {
  const int lm = 10, rm = 10, gap = 3, adj = 6;
  int totalW = tft.width() - lm - rm;
  int speedW = totalW / 3 + adj;
  int layerW = totalW - speedW - gap;
  int lx = lm;

  // LAYER
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(lx + (layerW - 5*24) / 2, 110);
  tft.print(F("LAYER"));
  tft.fillRoundRect(lx, 160, layerW, 120, 10, LIGHTGREY);

  // Current layer
  tft.setTextColor(BLACK);
  tft.setTextSize(8);
  char buf[10]; sprintf(buf, "%.1f", currentLayer);
  tft.setCursor(lx + 30, 180);
  tft.print(buf);

  // Slash
  tft.setCursor(lx + layerW/2 - 20, 180);
  tft.print(F("/"));

  // Target layer
  tft.setTextColor(GREEN);
  sprintf(buf, "%.1f", targetLayers);
  tft.setCursor(lx + layerW/2 + 30, 180);
  tft.print(buf);

  // Target mode
  tft.fillRect(lx + 20, 288, layerW - 40, 16, BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(lx + 20, 290);
  tft.print(F("Target mode: "));
  tft.setTextColor(targetModeActive ? GREEN : WHITE);
  tft.setCursor(lx + layerW*2/3, 290);
  tft.print(targetModeActive ? F("ACTIVE") : F("PASIVE"));

  // SPEED
  int sx = lm + layerW + gap;
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(sx + (speedW - 5*24) / 2, 110);
  tft.print(F("SPEED"));
  tft.fillRoundRect(sx, 160, speedW, 120, 10, LIGHTGREY);
  updateSpeedDisplay(currentSpeed);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(sx + 20, 290);
  tft.print(F("Target: --- RPM"));

  // Filament width
  tft.fillRect(lm + 20, 338, 200, 16, BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(lm + 20, 340);
  tft.print(F("Filament: "));
  // Numeric width in light blue
  tft.setTextColor(0x07FF);
  tft.print(filamentWidth, 2);
  // Unit in white
  tft.setTextColor(WHITE);
  tft.print(F(" mm"));

  // Accel & Slowdown
  tft.setCursor(lm + 270, 340);
  tft.print(F("Accel: "));
  tft.print(acceleration, 1);
  tft.setCursor(lm + 470, 340);
  tft.print(F("Slowdown: "));
  tft.print(slowdownLayers, 1);
  tft.print(F(" layers"));
}

void updateSpeedDisplay(int speed) {
  const int lm=10, rm=10, gap=3, adj=6;
  int totalW = tft.width() - lm - rm;
  int speedW = totalW/3 + adj;
  int layerW = totalW - speedW - gap;
  int x      = lm + layerW + gap;

  tft.fillRect(x, 160, speedW, 120, LIGHTGREY);
  tft.setTextSize(10);
  tft.setTextColor(RED, LIGHTGREY);
  char buf[5]; sprintf(buf, "%d", speed);
  int tw = strlen(buf)*6*10;
  tft.setCursor(x + (speedW - tw)/2, 180);
  tft.print(buf);
  tft.setTextSize(3);
  tft.setTextColor(RED, LIGHTGREY);
  tft.setCursor(x + speedW - 58, 190);
  tft.print(F("RPM"));
}

void drawSpeedButtons() {
  const int lm=10, rm=10;
  int availW = tft.width() - lm - rm;
  int bw     = availW/5 - 2;
  char buf[5];
  for (uint8_t i = 0; i < 5; i++) {
    uint16_t col = (i == activeButton) ? YELLOW : LIGHTGREY;
    drawBlock(lm + i*(bw+2), 400, bw, 60, col, nullptr, 5, BLACK);
    sprintf(buf, "%d", speedValues[i]);
    int tw = strlen(buf)*6*5;
    tft.setTextSize(5);
    tft.setTextColor(BLACK);
    tft.setCursor(lm + i*(bw+2) + (bw - tw)/2, 415);
    tft.print(buf);
  }
}

void checkControls() {
  for (uint8_t idx = 0; idx < 2; idx++) {
    auto &e = encoders[idx];
    int sw = digitalRead(e.sw);
    if (sw != e.lastSw) {
      if (idx == 0 && sw == LOW && currentSpeed == 0) {
        lockControls = true;
        lockStart    = millis();
        // Flash SETUP and grey WAITING
        tft.fillRoundRect(10, 5, (tft.width()-20)/4, 60, 8, GREEN);
        tft.setTextColor(BLACK); tft.setTextSize(3);
        tft.setCursor(10 + (((tft.width()-20)/4) - strlen("SETUP")*18)/2, 25);
        tft.print(F("SETUP"));
        int lm=10, bw=(tft.width()-20)/4-2, xw=lm+(bw+2)*1;
        tft.fillRoundRect(xw,5,bw,60,8,LIGHTGREY);
        tft.setTextColor(BLACK); tft.setTextSize(3);
        char bufW[10]; strcpy_P(bufW,(char*)pgm_read_ptr(&(modes[1])));
        int tw=strlen(bufW)*18; tft.setCursor(xw+(bw-tw)/2,25); tft.print(bufW);
      }
      if (idx == 1 && sw == LOW) {
        targetModeActive = !targetModeActive;
        drawMainInfo();
      }
      e.lastSw = sw;
      delay(25);
    }
    if (lockControls) continue;
    int clk = digitalRead(e.clk), dt = digitalRead(e.dt);
    if (clk != e.lastClk && clk==LOW && e.lastClk==HIGH) {
      if (idx==0) {
        if (dt==LOW) currentSpeed=max(MIN_SPEED,currentSpeed-SPEED_STEP);
        else          currentSpeed=min(MAX_SPEED,currentSpeed+SPEED_STEP);
        updateSpeedDisplay(currentSpeed);
        activeButton=-1;
        for(uint8_t i=0;i<5;i++) if(currentSpeed==speedValues[i]){activeButton=i;break;}
        drawSpeedButtons();
      } else {
        if (dt==HIGH) targetLayers=min(MAX_LAYERS,targetLayers+0.5f);
        else           targetLayers=max(0.5f,targetLayers-0.5f);
        drawMainInfo();
      }
    }
    e.lastClk=clk; e.lastDt=dt;
  }
}

// Update RGB LED: blink normally, override to solid red when speed buttons pressed
void updateRGB() {
  // Override when switch D22 is closed
  if (digitalRead(SWITCH_PIN) == LOW) {
    rgb.on(255, 0, 0);  // solid red when switch closed
    return;
  }
  // If any speed button is pressed (INPUT_PULLUP, LOW when pressed)
  for (uint8_t i = 0; i < sizeof(speedPins)/sizeof(speedPins[0]); i++) {
    if (digitalRead(speedPins[i]) == LOW) {
      rgb.on(255, 0, 0);  // solid red
      return;
    }
  }
  // Normal blinking
  unsigned long now = millis();
  if (now - previousMillis >= currentInterval) {
    previousMillis   = now;
    ledState         = !ledState;
    currentInterval  = ledState ? onInterval : offInterval;
    if (ledState) rgb.on(r, g, b);
    else          rgb.off();
  }
}


