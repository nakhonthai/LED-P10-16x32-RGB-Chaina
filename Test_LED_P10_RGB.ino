#include "Adafruit_GFX.h"
#include "DHT.h"

DHT dht;
float humidity;
float temperature;

//**************************************************************//
// Following codes simply shifts out data for 16x32 LED matrix.
//****************************************************************
int LP = 10;            // Latch Pin
int ClkP = 11;           // Clock Pin
int R1P = 3;          // R1 Pin
int B1P = 4;          // B1 Pin
int G1P = 5;           // G1 Pin
int R2P = 6;           // R2 Pin
int B2P = 7;           // B2 Pin
int G2P = 8;           // G2 Pin
int AP = A0;            // A Pin
int BP = A1;            // B Pin
int CP = A2;            // C Pin
int DP = A3;            // D Pin
int OEP = 9;          // OE Pin
int row = 0;

uint8_t actScan = 0;
uint64_t row1[16];
uint64_t temp;
uint8_t mem[16][32];

class RGBmatrix : public Adafruit_GFX {
  public:

    // Constructor for 16x32 panel:
    RGBmatrix(uint8_t a, uint8_t b, uint8_t c,
              uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);
    void
    drawPixel(int16_t x, int16_t y, uint16_t c),
              fillScreen(uint16_t c);
    uint16_t
    Color333(uint8_t r, uint8_t g, uint8_t b);
};

// Constructor for 16x32 panel:
RGBmatrix::RGBmatrix(
  uint8_t a, uint8_t b, uint8_t c,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf) :
  Adafruit_GFX(32, 16) {
}

// Promote 3/3/3 RGB to Adafruit_GFX 5/6/5
uint16_t RGBmatrix::Color333(uint8_t r, uint8_t g, uint8_t b) {
  // RRRrrGGGgggBBBbb
  return ((r & 0x7) << 13) | ((r & 0x6) << 10) |
         ((g & 0x7) <<  8) | ((g & 0x7) <<  5) |
         ((b & 0x7) <<  2) | ((b & 0x6) >>  1);
}

void RGBmatrix::drawPixel(int16_t x, int16_t y, uint16_t c) {
  uint8_t r, g, b, bit = 1, limit, *ptr;
  uint8_t color = 0;

  if ((x < 0) || (x >= 32) || (y < 0) || (y >= 16)) return;

  // Adafruit_GFX uses 16-bit color in 5/6/5 format, while matrix needs
  // 4/4/4.  Pluck out relevant bits while separating into R,G,B:
  r =  c >> 12;        // RRRRrggggggbbbbb
  g = (c >>  7) & 0xF; // rrrrrGGGGggbbbbb
  b = (c >>  1) & 0xF; // rrrrrggggggBBBBb

  if (y < 8) {
    color &= ~B00000111;            // Mask out R,G,B in one op
    if (r & bit) color |= B00000001; // Plane N R: bit 2
    if (b & bit) color |= B00000010; // Plane N G: bit 3
    if (g & bit) color |= B00000100; // Plane N B: bit 4
  } else {
    color &= ~B00111000;           // Mask out R,G,B in one op
    if (r & bit) color |= B00001000; // Plane N R: bit 5
    if (b & bit) color |= B00010000; // Plane N G: bit 6
    if (g & bit) color |= B00100000; // Plane N B: bit 7
  }
  mem[y][x] = color;
}

void RGBmatrix::fillScreen(uint16_t c) {
  if ((c == 0x0000) || (c == 0xffff)) {
    // For black or white, all bits in frame buffer will be identically
    // set or unset (regardless of weird bit packing), so it's OK to just
    // quickly memset the whole thing:
    memset(mem, c, 512);
  } else {
    // Otherwise, need to handle it the long way:
    Adafruit_GFX::fillScreen(c);
  }
}

#define DATAPORT PORTD
#define CLK 11  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT 10
#define OE  9
#define A   A0
#define B   A1
#define C   A2
#define D   A3
RGBmatrix matrix(A, B, C, CLK, LAT, OE, false);

int timer1_counter;

void setup() {

  dht.setup(2, dht.AM2302); // data pin 2
  //set pins to output so you can control the shift register
  pinMode(LP, OUTPUT);
  pinMode(ClkP, OUTPUT);
  pinMode(R1P, OUTPUT);
  pinMode(B1P, OUTPUT);
  pinMode(G1P, OUTPUT);
  pinMode(R2P, OUTPUT);
  pinMode(B2P, OUTPUT);
  pinMode(G2P, OUTPUT);
  pinMode(AP, OUTPUT);
  pinMode(BP, OUTPUT);
  pinMode(CP, OUTPUT);
  pinMode(DP, OUTPUT);
  pinMode(OEP, OUTPUT);

  digitalWrite(AP, LOW);
  digitalWrite(BP, LOW);
  digitalWrite(CP, LOW);
  digitalWrite(DP, LOW);
  digitalWrite(OEP, HIGH);
  digitalWrite(LP, LOW);

  // Set up Timer1 for interrupt:
  TCCR1A  = _BV(WGM11); // Mode 14 (fast PWM), OC1A off
  TCCR1B  = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Mode 14, no prescale
  ICR1    = 100;
  TIMSK1 |= _BV(TOIE1); // Enable Timer1 interrupt
  sei();                // Enable global interrupts

  // fill the screen with 'black'
  matrix.fillScreen(matrix.Color333(0, 0, 0));
}

// -------------------- Interrupt handler stuff --------------------
//
uint16_t thc = 0;
ISR(TIMER1_OVF_vect, ISR_BLOCK) { // ISR_BLOCK important -- see notes later
  scan();   // Call refresh func for active display
  TIFR1 |= TOV1;                  // Clear Timer1 interrupt flag
}

void loop() {
  uint8_t h, m, s;

  noInterrupts();
  humidity = dht.getHumidity();
  temperature = dht.getTemperature();
  interrupts();
  String st = String(temperature, 1);
  String sh = String(humidity, 1);

  matrix.fillScreen(matrix.Color333(0, 0, 0));

  matrix.setCursor(0, 0);   // start at top left, with one pixel of spacing
  matrix.setTextSize(1);    // size 1 == 8 pixels high

  // print each letter with a rainbow color
  matrix.setTextColor(matrix.Color333(0, 7, 7));
  matrix.print(st + 'C');

  matrix.setCursor(0, 8);   // start at top left, with one pixel of spacing
  matrix.setTextSize(1);    // size 1 == 8 pixels high

  // print each letter with a rainbow color
  matrix.setTextColor(matrix.Color333(7, 0, 7));
  matrix.print(sh + '%');

  delay(1000);
}

void scan()
{
  uint8_t i, b = 1;
  ICR1      = 4000; // Set interval for next interrupt
  TCNT1     = 0;        // Restart interrupt timer
  digitalWrite(OEP, HIGH);
  digitalWrite(LP, LOW);

  digitalWrite(AP, !!(row & B00000001));
  digitalWrite(BP, !!(row & B00000010));

  for (i = 0; i < 16; i++) {
    digitalWrite(R1P, !!(mem[row + 4][i] & 0x1));
    digitalWrite(B1P, !!(mem[row + 4][i] & 0x2));
    digitalWrite(G1P, !!(mem[row + 4][i] & 0x4));
    digitalWrite(R2P, !!(mem[row + 12][i] & 8));
    digitalWrite(B2P, !!(mem[row + 12][i] & 16));
    digitalWrite(G2P, !!(mem[row + 12][i] & 32));
    digitalWrite(ClkP, HIGH);
    digitalWrite(ClkP, LOW);
  }
  for (i = 0; i < 16; i++) {
    digitalWrite(R1P, !!(mem[row][i] & 1));
    digitalWrite(B1P, !!(mem[row][i] & 2));
    digitalWrite(G1P, !!(mem[row][i] & 4));
    digitalWrite(R2P, !!(mem[row + 8][i] & 8));
    digitalWrite(B2P, !!(mem[row + 8][i] & 16));
    digitalWrite(G2P, !!(mem[row + 8][i] & 32));
    digitalWrite(ClkP, HIGH);
    digitalWrite(ClkP, LOW);
  }
  for (i = 0; i < 16; i++) {
    digitalWrite(R1P, !!(mem[row + 4][i + 16] & 1));
    digitalWrite(B1P, !!(mem[row + 4][i + 16] & 2));
    digitalWrite(G1P, !!(mem[row + 4][i + 16] & 4));
    digitalWrite(R2P, !!(mem[row + 12][i + 16] & 8));
    digitalWrite(B2P, !!(mem[row + 12][i + 16] & 16));
    digitalWrite(G2P, !!(mem[row + 12][i + 16] & 32));
    digitalWrite(ClkP, HIGH);
    digitalWrite(ClkP, LOW);
  }
  for (i = 0; i < 16; i++) {
    digitalWrite(R1P, !!(mem[row][i + 16] & 1));
    digitalWrite(B1P, !!(mem[row][i + 16] & 2));
    digitalWrite(G1P, !!(mem[row][i + 16] & 4));
    digitalWrite(R2P, !!(mem[row + 8][i + 16] & 8));
    digitalWrite(B2P, !!(mem[row + 8][i + 16] & 16));
    digitalWrite(G2P, !!(mem[row + 8][i + 16] & 32));
    digitalWrite(ClkP, HIGH);
    digitalWrite(ClkP, LOW);
  }

  digitalWrite(LP, HIGH);
  digitalWrite(LP, LOW);
  digitalWrite(OEP, LOW);

  if (++row > 3) row = 0;
}
