#include <FastLED.h>

#include <Wire.h>
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define DEBOUNCE_TIME         150
#define VOLT_AVG_COUNT        10
#define NUM_FIRE_BLOBS        10
#define FIRE_BLOB_INTERVAL    100
#define FIRE_BLOB_MAX_RADIUS  4
#define FIRE_FADE_HEIGHT      10.0f
#define FIRE_BASE_HEIGHT      5.0f
#define MAX_BURST_LIFE        2000

//#define FASTLED_FORCE_SOFTWARE_SPI

#define SHIFT_BITS  6
#define FIXED_MULT  (float)((pow(2, (SHIFT_BITS))))   // unsigned part of signed range
#define RANGE_ABS   (float)((FIXED_MULT) - 1)   // unsigned part of signed range
#define TO_FIXED(FV) (int16_t)((FV) * FIXED_MULT)
#define TO_FLOAT(IV) (float)((IV) / FIXED_MULT)

#define TCAADDR 0x70

void tcaselect(uint8_t i);

class sensor {
  public:
    Adafruit_LIS3DH *_s = nullptr;
    uint8_t _addr = 0x18;
    uint8_t _i2c_index = 0;
    int16_t _x = 0;
    int16_t _y = 0;
    int16_t _r = TO_FIXED(1.0f);
    int16_t _g = TO_FIXED(1.0f);
    int16_t _b = TO_FIXED(1.0f);
    int16_t _speed = TO_FIXED(25.0f);
    sensor(Adafruit_LIS3DH *s, uint8_t addr, uint8_t i2c_index, int16_t x, int16_t y, int16_t r, int16_t g, int16_t b, int16_t speed) {
      _s = s;
      _addr = addr;
      _i2c_index = i2c_index;
      _x = x;
      _y = y;
      _r = r;
      _g = g;
      _b = b;
      _speed = speed;
    };
    boolean begin() {
      if (!_s->begin(_addr)) {
        return false;
      }
      _s->setRange(LIS3DH_RANGE_8_G);   // 2, 4, 8 or 16 G!
      
      return true;
    }
    void select() {
      tcaselect(_i2c_index);
    }
  protected:
};

Adafruit_LIS3DH lis1 = Adafruit_LIS3DH();
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH();

sensor accs[2 ] = {
  {&lis1, 0x18, 0, TO_FIXED(0.0f), TO_FIXED(25.0f), TO_FIXED(1.0f), TO_FIXED(1.0f), TO_FIXED(1.0f), TO_FIXED(60.0f)},
  {&lis2, 0x18, 1, TO_FIXED(0.0f), TO_FIXED(25.0f), TO_FIXED(1.0f), TO_FIXED(1.0f), TO_FIXED(1.0f), TO_FIXED(60.0f)}  // {&lis3, 0x18, 0, 12.0f, 1.0f, 0.0f, 0.0f, 35.0f}
};

#define NUM_LEDS_PER_STRIP 90
#define NUM_STRIPS 4

CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

typedef struct {
  int16_t r;
  int16_t g;
  int16_t b;
  int16_t x;
  int16_t y;
} float_rgb_loc;

float_rgb_loc float_leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

class led_segment {
  public:
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
    int16_t strip;
    int16_t start_led;
    int16_t num_leds;

  uint16_t index(uint16_t led_num) {
    return start_led + led_num;
  }
};

class burst {
  public:
    int16_t x;
    int16_t y;
    int16_t speed;
    int16_t size;
    unsigned long start_time;
};
/*
led_segment segments[6] = {{ TO_FIXED(0.0),   TO_FIXED(0),     TO_FIXED(0),     TO_FIXED(13),    3, 0, 60}, // torso
                           { TO_FIXED(-0.5),  TO_FIXED(12.5),  TO_FIXED(-5.5),  TO_FIXED(10),    1, 0, 60}, // l arm
                           { TO_FIXED(0.5),   TO_FIXED(12.5),  TO_FIXED(5.5),   TO_FIXED(10),    2, 0, 60}, // r arm
                           { TO_FIXED(-0.25), TO_FIXED(9.5),   TO_FIXED(-2.0),  TO_FIXED(1),     2, 0, 60}, // l leg
                           { TO_FIXED(0.25),  TO_FIXED(9.5),   TO_FIXED(2.0),   TO_FIXED(1),     1, 0, 12}, // r leg
                           { TO_FIXED(-0.75), TO_FIXED(14.5),  TO_FIXED(0.75),  TO_FIXED(14.5),  1, 0, 3}, // head
                           };
*/
led_segment segments[20] = {{ TO_FIXED(2.1),  TO_FIXED(0),     TO_FIXED(2.43),  TO_FIXED(8.64),   3, 0, 14  }, // torso middle
                           { TO_FIXED(1.87),  TO_FIXED(9.07),  TO_FIXED(0),     TO_FIXED(10.64),  3, 15,5   }, // torso middle
                           { TO_FIXED(0),     TO_FIXED(11.22), TO_FIXED(0),     TO_FIXED(16.1),   3, 19, 10 }, // torso middle
                           { TO_FIXED(0.625), TO_FIXED(16.35), TO_FIXED(2.5),   TO_FIXED(17.4),   3, 29, 5  }, // torso middle
                           { TO_FIXED(2.6),   TO_FIXED(18.2),  TO_FIXED(5.0),   TO_FIXED(25),     3, 34, 9  }, // torso middle
                           { TO_FIXED(5.25),  TO_FIXED(25.25), TO_FIXED(9.5),   TO_FIXED(22),     3, 43, 10 }, // left shoulder
                           { TO_FIXED(9.5),   TO_FIXED(21.25), TO_FIXED(9.5),   TO_FIXED(19),     3, 53, 5  }, // left arm
                           { TO_FIXED(10),    TO_FIXED(20.8),  TO_FIXED(10.5),  TO_FIXED(17.5),   3, 58, 6  }, // left arm
                           { TO_FIXED(10.75), TO_FIXED(20.8),  TO_FIXED(13),    TO_FIXED(17.5),   3, 64, 5  }, // left arm

                           { TO_FIXED(-2.1),  TO_FIXED(0),     TO_FIXED(-2.43), TO_FIXED(8.64),   2, 0, 14  }, // torso right
                           { TO_FIXED(-1.87), TO_FIXED(9.07),  TO_FIXED(0),     TO_FIXED(10.64),  2, 15,5   }, // torso right
                           { TO_FIXED(0),     TO_FIXED(11.22), TO_FIXED(0),     TO_FIXED(16.1),   2, 19, 7 }, // torso right
                           { TO_FIXED(-0.625),TO_FIXED(16.35), TO_FIXED(-2.5),  TO_FIXED(17.4),   2, 26, 5  }, // torso right
                           { TO_FIXED(-2.6),  TO_FIXED(18.2),  TO_FIXED(-5.0),  TO_FIXED(25),     2, 31, 9  }, // torso right
                           { TO_FIXED(-5.25), TO_FIXED(25.25), TO_FIXED(-9.5),  TO_FIXED(22),     2, 40, 10 }, // right shoulder
                           { TO_FIXED(-9.5),  TO_FIXED(21.25), TO_FIXED(-9.5),  TO_FIXED(19),     2, 50, 5  }, // right arm
                           { TO_FIXED(-10),   TO_FIXED(20.8),  TO_FIXED(-10.5), TO_FIXED(17.5),   2, 55, 6  }, // right arm
                           { TO_FIXED(-10.75),TO_FIXED(20.8),  TO_FIXED(-13),   TO_FIXED(17.5),   2, 61, 5  }, // right arm
                           
                           { TO_FIXED(8),  TO_FIXED(0),  TO_FIXED(16),  TO_FIXED(-36),     0, 0, 89  }, // leg left
                           { TO_FIXED(-8),  TO_FIXED(0),  TO_FIXED(-16),  TO_FIXED(-36),     1, 0, 89  }, // leg right
                           
                           };
/*                           
                           { TO_FIXED(-0.5),  TO_FIXED(12.5),  TO_FIXED(-5.5),  TO_FIXED(10),    1, 0, 60}, // l arm
                           { TO_FIXED(0.5),   TO_FIXED(12.5),  TO_FIXED(5.5),   TO_FIXED(10),    1, 0, 60}, // r arm
                           { TO_FIXED(-0.25), TO_FIXED(9.5),   TO_FIXED(-2.0),  TO_FIXED(1),     1, 0, 60}, // l leg
                           { TO_FIXED(0.25),  TO_FIXED(9.5),   TO_FIXED(2.0),   TO_FIXED(1),     1, 0, 12}, // r leg
                           { TO_FIXED(-0.75), TO_FIXED(14.5),  TO_FIXED(0.75),  TO_FIXED(14.5),  1, 0, 3}, // head
*/
burst bursts[2] = { { TO_FIXED(18), TO_FIXED(16), TO_FIXED(50.0f), 0, 0 },
                    { TO_FIXED(-18),  TO_FIXED(16), TO_FIXED(50.0f), 0, 0 },
                  };

burst fire_blobs[NUM_FIRE_BLOBS];

long burst_interval = 1000;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("in setup");

  Wire.begin();

  analogReadRes(12);

  FastLED.addLeds<APA102,7,13,BGR, DATA_RATE_MHZ(16)>(leds[0],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<APA102,7,14,BGR, DATA_RATE_MHZ(16)>(leds[1],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<APA102,11,13,BGR, DATA_RATE_MHZ(16)>(leds[2],NUM_LEDS_PER_STRIP);
  FastLED.addLeds<APA102,11,14,BGR, DATA_RATE_MHZ(16)>(leds[3],NUM_LEDS_PER_STRIP);

  LEDS.setBrightness(128);

  for (int seg = 0; seg < sizeof(segments) / sizeof(segments[0]); seg++) {
    for (int i = 0; i <= segments[seg].num_leds; i++) {
      float_leds[segments[seg].strip][segments[seg].index(i)].x = lerp(segments[seg].x1, segments[seg].x2, segments[seg].num_leds, i);
      float_leds[segments[seg].strip][segments[seg].index(i)].y = lerp(segments[seg].y1, segments[seg].y2, segments[seg].num_leds, i);
      float_leds[segments[seg].strip][segments[seg].index(i)].r = 0;
      float_leds[segments[seg].strip][segments[seg].index(i)].g = 0;
      float_leds[segments[seg].strip][segments[seg].index(i)].b = 0;
    }
  }

  randomSeed(analogRead(0));
  accs[0].select();
  if (! accs[0].begin()) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start left");
  }
  Serial.println("Left side LIS3DH found!");
  accs[1].select();
  if (! accs[1].begin()) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start right");
  }
  Serial.println("Right side LIS3DH found!");  
/*
  if (! accs[2].begin()) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start head");
  }
  Serial.println("Head LIS3DH found!");  
  */
}

void loop() {
  static long iterations = 0;
  static float voltages[VOLT_AVG_COUNT];
  static uint8_t volt_index = 0;
  static unsigned long last_blob = 0;
  static uint8_t blob_index = 0;
  int16_t adc = 0;

  unsigned long seconds_fixed = 0;
  
  for (int i = 0; i < NUM_STRIPS; i++) {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      float_leds[i][j].r = TO_FIXED(0.0f);
      float_leds[i][j].g = TO_FIXED(0.0f);
      float_leds[i][j].b = TO_FIXED(0.1f);
    }
  }

  sensors_event_t event; 
  accs[0].select();
  accs[0]._s->getEvent(&event);  
  if ((event.acceleration.z > 15.0f || event.acceleration.z < -15.0f) && millis() > bursts[0].start_time + DEBOUNCE_TIME) {
    bursts[0].start_time = millis();
  }

  accs[1].select();

  adc = accs[1]._s->readADC(1);
  LEDS.setBrightness(min(255, map(adc, -32768, 32768, 10, 512)));

  accs[1]._s->getEvent(&event);  
  if ((event.acceleration.z > 15.0f || event.acceleration.z < -15.0f) && millis() > bursts[1].start_time + DEBOUNCE_TIME) {
    bursts[1].start_time = millis();
  }
  
/*
  accs[2]._s->getEvent(&event);  
  if ((event.acceleration.z > 25.0f || event.acceleration.z < -25.0f) && millis() > bursts[1].start_time + DEBOUNCE_TIME) {
    bursts[2].start_time = millis();
  }
  */
  
  for (int seg = 0; seg < sizeof(segments) / sizeof(segments[0]); seg++) {
    for (int i = 0; i <= segments[seg].num_leds; i++) {
      float_leds[segments[seg].strip][segments[seg].index(i)].r = TO_FIXED(0.0f);
      float_leds[segments[seg].strip][segments[seg].index(i)].g = TO_FIXED(0.0f);
      float_leds[segments[seg].strip][segments[seg].index(i)].b = TO_FIXED(0.1f);
      
      for (int b = 0; b < sizeof(bursts) / sizeof(bursts[0]); b++) {  
        if (bursts[b].start_time < millis() + MAX_BURST_LIFE) {
          int16_t dx = float_leds[segments[seg].strip][segments[seg].index(i)].x - bursts[b].x;
          int16_t dy = float_leds[segments[seg].strip][segments[seg].index(i)].y - bursts[b].y;
          //float dist = sqrt3(TO_FLOAT(dx)*TO_FLOAT(dx) + TO_FLOAT(dy)*TO_FLOAT(dy)) - (float)(millis() - bursts[b].start_time) / 1000.0f * TO_FLOAT(bursts[b].speed);
          int16_t dist = TO_FIXED(sqrt3(TO_FLOAT((dx*dx) / FIXED_MULT + (dy*dy) / FIXED_MULT)) - (float)(millis() - bursts[b].start_time) / 1000.0f * TO_FLOAT(bursts[b].speed));
          
          int16_t brightness = TO_FIXED(0.0f);
          if (abs(dist) < TO_FIXED(3.0f)) {
            brightness = TO_FIXED(0.15f);
          }
          if (abs(dist) < TO_FIXED(1.0)) {
            brightness = TO_FIXED(0.25f);
          }
          if (abs(dist) < TO_FIXED(0.5f)) {
            brightness = TO_FIXED(1.0f);
          }
          //float_leds[segments[seg].index(i)].r = 1.0f - (1.0f - float_leds[segments[seg].index(i)].r) * (1.0f - brightness);
          float_leds[segments[seg].strip][segments[seg].index(i)].g = TO_FIXED(1.0f) - ((TO_FIXED(1.0f) - float_leds[segments[seg].strip][segments[seg].index(i)].g) * (TO_FIXED(1.0f) - brightness)) / FIXED_MULT;
          float_leds[segments[seg].strip][segments[seg].index(i)].b = TO_FIXED(1.0f) - ((TO_FIXED(1.0f) - float_leds[segments[seg].strip][segments[seg].index(i)].b) * (TO_FIXED(1.0f) - brightness)) / FIXED_MULT;
        }
      }

      /*
      for (int b = 0; b < sizeof(fire_blobs) / sizeof(fire_blobs[0]); b++) {  
        int16_t blob_age = (int16_t)((float)(millis() - fire_blobs[b].start_time) / 1000.0f);
        float bloby = fire_blobs[b].y + blob_age * fire_blobs[b].speed;
        if (fire_blobs[b].start_time > 0) {
          float dx = float_leds[segments[seg].strip][segments[seg].index(i)].x - fire_blobs[b].x;
          float dy = float_leds[segments[seg].strip][segments[seg].index(i)].y - bloby;
          float dist = sqrt3(dx*dx+dy*dy);

          float brightness = 0;

          if (dist <= fire_blobs[b].size) {
            brightness = (1.0f - (dist / fire_blobs[b].size));
            brightness *= (1.0f - ((1.0f + min(FIRE_FADE_HEIGHT, bloby)) / FIRE_FADE_HEIGHT));
          }
          if (float_leds[segments[seg].strip][segments[seg].index(i)].y <= FIRE_BASE_HEIGHT) {
            float fadeadd = 1.0f - ((float)float_leds[segments[seg].strip][segments[seg].index(i)].y) / (FIRE_BASE_HEIGHT);
            brightness = 1.0f - ((1.0f - fadeadd / 15.0f) * (1.0f - brightness));
            if (brightness > 1.0f) {
              brightness = 1.0f;
            }
          }
          
          float_leds[segments[seg].strip][segments[seg].index(i)].r = 1.0f - (1.0f - float_leds[segments[seg].strip][segments[seg].index(i)].r) * (1.0f - brightness);
        }
      }
      */
    }
  }

  for (int i = 0; i < NUM_STRIPS ; i++) {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[i][j].setRGB( (float_leds[i][j].r * 255) / FIXED_MULT, (float_leds[i][j].g * 255) / FIXED_MULT, (float_leds[i][j].b * 255) / FIXED_MULT );
//      leds[i][j].setRGB(255,255,255);
    }
  }

  LEDS.show();
  LEDS.delay(1);

  float div = 0.210f;
  voltages[volt_index++] = ((float)analogRead(A11)/4096.0f * 3.3f)/div;
  if (volt_index >= VOLT_AVG_COUNT) {
    volt_index = 0;
  }

  float volt_total = 0.0f;
  float volt_average = 0.0f;
  for (int i = 0; i < VOLT_AVG_COUNT; i++) {
    volt_total += voltages[i];
  }
  volt_average = volt_total / VOLT_AVG_COUNT;

//  Serial.print("Battery voltage average: ");
//  Serial.println(volt_average);

  float seconds = (float)millis() / 1000.0f;
//  Serial.print((float)iterations / seconds); 
//  Serial.println(" reads per second"); 
  iterations++;
/*
  if (millis() > last_blob + FIRE_BLOB_INTERVAL) {
    last_blob = millis();
    fire_blobs[blob_index].start_time = millis();

    fire_blobs[blob_index].x = random(-6, 6);
    fire_blobs[blob_index].y = 0;
    fire_blobs[blob_index].size = random(0.5, FIRE_BLOB_MAX_RADIUS);
    fire_blobs[blob_index].speed = 20.0f;
    
    blob_index++;
    if (blob_index >= NUM_FIRE_BLOBS) {
      blob_index = 0;  
    }

    
  }
*/
}

float sqrt3(const float x)  
{
  union
  {
    int i;
    float x;
  } u;

  u.x = x;
  u.i = (1<<29) + (u.i >> 1) - (1<<22); 
  return u.x;
} 

/*
uint16_t int_sqrt32(uint32_t x)
{
    uint16_t res=0;
    uint16_t add= 0x8000;   
    int i;
    for(i=0;i<16;i++)
    {
        uint16_t temp=res | add;
        uint32_t g2=temp*temp;      
        if (x>=g2)
        {
            res=temp;           
        }
        add>>=1;
    }
    return res;
}
 */
int16_t lerp(int16_t val1, int16_t val2, int16_t num_leds, int16_t index) {
  return val1 + (val2 - val1) * index / num_leds;
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
