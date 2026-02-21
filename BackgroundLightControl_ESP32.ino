// **  Adressen (vastgelegd in BASEADDRESS; gelijk voor DCC en LocoNet):
// **  BASEADDRESS    R      Preset 1 (dag)
// **  BASEADDRESS    G      Preset 2 (avond)
// **  BASEADDRESS+1  R      Preset 3 (nacht)
// **  BASEADDRESS+1  G      Preset 4 (ochtend)
// **  BASEADDRESS+2  R / G  Vol wit uit / aan
// **  BASEADDRESS+3  R / G  Disco uit / aan

#include "enums.h"
#include <NmraDcc.h>
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <arduinoFFT.h>
#include <math.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h> // Standard tablatronix version
#include <Preferences.h>
#include <ESPmDNS.h>
#include <WebServer.h>

#define FASTLED_ALLOW_INTERRUPTS 0
//#define FASTLED_INTERRUPT_RETRY_COUNT 0
#include <FastLED.h>
 
#define LED_TYPE TM1809 //TM1809   (TM1803, WS2815)

/* ---------------- CONFIG ---------------- */
const uint16_t CAN_ID_MODEL_TIME = 0x0033; // CS3 fast clock CAN ID
uint8_t packetBuffer[64];


/* ---------------- GLOBALS ---------------- */
WiFiClient client;
WiFiManager wifiManager;
Preferences preferences;

WebServer server(80);
static bool serverStarted = false;

volatile bool wifiReady = false;
volatile bool startPortal = false;
volatile bool inPortal = false;

uint16_t cs3Port = 15731;
char cs3IpStr[16] = "192.168.1.119";
char cs3PortStr[6] = "15731"; // Char buffer for WiFiManager
IPAddress cs3Ip;
bool modelTimeEnabled = false;   // default

// Clock State
int mH = 0, mM = 0;
int lastPrintedMinute = -1;
float timeScale = 1.0; 
unsigned long lastSyncMillis = 0;
unsigned long lastLocalTick = 0;
bool firstSyncDone = false;
bool scaleCalculated = false;

// DCC
NmraDcc  Dcc ;
DCC_MSG  Packet ;

// Define the Arduino input Pin number for the DCC Signal 
#define DCC_PIN     33

// DCC Address
#define BASEADDRESS 101        // <-- Pas aan om de adressen te veranderen

// LED strip pins
#define WARMWHITE_PIN 12
#define COLDWHITE_PIN 13

// PWM settings
const int pwmFreq = 4000;     // 1220 Hz
const int pwmResolution = 10;  // 8-bit resolution (0–255)
#define MAX_DUTY   (uint32_t)((1U << pwmResolution) - 1)
#define DUTY_SHIFT_BITS (8-pwmResolution >= 0 ? 8-pwmResolution : 0)

// =============================================================
// FASTLED CONFIG
// =============================================================
#define LEDDATA_PIN   23
#define NUM_LEDS  47
CRGB leds[NUM_LEDS];

// =============================================================
// AUDIO CONFIG (WM8782)
// =============================================================
#define I2S_SAMPLE_RATE 48000
#define I2S_BCLK_PIN    26
#define I2S_LRCLK_PIN   25
#define I2S_DATA_PIN    22

// =============================================================
// FFT (BASS-ONLY)
// =============================================================
#define FFT_SIZE        256
#define WINDOW_GAIN     0.35875f

// Bass band: ~40–160 Hz
#define BASS_MIN_HZ     40
#define BASS_MAX_HZ     160

// =============================================================
// BASS DETECTION TUNING
// =============================================================
#define BASS_ATTACK       0.75f
#define BASS_RELEASE      0.5f
#define BEAT_SENSITIVITY  1.35f
#define AUDIO_PRESENT_TH  0.010f

// =============================================================
// TIMING
// =============================================================
#define AMBIENT_ENTER_MS  15000
#define AUTO_GENRE_WARMUP 3000

#define READ_BUFFER_SIZE (FFT_SIZE * 8)

// Day/Night settings
#define NUM_LEDS 47 // 5 segments/meter
#define NUM_DIF_SEGMENTS 8
#define DAY_TO_EVENING_ANIM_LENGHT 15
#define EVENING_TO_NIGHT_ANIM_LENGHT 8
#define NIGHT_TO_MORNING_ANIM_LENGHT 10
#define MORNING_TO_DAY_ANIM_LENGHT 10
#define MAX_LIGHTNING 5

struct RgbWColor {
  public:
    uint8_t r;
    uint8_t g;
    uint8_t b;    
    uint8_t w;
};

const RgbWColor PRESET_DAY = { 76,93,79,214 };
const RgbWColor PRESET_EVENING = { 255,56,33,52 };
const RgbWColor PRESET_NIGHT = { 0,0,40,3 };
const RgbWColor PRESET_MORNING = { 243,62,4,20 };

RgbWColor dayToEvening[DAY_TO_EVENING_ANIM_LENGHT] {
  PRESET_DAY,
  { 89, 90, 76, 202 },
  { 102, 88, 72, 191 },
  { 114, 85, 69, 179 },
  { 127, 82, 66, 168 },
  { 140, 80, 63, 156 },
  { 153, 77, 59, 145 },
  { 166, 75, 56, 133 },
  { 178, 72, 53, 121 },
  { 191, 69, 49, 110 },
  { 204, 67, 46, 98 },
  { 217, 64, 43, 87 },
  { 229, 61, 40, 75 },
  { 242, 59, 36, 64 },
  //{  },
  PRESET_EVENING
};

RgbWColor eveningToNight[EVENING_TO_NIGHT_ANIM_LENGHT] {
  PRESET_EVENING,
  { 219, 48, 34, 45 },
  { 182, 40, 35, 38 },
  { 146, 32, 36, 31 },
  { 109, 24, 37, 24 },
  { 73, 16, 38, 17 },
  { 36, 8, 39, 10 },
  PRESET_NIGHT
};

RgbWColor nightToMorning[NIGHT_TO_MORNING_ANIM_LENGHT] {
  PRESET_NIGHT,
  { 27, 7, 36, 5 },
  { 54, 14, 32, 7 },
  { 81, 21, 28, 9 },
  { 108, 28, 24, 11 },
  { 135, 34, 20, 12 },
  { 162, 41, 16, 14 },
  { 189, 48, 12, 16 },
  { 216, 55, 8, 18 },
  PRESET_MORNING
};

RgbWColor morningToDay[MORNING_TO_DAY_ANIM_LENGHT] {
  PRESET_MORNING,
  { 224, 65, 12, 42 },
  { 206, 69, 21, 63 },
  { 187, 72, 29, 85 },
  { 169, 76, 37, 106 },
  { 150, 79, 46, 128 },
  { 132, 83, 54, 149 },
  { 113, 86, 62, 171 },
  { 95, 90, 71, 192 },  
  PRESET_DAY
};

const RgbWColor EMPTY_COLOR = {0,0,0,0};
const CRGB CRGB_Lightning = {255,255,255};

struct ledStepData {  
  public:
    uint8_t step;
};

ledStepData ledsData[NUM_LEDS];

struct ledLightningData {  
  public:  
    bool active = false;
    uint8_t led = 0;
    uint8_t flashCount = 0;
    uint8_t isOn = 0;
    uint16_t cooldown = 0;    
};

ledLightningData lightningData[MAX_LIGHTNING];

uint8_t segmentLengths[NUM_DIF_SEGMENTS] = { 6,3,2,1,1,1,2,6 };
//uint8_t segmentLengths[NUM_DIF_SEGMENTS] = { 2,1,1,1,1,1,2,6 }; // test

// =============================================================
// VISUAL TUNING
// =============================================================
const uint8_t HUE_JUMP = 18;

#define IDLE_TIMEOUT_MS     1800   // how long without bass before ambience starts
#define IDLE_SPAWN_INTERVAL 1200    // ms between idle flashers
#define IDLE_MAX_SPAWN      1      // max flashers per idle tick
float bassEnv = 0.0f;
float prevBassEnv = 0.0f;
unsigned long audioStartTime = 0;

// EMAs for genre selection
float bassEnergyEMA      = 0.0f;
float bassHitRateEMA     = 0.0f;
float transientRateEMA   = 0.0f;

// =============================================================
// INTERNALS
// =============================================================
static i2s_chan_handle_t rx_chan;
static uint8_t i2s_buffer[READ_BUFFER_SIZE] __attribute__((aligned(4)));

static double vReal[FFT_SIZE];
static double vImag[FFT_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, I2S_SAMPLE_RATE);

int bassBinStart, bassBinEnd;
bool bassDetected = false;
bool transientDetected = false;
uint8_t hueOffset = 0;

// =============================================================
// Flashing LED Engine
// =============================================================
#define MAX_ACTIVELEDFLASHERS 20
struct FlashLED {
  int index;               // LED index (0..NUM_LEDS-1)
  uint8_t state;           // 0=idle, 1=on, 2=fading
  unsigned long endTime;   // when to stop full brightness (state 1)
  unsigned long cooldownEnd;// when LED becomes eligible again (after state 2)
};
FlashLED activeFlashers[MAX_ACTIVELEDFLASHERS]; // up to 20 flashers

// timing/config
const uint16_t ON_TIME_MIN = 40;    // ms LED stays on
const uint16_t ON_TIME_MAX = 160;   // ms
const uint8_t  FADE_BY     = 26;    // fadeToBlackBy amount per loop in fade state
const uint16_t COOLDOWN_MIN = 90;   // ms before same LED can be reused
const uint16_t COOLDOWN_MAX = 150;  // ms

AudioGenre::GenreMode currentGenre = AudioGenre::GenreMode::GENRE_EDM;

struct GenreConfig {
  float bassTriggerMul;
  float bassSpawnGain;
  float transientGain;

  uint8_t baseHue;
  uint8_t fadeBy;

  unsigned long idleTimeout;
  unsigned long idleInterval;
  uint8_t maxSpawn;
};

// tuned parameters for each genre
GenreConfig genres[AudioGenre::GenreMode::GENRE_COUNT] = {
  {1.0f, 1.4f, 0.15f, 160, 14, 3000, 900, 6}, // EDM
  {0.85f,0.8f,1.0f,0,32,2000,600, 4},         // ROCK
  {1.6f,0.3f,0.0f,96,8,1200,400, 1},          // AMBIENT
  {0.75f,1.6f,0.05f,200,10,3500,1200, 3},     // HIPHOP
  {0.9f,1.0f,0.05f,100,18,2500,700, 5},       // TECHNO
  {1.1f,0.7f,1.3f,32,30,1500,400, 6},         // DNB
  {0.3f,1.2f,0.2f,180,6,800,300, 3},          // CINEMATIC
  {0.95f,1.1f,0.25f,140,20,2800,800, 5}       // HOUSE
};

// =============================================================
// AUDIO STATE
// =============================================================
float bassRMS = 0;
unsigned long lastIdleSpawnTime = 0;
unsigned long lastAudioActivityTime = 0;
uint32_t lastBassHitTime = 0;
uint32_t lastGenreChange = 0;

// =============================================================
// CONSTANTS
// =============================================================
const float BASS_LPF = 0.12f;
const float VAR_LPF  = 0.10f;

const float BASS_TRIGGER = 0.030f;
const float TRANSIENT_THRESHOLD = 0.02f;

const uint32_t GENRE_HOLD_TIME = 8000; // ms before changing

// =============================================================
// GENRE PROFILES (for auto-selection)
// =============================================================
struct GenreProfile {
  float bassEnergy;
  float bassRate;
  float transientRate;
};

GenreProfile genreProfiles[AudioGenre::GenreMode::GENRE_COUNT] = {
  {0.05f, 0.8f, 0.4f},  // EDM
  {0.04f, 0.5f, 0.3f},  // ROCK
  {0.01f, 0.0f, 0.0f},   // AMBIENT
  {0.05f, 0.3f, 0.5f},  // HIPHOP
  {0.04f, 0.6f, 0.2f},  // TECHNO
  {0.06, 0.7f, 1.0f},  // DNB
  {0.02f, 0.1f, 0.2f},  // CINEMATIC
  {0.05, 0.6f, 0.4f}   // HOUSE
};

// End - Disco mode parameters

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;

// constants won't change:
const long interval = 25;           // interval at which to blink (milliseconds)
uint8_t doStuff = 0;
long randNumber;

const uint8_t TRANSITION_STEP_DELAY = 4;
bool isInTransition = false;
bool maxStepsReached = false;

enum LightMode::Mode lightMode = LightMode::Mode::DAYNIGHT;
enum DaytimeOverlay::Overlay daytimeOverlay = DaytimeOverlay::Overlay::NONE;

enum Daytimes::Daytime currentDaytime;
enum Daytimes::Daytime newDaytime;
uint8_t maxSteps = 0;
uint8_t fieldsCounter = 0;

float dim = 1.0; // 1= full brightness, 0 = no brightness;
float dimWW = 0.35; // 1= full brightness, 0 = no brightness;
float dimCW = 0.25; // 1= full brightness, 0 = no brightness;
float dimDynamic = 1.0; // weather

uint8_t ddcIgnore = 0;

uint8_t currentHours   = 0;
uint8_t currentMinutes   = 0;
bool timeReceived = false;


/* ---------------- WiFi Event Handler ---------------- */
void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("\n[WiFi] Connected! IP: ");
            Serial.println(WiFi.localIP());
            if (MDNS.begin("cs-light-bridge")) {
               // Serial.println("[mDNS] http://cs-light-bridge.local");
            }

            registerUrls();
            
            wifiReady = true;            
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            wifiReady = false;
            break;
        default: break;
    }
}

/* ---------------- WiFi Task (Core 0) ---------------- */
void wifiTask(void *pvParameters) {
  WiFi.onEvent(WiFiEvent);
  
  WiFiManagerParameter custom_cs3_ip("cs3_ip", "CS3 IP Address", cs3IpStr, 16);
  WiFiManagerParameter custom_cs3_port("cs3_port", "TCP Port (15731)", cs3PortStr, 6);
  
  WiFiManagerParameter custom_model_time(
     "modelTime",
    "Enable Model Time",
    modelTimeEnabled ? "T" : "",
    2,
    "type=\"checkbox\""
    );
  
  wifiManager.addParameter(&custom_cs3_ip);
  wifiManager.addParameter(&custom_cs3_port);
  wifiManager.addParameter(&custom_model_time);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(180); 

  wifiManager.setSaveConfigCallback([&]() {
      preferences.begin("CS3cfg", false);
      preferences.putString("cs3ip", custom_cs3_ip.getValue());
      preferences.putUShort("port", atoi(custom_cs3_port.getValue()));

    // Checkbox logic
    bool enabled = false;
    if (wifiManager.server->hasArg("modelTime")) {
        enabled = wifiManager.server->arg("modelTime") == "T";
    }
      
      preferences.end();
      //Serial.println("Settings Saved. Restarting...");
      delay(1000);
      ESP.restart();
  });

  if(wifiManager.autoConnect("CS-Light-Setup")) {
      wifiManager.startWebPortal(); 
  }

  while(true) {
      wifiManager.process();

      if (startPortal) {
        startPortal = false;
        //Serial.println("[CFG] Starting captive portal");
    
        // Stop your server entirely to free port 80
        server.stop();
    
        // Optional: short pause to ensure WiFi stack is ready
        delay(200);
    
        // Ensure WiFiManager blocks and fully starts portal
        wifiManager.setConfigPortalBlocking(true);
        wifiManager.startConfigPortal("CS-Light-Setup");
    
        // Portal finished: reboot
        //Serial.println("[CFG] Portal finished, rebooting");
        delay(1000);
        ESP.restart();
    }
    
      vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void wifiPacketTask(void *pvParameters) {
  while(true) {
      
      if (wifiReady ) {
        if (!inPortal) {
          server.handleClient();
        }
  
        if (modelTimeEnabled) {
        
          // 1. Maintain TCP Connection
          if (!client.connected()) {
            static unsigned long lastConnectTry = 0;
            if (millis() - lastConnectTry > 10000) { // Try every 10 seconds
              lastConnectTry = millis();
              if (client.connect(cs3Ip, cs3Port)) {
                //Serial.println("[CS3] Connection established.");
              }
            }
          }
      
          // 2. Read Packets
          if (client.available() >= 13) {
            uint8_t p[13];
            client.read(p, 13);
      
            // Fast Clock Marker (0x20)
            if (p[9] == 0x20) {
              int nH = p[10];
              int nM = p[11];
      
              if (firstSyncDone) {
                float realElapsed = (millis() - lastSyncMillis) / 1000.0;
                int modelMinsPast = (nM - mM);
                if (nH != mH) modelMinsPast += (nH - mH) * 60;
                if (modelMinsPast < 0) modelMinsPast += 1440;
      
                if (realElapsed > 5.0 && modelMinsPast > 0) {
                  timeScale = (float)(modelMinsPast * 60) / realElapsed;
                  if (timeScale > 12 || timeScale < 0) { // possible when adjusting the time
                    timeScale = 0;
                    scaleCalculated = false;
                  } else {
                    scaleCalculated = true;                
                  }
                }
              }
              mH = nH; mM = nM;
              lastSyncMillis = millis();
              firstSyncDone = true;
              setDaytimeBasedOnTime(mH, mM);
              //Serial.printf("\n--- SYNC FROM CS3: %02d:%02d (Detected Scale: %0.1fx) ---\n", mH, mM, timeScale);
            }
          }   
        }
      }
      vTaskDelay(pdMS_TO_TICKS(modelTimeEnabled ? 50 : 5000)); 
  }
}

void setup() {
  pinMode(WARMWHITE_PIN, OUTPUT);
  pinMode(COLDWHITE_PIN, OUTPUT);

  //Serial.begin(115200);
  //uint8_t maxWaitLoops = 255;
  //while(!Serial && maxWaitLoops--)
  //  delay(20);
    
    //Serial.println("Daylight");

    // Configure PWM channels
  ledcAttach(WARMWHITE_PIN, pwmFreq, pwmResolution);
  ledcAttach(COLDWHITE_PIN, pwmFreq, pwmResolution);
  ledcWrite(WARMWHITE_PIN, 0);
  ledcWrite(COLDWHITE_PIN, 0);
  
    // sanity check delay - allows reprogramming if accidently blowing power w/leds
    delay(2000);
    FastLED.addLeds<LED_TYPE, LEDDATA_PIN, RBG>(leds, NUM_LEDS);  // WS2812    
    FastLED.clear(true);
    
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));
  
    i2s_std_slot_config_t slot_cfg =
        I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_24BIT,
            I2S_SLOT_MODE_STEREO);
  
    slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
  
    i2s_std_config_t std_cfg = {
      .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
      .slot_cfg = slot_cfg,
      .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = (gpio_num_t)I2S_BCLK_PIN,
        .ws   = (gpio_num_t)I2S_LRCLK_PIN,
        .dout = I2S_GPIO_UNUSED,
        .din  = (gpio_num_t)I2S_DATA_PIN,
        .invert_flags = {0}
      }
    };
  
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));    

// Load Settings from Flash
  preferences.begin("CS3cfg", true);
  String savedIp = preferences.getString("cs3ip", "192.168.1.119");
  cs3Port = preferences.getUShort("port", 15731);
  modelTimeEnabled = preferences.getBool("modelTime", false);
  preferences.end();

  savedIp.toCharArray(cs3IpStr, 16);
  cs3Ip.fromString(cs3IpStr);
  sprintf(cs3PortStr, "%d", cs3Port);

  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(wifiPacketTask, "WiFiPacketTask", 8192, NULL, 1, NULL, 0);
  
    //gettime (should be from Fast Clock)
    RgbWColor initColor;
    CRGB initCrgb = CRGB::Black;
    uint8_t preset_max;
    
    initColor = PRESET_DAY;    
    preset_max = MORNING_TO_DAY_ANIM_LENGHT;
    currentDaytime = Daytimes::Daytime::DAY;
    initCrgb = RgbWColor2Crgb(initColor);
    
    for (uint8_t i=0;i<NUM_LEDS;i++) {
      leds[i] = initCrgb;
      ledsData[i] = { preset_max };
    }
    FastLED.show();    
    // WarmWhite    
    ledcWrite(WARMWHITE_PIN, (initColor.w << DUTY_SHIFT_BITS)*dimWW*dim*dimDynamic);  // esp32 doesn't have analogeWrite ??    

    randomSeed(analogRead(0));

  #ifdef digitalPinToInterrupt
    Dcc.pin(DCC_PIN, 0);
  #else
    Dcc.pin(0, DCC_PIN, 1);
  #endif
          
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

currentGenre = AudioGenre::GenreMode::GENRE_EDM;   // explicit default
lastGenreChange = millis();

  //Serial.print("DCC Baseaddress: ");
    //Serial.println(BASEADDRESS);
}

void loop() {
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  if (ddcIgnore > 0)
    ddcIgnore--;
    
  Dcc.process();  

  // 3. Local Minute Interpolation
    if (modelTimeEnabled && scaleCalculated) {
      // Interval for 1 model MINUTE (60 * 1000ms / scale)
      unsigned long msPerModelMinute = 60000.0 / timeScale;

      if (millis() - lastLocalTick >= msPerModelMinute) {
        lastLocalTick = millis();
        
        if (++mM >= 60) { 
          mM = 0; 
          if (++mH >= 24) mH = 0; 
        }

        if (mM != lastPrintedMinute) {
          //Serial.printf("Model Time: %02d:%02d\n", mH, mM);
          setDaytimeBasedOnTime(mH, mM);
          lastPrintedMinute = mM;
        }
      }
    }

  unsigned long currentMillis = millis();  
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;  

    // do stuff    
    switch (lightMode) {      
      case LightMode::Mode::PARTY:
        doParty();
        break;
      case LightMode::Mode::WORK:
        // doWorkLight();
        break;
      case LightMode::Mode::DAYNIGHT:
      default:
         if (doStuff++ >= TRANSITION_STEP_DELAY) {
            doStuff = 0;
            doDaytimeTransitions();
        }
        switch (daytimeOverlay) {
          case DaytimeOverlay::Overlay::THUNDER:
            doThunder();
            break;
        }
        break;
    }    
  }
}

void setDaytimeBasedOnTime(uint8_t hours, uint8_t minutes) {
    if (lightMode != LightMode::Mode::DAYNIGHT)
      return;

    currentHours = hours;
    if (currentHours >= 9 && currentHours <= 17 && currentDaytime != Daytimes::Daytime::DAY) {
      newDaytime = Daytimes::Daytime::DAY;      
    } else if (currentHours >= 18 && currentHours <= 21 && currentDaytime != Daytimes::Daytime::EVENING) {
      newDaytime = Daytimes::Daytime::EVENING;
    } else if (((currentHours >= 22 && currentHours <= 23) || currentHours <= 6) && currentDaytime != Daytimes::Daytime::NIGHT) {
      newDaytime = Daytimes::Daytime::NIGHT;
    } else if (currentHours >= 7 && currentHours <= 8 && currentDaytime != Daytimes::Daytime::MORNING) {
      newDaytime = Daytimes::Daytime::MORNING;
    }
    timeReceived = true;
}

// Start - Disco mode functions
// helper: returns true if an active or cooling flasher occupies idx or its neighbor
bool isIdxBusyOrNeighbor(int idx, unsigned long now) {
  for (int i = 0; i < MAX_ACTIVELEDFLASHERS; ++i) {
    // If flasher is active (on or fading), block neighbors/itself
    // Also block if flasher is in cooldown and still cooling
    if (activeFlashers[i].state != 0 || activeFlashers[i].cooldownEnd > now) {
      int a = activeFlashers[i].index;
      if (a == idx || (a > 0 && a - 1 == idx) || (a < NUM_LEDS && a + 1 == idx )) return true;
    }
  }
  return false;
}

// pick a random available LED index, respecting neighbor & cooldown rules
int pickRandomAvailableLED() {
  unsigned long now = millis();

  // First a small deterministic sweep to try to pick a mostly-unused LED:
  // this helps avoid bias toward lower indices in some random libs.
  const int attempts = MAX_ACTIVELEDFLASHERS*2;
  for (int a = 0; a < attempts; ++a) {
    int idx = random(0, NUM_LEDS+10);
    if (idx < NUM_LEDS && !isIdxBusyOrNeighbor(idx, now)) return idx;
  }

  // nothing free
  return -1;
}

// spawn a flasher into the first free flasher slot
void spawnFlasher(uint8_t hue) {
  unsigned long now = millis();

  for (int i = 0; i < MAX_ACTIVELEDFLASHERS; ++i) {
    if (activeFlashers[i].state == 0 && activeFlashers[i].cooldownEnd <= now) {
      int idx = pickRandomAvailableLED();
      if (idx < 0) return; // no candidate found

      // assign
      activeFlashers[i].index = idx;
      activeFlashers[i].state = 1;
      activeFlashers[i].endTime = now + random(ON_TIME_MIN, ON_TIME_MAX);
      // note: cooldownEnd will be set later once fully faded

      // set LED color (full)
      uint8_t randBehave = random(4);
      if (randBehave >= 2) {
        leds[idx] = CHSV(random8(), 255, 255); // random8()
      } else {
        leds[idx] = CHSV(hue, 255, 255); // random8()
      }
      return;
    }
  }
}

// update flashers: handle on->fade, fade->done, robustly clear tiny residues
void updateFlashers() {
  unsigned long now = millis();
  bool updateHue = bassDetected;
  if (bassDetected) {
    hueOffset += HUE_JUMP;
    bassDetected = false;
  }
  uint8_t fadeBy = genres[currentGenre].fadeBy;

  for (int i = 0; i < MAX_ACTIVELEDFLASHERS; ++i) {
    if (activeFlashers[i].state == 0) continue;

    int idx = activeFlashers[i].index;
    if (idx < 0 || idx >= NUM_LEDS) { // safety
      activeFlashers[i].state = 0;
      continue;
    }

    if (activeFlashers[i].state == 1) {
      if (updateHue) {        
        CHSV hsv = rgb2hsv_approximate(leds[idx]);  // convert to HSV
        hsv.hue += hueOffset;
        leds[idx] = hsv;
      }
      // on: wait until endTime then start fading
      if (now >= activeFlashers[i].endTime) {
        activeFlashers[i].state = 2;
      }
    } else if (activeFlashers[i].state == 2) {
      // fading: apply fade each loop
      leds[idx].fadeToBlackBy(fadeBy);

      // If very dim, force off and set cooldown (this prevents tiny residue)
      if (leds[idx].getLuma() <= 6) {
        leds[idx] = CRGB::Black;
        activeFlashers[i].state = 0;
        activeFlashers[i].cooldownEnd = now + random(COOLDOWN_MIN, COOLDOWN_MAX);
      }
    }
  }
}

bool anyFlasherActive() {
  for (int i = 0; i < 20; i++) {
    if (activeFlashers[i].state != 0) return true;
  }
  return false;
}

// compute Euclidean distance
float scoreGenre(int g) {
  float db = bassEnergyEMA - genreProfiles[g].bassEnergy;
  float dr = bassHitRateEMA - genreProfiles[g].bassRate;
  float dt = transientRateEMA - genreProfiles[g].transientRate;
  return db*db + dr*dr + dt*dt;
}

void autoSelectGenre() {
  if (millis() - lastGenreChange < GENRE_HOLD_TIME)
    return;

  AudioGenre::GenreMode best = currentGenre;
  float bestScore = scoreGenre(currentGenre);

  for (int g = 0; g < AudioGenre::GenreMode::GENRE_COUNT; g++) {
    float s = scoreGenre(g);
    if (s < bestScore * 0.75 && g != AudioGenre::GenreMode::GENRE_AMBIENT) { // confidence margin
      best = (AudioGenre::GenreMode)g;
      bestScore = s;
    }
  }

  if (best != currentGenre)
    lastGenreChange = millis();
    currentGenre = best;
}


// End - Disco mode functions

// Start - DCC functions
// address: het adres natuurlijk
// direction: ROOD == 0, GROEN != 0
void ProcessCommand (uint16_t address, uint8_t direction) {
  if (isInTransition)
    return;

  switch(address){
    case BASEADDRESS:
      if(direction == 0){     // R: Day
        newDaytime = Daytimes::Daytime::DAY;
        //Serial.println("DCC to Daytime: Day");
      } else {                // G: Evening
        newDaytime = Daytimes::Daytime::EVENING;
        //Serial.println("DCC to Daytime: Evening");
      }
      break;
    case BASEADDRESS+1:
      if(direction == 0){     // R: Night
        newDaytime = Daytimes::Daytime::NIGHT;
        //Serial.println("DCC to Daytime: Night");
      } else {                // G: Morning
        newDaytime = Daytimes::Daytime::MORNING;
        //Serial.println("DCC to Daytime: Morning");
      }
      break;
    case BASEADDRESS+2:
      if(direction == 0){     // R: Off
        disableWorkLight();
        //Serial.println("DCC Worklight: Off");
      } else {                // G: On
        if (ddcIgnore > 0)
          return;

        ddcIgnore = 255;
        
        bool doCWdim = false;
        if (lightMode == LightMode::Mode::WORK) {
          doCWdim = true;
        }
          
        enableWorkLight();

        if (doCWdim) {
          dimCW += 0.25;
          if (dimCW >= 1.0)
            dimCW = 0.25;
          //Serial.println("DCC Worklight: On");
        }
      }
      break;
    case BASEADDRESS+3:
      if(direction == 0){     // R: Off
        disablePartyMode();
        //Serial.println("DCC PartyMode: Off");
      } else {                // G: On
        enablePartyMode();
        //Serial.println("DCC PartyMode: On");
      }
      break;
  }
}

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  if (OutputPower !=0 && Addr >= BASEADDRESS && Addr <= BASEADDRESS+3) {
    ProcessCommand (Addr, Direction);
  }
}
// End - DCC functions

void disableThunder() {
  //Serial.println("Disable Thunder");
  if (daytimeOverlay != DaytimeOverlay::Overlay::THUNDER)
    return;

  daytimeOverlay = DaytimeOverlay::Overlay::NONE;

  dimDynamic = 1.0;  

  clearThunder();

  returnToDaytimeLedData();
}

void clearThunder() {
  for (uint8_t i=0;i<MAX_LIGHTNING;i++) {
    if (lightningData[i].active && lightningData[i].flashCount >= 0) {
      uint8_t ledIdx = lightningData[i].led;
        lightningData[i].isOn = 0;
        leds[ledIdx] = RgbWColor2Crgb(nextDaytimeStepColor(currentDaytime, ledsData[ledIdx].step));
        lightningData[i].led = 0;
        lightningData[i].active = false;
    }
  }
}

void enableThunder() {
  //Serial.println("Enable Thunder");
  if (lightMode != LightMode::Mode::DAYNIGHT || daytimeOverlay == DaytimeOverlay::Overlay::THUNDER)
    return;

  dimDynamic = 0.5;

  returnToDaytimeLedData();

  for (uint8_t i=0;i<MAX_LIGHTNING;i++) {
    lightningData[i] = ledLightningData{};
  }
  
  daytimeOverlay = DaytimeOverlay::Overlay::THUNDER;
}

void enablePartyMode() {
  if (lightMode == LightMode::Mode::PARTY)
    return;

   if (daytimeOverlay == DaytimeOverlay::Overlay::THUNDER) {
      clearThunder();
   }
  
  //Serial.println("Enable Party Mode");
  if (lightMode != LightMode::Mode::PARTY) {        
    FastLED.clear(true);
  }
  ledcWrite(WARMWHITE_PIN, 0);
  ledcWrite(COLDWHITE_PIN, 0);

  audioStartTime = millis();
  lastAudioActivityTime = 0;
  lightMode = LightMode::Mode::PARTY;
}

void disablePartyMode() {
  //Serial.println("Disable Party Mode");
  if (lightMode != LightMode::Mode::PARTY)
    return;
  
  lightMode = LightMode::Mode::DAYNIGHT;

  returnToDaytimeLedData();
}

void enableWorkLight() {
   if (daytimeOverlay == DaytimeOverlay::Overlay::THUNDER) {
      clearThunder();
   }
  
  //Serial.println("Enable WorkLight");
  if (lightMode != LightMode::Mode::WORK) {    
    FastLED.clear(true);
  }
  
  // WarmWhite
  ledcWrite(WARMWHITE_PIN, 0);
  ledcWrite(COLDWHITE_PIN,  (255 << DUTY_SHIFT_BITS)*dimCW);
  
  lightMode = LightMode::Mode::WORK;
}

void disableWorkLight() {
  //Serial.println("Disable WorkLight");
  if (lightMode != LightMode::Mode::WORK)
    return;  

  returnToDaytimeLedData();
  
  ledcWrite(COLDWHITE_PIN, 0);
  
  lightMode = LightMode::Mode::DAYNIGHT; // should be old lightmode?
}

void doThunder() {  
  uint8_t doStrikes = random(MAX_LIGHTNING);
  for (uint8_t i=0;i<doStrikes;i++) {
    uint8_t freeIndex = 0;
    bool isFreeIndex = false;
    uint8_t strikeOnSegment = (uint8_t)random(NUM_LEDS);
        //Serial.print("Strike on Segment:");
        //Serial.println(strikeOnSegment,DEC) ;
    bool strikeFound = false;
    for (uint8_t f=0;f<MAX_LIGHTNING && !strikeFound;f++) {      
      if (!isFreeIndex && !lightningData[f].active && lightningData[f].cooldown == 0) {
        freeIndex = f;        
        isFreeIndex = true;   
      }
      if (lightningData[f].led == strikeOnSegment && lightningData[f].active) {
        strikeFound = true;
      }  
    }

    if (!strikeFound && isFreeIndex)  {
      //Serial.print("freeIndex:");
      //Serial.println(freeIndex,DEC) ;
      ledLightningData newLightning = ledLightningData{};
        newLightning.led = strikeOnSegment;
        newLightning.active = true;
        newLightning.flashCount = random(2, 10);
        newLightning.isOn = 0;
        newLightning.cooldown = 0;           

      lightningData[freeIndex]  = newLightning;
    }
  }

  for (uint8_t i=0;i<MAX_LIGHTNING;i++) {
    if (lightningData[i].active && lightningData[i].flashCount >= 0) {
      uint8_t ledIdx = lightningData[i].led;
      lightningData[i].flashCount--;
      if (lightningData[i].flashCount == 0) {
        lightningData[i].isOn = 0;
        leds[ledIdx] = RgbWColor2Crgb(nextDaytimeStepColor(currentDaytime, ledsData[ledIdx].step));
        lightningData[i].led = 0;
        lightningData[i].active = false;
        lightningData[i].cooldown = random(48,1024);
      } else {
        // flash        
        if (random(4) > 1 && lightningData[i].isOn < 2) {          
          leds[ledIdx] = CRGB_Lightning;
          lightningData[i].isOn++;
        } else {
          lightningData[i].isOn = 0;
          leds[ledIdx] = RgbWColor2Crgb(nextDaytimeStepColor(currentDaytime, ledsData[ledIdx].step)); // Dark contrasts better to lightning color than the old color.
        }
      }
    } else if (lightningData[i].cooldown > 0){
      lightningData[i].cooldown--;
    }
  }
  FastLED.show();
}

void returnToDaytimeLedData() {
  FastLED.clear();  

  RgbWColor tmpColor;
  CRGB tmpRGB = CRGB::Black;  
  for (uint8_t i=0;i<NUM_LEDS;i++) {
    if (i == 0 || ledsData[i].step != ledsData[i-1].step) {
      tmpColor = nextDaytimeStepColor(currentDaytime, ledsData[i].step);
      tmpRGB = RgbWColor2Crgb(tmpColor); 
    }
    leds[i] = tmpRGB;
  }
  FastLED.show();  
  // WarmWhite
  ledcWrite(WARMWHITE_PIN,  (tmpColor.w << DUTY_SHIFT_BITS)*dimWW*dim*dimDynamic);
}

void doParty() {
 // ==== READ AUDIO ====
  size_t bytes_read = 0;
  i2s_channel_read(rx_chan, i2s_buffer, READ_BUFFER_SIZE, &bytes_read, portMAX_DELAY);

  
  // Count bass hits for rate
  bool bassHit = processBassFFT((int32_t *)i2s_buffer);
  bool audioActive = isAudioActive();
  
  unsigned long now = millis();
  if (audioActive) {
    lastAudioActivityTime = now;

    if (currentGenre == AudioGenre::GenreMode::GENRE_AMBIENT) {
      if (bassEnergyEMA > 0.08f)
        currentGenre = AudioGenre::GenreMode::GENRE_TECHNO;
      else
        currentGenre = AudioGenre::GenreMode::GENRE_HOUSE;
        
      lastGenreChange = now;
    } else if (now - audioStartTime > AUTO_GENRE_WARMUP &&
      currentGenre != AudioGenre::GenreMode::GENRE_AMBIENT) {
  
      autoSelectGenre();
    }
  } else if (   // ----- FAST EXIT FROM AMBIENT -----
      now - audioStartTime > AUTO_GENRE_WARMUP && (now - lastAudioActivityTime > 15000 || (lastAudioActivityTime == 0 && now - audioStartTime > 15000)) &&
      currentGenre != AudioGenre::GenreMode::GENRE_AMBIENT) {

      currentGenre = AudioGenre::GenreMode::GENRE_AMBIENT;
      lastGenreChange = now;
    }  
  
  GenreConfig& g = genres[currentGenre];
  for (int i = 0; i < NUM_LEDS; i++) leds[i].fadeToBlackBy(12);

  // ---------- BASS ----------
  if (bassHit) {
    bassDetected = true;
    float bassEnergy = constrain(bassEnv * 6.0f, 0.0f, 1.0f);
    int spawnCount = constrain( bassEnergy*3*g.bassSpawnGain, 1, g.maxSpawn );
    while (spawnCount--)
      spawnFlasher(g.baseHue);
  }
  
  // ---------- TRANSIENTS ----------ambient
  if (transientDetected && random(100) < g.transientGain * 100)
    spawnFlasher(g.baseHue + random8(40));

  // ---------- IDLE ----------
  if (!anyFlasherActive() &&
      ((now - lastAudioActivityTime) > g.idleTimeout || (lastAudioActivityTime == 0 && now - audioStartTime > g.idleTimeout)) &&
      (now - lastIdleSpawnTime) > g.idleInterval) {

    lastIdleSpawnTime = now;
    int idleSpawns = random(1, IDLE_MAX_SPAWN + 1);
      while (idleSpawns--) spawnFlasher(g.baseHue);
  }

  updateFlashers();

  FastLED.show();
}

bool isAudioActive() {
    return bassEnergyEMA > 0.008f
        || (millis() - lastBassHitTime) < 800
        || transientRateEMA > 0.05f;
}

// =============================================================
// FFT PROCESSOR
// =============================================================
bool processBassFFT(int32_t *raw) {

  double mean = 0.0;
  for (int i = 0; i < FFT_SIZE; i++) {
    vReal[i] = (int16_t)(raw[i * 2] >> 16);
    mean += vReal[i];
  }
  mean /= FFT_SIZE;

  for (int i = 0; i < FFT_SIZE; i++) {
    vReal[i] = (vReal[i] - mean) / 32768.0;
    vImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Blackman_Harris, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  const double norm = (FFT_SIZE / 2.0) * WINDOW_GAIN;
  for (int i = 0; i < FFT_SIZE / 2; i++) vReal[i] /= norm;

  double sumSq = 0.0;
  int count = 0;
  for (int i = bassBinStart; i <= bassBinEnd; i++) {
    sumSq += vReal[i] * vReal[i];
    count++;
  }

  bassRMS = (count > 0) ? sqrt(sumSq / count) : 0.0;

  float coeff = bassRMS > bassEnv ? BASS_ATTACK : BASS_RELEASE;
  bassEnv += (bassRMS - bassEnv) * coeff;

// ------------------------------
    // Transient detection (high-frequency peaks)
    // Example: bins above bassBinEnd up to ~8 kHz
    // ------------------------------
    transientDetected = false;
    double hfSumSq = 0.0;
    int hfStart = bassBinEnd + 1;
    int hfEnd   = (8000.0 * FFT_SIZE) / I2S_SAMPLE_RATE;
    hfEnd = min(hfEnd, FFT_SIZE/2 - 1);

    for (int i = hfStart; i <= hfEnd; i++) {
        hfSumSq += vReal[i] * vReal[i];
    }
    double hfRMS = sqrt(hfSumSq / max(1, hfEnd-hfStart+1));
    if (hfRMS > TRANSIENT_THRESHOLD) transientDetected = true;
  
    // Update EMAs
    float alpha = 0.05f;
    bassEnergyEMA    = alpha * bassEnv + (1 - alpha) * bassEnergyEMA;
    bassHitRateEMA   = alpha * (bassRMS > 0.015f ? 1.0f : 0.0f) + (1 - alpha) * bassHitRateEMA;
    transientRateEMA = alpha * ((bassRMS - prevBassEnv) > 0.01f ? 1.0f : 0.0f) + (1 - alpha) * transientRateEMA;

  bool hit = false;
  if (bassEnv > prevBassEnv * BEAT_SENSITIVITY &&
      bassEnv > 0.015f) {
        lastBassHitTime = millis();
    hit = true;
  }

  prevBassEnv = bassEnv;
  return hit;
}

uint8_t maxDaytimeSteps(enum Daytimes::Daytime daytime) {  
  switch (daytime) {
    case Daytimes::Daytime::DAY:
      //Serial.println("Day");
      return MORNING_TO_DAY_ANIM_LENGHT;
      break;
    case Daytimes::Daytime::EVENING:
    //Serial.println("Evening");
      return DAY_TO_EVENING_ANIM_LENGHT;
      break;
    case Daytimes::Daytime::NIGHT:
    //Serial.println("Night");
      return EVENING_TO_NIGHT_ANIM_LENGHT;
      break;
    case Daytimes::Daytime::MORNING:
    //Serial.println("Morning");
      return NIGHT_TO_MORNING_ANIM_LENGHT;
      break;    
  }
  return 0;
}

RgbWColor nextDaytimeStepColor(enum Daytimes::Daytime daytime, uint8_t step) {
  switch (daytime) {
    case Daytimes::Daytime::DAY:
      if (step < MORNING_TO_DAY_ANIM_LENGHT)
        return morningToDay[step];
      else 
        return morningToDay[MORNING_TO_DAY_ANIM_LENGHT-1];
      break;
    case Daytimes::Daytime::EVENING:
      if (step < DAY_TO_EVENING_ANIM_LENGHT)
        return dayToEvening[step];
      else 
        return dayToEvening[DAY_TO_EVENING_ANIM_LENGHT-1];
      break;
    case Daytimes::Daytime::NIGHT:
      if (step < EVENING_TO_NIGHT_ANIM_LENGHT)
        return eveningToNight[step];
      else 
        return eveningToNight[EVENING_TO_NIGHT_ANIM_LENGHT-1];
      break;
    case Daytimes::Daytime::MORNING:
      if (step < NIGHT_TO_MORNING_ANIM_LENGHT)      
        return nightToMorning[step];
      else 
        return nightToMorning[NIGHT_TO_MORNING_ANIM_LENGHT-1];
      break;    
  }
}

void doDaytimeTransitions() {
  if (currentDaytime != newDaytime && !isInTransition) {
      // 
        switch (currentDaytime) {          
          case Daytimes::Daytime::MORNING:
            //Serial.println("Switch to Day");
            currentDaytime = Daytimes::Daytime::DAY;
            break;
          case Daytimes::Daytime::NIGHT:
          //Serial.println("Switch to Morning");
            currentDaytime = Daytimes::Daytime::MORNING;
            break;    
          case Daytimes::Daytime::EVENING:
          //Serial.println("Switch to Night");
            currentDaytime = Daytimes::Daytime::NIGHT;
            break;
          case Daytimes::Daytime::DAY:
            //Serial.println("Switch to Evening");
            currentDaytime = Daytimes::Daytime::EVENING;
            break;                   
        }
        for (uint8_t i=0;i<NUM_LEDS;i++) {          
          ledsData[i] = { 0 };
        }
        isInTransition = true;
        maxStepsReached = false;
        maxSteps = maxDaytimeSteps(currentDaytime);   
        //Serial.print("Max steps:");
        //Serial.println(maxSteps,DEC) ;
        fieldsCounter =0;
    } 
  if (isInTransition) {
    if (!maxStepsReached) {
      //Serial.println("Next step");
      next_step();      
    } else {
      //Serial.println("Transition finished");
      isInTransition = false;
    }
  }
}

CRGB RgbWColor2Crgb(RgbWColor rgbw) {
   return CRGB(rgbw.r*dim*dimDynamic,rgbw.g*dim*dimDynamic,rgbw.b*dim*dimDynamic);
}

void next_step() {
  // field
  uint8_t setLed = 0;
  
  if (fieldsCounter < NUM_DIF_SEGMENTS) {
    fieldsCounter++;
  }

  RgbWColor newColor = EMPTY_COLOR;
  CRGB newRgb = CRGB::Black;
  for (uint8_t field=0;field<fieldsCounter;field++) {
    uint8_t fieldLength = segmentLengths[field];
    for (uint8_t i=0;i<fieldLength;i++) {
      if (setLed < NUM_LEDS) {
        if (i == 0) {
          newColor = nextDaytimeStepColor(currentDaytime, ledsData[setLed].step);
          newRgb = RgbWColor2Crgb(newColor);
        }
        leds[setLed] = newRgb;    
        if (ledsData[setLed].step < maxSteps) {
          ledsData[setLed].step++;
        }
        setLed++;
        //Serial.print("Set led 1: ");
        //Serial.println(setLed,DEC) ;
      }
    }    
  }
  if (fieldsCounter == NUM_DIF_SEGMENTS) {
    while (setLed < NUM_LEDS) {      
      leds[setLed] = newRgb;
      if (ledsData[setLed].step < maxSteps) {
        ledsData[setLed].step++;
      }
      setLed++;
      //Serial.print("Set led 2: ");
        //Serial.println(setLed,DEC) ;
    }
  }
  FastLED.show();
    maxStepsReached = (ledsData[NUM_LEDS-1].step>= maxSteps);
    if (!maxStepsReached) {
      // WarmWhite
      ledcWrite(WARMWHITE_PIN,  (newColor.w << DUTY_SHIFT_BITS)*dimWW*dim*dimDynamic);
    }
    //Serial.print("Max steps reached: ");
        //Serial.println(maxStepsReached,DEC) ;
}

// ----------- Webrequests ---------------------

void registerUrls() {
      if (serverStarted)
        return;
        
      server.on("/", handleRoot);
      server.on("/daytimeMorning", handleDaytimeMorning);
      server.on("/daytimeDay", handleDaytimeDay);
      server.on("/daytimeEvening", handleDaytimeEvening);
      server.on("/daytimeNight", handleDaytimeNight);
      server.on("/daytimeWork", handleDaytimeWork);
      server.on("/daytimeParty", handleDaytimeParty);
      server.on("/overlayThunder", handleOverlayThunder);
      server.on("/settings", handleSettings);
      server.begin();
      serverStarted = true;
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>"
    "body{font-family:Arial;text-align:center;}"
    "button{font-size:16px;padding:10px;margin:10px;width:200px;}"
    "</style></head><body>";
  html += "<h1>CS Light Control</h1>";
  html += "<button onclick=\"location.href='/daytimeMorning'\">Ochtend</button>";  
  html += "<button onclick=\"location.href='/daytimeDay'\">Dag</button>";
  html += "<button onclick=\"location.href='/daytimeEvening'\">Avond</button>";
  html += "<button onclick=\"location.href='/daytimeNight'\">Nacht</button><br><br>";
  html += "<button onclick=\"location.href='/overlayThunder'\">Bliksem</button>";
  html += "<button onclick=\"location.href='/daytimeWork'\">Werk</button>";
  html += "<button onclick=\"location.href='/daytimeParty'\">Disco</button><hr>";
  html += "<a href=\"/settings\">WiFi Settings</a>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleOverlayThunder() {    
  if (daytimeOverlay != DaytimeOverlay::Overlay::THUNDER && lightMode == LightMode::Mode::DAYNIGHT) {
    enableThunder();
  } else if (daytimeOverlay == DaytimeOverlay::Overlay::THUNDER) {
    disableThunder();
  }
  
  server.sendHeader("Location", "/");
  server.send(303); // Redirect back to root page
}


void handleDaytimeMorning() {    
  newDaytime = Daytimes::Daytime::MORNING;
  server.sendHeader("Location", "/");
  server.send(303); // Redirect back to root page
}

void handleDaytimeDay() {    
  newDaytime = Daytimes::Daytime::DAY;
  server.sendHeader("Location", "/");
  server.send(303); // Redirect back to root page
}

void handleDaytimeEvening() {    
  newDaytime = Daytimes::Daytime::EVENING;
  server.sendHeader("Location", "/");
  server.send(303); // Redirect back to root page
}

void handleDaytimeNight() {    
  newDaytime = Daytimes::Daytime::NIGHT;
  server.sendHeader("Location", "/");
  server.send(303); // Redirect back to root page
}

void handleDaytimeWork() {     
  bool doCWdim = false;     
  if (lightMode == LightMode::Mode::WORK) {
    doCWdim = true;
  }

  enableWorkLight();

  if (doCWdim) {
    dimCW += 0.25;
    if (dimCW > 1.0) {
      dimCW = 0.25;
      disableWorkLight();
    }          
  }
  
  server.sendHeader("Location", "/");
  server.send(303); // Redirect back to root page
}

void handleDaytimeParty() {      
  if (lightMode != LightMode::Mode::PARTY) {
    enablePartyMode();
  } else {
    disablePartyMode();
  }
  server.sendHeader("Location", "/");
  server.send(303); // Redirect back to root page
}

void handleSettings() {
  startPortal = true;
  inPortal = true;

  server.send(200, "text/html",
    "<html><body style='text-align:center;'>"
    "<h2>Entering setup mode</h2>"
    "<p>Please reconnect to <b>CS-Light-Setup</b></p>"
    "</body></html>"
  );
}
