
#include <mbed.h>
#include <rtos.h>
#include <mbed_wait_api.h>
#include <platform/CircularBuffer.h>
#include <platform/Callback.h>
#include "DFRobotDFPlayerMini.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoBLE.h>


using namespace rtos;

// Semaphores manage access to a shared resource
// This semaphore allows one thread to access the shared buffer
Semaphore stripSemaphore(1);
Semaphore bleSemaphore(1);
Semaphore colorSemaphore(1);
Semaphore navBeaconSemaphore(1);
Semaphore audioSemaphore(1);

Thread threadBluetooth;
Thread threadFireflyDrive;
Thread threadGravRing;
Thread threadNavBeacons;
Thread threadTurbines;
Thread threadPixels;
Thread threadAnimation;
Thread threadAudio;

BLEService serenityService("12E10000-E8F2-537E-4F6C-D104768A1214"); // BLE Serenity Service

BLEIntCharacteristic charFireflyDriveState("12E10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charGravRingState("12E10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charTurbinesState("12E10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charCabinLightsState("12E10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charCockpitLightsState("12E10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charNavBeaconsState("12E10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charLandingLightsState("12E10007-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEIntCharacteristic charFireflyDriveColor("12E10201-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charGravRingColor("12E10202-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charTurbinesColor("12E10203-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charCabinLightsColor("12E10204-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic charCockpitLightsColor("12E10205-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEIntCharacteristic charTurbinesPosition("12E10301-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEIntCharacteristic charSceneControl("12E10401-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);






const int neopixel_pin = 6;
const int neopixel_gravring_count = 12;
const int neopixel_fireflydrive_count = 16;
const int neopixel_turbines_count = 2;
const int neopixel_cabin_count = 3;

const int px_port_nacelle = 0;
const int px_starboard_nacelle = 1;
const int px_gravring_start = 2;
const int px_gravring_end = (px_gravring_start + (neopixel_gravring_count-1));
const int px_fireflydrive_start = (px_gravring_end + 1);
const int px_fireflydrive_end = (px_fireflydrive_start + (neopixel_fireflydrive_count-1));

const int px_aft_cabin_starboard = (px_fireflydrive_end + 1);
const int px_aft_cabin_port = (px_aft_cabin_starboard + 1);
const int px_fore_cabin = (px_aft_cabin_port + 1);
const int px_cockpit = (px_fore_cabin + 1);

const int neopixel_count = (px_cockpit + 1);

const int px_turbines_start = px_port_nacelle;
const int px_turbines_end = px_starboard_nacelle;
const int px_cabin_start = px_aft_cabin_starboard;
const int px_cabin_end = px_fore_cabin;

const int led_landing_lights = 4;
const int led_nav_beacons = 6;

const int srvo_turbine_port = 0;
const int srvo_turbine_starboard = 2;

const int STATE_OFF = 0;
const int STATE_TURNING_ON = 1;
const int STATE_FADING_UP = 2;

const int STATE_TURNING_OFF = 3;
const int STATE_FADING_DOWN = 4;
const int STATE_FADING = 5;
const int STATE_STANDARD_ON = 10;

const int STATE_AUDIO_WAITING = 0;
const int STATE_AUDIO_PLAYING = 1;
const int STATE_AUDIO_LOOPING = 2;
const int STATE_AUDIO_BUSY = 10;

const int DIRECTION_CLOCKWISE = 0;
const int DIRECTION_COUNTERCLOCKWISE = 1;

const bool FADE_UP = true;
const bool FADE_DOWN = false;


const int SERVOMIN = 140; // 150 // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 520; // 600 // This is the 'maximum' pulse length count (out of 4096)
const int SERVOMSMIN = 685; // in microseconds for writemicroseconds()
const int SERVOMSMAX = 2500; // in microseconds for writemicroseconds()
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates


DFRobotDFPlayerMini player;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(neopixel_count, neopixel_pin, NEO_GRB + NEO_KHZ800);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


Adafruit_NeoPixel getStrip(){
  return strip;
}


/*
 * 
 * * * * * COLORS * * * *
 */

uint32_t colorOff = strip.Color(0, 0, 0);
uint32_t colorOrange = strip.Color(95, 55, 5);
uint32_t colorBlue = strip.Color(23, 127, 144);
uint32_t colorGreen = strip.Color(0, 200, 5);
uint32_t colorFire = strip.Color(250, 60, 0);


uint32_t pixelBaseColors[neopixel_count] = { };

void setPixelBaseColor( int pixel, uint32_t basecolor ){
  colorSemaphore.wait();
  pixelBaseColors[pixel] = basecolor;
  colorSemaphore.release();
}

uint32_t getPixelBaseColor( int pixel ){
  uint32_t baseColor=colorOff;
  colorSemaphore.wait();
  baseColor=pixelBaseColors[pixel];
  colorSemaphore.release();
  return baseColor;
}


uint8_t getRedFromColor(uint32_t c)
{
  return (uint8_t) ((c >> 16) & 0xFF);
}
uint8_t getGreenFromColor(uint32_t c) 
{
  return (uint8_t) ((c >> 8) & 0xFF);
}
uint8_t getBlueFromColor(uint32_t c) 
{
  return (uint8_t) (c & 0xFF);
}

uint8_t scale8_videoX( uint8_t i, uint8_t scale)
{
    uint8_t j = (((int)i * (int)scale) >> 8) + ((i&&scale)?1:0);
    /*uint8_t j=0;
    asm volatile(
        "  tst %[i]\n\t"
        "  breq L_%=\n\t"
        "  mul %[i], %[scale]\n\t"
        "  mov %[j], r1\n\t"
        "  clr __zero_reg__\n\t"
        "  cpse %[scale], r1\n\t"
        "  subi %[j], 0xFF\n\t"
        "L_%=: \n\t"
        : [j] "+a" (j)
        : [i] "a" (i), [scale] "a" (scale)
        : "r0", "r1"
    );*/
    return j;
}

// Helper function that blends one uint8_t toward another by a given amount
uint8_t nblendU8TowardU8( uint8_t cur, const uint8_t target, uint8_t amount)
{
  uint8_t returnVal=cur;
  if( cur == target) return cur;
  
  if( cur < target ) {
    uint8_t delta = target - cur;
    delta = scale8_videoX( delta, amount);
    cur += delta;
  } else {
    uint8_t delta = cur - target;
    delta = scale8_videoX( delta, amount);
    cur -= delta;
  }
  return cur;
}

uint32_t fadeTowardColor( uint32_t cur, const uint32_t target, uint8_t amount)
{
  Adafruit_NeoPixel strip = getStrip();  
  return strip.Color( nblendU8TowardU8( getRedFromColor(cur),   getRedFromColor(target),   amount), nblendU8TowardU8( getGreenFromColor(cur), getGreenFromColor(target), amount), nblendU8TowardU8( getBlueFromColor(cur),  getBlueFromColor(target),  amount) );
}





/*
 * 
 * * * * * FIREFLY DRIVE * * * *
 * 
 */
void fireflyDriveInit();
void fireflyDriveCycle();
void setFireflyDriveColor(uint32_t targetColor);
void setFireflyDriveInited( bool inited);
bool getFireflyDriveInited();

 
int currentFireflyDriveState = STATE_STANDARD_ON;
uint32_t fireflyDriveBaseColor = colorOrange;
uint32_t fireflyDriveCurrentColor = colorOff;
bool fireflyDriveInited = false;

void fireflyDriveInit(){
  if(! getFireflyDriveInited() ){
    setFireflyDriveColor(fireflyDriveBaseColor);
    setFireflyDriveInited(true);
  }
}

 
void fireflyDriveCycle(){
    // int diversity = 5;
    // int randomQty = random(0,neopixel_fireflydrive_count);
    int randomPixel;
    while(true){
      if(getFireflyDriveInited()){
        if(currentFireflyDriveState==STATE_STANDARD_ON){
          stripSemaphore.wait();
          // for( int pixel=0; pixel<randomQty; pixel++){
            randomPixel = random(px_fireflydrive_start, px_fireflydrive_end);
            strip.setPixelColor(randomPixel, strip.Color( random(0,255), random(0,255), random(0,255)));      
          stripSemaphore.release();  
        }
      }
      
      ThisThread::sleep_for(500);
    }  
}

void setFireflyDriveColor(uint32_t targetColor){
    for( int pixel=px_fireflydrive_start; pixel<=px_fireflydrive_end; pixel++){
        setPixelBaseColor( pixel, targetColor);
    }
}

void setFireflyDriveInited( bool inited){
  fireflyDriveInited = inited;
}

bool getFireflyDriveInited(){
  return fireflyDriveInited;
}

/*
 * 
 * * * * * GRAV RING * * * *
 * 
 */
void gravRingInit();
void gravRingCycle();
void setGravRingColor(uint32_t targetColor);
void setGravRingInited( bool inited);
bool getGravRingInited();
 
uint32_t gravRingBaseColor = colorBlue;
int currentGravRingState = STATE_STANDARD_ON;
int gravRingHighlight = 0;
int gravRingDirection = DIRECTION_COUNTERCLOCKWISE;
bool gravRingInited = false;

void gravRingInit(){
  if(! gravRingInited){
    setGravRingColor(gravRingBaseColor);
    setGravRingInited(true);
  }
}

void gravRingCycle(){
  while(true){
    if( getGravRingInited() ){
      if(currentGravRingState==STATE_STANDARD_ON){
        if(gravRingDirection == DIRECTION_CLOCKWISE){
          gravRingHighlight--;
          if( gravRingHighlight < 0){ 
            gravRingHighlight = (px_gravring_end - px_gravring_start); 
          }
        } else {
          gravRingHighlight++;
          if( gravRingHighlight >= neopixel_gravring_count){ 
            gravRingHighlight = 0; 
          }
        }
        stripSemaphore.wait();
        strip.setPixelColor((px_gravring_start + gravRingHighlight), colorOff);
        strip.show();
        stripSemaphore.release();
      }
    }
    ThisThread::sleep_for(100);
  }
}

void setGravRingColor(uint32_t targetColor){
    for( int pixel=px_gravring_start; pixel<=px_gravring_end; pixel++){
      setPixelBaseColor( pixel, targetColor);
    }
}

void setGravRingInited( bool inited){
  gravRingInited = inited;
}

bool getGravRingInited(){
  return gravRingInited;
}
/*
 * 
 * * * * * NACELLE TURBINES * * * *
 * 
 */
void turbinesInit();
void turbinesCycle();
void setPortTurbineColor( uint32_t targetColor );
void setStarboardTurbineColor( uint32_t targetColor );
void setTurbinesColor( uint32_t targetColor );
void setTurbinesInited( bool inited );
bool getTurbinesInited();
 
int currentTurbinesState = STATE_OFF;
uint32_t turbinesBaseColor = colorOrange;
bool turbinesInited = false;
 
void turbinesInit(){
  if(! getTurbinesInited() ){
    if(currentTurbinesState==STATE_STANDARD_ON){
      setTurbinesColor( turbinesBaseColor );
    } else {
      setTurbinesColor( colorOff );
    }
    setTurbinesInited(true);
  }
}

void turbinesCycle(){
    int randomMode;
    uint32_t randomColor;
    while(true){
      // Serial.println(currentFireflyDriveState);
      if( getTurbinesInited() ){
        if(currentTurbinesState==STATE_STANDARD_ON){
            randomMode = random(0,10);
            stripSemaphore.wait();
            randomColor=strip.Color( random(0,255), random(0,255), random(0,255) );
            switch(randomMode){
              case 0:
                // do nothing
                break;
              case 1 ...3:
                // fill both with the same color
                strip.fill(randomColor,px_turbines_start,neopixel_turbines_count);
                break;
              case 4 ...6:
                // fill port with color
                strip.setPixelColor(px_port_nacelle,randomColor);
                break;
              case 7 ...9:
                // fill starboard with color
                strip.setPixelColor(px_starboard_nacelle,randomColor);
                break;
            }
            strip.show();
            stripSemaphore.release();
        }
      }
      ThisThread::sleep_for(100);
    }
}

void setPortTurbineColor( uint32_t targetColor){
  setPixelBaseColor( px_port_nacelle, targetColor);
}
void setStarboardTurbineColor( uint32_t targetColor){
  setPixelBaseColor( px_starboard_nacelle, targetColor);
}

void setTurbinesColor( uint32_t targetColor){
  setStarboardTurbineColor( targetColor);
  setStarboardTurbineColor( targetColor);
}

void setTurbinesInited( bool inited){
  turbinesInited = inited;
}

bool getTurbinesInited(){
  return turbinesInited;
}


/*
 * 
 * * * * * MISC PIXELS * * * *
 * 
 */
void pixelsInit();
void setCabinColor(uint32_t targetColor);
void setCockpitColor(uint32_t targetColor);
void pixelCleanup();
void setPixelsInited( bool inited);
bool getPixelsInited();

bool pixelsInited = false;
int currentMiscPixelState = STATE_OFF;
int currentCabinForePixelState = STATE_OFF;
int currentCabinAftStarboardPixelState = STATE_OFF;
int currentCabinAftPortPixelState = STATE_OFF;
int currentCockpitPixelsState = STATE_OFF;

uint32_t cabinForeBaseColor = colorOrange;
uint32_t cabinAftStarboardBaseColor = colorOrange;
uint32_t cabinAftPortBaseColor = colorOrange;
uint32_t cockpitBaseColor = colorOrange;

void pixelsInit(){
  if(! getPixelsInited() ){
    strip.begin();
    strip.setBrightness(250);
    strip.show(); // Initialize all pixels to 'off'
    // give off OKAY signal
    strip.fill(colorGreen,0,neopixel_count);
    strip.show();
    delay(250);
    Serial.println("all pixels activated");
    Serial.println(" ");
    strip.fill(colorOff,0,neopixel_count);
    strip.show();
 
    for( int pixel=0; pixel<neopixel_count; pixel++){
      setPixelBaseColor( pixel, colorOff);
    }


    Serial.println("pixels inited");
    if( currentCabinAftStarboardPixelState== STATE_STANDARD_ON){
      setPixelBaseColor( px_aft_cabin_starboard, cabinAftStarboardBaseColor);
    } else {
      setPixelBaseColor( px_aft_cabin_starboard, colorOff);
    }

    if( currentCabinAftPortPixelState== STATE_STANDARD_ON){
      setPixelBaseColor( px_aft_cabin_port, cabinAftPortBaseColor);
    } else {
      setPixelBaseColor( px_aft_cabin_port, colorOff);
    }
    
    if( currentCabinForePixelState== STATE_STANDARD_ON){
      setPixelBaseColor( px_fore_cabin, cabinForeBaseColor);
    } else {
      setPixelBaseColor( px_fore_cabin, colorOff);
    }

    if( currentCockpitPixelsState== STATE_STANDARD_ON){
      setPixelBaseColor( px_cockpit, cockpitBaseColor);
    } else {
      setPixelBaseColor( px_cockpit, colorOff);
    }

    setPixelsInited(true);
  
  }
}

void setCabinAftStarboardColor(uint32_t targetColor){
    setPixelBaseColor( px_aft_cabin_starboard, targetColor);
}
void setCabinAftPortColor(uint32_t targetColor){
    setPixelBaseColor( px_aft_cabin_port, targetColor);
}
void setCabinForeColor(uint32_t targetColor){
    setPixelBaseColor( px_fore_cabin, targetColor);
}
void setCabinColor(uint32_t targetColor){
    setCabinAftStarboardColor(targetColor);
    setCabinAftPortColor(targetColor);
    setCabinForeColor(targetColor);
}

void setCockpitColor(uint32_t targetColor){
  setPixelBaseColor( px_cockpit, targetColor);
}

void pixelCleanup()
{
  uint8_t fadeAmount=15;
  uint32_t currentColor, targetColor;
  uint16_t i;
  while(true){
    if( getPixelsInited() ){
      stripSemaphore.wait();
      for( i = 0; i < neopixel_count; i++) {
        currentColor = strip.getPixelColor(i);
        targetColor = pixelBaseColors[i];
        if(currentColor != targetColor){
          strip.setPixelColor( i, fadeTowardColor(currentColor, targetColor, fadeAmount) );
        }
      }
      strip.show();
      stripSemaphore.release();
    }
    
    ThisThread::sleep_for(50);
  }
}

void setPixelsInited( bool inited){
  pixelsInited = inited;
}

bool getPixelsInited(){
  return pixelsInited;
}

/*
 * 
 * * * * * NAVIGATION BEACONS * * * *
 * 
 */
bool navBeaconsInited = false;
int currentNavBeaconState = STATE_STANDARD_ON;
// int navBeaconCounter = 0;
// bool navBeaconsLit = false;

void navBeaconsInit(){
  if(! getNavBeaconsInited() ){
    // the nav beacons use the PWM, which should already be inited  
    setNavBeaconsInited(true);
  }
}


void navBeaconsSetLit(int lit){
  if(lit){
    pwm.setPWM(led_nav_beacons,1, 3000);
    // navBeaconsLit=true;
  } else {
    pwm.setPWM(led_nav_beacons,0, 0);
    // navBeaconsLit=false;
  } 
}

void navbeaconsCycle()
{  
  while(true){
    if( getNavBeaconsInited() ){
      // Serial.print("Current beacon state: ");
      // Serial.println(currentNavBeaconState);
      if( getNavBeaconsState()==STATE_STANDARD_ON){
        navBeaconsSetLit(true);
        ThisThread::sleep_for(1200);
        navBeaconsSetLit(false);
        ThisThread::sleep_for(600);
      }
    }
  }
}

void setNavBeaconsInited(bool inited ){
  navBeaconSemaphore.wait();
  navBeaconsInited=inited;
  navBeaconSemaphore.release();
}

bool getNavBeaconsInited(){
  navBeaconSemaphore.wait();
  bool inited = navBeaconsInited;
  navBeaconSemaphore.release();
  return inited;
}

void setNavBeaconsState(int state ){
  navBeaconSemaphore.wait();
  currentNavBeaconState=state;
  navBeaconSemaphore.release();
}

int getNavBeaconsState(){
  navBeaconSemaphore.wait();
  bool state = currentNavBeaconState;
  navBeaconSemaphore.release();
  return state;
}


/*
 * 
 * * * * * SERVOS * * * * 
 * 
 */
bool servosInited = false;
bool currentservostate = 0;
bool servoslinked = true;
int servocounter = 0;
int servopositions[3] = { 0, 0, 0};
int servotargets[3] = { 0, 0, 0};
 
void servosInit(){
  if(! servosInited){
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    pwm.setPWM(0,0, 0);
    pwm.setPWM(2,0, 0);
    Serial.println("PWM started");
    Serial.println(" ");
    Serial.println("Resetting servos");
    pwm.writeMicroseconds(0, SERVOMSMAX);
    pwm.writeMicroseconds(2, SERVOMSMIN);
    
    wait_us(1500);
    pwm.setPWM(0, 0, 0);
    pwm.setPWM(2, 0, 0);
    Serial.println(" ");

    servosInited=true;
  }
}

void setServoPosition(int pos, bool linked=true, int servo=-1){
  int correctedPos = pos + SERVOMSMIN;
    if(servosInited){
      if(linked){
        pwm.writeMicroseconds(0, ((SERVOMSMAX-correctedPos)+SERVOMSMIN));
        servopositions[srvo_turbine_port]=((SERVOMSMAX-correctedPos)+SERVOMSMIN);
      } else {
        pwm.writeMicroseconds(0, correctedPos);
        servopositions[srvo_turbine_port]=correctedPos;
      }
      pwm.writeMicroseconds(2, correctedPos);
      servopositions[srvo_turbine_starboard]=correctedPos;
    }
}



/*
 * 
 * * * * * LANDING LIGHTS * * * * 
 * 
 */
int currentLandingLightsState = STATE_OFF;

void setLandingLightsState(int state){
  if(state==STATE_STANDARD_ON){
     pwm.setPWM(led_landing_lights,1, 3000);
  } else {
     pwm.setPWM(led_landing_lights, 0, 0);
  }
  currentLandingLightsState=state;
}
 



bool printedCentralInfo = false;

/*
 * 
 * * * * * AUDIO * * * *
 * 
 */
void audioInit();
void setAudioState(int state);
int getAudioState();
void stopAllAudio();
void playLoopedAudio(int audioSlot, int audioVolume = 30);
bool previousAudioComplete();
void playAudio(int audioSlot, int audioVolume = 30);
void playAndWaitForAudio(int audioSlot, int audioVolume = 30);
void audioStatus();
 
bool audioInited = false;
// bool audioStatus = STATE_AUDIO_WAITING;
bool currentAudioState = STATE_AUDIO_WAITING;

void audioInit(){
    if(!audioInited){
        Serial1.begin(9600);
        wait_us(5000);
        if(Serial1){
            Serial.println("checking audio");
            // Start communication with DFPlayer Mini
            if (player.begin(Serial1)) {
                Serial.println("audio ready");
                audioInited = true;
                playAndWaitForAudio( 1,30 );
            } else {
                Serial.println("audio failed to init");
            }
        } else {
            Serial.println("Serial (audio) may not be connected");
        }
    } 
}

void setAudioState(int state){
  audioSemaphore.wait();
  currentAudioState=state;
  audioSemaphore.release();
}

int getAudioState(){
  int audioState=STATE_AUDIO_WAITING;
  audioSemaphore.wait();
  audioState=currentAudioState;
  audioSemaphore.release();
  return audioState;
}

void stopAllAudio(){
    if( audioInited ){
        Serial.println("Stopping audio");
        audioSemaphore.wait();
        player.stop();
        audioSemaphore.release();
        setAudioState(STATE_AUDIO_WAITING);
        // need some sort of callback mechanism to determine when audio finishes
    }    
}

void playLoopedAudio(int audioSlot, int audioVolume){
    if( audioInited ){
        Serial.print("Playing looped audio: ");
        Serial.println( audioSlot);
        audioSemaphore.wait();
        player.stop();
        player.volume(audioVolume);
        player.loop(audioSlot);
        audioSemaphore.release();
        setAudioState(STATE_AUDIO_PLAYING);
        // need some sort of callback mechanism to determine when audio finishes
    }    
}

bool previousAudioComplete(){
  if(getAudioState()==STATE_AUDIO_WAITING){
    return true;
  }
  return false;
}

void playAudio(int audioSlot, int audioVolume){
    if( audioInited ){
        Serial.print("Playing audio: ");
        Serial.println( audioSlot);
        audioSemaphore.wait();
        player.stop();
        player.volume(audioVolume);
        player.play(audioSlot);
        audioSemaphore.release();
        setAudioState(STATE_AUDIO_PLAYING);
        // need some sort of callback mechanism to determine when audio finishes
    }    
}

void playAndWaitForAudio(int audioSlot, int audioVolume){
    if( audioInited ){
        playAudio(audioSlot, audioVolume);
        // wait for it to start
        bool audioPlaying=true;
        while( getAudioState() == STATE_AUDIO_PLAYING ){
          audioSemaphore.wait();
          if (player.available() && player.readType() == DFPlayerPlayFinished ){
            audioPlaying=false;
          }
          audioSemaphore.release();
          ThisThread::sleep_for(50);
        }
        setAudioState(STATE_AUDIO_WAITING);
    }
}

void audioStatus(){
  bool playerAvailable;
  int playerStatusType;
  
  while(true){
    if( audioInited ){
       audioSemaphore.wait();
       playerAvailable=player.available();
       audioSemaphore.release();
       if (playerAvailable){
          audioSemaphore.wait();
          playerStatusType=player.readType();
          audioSemaphore.release();
          if( playerStatusType == DFPlayerPlayFinished ){
            setAudioState(STATE_AUDIO_WAITING);
          }
       }
       
    }
    ThisThread::sleep_for(500);
  }
}


/*
 * 
 * * * * * ANIMATION SCENE ENGINE * * * *
 */
const int ANIMATION_STANDARD_FLIGHT = 1;
const int ANIMATION_LANDING = 2;
const int ANIMATION_HARD_BURN = 3;
const int ANIMATION_CRAZY_IVAN = 4;
const int ANIMATION_OUT_OF_GAS = 5;

 
int currentAnimationScene = 0;
bool animationsInited = false;

void animationsInit(){
  if(! getAnimationsInited() ) {
    

    setAnimationsInited(true);
  }
}

void animationCycle(){
  while(true){  
    if( getAnimationsInited() ){
      if( currentAnimationScene > 0 ){
        switch(currentAnimationScene){
        case ANIMATION_STANDARD_FLIGHT: // Standard Flight
          Serial.println("Run Standard Flight Scene");
          // currentFireflyDriveState = STATE_FADING_UP;
          break;
        case ANIMATION_LANDING: // Landing
          Serial.println("Run Landing Sequence Scene");
          // runLandingSequence();
          // currentFireflyDriveState = STATE_FADING_DOWN;
          break;
        case ANIMATION_HARD_BURN: // Hard Burn
          Serial.println("Run Hard Burn Scene");
          // currentTurbineState = STATE_FADING_UP;
          break;
        case ANIMATION_CRAZY_IVAN: { // Crazy Ivan
          Serial.println("Run Crazy Ivan Scene");
          // currentTurbineState = STATE_FADING_DOWN;
          
          // set cockpit color to red
          playAudio(2,30);
          stripSemaphore.wait();
            setCockpitColor( strip.Color(160,0,0) );
          stripSemaphore.release();
           
          ThisThread::sleep_for( random(200, 1800) );

          stripSemaphore.wait();
            setTurbinesColor( strip.Color(220,220,70) );
            setFireflyDriveColor( strip.Color(50,20,7) );
          stripSemaphore.release();

          ThisThread::sleep_for( random(3000, 3800) );

          setServoPosition(200,true);
          while( ! previousAudioComplete() ){ ThisThread::sleep_for(10); }
          
          playAudio(3,30);
          ThisThread::sleep_for( 13000 );
          
          stripSemaphore.wait();
            setTurbinesColor( colorOff );
          stripSemaphore.release();
 
          setServoPosition(1800,false);

          ThisThread::sleep_for( 500 );

          stripSemaphore.wait();
            setTurbinesColor( strip.Color(220,220,70) );
          stripSemaphore.release();

          ThisThread::sleep_for( 2600 );
          
          stripSemaphore.wait();
            setTurbinesColor( colorOff );
          stripSemaphore.release();

          ThisThread::sleep_for( 500 );

          setServoPosition(0,true);

          ThisThread::sleep_for( 500 );
          
          while( ! previousAudioComplete() ) { ThisThread::sleep_for(10); }
          playAudio(4,30);
          pwm.setPWM(0, 0, 0);
          pwm.setPWM(2, 0, 0);

          stripSemaphore.wait();
            setTurbinesColor( strip.Color(220,220,70) );
          stripSemaphore.release();

          ThisThread::sleep_for( random(200, 1800) );

          stripSemaphore.wait();
            setFireflyDriveColor( strip.Color(220,220,70) );
          stripSemaphore.release();

          ThisThread::sleep_for( random(2000, 2800) );

          stripSemaphore.wait();
            setFireflyDriveColor( colorOrange );
          stripSemaphore.release();

          ThisThread::sleep_for( 500 );

          stripSemaphore.wait();
            setCockpitColor( colorOrange );
          stripSemaphore.release();
          
          currentAnimationScene=0;
          
          break;
        }
        case ANIMATION_OUT_OF_GAS: // Out of Gas
          Serial.println("Run Out of Gas Scene");
          // turnNavBeaconsOn=true;
          break;

        }
      }
    }
  }
}

void setAnimationsInited( bool inited){
  animationsInited = inited;
}

bool getAnimationsInited(){
  return animationsInited;
}




/*
 * 
 * * * * * BLUETOOTH * * * *
 * 
 */
bool bluetoothReady = true;
bool bluetoothInited = false;
 
void bluetoothInit(){
  if(! bluetoothInited){
    if (!BLE.begin()) {   // initialize BLE
      Serial.println("starting BLE failed!");
      
    } else {
    
      BLE.setLocalName("Serenity");  // Set name for connection
      BLE.setAdvertisedService(serenityService); // Advertise service
    
      BLE.addService(serenityService); // Add service
      charNavBeaconsState.setValue(0); // Set greeting string
    
      BLE.advertise();  // Start advertising
      Serial.print("Peripheral device MAC: ");
      Serial.println(BLE.address());
      Serial.println("Waiting for connections...");
    
      
      BLE.setLocalName("Serenity");
      BLE.setAdvertisedService(serenityService);
      serenityService.addCharacteristic(charFireflyDriveState);
      serenityService.addCharacteristic(charGravRingState);
      serenityService.addCharacteristic(charTurbinesState);
      serenityService.addCharacteristic(charCabinLightsState);
      serenityService.addCharacteristic(charCockpitLightsState);
      serenityService.addCharacteristic(charNavBeaconsState);
      serenityService.addCharacteristic(charLandingLightsState);
    
    
      serenityService.addCharacteristic(charFireflyDriveColor);
      serenityService.addCharacteristic(charGravRingColor);
      serenityService.addCharacteristic(charTurbinesColor);
      serenityService.addCharacteristic(charCabinLightsColor);
      serenityService.addCharacteristic(charCockpitLightsColor);
    
      serenityService.addCharacteristic(charTurbinesPosition);
      
      serenityService.addCharacteristic(charSceneControl);
    
    
      // add service
      BLE.addService(serenityService);  
      
      charFireflyDriveState.setValue(0);
      charGravRingState.setValue(0);
      charTurbinesState.setValue(0);
      charCabinLightsState.setValue(0);
      charCockpitLightsState.setValue(0);
      charNavBeaconsState.setValue(1);
      charLandingLightsState.setValue(0);
    
      charFireflyDriveColor.setValue(0);
      charGravRingColor.setValue(0);
      charTurbinesColor.setValue(0);
      charCabinLightsColor.setValue(0);
      charCockpitLightsColor.setValue(0);
    
      charTurbinesPosition.setValue(0);

      charSceneControl.setValue(0);
    
      BLE.advertise();
      bluetoothInited=true;
    }
  }
}


void bluetoothCycle()
{  
  uint32_t colorIncoming;
  while(true){
    if(bluetoothInited){
      // bleSemaphore.wait();
      BLEDevice central = BLE.central();  // Wait for a BLE central to connect
    
      // if a central is connected to Serenity:
      if (central) {
        if(!printedCentralInfo){
          Serial.print("Connected to central MAC: ");
          // print the central's BT address:
          Serial.println(central.address());
          printedCentralInfo=true;
        }    
        if (charFireflyDriveState.written()) {
          if(charFireflyDriveState.value()==1){
            setFireflyDriveColor( fireflyDriveBaseColor );
            // fireflyDriveFade(FADE_UP);
            currentFireflyDriveState=STATE_STANDARD_ON;
            Serial.println("Firefly Drive on");
          } else {
            setFireflyDriveColor( colorOff );
            // fireflyDriveFade(FADE_DOWN);
            currentFireflyDriveState=STATE_OFF;
            Serial.println("Firefly Drive off");
          }
        }
  
        if (charGravRingState.written()) {
          if(charGravRingState.value()==1){
            // gravRingFade(FADE_UP);
            setGravRingColor( gravRingBaseColor );
            currentGravRingState=STATE_STANDARD_ON;
            Serial.println("Grav Ring on");
          } else {
            // gravRingFade(FADE_DOWN);
            setGravRingColor( colorOff );
            currentGravRingState=STATE_OFF;
            Serial.println("Grav Ring off");
          }
        }
        if (charTurbinesState.written()) {
          if(charTurbinesState.value()==1){
            // turbinesFade(FADE_UP);
            Serial.println("Nacelles on");
          } else {
            // turbinesFade(FADE_DOWN);
            Serial.println("Nacelles off");
          }
        }
        if (charCabinLightsState.written()) {
          if(charCabinLightsState.value()==1){
            setCabinColor( cabinAftStarboardBaseColor );
            currentCabinForePixelState = STATE_STANDARD_ON;
            currentCabinAftStarboardPixelState = STATE_STANDARD_ON;
            currentCabinAftPortPixelState = STATE_STANDARD_ON;
            Serial.println("Cabin lights on");
          } else {
            setCabinColor( colorOff );
            currentCabinForePixelState = STATE_OFF;
            currentCabinAftStarboardPixelState = STATE_OFF;
            currentCabinAftPortPixelState = STATE_OFF;
            Serial.println("Cabin lights off");
          }
        }
        if (charCockpitLightsState.written()) {
          if(charCockpitLightsState.value()==1){
            setCockpitColor( cockpitBaseColor );
            currentCockpitPixelsState=STATE_STANDARD_ON;
            Serial.println("Cockpit lights on");
          } else {
            setCockpitColor( colorOff );
            currentCockpitPixelsState=STATE_OFF;
            Serial.println("Cockpit lights off");
          }
        }
        if (charNavBeaconsState.written()) {
          if(charNavBeaconsState.value()==1){
            setNavBeaconsState( STATE_STANDARD_ON );
            Serial.println("Nav beacons on");
          } else {
            setNavBeaconsState( STATE_OFF );
            Serial.println("Nav beacons off");
          }
        }
        if (charLandingLightsState.written()) {
          if(charLandingLightsState.value()==1){
            setLandingLightsState(STATE_STANDARD_ON);
            Serial.println("Landing lights on");
          } else {
            setLandingLightsState(STATE_OFF);
            Serial.println("Landing lights off");
          }
        }
        if (charFireflyDriveColor.written()) {
          colorIncoming=charFireflyDriveColor.value();
          setFireflyDriveColor( colorIncoming );
          fireflyDriveBaseColor=colorIncoming;
          Serial.print("Set Firefly Drive color: ");
          Serial.println( colorIncoming );
          strip.show();
        }
        if (charGravRingColor.written()) {
          colorIncoming=charGravRingColor.value();
          setGravRingColor( colorIncoming );
          gravRingBaseColor=colorIncoming;
          Serial.print("Set Grav Ring color: ");
          Serial.println( colorIncoming );
          strip.show();
        }
  
        if (charTurbinesPosition.written()) {
          // charTurbinesPosition.value();
          setServoPosition(charTurbinesPosition.value());
        }
        if (charSceneControl.written()) {
          Serial.println("Scene control");
          currentAnimationScene=charSceneControl.value();
        }
        
        // bleSemaphore.release();
  
      }    
    
    }



  }
}




/*
 * 
 * * * * * PWM BOARD * * * *
 */
bool pwmInited = false;

void pwmInit(){
  if(! pwmInited){
    pwm.begin();
    wait_us(2500);
    // if(pwm){
      pwmInited=true;
    // }
  }
}



/*
 * 
 * * * * * THREADS * * * *
 */


bool threadsInited=false;

void threadsInit(){
  if(!threadsInited){
    Serial.println("starting threads");
    if(pixelsInited){
       threadPixels.start(pixelCleanup);
      Serial.println("Pixel Thread running");
    }
    if( navBeaconsInited ){
       threadNavBeacons.start(navbeaconsCycle);
      Serial.println("Nav Beacon Thread running");
    }
    if( getFireflyDriveInited() ){
       threadFireflyDrive.start(fireflyDriveCycle);
      Serial.println("Firefly Drive Thread running");
    }
    if( getGravRingInited() ){
       threadGravRing.start(gravRingCycle);
      Serial.println("Grav Ring Thread running");
    }
    if( turbinesInited ){
       threadTurbines.start(turbinesCycle);
      Serial.println("Turbines Thread running");
    }
    if( getAnimationsInited() ){
      threadAnimation.start(animationCycle);
      Serial.println("Turbines Thread running");
    }
    if(audioInited){
      threadAudio.start(audioStatus);
      Serial.println("Audio Thread running");
    }
    if(bluetoothInited){
       threadBluetooth.start(bluetoothCycle);
      Serial.println("Bluetooth Thread running");
    }

    
    threadsInited=true;
  }
}




void setup() {
  delay(1000);
  randomSeed(analogRead(0));
  Serial.begin(9600);
  delay(2000);
  // while(!Serial){ delay(10); }

  audioInit();
    
  pwmInit();
  
  servosInit();
 
  delay(2000);
  pixelsInit();
    
  
  navBeaconsInit();
  fireflyDriveInit();
  gravRingInit();
  turbinesInit();
  animationsInit();

  bluetoothInit();
  threadsInit();
}

void loop() {
  // wait_us(10000);
  // setFireflyDriveColor( colorBlue );
  // fireflyDriveBaseColor=colorBlue;
}
