/**
 * Teensy 3.1 HP Led
 * (C)2015 Anouk Wipprecht <info@anoukwipprecht.nl>
 */

#include <XBee.h>

#define DRESS_1    '1'
#define DRESS_2    '2'
#define DRESS_ALL  'A'

/* define who we are (DRESS_1 or DRESS_2) */
#define MY_ID  DRESS_1

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

/* High Power LED BUCKPUCK Control pins */
#define LED1  23
#define LED2  22
#define LED3  20

#define CONSOLE   Serial
#define XBEEUART  Serial2

/* Minimal cooldown time between animations */
#define ANIMATION_COOLDOWN_DELAY  2000 /*ms */

/* Distance sample rate */
#define DISTANCE_READING_DELAY    100 /*ms */

/* MAXBOTIX Analog pin */
#define MAXBOTIX  A12


/* Zones:
--- void < 50
--- 55 <= fade < 70
--- 100 <= freakout < 120  ( heartbeat )
--- 125 <= notice < 135  ( not implemented )
*/

#define MAXBOTIX_ZONE_1_MIN 55
#define MAXBOTIX_ZONE_1_MAX 70
#define MAXBOTIX_ZONE_2_MIN 100
#define MAXBOTIX_ZONE_2_MAX 120 

unsigned long cooldown_start_millis;
unsigned long distance_reading_millis;
volatile uint16_t maxbotix_value;

enum {
  STATE_WAITING = 0,
  STATE_ZONE_1_START,
  STATE_ZONE_1_BUSY,
  STATE_ZONE_2_START,
  STATE_ZONE_2_BUSY,
  STATE_XBEE_OVERRIDE_START,
  STATE_XBEE_OVERRIDE_BUSY,
  STATE_ANIMATION_DONE,
  STATE_COOLDOWN,
  STATE_NONE,
};


/** Animations */
#include "animations.h"
animationStc animation;

/* the animation frames */
/* add animations defined in animations.h here (not strictly necessary) */
extern frameStc anim_zone_2_frames[];
extern frameStc anim_zone_1_frames[];
extern frameStc anim_heartbeat_frames[];
extern frameStc anim_quick_fade_frames[];

/***************/

uint8_t current_state;
uint8_t previous_state;

static void set_next_state(uint8_t state)
{
  previous_state = current_state;
  current_state = state;
}


static void do_animation(void)
{
  animationStc *anim = &animation;
  frameStc *frame = &anim->frames[anim->frame];

  if (millis() - anim->start_millis < anim->frame_delay)
    return;

/*
  CONSOLE.print("Frame: ");
  CONSOLE.print(anim->frame);
  CONSOLE.print(" of: ");
  CONSOLE.println(anim->num_frames);
  CONSOLE.print("LED1: ");
  CONSOLE.println(frame->led1);
  CONSOLE.print("LED2: ");
  CONSOLE.println(frame->led2); 
*/

  CONSOLE.print(".");

  analogWrite(LED1, frame->led1);
  analogWrite(LED2, frame->led2);
  //TODO: implement third LED
  
  anim->frame_delay = frame->delay;
  anim->start_millis = millis();
  
  if (++anim->frame >= anim->num_frames) {
    if (--anim->repeat <= 0) {
      analogWrite(LED1, 0);
      analogWrite(LED2, 0);
      analogWrite(LED3, 0);
      set_next_state(STATE_ANIMATION_DONE);
    } else {
      CONSOLE.print("REPEAT: ");
      CONSOLE.println(anim->repeat);
      anim->frame = 0;
    }
  }
}

static void handle_xbee_data(uint8_t data)
{
  if (data == MY_ID || data == DRESS_ALL) {
    set_next_state(STATE_XBEE_OVERRIDE_START);
  }
}

static inline uint16_t running_avg(uint16_t current, uint16_t value, float alpha)
{
  float newval = ((float)current * (1.0 - alpha) + (float)value * alpha);
  return round(newval);
}

void do_distance_reading(void)
{
  maxbotix_value = running_avg(maxbotix_value, analogRead(MAXBOTIX), 0.5);
  CONSOLE.print("Dist: ");
  CONSOLE.println(maxbotix_value);

  if (current_state != STATE_WAITING)
    return;
    
  if (maxbotix_value >= MAXBOTIX_ZONE_1_MIN &&
      maxbotix_value < MAXBOTIX_ZONE_1_MAX) {
        set_next_state(STATE_ZONE_1_START);
  } else if (maxbotix_value >= MAXBOTIX_ZONE_2_MIN &&
      maxbotix_value < MAXBOTIX_ZONE_2_MAX) {
        set_next_state(STATE_ZONE_2_START);
  }
}

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, HIGH); //BuckPucks want to have a HIGH level to be deactivated, they are active on LOW.
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  
  CONSOLE.begin(115200);
  XBEEUART.begin(9600);
  xbee.setSerial(XBEEUART);

  current_state = previous_state = STATE_WAITING;
  
  maxbotix_value = analogRead(MAXBOTIX);
  for (int i=0; i<10; i++) {
    maxbotix_value = running_avg(maxbotix_value, analogRead(MAXBOTIX), 0.7);
  }
  //maxbotix_timer.begin(do_distance_reading, 500000);
  distance_reading_millis = millis();
}

void loop() {

   
  /* xbee.readPacket();  // XBEE code start

  if (xbee.getResponse().isAvailable()) {   
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      uint8_t option = 0;
      uint8_t data = 0;
      
      xbee.getResponse().getRx16Response(rx16);
      option = rx16.getOption();
      data = rx16.getData(0);
      
      CONSOLE.print("XBEE Data received: ");
      CONSOLE.println(data);
      
      handle_xbee_data(data);
      
    } else if (xbee.getResponse().isError()) {
      CONSOLE.println("Error reading packet.  Error code: ");  
      CONSOLE.println(xbee.getResponse().getErrorCode());
    }
  } // XBEE code end
*/
  if (millis() - distance_reading_millis >= DISTANCE_READING_DELAY) {
    do_distance_reading();
    distance_reading_millis = millis();
  }
  
  switch (current_state)
  {
    case STATE_WAITING:
      break;
    case STATE_ZONE_1_START:
      /* change animation.frames and animation.num_frames to the wanted animation frames structure */
      animation.frames = (frameStc*)anim_quick_fade_frames;
      animation.num_frames = sizeof(anim_quick_fade_frames) / sizeof(frameStc);
      /***/
      animation.frame = 0;
      animation.frame_delay = 0;
      animation.repeat = 0;
      animation.start_millis = millis();

      CONSOLE.print("Starting animation: ");
      CONSOLE.print(animation.num_frames);
      CONSOLE.println(" frames");
      
      set_next_state(STATE_ZONE_1_BUSY);
    case STATE_ZONE_1_BUSY:
      do_animation();
      break;
      
    case STATE_ZONE_2_START:
      /* change animation.frames and animation.num_frames to the wanted animation frames structure */
      animation.frames = (frameStc*)anim_heartbeat_frames;
      animation.num_frames = sizeof(anim_heartbeat_frames) / sizeof(frameStc);
      /***/
      animation.frame = 0;
      animation.frame_delay = 0;
      animation.repeat = 0;
      animation.start_millis = millis();

      CONSOLE.print("Starting animation: ");
      CONSOLE.print(animation.num_frames);
      CONSOLE.println(" frames");
      
      set_next_state(STATE_ZONE_2_BUSY);
    case STATE_ZONE_2_BUSY:
      do_animation();
      break;
      
    case STATE_XBEE_OVERRIDE_START:
      /* change animation.frames and animation.num_frames to the wanted animation frames structure */
      animation.frames = (frameStc*)anim_xbee_override_frames;
      animation.num_frames = sizeof(anim_xbee_override_frames) / sizeof(frameStc);
      /***/
      animation.frame = 0;
      animation.frame_delay = 0;
      animation.repeat = 5;
      animation.start_millis = millis();
      
      CONSOLE.print("Starting XBEE animation: ");
      CONSOLE.print(animation.num_frames);
      CONSOLE.println(" frames");
      
      set_next_state(STATE_XBEE_OVERRIDE_BUSY);
    case STATE_XBEE_OVERRIDE_BUSY:
      do_animation();
      break;
      
    case STATE_ANIMATION_DONE:
      cooldown_start_millis = millis();
      CONSOLE.println("Animation done, cooling down");
      set_next_state(STATE_COOLDOWN);

    case STATE_COOLDOWN:
      if (millis() - cooldown_start_millis >= ANIMATION_COOLDOWN_DELAY) {
        CONSOLE.println("Done cooling down");
        set_next_state(STATE_WAITING);
      }
      break;

    case STATE_NONE:
    default:
      set_next_state(STATE_WAITING);
  }
}
