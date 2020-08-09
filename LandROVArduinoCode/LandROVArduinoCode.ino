#include <Servo.h>
#include <avr/wdt.h>

#define DIST_PER_ENC_TICK 0.01

#define LEFT_PWM_PIN 5
#define LEFT_FW_PIN 7
#define LEFT_RV_PIN 6
#define RIGHT_PWM_PIN 3
#define RIGHT_FW_PIN 4
#define RIGHT_RV_PIN 2

const byte LEFT_REAR_ENC_PIN_A = A0;
const byte LEFT_REAR_ENC_PIN_B = A1;
const byte LEFT_FRONT_ENC_PIN_A = A3;
const byte LEFT_FRONT_ENC_PIN_B = A2;
const byte RIGHT_REAR_ENC_PIN_A = 9;
const byte RIGHT_REAR_ENC_PIN_B = 10;
const byte RIGHT_FRONT_ENC_PIN_A = 11;
const byte RIGHT_FRONT_ENC_PIN_B = 12;

double distance_left;
double distance_right;
double velocity_left;
double velocity_right;

void setup() {
  wdt_disable();
  Serial.begin(115200);
  
  pinMode(LEFT_REAR_ENC_PIN_A, INPUT);
  pinMode(LEFT_REAR_ENC_PIN_B, INPUT);
  pinMode(LEFT_FRONT_ENC_PIN_A, INPUT);
  pinMode(LEFT_FRONT_ENC_PIN_B, INPUT);
  pinMode(RIGHT_REAR_ENC_PIN_A, INPUT);
  pinMode(RIGHT_REAR_ENC_PIN_B, INPUT);
  pinMode(RIGHT_FRONT_ENC_PIN_A, INPUT);
  pinMode(RIGHT_FRONT_ENC_PIN_B, INPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_FW_PIN, OUTPUT);
  digitalWrite(LEFT_FW_PIN, LOW);
  pinMode(LEFT_RV_PIN, OUTPUT);
  digitalWrite(LEFT_RV_PIN, LOW);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_FW_PIN, OUTPUT);
  digitalWrite(RIGHT_FW_PIN, LOW);
  pinMode(RIGHT_RV_PIN, OUTPUT);
  digitalWrite(RIGHT_RV_PIN, LOW);

  cli();
  *digitalPinToPCMSK(LEFT_REAR_ENC_PIN_A) |= bit (digitalPinToPCMSKbit(LEFT_REAR_ENC_PIN_A));  // enable pin
  *digitalPinToPCMSK(RIGHT_REAR_ENC_PIN_A) |= bit (digitalPinToPCMSKbit(RIGHT_REAR_ENC_PIN_A));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(LEFT_REAR_ENC_PIN_A)); // clear any outstanding interrupt
  PCIFR  |= bit (digitalPinToPCICRbit(RIGHT_REAR_ENC_PIN_A)); // clear any outstanding interrupt
  //PCICR  |= bit (digitalPinToPCICRbit(LEFT_REAR_ENC_PIN_A)); // enable interrupt for the group
  //PCICR  |= bit (digitalPinToPCICRbit(RIGHT_REAR_ENC_PIN_A)); // enable interrupt for the group
  sei();
  
  TCCR0B = TCCR0B & B11111000 | B00000011; // Change pwm timer prescalar to increase frequency
  TCCR2B = TCCR2B & B11111000 | B00000011; // Change pwm timer prescalar to increase frequency

  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  
  //wdt_enable(WDTO_2S);

}

void loop() {
  double code;
  int pwm_value;
  
  //wdt_reset();
  delay(50);
  /*digitalWrite(LEFT_RV_PIN, LOW);
  digitalWrite(LEFT_FW_PIN, HIGH);
  analogWrite(LEFT_PWM_PIN, 50);

  delay(300);
  digitalWrite(RIGHT_RV_PIN, LOW);
  digitalWrite(RIGHT_FW_PIN, HIGH);
  analogWrite(RIGHT_PWM_PIN, 50);*/
          
  if (Serial.available())
  {
    code = (double) Serial.read();
    if (code >= 128) {
      if (code > 191) {
        pwm_value = (int) round(255*(code - 191)/64);
        digitalWrite(LEFT_RV_PIN, LOW);
        digitalWrite(LEFT_FW_PIN, HIGH);
        analogWrite(LEFT_PWM_PIN, pwm_value);
      } else if (code < 191) {
        pwm_value = (int) round(255*(191 - code)/64);
        digitalWrite(LEFT_FW_PIN, LOW);
        digitalWrite(LEFT_RV_PIN, HIGH);
        analogWrite(LEFT_PWM_PIN, pwm_value);
      } else {
        digitalWrite(LEFT_FW_PIN, LOW);
        digitalWrite(LEFT_RV_PIN, LOW);
        analogWrite(LEFT_PWM_PIN, 0);
      }
    } else {
        //Right motor speed set: 0-127, 63 stationary
        if (code > 63) {
          pwm_value = (int) round(255*(code - 63)/64);
          digitalWrite(RIGHT_RV_PIN, LOW);
          digitalWrite(RIGHT_FW_PIN, HIGH);
          analogWrite(RIGHT_PWM_PIN, pwm_value);
        } else if (code < 63) {
          pwm_value = (int) round(255*(63 - code)/64);
          digitalWrite(RIGHT_FW_PIN, LOW);
          digitalWrite(RIGHT_RV_PIN, HIGH);
          analogWrite(RIGHT_PWM_PIN, pwm_value);
        } else {
          digitalWrite(RIGHT_FW_PIN, LOW);
          digitalWrite(RIGHT_RV_PIN, LOW);
          analogWrite(RIGHT_PWM_PIN, 0);
        }
    }
  }
      /*case 'D':
        //Return track distance travelled
        if (code_1 == 'R') Serial.println(distance_right);
        else if (code_1 == 'L') Serial.println(distance_left);
        else Serial.println("second char not recognised");
        break;
      case 'V':
        //Return current track velocity
        if (code_1 == 'R') Serial.println(velocity_right);
        else if (code_1 == 'L') Serial.println(velocity_left);
        else Serial.println("Second char not recognised");
        break;
      default:
        Serial.println("Incorrect Input, drive speed: __ L/R 0-5-9, get distance: __ D R/L, get velocity: __ V R/L");*/
}

///////////////////////////////////////////////
//  Left Encoder incrimenting/decrementing
///////////////////////////////////////////////
ISR (PCINT0_vect)
{
  cli();
  // Interrupt on phase A pin
  if (digitalRead(RIGHT_FRONT_ENC_PIN_A) == digitalRead(RIGHT_FRONT_ENC_PIN_B)) distance_right -= DIST_PER_ENC_TICK;
  else distance_right += DIST_PER_ENC_TICK;
  Serial.println("hello from right enc!");
  sei();
}

///////////////////////////////////////////////
//  Right Encoder incrimenting/decrementing
///////////////////////////////////////////////
ISR (PCINT1_vect)
{
  cli();
  // Interrupt on phase A pin
  if (digitalRead(LEFT_FRONT_ENC_PIN_A) == digitalRead(LEFT_FRONT_ENC_PIN_B)) distance_left -= DIST_PER_ENC_TICK;
  else distance_left += DIST_PER_ENC_TICK;
  Serial.println("hello from left enc!");
  sei();
}
