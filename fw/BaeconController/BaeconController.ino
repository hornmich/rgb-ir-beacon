#define F_CPU 8000000
#include <IRremote.h>
#include <IRremoteInt.h>

IRrecv receiver(3);
decode_results results;
int actColor, lastColor;

void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);  /* R */
  pinMode(1, OUTPUT);  /* G */
  pinMode(2, OUTPUT);  /* B */
  actColor = 0;
  lastColor = 0;
  receiver.enableIRIn();
  digitalWrite(1, HIGH);
  delay(5000);
  digitalWrite(1, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(receiver.decode(&results)) {
    switch (results.value) {
    case 0xFF906F:
      /* R On */
      digitalWrite(0, HIGH);
      delay(100);
      digitalWrite(0, LOW);
      actColor = 0x1;
      break;
    case 0xFF10EF:
      /* G On */
      digitalWrite(1, HIGH);
      delay(100);
      digitalWrite(1, LOW);
      actColor = 0x2;
      break;
    case 0xFF50AF:
      /* B On */
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      actColor = 0x4;
      break;
    case 0xFF609F:
      /* Off */
      if (actColor != lastColor) {
        lastColor = actColor;
        actColor = 0;
      }
      break;
    case 0xFFE01F:
      /* On */
      actColor = lastColor;
      break;
    default:
      digitalWrite(0, HIGH);
      delay(50);
      digitalWrite(0, LOW);
      delay(100);
      digitalWrite(0, HIGH);
      delay(50);
      digitalWrite(0, LOW);
    }

    digitalWrite(0, (actColor & 0x1) ? HIGH : LOW);
    digitalWrite(1, (actColor & 0x2) ? HIGH : LOW);
    digitalWrite(2, (actColor & 0x4) ? HIGH : LOW);
   
    receiver.resume();
  }
}
