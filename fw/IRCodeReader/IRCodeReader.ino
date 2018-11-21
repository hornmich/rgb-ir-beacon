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
  Serial.begin(9600);
  actColor = 0;
  lastColor = 0;
  receiver.enableIRIn();
}

void loop() {
  // put your main code here, to run repeatedly:

  if(receiver.decode(&results)) {
    Serial.println(results.value, HEX);
    switch (results.value) {
    case 0xFF906F:
      /* R On */
      actColor = 0x1;
      break;
    case 0xFF10EF:
      /* G On */
      actColor = 0x2;
      break;
    case 0xFF50AF:
      /* B On */
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
      actColor = 0xFF;
    }
    receiver.resume();
    Serial.println(actColor, HEX);
  }
}
