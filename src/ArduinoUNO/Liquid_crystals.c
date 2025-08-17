#include <LiquidCrystal.h>

const int rs = 2, 
          en = 3, 
          d4 = 6, 
          d5 = 7, 
          d6 = 8, 
          d7 = 9;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  lcd.begin(16, 2);
  Serial.begin(115200);
  lcd.setCursor(0, 0);
  lcd.print("Hello world");
}
