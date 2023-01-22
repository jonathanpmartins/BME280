#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

class LCD_I2C : public LiquidCrystal_I2C {

  public:

    LCD_I2C(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows) 
      : LiquidCrystal_I2C (lcd_Addr, lcd_cols, lcd_rows ) {
    }

    void printOnLcd(const char * string, int column, int row) {
      LCD_I2C::clear();
      LCD_I2C::setCursor(column, row);
      LCD_I2C::print(string);
    }

    void printOnCursor(int x, int y, String string){
        LCD_I2C::setCursor(x, y);
        LCD_I2C::print(string);
    }

    void firstLine(String string) {
        LCD_I2C::printOnCursor(0, 0, string);
    }

    void secondLine(String string) {
        LCD_I2C::printOnCursor(0, 1, string);
    }  
};