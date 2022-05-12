// dvdrom_meer40t version 0.01
// 2022-05-12 
// Author : Danijel Ptičar


#include "serial.h"

float steps_in_mm = 108;   // steps in mm (set this value to match your steppers)

// in normal mode
#define startMoveSpeed  300
#define endMoveSpeed    100
#define acc             1

uint8_t acrc = 0;     // used in debuging 

// set in speedcode command
uint16_t  CUTENGRAVE_SPEED = 4000;
uint16_t  CUT_SPEED_M = 5044;     // diagonal speed -> 0.261

char speedcode[20]   = "1721821020000211";
uint16_t scode;
uint8_t accel;
uint8_t step_value;
uint16_t dia_corr;
uint8_t suffix_c = 0;
uint8_t raster_step = 3;
float speed_in_mms = 5.0;

#define bit(n) (1 << n)

#define LASER_PIN   3         // D11
#define LASER_PORT  PORTB

#define LASER_ON()  LASER_PORT |=bit(LASER_PIN)
#define LASER_OFF()  LASER_PORT &= (~bit(LASER_PIN))

#define Y_DIR_PIN   3       // D3
#define Y_DIR_PORT   PORTD
#define Y_STEP_PIN  6       //  D6
#define Y_STEP_PORT PORTD

#define X_DIR_PIN   2       // D2
#define X_DIR_PORT   PORTD
#define X_STEP_PIN  5       // D5
#define X_STEP_PORT  PORTD

#define XY_STEP_PORT PORTD
#define XY_DIR_PORT PORTD

#define RAIL_PIN    0         // D8
#define RAIL_PORT   PORTB

uint16_t X_mag = 0;
uint16_t Y_mag = 0;

volatile  uint16_t   isrX_mag = 0;      // TB
volatile  uint16_t   isrY_mag = 0;      // LR
volatile  uint8_t    isrBusy = 0;

//#define lockRails_pin   8   // 0 = enable
//#define laser_pin       11

#define UNLOCK_RAILS()    RAIL_PORT|=bit(RAIL_PIN)
#define LOCK_RAILS()      RAIL_PORT&=~bit(RAIL_PIN)

#define Y_step_HIGH()   Y_STEP_PORT|=bit(Y_STEP_PIN)
#define Y_step_LOW()    Y_STEP_PORT&=~bit(Y_STEP_PIN)
#define X_step_HIGH()   X_STEP_PORT|=bit(X_STEP_PIN)
#define X_step_LOW()    X_STEP_PORT&=~bit(X_STEP_PIN)

#define X_dir_LEFT()     X_DIR_PORT|=bit(X_DIR_PIN)   
#define X_dir_RIGHT()      X_DIR_PORT&=~bit(X_DIR_PIN)  
#define Y_dir_DOWN()      Y_DIR_PORT|=bit(Y_DIR_PIN)
#define Y_dir_UP()        Y_DIR_PORT&=~bit(Y_DIR_PIN)

#define X_dir_INVERT()      X_DIR_PORT^=bit(X_DIR_PIN)   
#define Y_dir_INVERT()      Y_DIR_PORT^=bit(Y_DIR_PIN)   

#define XY_step_HIGH()  XY_STEP_PORT|=bit(X_STEP_PIN) | bit(Y_STEP_PIN)
#define XY_step_LOW()   XY_STEP_PORT&=~( bit(X_STEP_PIN) | bit(Y_STEP_PIN) )

#define LED13_ON()  PORTB |= bit(5)
#define LED13_OFF()  PORTB &= ~bit(5)

#define L_bit  0        // 1=(L)Up   0=(R)Down
#define T_bit  1        // 1=(T)Left 0=(B)Right
#define LR_bit 2        // last axis Y  -> can be in one bit, its LR or TB
#define TB_bit 3        // last axis X  -> can be in one bit, its LR or TB
#define  M_bit 4        // 
volatile uint8_t dirStatus = 0;

inline void setLflag()
{
  dirStatus |= bit(L_bit);
  dirStatus |= bit(LR_bit);
  dirStatus &= ~bit(TB_bit);
  dirStatus &= ~bit(M_bit);
}

inline void setRflag()
{
  dirStatus &= ~bit(L_bit);
  dirStatus |= bit(LR_bit);
  dirStatus &= ~bit(TB_bit);
  dirStatus &= ~bit(M_bit);
}

inline void setTflag()
{
  dirStatus |= bit(T_bit);
  dirStatus |= bit(TB_bit);
  dirStatus &= ~bit(LR_bit);
  dirStatus &= ~bit(M_bit);
}

inline void setBflag()
{
  dirStatus &= ~bit(T_bit);
  dirStatus |= bit(TB_bit);
  dirStatus &= ~bit(LR_bit);
  dirStatus &= ~bit(M_bit);
}

inline void setMflag()
{
  dirStatus |= bit(M_bit);
}




#define issetLRflag() !!(dirStatus & bit(LR_bit))
#define issetTBflag() !!(dirStatus & bit(TB_bit))

#define issetLflag()   (dirStatus & bit(L_bit))
#define issetTflag() !!(dirStatus & bit(T_bit))

#define issetMflag()   (dirStatus & bit(M_bit))


#define EN_bit      0 // 0=Normal mode(N),  1=compact mode (E) 
#define CG_bit      1 // 1=CUT,  0=G(engrave)
#define LASER_bit   2 // 1=on 0=0ff

volatile uint8_t boardStatus = 0x00;

void setDflag()
{
  boardStatus |= bit(LASER_bit);
}

void setUflag()
{
  boardStatus &= ~bit(LASER_bit);
}

// cut
void setCflag()
{
  boardStatus |= bit(CG_bit);
}

// engrave
void setGflag()
{
  boardStatus &= ~bit(CG_bit);
}

void setEflag()
{
  boardStatus |= bit(EN_bit);
}

void setNflag()
{
  boardStatus &= ~bit(EN_bit);
}

#define issetCflag() !!(boardStatus & bit(CG_bit))
#define issetEflag() !!(boardStatus & bit(EN_bit))
#define issetNflag() !(boardStatus & bit(EN_bit))

#define issetDflag() !!(boardStatus & bit(LASER_bit))
#define issetUflag() !(boardStatus & bit(LASER_bit))

// 0->A->B->OVF->0->A->...

ISR(TIMER1_OVF_vect)
{

  if (isrBusy == 0)
  {
    if (!issetCflag() && issetEflag() ) LASER_OFF();  // engrave. mode . buff underflow ?
    return; 
  }
  if (isrX_mag > 0 && isrY_mag > 0)
  {

    XY_step_HIGH();
    if (issetDflag()) LASER_ON();
    else LASER_OFF();
    isrX_mag--;
    isrY_mag--;
    if (isrX_mag == 0 && isrY_mag == 0) isrBusy = 0;
    XY_step_LOW();   // 1us is min. step puls width for A4988
    return;
  }

  if (isrX_mag > 0)
  {

    X_step_HIGH();
    if (issetDflag()) LASER_ON();
    else LASER_OFF();
    isrX_mag--;
    if (isrX_mag == 0) isrBusy = 0;
    XY_step_LOW();
    return;
  }

  if (isrY_mag > 0)
  {

    Y_step_HIGH();
    if (issetDflag()) LASER_ON();
    else LASER_OFF();
    isrY_mag--;
    if (isrY_mag == 0) isrBusy = 0;
    XY_step_LOW();
    return;
  }

  isrBusy = 0;
}

ISR(TIMER1_COMPA_vect)
{
  /*
    if (isrBusy == 0) return;

    if (isrX_mag == 0 && isrY_mag == 0) {
    isrBusy = 0;
    return;
    };
  */
}

// not used
ISR(TIMER1_COMPB_vect)
{

}

void setup()
{
  DDRB = 0x00;
  DDRC = 0x00;
  DDRD = 0x00;
  
  DDRB = DDRB | bit(5);           //  D13 LED
  DDRB = DDRB | bit(RAIL_PIN);    //  D8
  DDRB = DDRB | bit(LASER_PIN);   //  D11

  DDRD = DDRD | bit(Y_DIR_PIN);   //  D3
  DDRD = DDRD | bit(Y_STEP_PIN);  //  D6
  DDRD = DDRD | bit(X_DIR_PIN);   //  D2
  DDRD = DDRD | bit(X_STEP_PIN);  //  D5

  for (int j = 0; j < 3; j++)
  {
    //for (uint32_t i = 0; i < 260000; i++) LED13_ON();
    //for (uint32_t i = 0; i < 260000; i++) LED13_OFF();
  }


  // disabe timer0 and isr's(delay, millis,  etc...)
  TCCR0A = 0;  TCCR0B = 0;  TIMSK0 = 0;



  TCCR1B = 0;     // stop timer
  TCNT1 = 0;
  TCCR1A =   bit(WGM11) ; //  D9,D10 normal pins
  TIMSK1 =  bit(TOIE1) ; // | bit(OCIE1A) | bit(OCIE1B)
  TCCR1B =  bit(CS11) | bit(WGM13) | bit(WGM12); // WGM=1110 , OVF on TOP, prescaler 8
  //OCR1A =  31;      
  //OCR1B =  100;     
  ICR1 = CUTENGRAVE_SPEED;

  serial_init();
  sei();
  acrc=0;
}

uint16_t tmpMag = 0;  
uint8_t  specialSymbol = 0;
uint8_t digitCounter = 0;
uint8_t byteData = 0;


//--------------------------------LOOP-------------------------------------------
void loop()
{
  if (issetNflag()) normalMode();
  else if (issetCflag()) cutMode();
  else engraveMode();
}



// not time sensitive, done without isr 
void executeNormal()
{
  // disable timer isr ?

  uint16_t _x_mag_stop, _x_mag_start;
  uint16_t _y_mag_stop, _y_mag_start;

  if ( (X_mag >> 1) < (startMoveSpeed - endMoveSpeed) / acc)
  {
    _x_mag_stop = (X_mag >> 1);
    _x_mag_start = (X_mag >> 1);
  } else
  {
    _x_mag_stop = X_mag - (startMoveSpeed - endMoveSpeed) / acc;
    _x_mag_start = (startMoveSpeed - endMoveSpeed) / acc;
  }

  if ( (Y_mag >> 1) < (startMoveSpeed - endMoveSpeed) / acc)
  {
    _y_mag_stop = (Y_mag >> 1);
    _y_mag_start = (Y_mag >> 1);
  } else
  {
    _y_mag_stop = Y_mag - (startMoveSpeed - endMoveSpeed) / acc;
    _y_mag_start = (startMoveSpeed - endMoveSpeed) / acc;
  }

  uint16_t currentSpeedX = startMoveSpeed;   
  uint16_t currentSpeedY = startMoveSpeed;
  uint16_t Xcounter = 0;
  uint16_t Ycounter = 0;
  while (X_mag > 0 || Y_mag > 0)
  {
    if (X_mag > 0) if (Xcounter == 0) X_step_HIGH();
    if (Y_mag > 0) if (Ycounter == 0) Y_step_HIGH();

    if (X_mag > 0) if (Xcounter == 2) X_step_LOW(), X_mag--;
    if (Y_mag > 0) if (Ycounter == 2) Y_step_LOW(), Y_mag--;

    Xcounter++;
    Ycounter++;

    if (X_mag > 0)
    {
      if (Xcounter >= currentSpeedX)
      {
        Xcounter = 0;
        if (X_mag > _x_mag_stop) currentSpeedX -= acc;
        else if (X_mag <= _x_mag_start) currentSpeedX += acc;
      }
    } else {
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
    }

    if (Y_mag > 0)
    {
      if (Ycounter >= currentSpeedY)
      {
        Ycounter = 0;
        if (Y_mag > _y_mag_stop) currentSpeedY -= acc;
        else if (Y_mag <= _y_mag_start) currentSpeedY += acc;
      }
    } else {
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
    }

    asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP");
  }

  for (int i = 0; i < 2000; i++) asm("NOP"); // some dalay after fast move

  // enable timer, reset interupt flags ??

}



//---------------------------------- N mode ------------------------------------
void normalMode()
{
  while (issetNflag())
  {
    while (serial_available() > 0)
    {
      if (isrBusy) continue;

      uint8_t c = serial_read();

      if (c == '|')
      {
        tmpMag += 25;
        specialSymbol = 1;
      }

      if (c >= 'a' && c <= 'z')
      {
        tmpMag += c - 'a' + 1;
        if (c == 'z' && !specialSymbol) tmpMag += 255 - 26;
        specialSymbol = 0;
        //continue;
      }

      if (c >= '0' && c <= '9') {
        byteData = byteData * 10 + (c - '0');
        digitCounter++;
        if (digitCounter == 3) tmpMag += byteData, byteData = 0, digitCounter = 0;
      }

      if (c == 'L')
      {
        Y_dir_UP();
        if (issetTBflag())
        {
          X_mag = +tmpMag;
          tmpMag = 0;
        }
        setLflag();
      }
      
      if (c == 'R')
      {
        Y_dir_DOWN();
        if (issetTBflag())
        {
          X_mag = +tmpMag;
          tmpMag = 0;
        }
        setRflag();
      }

      if (c == 'T')
      {
        X_dir_LEFT();
        if (issetLRflag())
        {
          Y_mag += tmpMag;
          tmpMag = 0;
        }
        setTflag();
        //continue;
      }
      
      if (c == 'B')
      {
        X_dir_RIGHT();
        if (issetLRflag())
        {
          Y_mag += tmpMag;
          if (issetLflag())
          {
            //Y_mag *= -1; // ???
          }
          tmpMag = 0;

        }
        setBflag();
        //continue;
      }

      if (c == 'N')
      {
        tmpMag = 0;
        //executeNormal();
      }

      if (c == 'S')
      {
        while (serial_available() == 0) ;
        uint8_t c_peek = serial_peek();
        if (c_peek == '1')
        {
          LOCK_RAILS();
          serial_read();
        } else if (c_peek == '2')
        {
          UNLOCK_RAILS();
          serial_read();
        }
        if (issetLRflag())
        {
          Y_mag += tmpMag;
        }
        if (issetTBflag())
        {
          X_mag += tmpMag;
        }
        tmpMag = 0;
        executeNormal();
        return;
      }
      if (c == 'I')
      {
        X_mag = 0;
        Y_mag = 0;
        tmpMag = 0;
        boardStatus = 0x00;
        return;
      }
      if (c == 'C')
      {
        setCflag();
      }
      if (c == 'V')
      {
        if (issetCflag()) loadCSpeedCode();
        else loadGSpeedCode();
        calc_speedcode();
        calc_ICR1_values();
        return;
      }
      if (c == 'E')
      {
        setEflag();
        return;
      }
      if (c == 'D')
      {
        setDflag();  
        LASER_ON();
      }
      if (c == 'U')
      {
        setUflag();
        LASER_OFF();
      }
    }
  }
}


//---------------------------------------------ENGRAVE MODE-----

void engraveMode()
{
  while (!issetNflag())
  {
    while (serial_available() > 0)
    {
      uint8_t c = serial_read();

      if (c == '|')
      {
        tmpMag += 25;
        specialSymbol = 1;
      }

      if (c >= 'a' && c <= 'z')
      {
        tmpMag += c - 'a' + 1;
        if (c == 'z' && !specialSymbol) tmpMag += 255 - 26;
        specialSymbol = 0;
      }

      if (c >= '0' && c <= '9') {
        byteData = byteData * 10 + (c - '0');
        digitCounter++;
        while (digitCounter != 3)
        {
          if (serial_available() != 0)
          {
            c = serial_read();
            byteData = byteData * 10 + (c - '0');
            digitCounter++;
          }
        }
        tmpMag = byteData, byteData = 0, digitCounter = 0;
      }

      if (tmpMag != 0)
      {
        if (issetLRflag()) Y_mag += tmpMag;   // prepare mag.
        else if (issetTBflag()) X_mag += tmpMag;
        tmpMag = 0;

        while (isrBusy);    // wait last step pulse

        cli();
        isrX_mag = X_mag;
        isrY_mag = Y_mag;
        ICR1 = CUTENGRAVE_SPEED;  
        isrBusy = 1;
        sei();
        X_mag = 0;
        Y_mag = 0;
        continue;
      }

      if (c == 'L')   // Up
      {
        setLflag();

        while (isrBusy);  
        
        setUflag();       
        Y_dir_UP();
        X_mag = raster_step; //   direction set in normal mode after speedcode
        cli();
        isrX_mag = X_mag;
        isrY_mag = 0;
        ICR1 = CUTENGRAVE_SPEED;

        isrBusy = 1;
        sei();
        X_mag = 0;
        Y_mag = 0;

      }
      
      if (c == 'R')  // Down
      {
        setRflag();

        while (isrBusy);
        
        setUflag();       
        Y_dir_DOWN();
        X_mag = raster_step; // direction set in normal mode after speedcode
        cli();
        isrX_mag = X_mag;
        isrY_mag = 0;
        ICR1 = CUTENGRAVE_SPEED;
        isrBusy = 1;
        sei();
        X_mag = 0;
        Y_mag = 0;
      }

      if (c == 'T') // Left
      {
        setTflag();
        
        while (isrBusy);

        setUflag();       
        X_dir_LEFT();
        Y_mag = raster_step; // direction set in normal mode after speedcode
        cli();
        isrX_mag = 0;
        isrY_mag = Y_mag;
        ICR1 = CUTENGRAVE_SPEED;
        isrBusy = 1;
        sei();
        X_mag = 0;
        Y_mag = 0;

      }
      
      if (c == 'B') // Right
      {
        setBflag();
       
        while (isrBusy);
       
        setUflag();
        X_dir_RIGHT();
        Y_mag = raster_step;
        cli();
        isrX_mag = 0;
        isrY_mag = Y_mag;
        ICR1 = CUTENGRAVE_SPEED;
        isrBusy = 1;
        sei();
        X_mag = 0;
        Y_mag = 0;
      }

      while (isrBusy) ;
      
      if (c == 'D')
      {
        setDflag();
        continue;
      }
      if (c == 'U')
      {
        setUflag();
        continue;
      }

      if (c == 'N')
      {
        setNflag();
        return;
      }

      if (c == 'F' || c == '@')
      {

        cli();
        isrBusy = 1;
        sei();
        while (isrBusy) ;
        setUflag();
        LASER_OFF();
      }

      if (c == 'I')
      {
        X_mag = 0;
        Y_mag = 0;
        tmpMag = 0;
        boardStatus = 0x00;
        dirStatus = 0x00;
        return;
      }
    }
  }
}

//---------------------------------- C mode ------------------------------------

void cutMode()
{
  while (issetCflag())
  {
    while (serial_available() > 0)
    {
      uint8_t c = serial_read();

      if (c == '|')
      {
        tmpMag += 25;
        specialSymbol = 1;
      }

      if (c >= 'a' && c <= 'z')
      {
        tmpMag += c - 'a' + 1;
        if (c == 'z' && !specialSymbol) tmpMag += 255 - 26;
        specialSymbol = 0;

      }  // else if

      if (c >= '0' && c <= '9') {
        byteData = byteData * 10 + (c - '0');
        digitCounter++;
        while (digitCounter != 3)
        {
          if (serial_available() != 0)
          {
            c = serial_read();
            byteData = byteData * 10 + (c - '0');
            digitCounter++;
          }
        }
        tmpMag = byteData, byteData = 0, digitCounter = 0;
      }

      if (tmpMag != 0)
      {
        if (issetMflag()) X_mag += tmpMag, Y_mag += tmpMag;
        else if (issetLRflag()) Y_mag += tmpMag;
        else if (issetTBflag()) X_mag += tmpMag;      // no need for if here
        tmpMag = 0;
        uint16_t workingSpeed = CUTENGRAVE_SPEED;
        if (issetMflag()) workingSpeed = CUT_SPEED_M;
        
        while (isrBusy) ;
        
        if (issetLflag()) Y_dir_UP();
        else  Y_dir_DOWN();

        if (issetTflag()) X_dir_LEFT();
        else X_dir_RIGHT();

        cli();
        isrX_mag = X_mag;
        isrY_mag = Y_mag;
        ICR1 = workingSpeed;
        isrBusy = 1;
        sei();
        X_mag = 0;
        Y_mag = 0;
        continue;

      }

      if (c == 'L')
      {
        setLflag();
        continue;
      }
      if (c == 'R')
      {
        setRflag();
        continue;
      }

      if (c == 'T')
      {
        setTflag();
        continue;
      }
      if (c == 'B')
      {
        setBflag();
        continue;
      }

      if (c == 'M')
      {
        setMflag();
        continue;
      }
      while (isrBusy) ;

      if (c == 'D')
      {
        setDflag();
        continue;
      }
      if (c == 'U')
      {
        setUflag();
        continue;
      }

      if (c == 'F' || c == '@')
      {
        cli();
        isrBusy = 1; // wait one more period
        sei();
        while (isrBusy) ;
        setUflag();
        LASER_OFF();
        return;
      }

      if (c == 'N')
      {
        cli();
        isrBusy = 1; // wait one more period
        sei();
        while (isrBusy) ;
        setUflag();
        LASER_OFF();
        setNflag();
        return;
      }

      if (c == 'I')
      {
        X_mag = 0;
        Y_mag = 0;
        tmpMag = 0;
        boardStatus = 0x00;
        dirStatus = 0x00;
        setUflag();
        LASER_OFF();
        return;
      }
    }
  }
}

// per_in_us/1000000 * 16000000/8 - 1 = per_in_us*2 -1    (for prescaler 8)
void calc_ICR1_values()
{
  float tmr_period_us = 1 / (speed_in_mms * steps_in_mm) * 1000000.0;
  CUTENGRAVE_SPEED = (2.0 * tmr_period_us - 1.0);
  CUT_SPEED_M = (2.0 * tmr_period_us - 1.0) * 1.261;
}

void calculateSpeed()
{
  float b = 5120;
  if (accel == 3) b = 5632;
  if (accel == 4) b = 6144;
  float m = 12120;
  if (suffix_c)
  {
    b = 8;
    m = m / 12;
  }
  float period_in_ms = (scode - b) / m;
  float frequency_kHz = 1 / period_in_ms;
  speed_in_mms = frequency_kHz * 25.4;
}

void calc_speedcode()
{
  scode = (uint16_t)decodeNumber(speedcode) * 256 + decodeNumber(speedcode + 3);
  scode = - scode;
  accel = speedcode[6] - '0';
  suffix_c = 0;
  if (speedcode[7] != 'G')
  {
    step_value = decodeNumber(speedcode + 7);
    dia_corr = (uint16_t)decodeNumber(speedcode + 10) * 256 + decodeNumber(speedcode + 10 + 3);
    suffix_c = speedcode[16] == 'C';
  }
  if (speedcode[7] == 'G')   raster_step = decodeNumber(speedcode + 8);

  calculateSpeed();
}

void loadCSpeedCode()
{
  uint8_t cnt = 0;
  while (cnt != 16)
  {
    if (serial_available() > 0) speedcode[cnt++] = serial_read();
  }
  while (serial_available() == 0) ;
  char c_peek = serial_peek();
  if (c_peek == 'C') speedcode[cnt++] = serial_read();
  speedcode[cnt] = 0;
}

void loadGSpeedCode()
{
  uint8_t cnt = 0;
  while (cnt != 11)
  {
    if (serial_available() > 0) speedcode[cnt++] = serial_read();
  }
  while (serial_available() == 0) ;
  char c_peek = serial_peek();
  if (c_peek == 'G')
    while (cnt != 15)
    {
      if (serial_available() > 0) speedcode[cnt++] = serial_read();
    }
  speedcode[cnt] = 0;
}

uint8_t decodeNumber(char *c)
{
  return (c[0] - '0') * 100 + (c[1] - '0') * 10 + (c[2] - '0') * 1;
}
