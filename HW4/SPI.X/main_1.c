#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON// allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7
#define SDI LATBbits.LATB8

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void ram_init() {
  // set up the chip select pin as an output
  // the chip select pin is used by the sram to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISBbits.TRISB7 = 0;
  RPB8Rbits.RPB8R = 0b011;
  //RPB7Rbits.RPB7R = 0b11;
  CS = 1;

  // Master - SPI1, pins are: SDI4(F4), SDO4(F5), SCK4(F13).  
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 4

                            // send a ram set status command.
                 // finish the command
}

void ram_write(unsigned short addr, const char data[], int len) {
  int i = 0;
  CS = 0;                        // enable the ram by lowering the chip select line
  spi_io(0x2);                   // sequential write operation
  spi_io((addr & 0xFF00) >> 8 ); // most significant byte of address
  spi_io(addr & 0x00FF);         // the least significant address byte
  for(i = 0; i < len; ++i) {
    spi_io(data[i]);
  }
  CS = 1;                        // raise the chip select line, ending communication
}

void ram_read(unsigned short addr, char data[], int len) {
  int i = 0;
  CS = 0;
  spi_io(0x3);                   // ram read operation
  spi_io((addr & 0xFF00) >> 8);  // most significant address byte
  spi_io(addr & 0x00FF);         // least significant address byte
  for(i = 0; i < len; ++i) {
    data[i] = spi_io(0);         // read in the data
  }
  CS = 1;
}
void set_voltage(unsigned int channel, unsigned int voltage){
    //format: channel bit, 111, data bits, 0000
    /*unsigned short data = 0b0111000000000000 | (channel <<15);
    data |= (voltage << 4);
    CS = 0;
    spi_io(data>>8);
    spi_io((data<<8)>>8);
    CS = 1;*/
    unsigned char bigdata = 0b01110000;
    bigdata |= (channel << 7);
    bigdata |= (voltage >> 4);
    unsigned char littledata = 0b0;
    littledata  |= ((voltage && 0b1111)<<4);
    CS = 0;
    spi_io(bigdata);
    spi_io(littledata);
    CS = 1;
    
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    ram_init(); 
    // do your TRIS and LAT commands here
    //TRISBCLR = 0b111;
    //TRISBSET = 0b1000;
    unsigned int sinwave[100];
    unsigned int sawwave[100];
    int i = 0;
    for (i; i<100;++i){
        sawwave[i] = (unsigned int) i * 255.0 / 100.0 ;
        sinwave[i] = (unsigned int) 255.0 /2.0 +255/2 * sin(2.0 * i * 3.14159 /100.0);
    }
    
    
     _CP0_SET_COUNT(0);
     unsigned int count = 0;
    while(1) {
        while(_CP0_GET_COUNT() < 24000){
            ;
        }
        _CP0_SET_COUNT(0);
        set_voltage(0, sinwave[count]);
        set_voltage(1, sawwave[count]);
        ++count;
        if(count >= 100){
            count = 0;
        }
    }
}
