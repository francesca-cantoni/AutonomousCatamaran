// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>
#include "parser.h"
#include <stdio.h>
#include <string.h>
#include <p30F4011.h>

// Define //
#define CONTROLLED 0
#define TIMEOUT 1
#define SAFE 2
#define MAX_TASKS 9
#define BUFFER_SIZE 100   // 9600/10 = 960 bytes per second
                          // 960/10 = 96 bytes per 100 ms
#define HB 20   // Heartbeat = 20ms (minimum value among all the periods of the tasks executed in the scheduler)

// Heartbeat structure //
typedef struct{
    int n;  // Internal incrementation
    int N;  // Number of heartbeats needed between two consecutive executions of the task
}heartbeat;

// Circular buffer //
typedef struct{
    char buffer[BUFFER_SIZE];
    int readIndex;
    int writeIndex;
}CircularBuffer;

// Global Variables //
int flagError = 0;  // 0-->correct functioning, 1-->problem (overflow or 
                    //                          overcoming of Heartbeat period by one task)
int currentState = 0;   // From 0 to 2 depending on the state
int countTemp = 0;  // Increment until 10 samples
float sumTemp = 0;  // Sum for the previous sample of temperature
float averTemp = 0; // Average temperature
int display = 0;    // Current configuration of the display
int previousS6; // Previous state for the button S6
int rpm1 = 0;   // rpm for motor 1 (left)
int rpm2 = 0;   // rpm for motor 2 (right)
int maxRPM = 8000;    // Limited by the propeller
int minRPM = -8000;   // Limited by the propeller
double halfDutyCycle = 1842.0;  // DC 50% = PTPER = 0 rpm
volatile CircularBuffer cbRX;     // Circular buffer for store char received via UART
char valueRX = ' ';
volatile CircularBuffer cbTX;     // Circular buffer for store char that has to be sent via UART
char valueTX = ' ';
int semaphore = 0;      // 0 --> none is writing in the shared resource
                        // 1--> some function is writing in the shared resource

// PARSER //
parser_state pstate = {.state = STATE_DOLLAR, .index_type = 0, .index_payload = 0};

// INITIALIZATION //
void init_GPIO();

void init_ADC();

void init_PWM();

void init_UART();

void init_SPI(int primary_prescaler,int secondary_prescaler);

// TIMER //
void choose_prescaler(int ms, int* prescaler, int* pr_counter);
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);

// DISPLAY //
void put_char_SPI(char c);
void LCD_write(char *text);
void move_cursor_begin();
void move_cursor_middle();
void clear_LCD();
void LCD_display();

// LEDS //
void blink_Leds();

// BUTTON //
void check_button();

// TEMPERATURE //
float get_SumTemperature();

// PWM //
void saturation_RPM();
void send_PWM();

// FEEDBACK TO PC //
void send_MCTEM();
void send_MCFBK();
void send_MCACK();

// PARSER //
int extract_integer(const char* str);
int next_value(const char* msg,int i);
void parse_HLREF(const char* msg);

int sat_requirements_check(int min, int max);
void parse_HLSAT(const char* msg);

// UART //
void write_buffer(volatile CircularBuffer* cb, char value);
void write_buffTX_string(char *text);
int read_bufferRX(volatile CircularBuffer* cb, char*  value);
int read_bufferTX(volatile CircularBuffer* cb, char*  value);
void read_msg();


// SCHEDULER //
// Tasks  
heartbeat schedInfo[MAX_TASKS]; 
heartbeat t1 = {.n = 0, .N = 500/HB};       // Every 500 ms (blink_Leds())
heartbeat t2 = {.n = -3, .N = 100/HB};      // Every 100 ms (get_temp())
heartbeat t3 = {.n = 0, .N = 20/HB};        // Every 20 ms (check_buttons())
heartbeat t4 = {.n = -1, .N = 100/HB};      // Every 100 ms (send duty_cycle())
heartbeat t5 = {.n = -1, .N = 100/HB};      // Every 100 ms (LCD_display())
heartbeat t6 = {.n = 0, .N = 100/HB};       // Every 100 ms (read_msg())
heartbeat t7 = {.n = 0, .N = 20/HB};        // Every 20 ms (send_MCACK())
heartbeat t8 = {.n = -2, .N = 200/HB};      // Every 200 ms (send_MCFBK())

// Function 
void scheduler();

// MAIN //
int main(void) { 
    init_GPIO();
    init_ADC();
    init_PWM();
    init_UART();
    init_SPI(3,6);      // primary prescaler --> 1:1 & secondary prescaler --> 2:1
    
    // Initialization of the timers
    int tmrHeartbeat = 1;
    int tmrUART = 2;
    tmr_wait_ms(tmrHeartbeat, 1000);     // Wait 1 s for the LCD to be ready
    tmr_setup_period(tmrHeartbeat, HB); // Every 20 ms
    
    tmr_setup_period(tmrUART, 5000); // Every 5 s
    
    // Initialization of the tasks for the scheduler
    schedInfo[1] = t1;      
    schedInfo[2] = t2;      
    schedInfo[3] = t3;       
    schedInfo[4] = t4;      
    schedInfo[5] = t5;      
    schedInfo[6] = t6;      
    schedInfo[7] = t7;
    schedInfo[8] = t8;
            
    T2CONbits.TON = 1;  // Starting tmr2 (tmrUART)  for TIMEOUT mode 
    T1CONbits.TON = 1;  // Starting tmr1 (tmrHeartbeat) for the hearbeat of the scheduler 
    
    while(1){       // To avoid the resets of the PIC
        scheduler();
        tmr_wait_period(tmrHeartbeat);
    }
    
    return 0;
}

// FUNCTION DECLARATION //
// GPIO //
void init_GPIO(){
    TRISBbits.TRISB0 = 0;       // Define RB0 (led D3) as output
    LATBbits.LATB0 = 0;         // Set the pin low   
    TRISBbits.TRISB1 = 0;       // Define RB1 (led D4) as output
    LATBbits.LATB1 = 0;         // Set the pin low  
    TRISDbits.TRISD0 = 1;       // Define RD0 (button S6) as input
    TRISEbits.TRISE8 = 1;       // Define RE8 (button S5) as input 
    IEC0bits.INT0IE = 1;        // Enable S5 (same pin as INT0) interrupt
}

// ADC //
void init_ADC(){
    ADCON1bits.ASAM = 1;    // Automatic start of the sampling period (at the end of the previous conversion)
    ADCON3bits.SAMC = 15;   // Sample time 15 Tad
    ADCON1bits.SSRC = 7;    // Automatic end of the sampling period (after time specified by SAMC) 
                            // and start of the conversion  
    ADCON2bits.CHPS = 0;    // Selection of CH0
    ADCON3bits.ADCS = 63;   // Duration of one Tad (taken as long as possible --> 111111 = 32*TCY = 63)
    ADCHSbits.CH0SA = 3;    // Select AN3 as positive input of the opamp related to CH0
    ADCHSbits.CH0NA = 0;    // Select Vref- as negative input of the opamp related to CH0
    ADPCFG = 0xFFFF;        // A/D Port Configuration Register 
    ADPCFGbits.PCFG3 = 0;   // Analog input pin AN3 in Analog mode, port read input disabled, A/D samples pin voltage
    ADCON2bits.SMPI = 0000; // Interrupt flag (IF) high after one conversion
    ADCON1bits.ADON = 1;    // Turn ADC ON
}

// PWM // 
void init_PWM(){
    PWMCON1bits.PEN1H = 1;  // PWM1H pin is enabled for PWM output (motor 1 --> left)
    PWMCON1bits.PEN2H = 1;  // PWM2H pin is enabled for PWM output (motor 2 --> right)
    PTPER = 1842;           // PTPER = [(Fcy/Fpwm*( prescaler))] - 1    with Fpwm = 1 kHz
                            // the result is encodable in 15 bits (2^15=32768) so prescaler 1:1 
    PTCONbits.PTMOD = 0;    // Free running mode
    PTCONbits.PTCKPS = 0;   // 1:1 Fcy Prescaler
    PTCONbits.PTOPS = 0;    // 1:1  post-scaler
    PTCONbits.PTSIDL = 0;   // pwm timer runs in idle mode
    PTCONbits.PTEN = 1;     // PWM time base is ON
} // Initialization of PWM with the corresponding pwm frequency

// UART //
void init_UART(){
    U2BRG = 11;     // [(7372800/4)/(16*9600)] - 1 = 11
    U2MODEbits.UARTEN = 1;  // Enable UART2
    U2STAbits.UTXEN = 1;    // Enable transmission via U2TX
    U2STAbits.URXISEL = 0;  // Interrupt flag is generated when a char is received 
    IEC0bits.T2IE = 1;  //Enable interrupt for tmr2 (tmrUART) 
    IEC1bits.U2RXIE = 1; //Enable interupt for UART
} // Setup UART2 communication at 9600 baudrate

// SPI //
void init_SPI(int primary_prescaler,int secondary_prescaler){  
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode
    SPI1CONbits.PPRE = primary_prescaler; 
    SPI1CONbits.SPRE = secondary_prescaler; 
    SPI1STATbits.SPIEN = 1; // enable SPI
} // Init SPI communication


// TIMER //
void choose_prescaler(int ms, int* prescaler, int* pr_counter){
    // fcy=(fosc/4) = (7.372800/4) = 1843200 clocks/s
    // 1843,2 clock ticks in 1 ms
    
    long ticks = 1843.2*ms;
    // if ticks is > 65535 it cannot be put in PR1 (only 16 bits )
    
    // no prescaler 
    if (ticks <= 65535){            // 2^(16) = 65535 max value
        *pr_counter = ticks;
        *prescaler = 0;
        return;
    }
    ticks = ticks/8;    // prescaler = 1:8    
    if (ticks <= 65535){           // 2^(16) = 65535 max value
        *pr_counter = ticks;
        *prescaler = 1;
        return;
    }
    ticks = ticks/8;   // prescaler = 1:64     
    if (ticks <= 65535){           // 2^(16) = 65535 max value
        *pr_counter = ticks;
        *prescaler = 2;
        return;
    }
    ticks = ticks/4;    // prescaler = 1:256  
    if (ticks <= 65535){           // 2^(16) = 65535 max value
        *pr_counter = ticks;
        *prescaler = 3;
        return;
    }
}
void tmr_setup_period(int timer, int ms){   
    
    // Common steps to all the timers
    int prescaler, pr_value;
    choose_prescaler(ms, &prescaler, &pr_value);
    
    switch (timer) {
        // Considering tmr1
        case 1: {
            // SAFETY WAY
            // 1) disable the counter
            T1CONbits.TON = 0;
            // 2) set the desired value of the prescaler and PRx
            T1CONbits.TCKPS = prescaler;
            PR1 = pr_value;
            // 3) reset the current value of the timer
            TMR1 = 0;
            break;
        }
        
        // Considering tmr2
        case 2: {
            // SAFETY WAY
            // 1) disable the counter
            T2CONbits.TON = 0;
            // 2) set the desired value of the prescaler and PRx
            T2CONbits.TCKPS = prescaler;
            PR2 = pr_value;
            // 3) reset the current value of the timer
            TMR2 = 0;
            break;
        }
    }
} // Setup the period of the timer
void tmr_wait_period(int timer) {
    switch (timer) {
        // Considering tmr1
        case 1: {
            if(IFS0bits.T1IF == 1){ // The timer has already expired before the end of the heartbeat
                flagError = 1;
            }
            else {
                while (IFS0bits.T1IF == 0) {
                    // T1IF becomes high only when the timer expires
                }
            }
            IFS0bits.T1IF = 0;         // Put manually down the flag to be able to 
                                       // recognize the next expiration
            break;
        }
    }
} // Function to stay synchronized with an event with the periodicity equal to the period of the timer
void tmr_wait_ms(int timer, int ms){
    // fcy=(fosc/4) = (7.372800/4) = 1843200 clocks/s

    int prescaler, pr_value;
    choose_prescaler(ms, &prescaler, &pr_value);
    
    switch (timer) {
        //considering timer 1
        case 1: {
            // SAFETY WAY
            // 1) disable the counter
            T1CONbits.TON = 0;
            // 2) set the desired value of the prescaler and PRx
            T1CONbits.TCKPS = prescaler;
            PR2 = pr_value;
            // 3) reset the current value of the timer
            TMR1 = 0;
            // 4) timer start
            T1CONbits.TON = 1;
            // 5) wait until the expiration
            while(IFS0bits.T1IF == 0)
            {
                // T1IF becomes high only when the timer expires
            }
            // 6) put down the flag
            IFS0bits.T1IF = 0;
            // 7) stop the timer
            T1CONbits.TON = 0;
            break;
        }
    }
    
}// Timer for waiting a given amount of time

void __attribute__ (( __interrupt__ , __auto_psv__ )) _T2Interrupt() {
    IFS0bits.T2IF = 0;  // Put down the flag
    T2CONbits.TON = 0;  // Disable the counter of tmr2 because we are in TIMEOUT mode
    if(currentState == CONTROLLED){
        currentState = TIMEOUT; // Enter timeout mode (more than 5 s without msg)
    }
}//INTERRUPT for tmr2 (timeout mode)


// DISPLAY //
void put_char_SPI(char c){
    while (SPI1STATbits.SPITBF == 1); // Wait for previous transmissions to finish
    SPI1BUF = c;
} // Write a single char on the LCD
void LCD_write(char *text){
    int i = 0;
    while(text[i] != '\0'){
        if (i == 15){
            while(SPI1STATbits.SPITBF == 1);    // Wait until the next transmission starts
            SPI1BUF = 0xC0;     // Second row of the display
        }
        else{
            put_char_SPI(text[i]);
            i++;
        }
    }
}   // Write a message on the LCD
void move_cursor_begin(){
    while(SPI1STATbits.SPITBF == 1);    // Wait until the next transmission starts
    SPI1BUF = 0x80;
}   // Move the cursor to the first element of the first row of the LCD
void move_cursor_middle(){
    while(SPI1STATbits.SPITBF == 1);    // Wait until the next transmission starts
    SPI1BUF = 0xC0; 
}   // Move the cursor to the first element of the second row of the LCD
void clear_LCD(){
    int i;
    move_cursor_begin();
    for(i = 0; i < 32 ; i++){   // For every cell of the LCD
        if(i == 15){
            move_cursor_middle();
        }
        while(SPI1STATbits.SPITBF == 1);    // Wait until the next transmission starts
        SPI1BUF = ' ';      // Clear one cell of the LCD screen
    }
    move_cursor_begin();
}

void LCD_display(){
    char strbuf[25];
    clear_LCD();
    
    // FIRST CONFIGURATION
    if (display == 0){
        // current status //
        switch(currentState){
            case 0: // CONTROLLED MODE
                sprintf (strbuf, "sta:C ");
                break;
            case 1: // TIMEOUT MODE
                sprintf (strbuf, "sta:T ");
                break;
            case 2: // SAFE MODE
                sprintf (strbuf, "sta:H ");
                break;
        }
        LCD_write(strbuf);
        // temperature //
        sprintf (strbuf, "temp:%3.1f", (double)averTemp);   // Float with 3 digits before the comma and 1 after
        LCD_write(strbuf);
        // applied rmp //
        move_cursor_middle();
        sprintf(strbuf,"rpm:%d,%d",rpm1,rpm2);
        LCD_write(strbuf);
    }
    
    // SECOND CONFIGURATION
    else{
        // min/max current saturation value //
        sprintf(strbuf,"sat:%d/%d",minRPM,maxRPM);
        LCD_write(strbuf);
        // duty cycle PWM registers //
        move_cursor_middle();
        sprintf (strbuf, "DC:%d,%d",PDC1,PDC2);
        LCD_write(strbuf);
    }
}

// LEDS //
void blink_Leds(){
    // led D3 behaviour
    if (flagError == 0){
        LATBbits.LATB0 = !LATBbits.LATB0;   // D3 always blink
    }
    else{
        LATBbits.LATB0 = 0; // To signal the error
    }
    // led D4 behaviour
    switch(currentState){
        case 1: // in the state "SAFE"
            LATBbits.LATB1 = !LATBbits.LATB1;   // D4 blink
            break;
        default: // in any other state 
            LATBbits.LATB1 = 0; // D4 turned off
    }
}   // Blink of the Leds accordingly to the state we are in

// BUTTONS //
void check_button(){
    // If S6 is pressed
    if(PORTDbits.RD0 == 0 && PORTDbits.RD0 != previousS6){
        display = !display;
    }
    
    previousS6 = PORTDbits.RD0;
}   // S6 has been pushed (S5 handled by an interrupt)

void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT0Interrupt() {
    IFS0bits.INT0IF = 0;        // Reset interrupt flag
    if(currentState != SAFE){
        currentState = SAFE;    // The catamaran enter the safe mode
        PDC1 = halfDutyCycle;   // Immediately put the velocity to 0
        PDC2 = halfDutyCycle;   // Immediately put the velocity to 0
        rpm1 = 0;
        rpm2 = 0;
    }
} //INTERRUPT for S5

// TEMPERATURE //
float get_SumTemperature(){
    // Since the converion is automatic, we don't have to check if there is something new to read
    int ADCValue = ADCBUF0;     // After one A/D conversion the result is written 
                                // at the beginning of the buffer so ADCBUF0
    // Conversion of the analogic value of the temperature to the temperature in degree
    float volt = ADCValue/1024.0*5.0;  // Convert the analogic value into a voltage
    sumTemp = sumTemp + (volt-0.75)/0.01 +25.0; // 10 mV/C voltage slope, 750mV at 25 degrees
    return(sumTemp);    // Convert the voltage to the corresponding temperature
}   // Automatically read the value from the ADC and convert it in temperature

// PWM //
void saturation_RPM(){
    if(rpm1 > maxRPM){rpm1 = maxRPM;}
    if(rpm1 < minRPM){rpm1 = minRPM;}
    if(rpm2 > maxRPM){rpm2 = maxRPM;}
    if(rpm2 < minRPM){rpm2 = minRPM;}
}   // Saturate the rpm according to the min and max values coming from the user
void send_PWM(){
    if(currentState == CONTROLLED){
        saturation_RPM();
        double duty1;
        double duty2;
        double maxrpm = 10000.0;   // Internal max rpm
        // formula:     (DC-50%)/(rmp-0) = (100%-50%)/(maxrpm-0)
        duty1 = halfDutyCycle*(rpm1/maxrpm) + halfDutyCycle;
        PDC1 = duty1;
        duty2 = halfDutyCycle*(rpm2/maxrpm) + halfDutyCycle;
        PDC2 = duty2;
    }
    else{   // Set velocities to 0 if TIMEOUT or SAFE (corresponding to a DC of half the maximum)
        PDC1 = halfDutyCycle;       // DC1 = 50% means rpm1 = 0 
        PDC2 = halfDutyCycle;       // DC2 = 50% means rpm2 = 0
        rpm1 = 0;
        rpm2 = 0;
    }
}   // Send the duty cycles to the motors

// FEEDBACK TO PC //
void send_MCTEM(){
    char text[25];
    int i = 0;
    sprintf(text,"$MCTEM,%3.1f*",(double)averTemp);
    while(text[i] != '\0'){
        while(U2STAbits.UTXBF == 1);    // Wait until it is possible to send again (transmit buffer is no more full)
        U2TXREG = text[i];      // Character to be written into the transmit buffer
        i++;
    }
}
void send_MCFBK(){
    char text[25];
    int i = 0;
    sprintf(text,"$MCFBK,%d,%d,%d*",rpm1,rpm2,currentState);
    while(text[i] != '\0'){
        while(U2STAbits.UTXBF == 1);    // Wait until it is possible to send again (transmit buffer is no more full)
        U2TXREG = text[i];      // Character to be written into the transmit buffer
        i++;
    }
}
void send_MCACK(){
    while(read_bufferTX(&cbTX,&valueTX) == 1){    // There is at least one unread char in the circular buffer
        while(U2STAbits.UTXBF == 1);    // Wait until you can send again (transmit buffer is no more full)
        U2TXREG = valueTX;      // Character to be sent to the transmit buffer
    }
    
}

// PARSER //
int extract_integer(const char* str) {
    int i = 0, number = 0, sign = 1;
    if (str[i] == '-') {
        sign = -1;
        i++;
    }
    else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    while (str[i] != ',' && str[i] != '\0') {
        number *= 10; // Multiply the current number by 10;
        number += str[i] - '0'; // Convert character to decimal number
        i++;
    }
    return sign*number;
}
int next_value(const char* msg,int i) {
    while (msg[i] != ',' && msg[i] != '\0') { i++;}
    if (msg[i] == ',') {i++;}
    return i;
}
void parse_HLREF(const char* msg){
    int i = 0;
    rpm1 = extract_integer(msg);
    i = next_value(msg,i);
    rpm2 = extract_integer(msg + i);
}

int sat_requirements_check(int min, int max){
    // Check if the values are not within the allowed range of the propeller
    // Check if min and max are not correcty set (min > max)
    // The zero value is not allowed 
    if(min < -8000 || max > 8000 || min > max || min > 0 || max < 0){
        return(0);      // The received values are not allowed so they are discharged
    } 
    // All the previous requirements are meet
    else{
        return(1);      // The received values are allowed so they are stored
    }    
}
void parse_HLSAT(const char* msg){
    char strbuf[25];
    int i = 0;
    int receivedMin;
    int receivedMax;
    receivedMin = extract_integer(msg);
    i = next_value(msg,i);
    receivedMax = extract_integer(msg + i);
    if(sat_requirements_check(receivedMin,receivedMax) == 1){
        // Change the current saturation values
        minRPM = receivedMin;
        maxRPM = receivedMax;
        // ACK feedback
        sprintf(strbuf, "$MCACK,SAT,1*");   // SUCCESSFUL --> the current saturation values are updated
        write_buffTX_string(strbuf);
    }
    else{
        // ACK feedback
        sprintf(strbuf, "$MCACK,SAT,0*");   // UNSUCCESSFUL --> the current saturation values are not updated
        write_buffTX_string(strbuf);
    }
}

// UART //
void write_buffer(volatile CircularBuffer* cb, char value){
    cb->buffer[cb->writeIndex] = value;
    if (cb->writeIndex == cb->readIndex-1)
        cb->writeIndex;     // Discharg the last char otherwise you are going 
                            // to read the most recent one instead of the oldest one
    else
        cb->writeIndex++;
    
    if (cb->writeIndex == BUFFER_SIZE){     // Circular behaviour
        cb->writeIndex = 0;
    }
}   // Write the char into the circular buffer for UART communication

void write_buffTX_string(char *text){
    int i=0;
    while(semaphore == 0){     // The shared data circular buffer TX is not used by another function
        while(text[i] != '\0'){
            semaphore =  1;     // Take the semaphore
            write_buffer(&cbTX, text[i]);      // Character to be sent to the transmit buffer
            i++;
        }
        semaphore = 0;          // Release the semaphore
    }
}

void __attribute__ (( __interrupt__ , __auto_psv__ ) ) _U2RXInterrupt() {
    IFS1bits .U2RXIF = 0;   // Put the flag down
    if(U2STAbits.OERR == 1){    // If an overflow occured
        flagError = 1;  // Signal an error for the overflow
        U2STAbits.OERR = 0;     // Put down manually the flag (loss of bytes)
    }
    char val = U2RXREG;     // Get the char received by UART
    write_buffer(&cbRX, val); // Write the received char in the circular buffer
} // INTERRUPT for read char from UART2

int read_bufferRX(volatile CircularBuffer* cb, char*  value){
    // Safe way to manage shared data
    // 1) disable the interrupt in reception 
    IEC1bits.U2RXIE = 0;
    if (cb->readIndex == cb->writeIndex){ // None new message
        IEC1bits.U2RXIE = 1;    // Renable the interrupt in reception
        return 0;
    }
    // 2) copy the data and act on it
    *value = cb->buffer[cb->readIndex];
    cb->readIndex++;
    if(cb->readIndex == BUFFER_SIZE){   // Circular behaviour
        cb->readIndex=0;
    }
    // 3) Renable the interrupt in reception
    IEC1bits.U2RXIE = 1; 
    return 1;
}   // Read the char stored in the circular buffer

int read_bufferTX(volatile CircularBuffer* cb, char*  value){
    // Safe way to manage shared data
    // 1) Take the sempaphore 
    semaphore = 1;      // Take the semaphore
    if (cb->readIndex == cb->writeIndex){ // None new message
        semaphore = 0;    // Release the semaphore
        return 0;
    }
    // 2) copy the data and act on it
    *value = cb->buffer[cb->readIndex];
    
    cb->readIndex++;
    if(cb->readIndex == BUFFER_SIZE){   // Circular behaviour
        cb->readIndex=0;
    }
    // 3) Release the seamphore
    semaphore = 0;    // Release the semaphore 
    return 1;
}   // Read the char stored in the circular buffer


void read_msg(){
    while(read_bufferRX(&cbRX,&valueRX) == 1){    // There is at least one unread char in the circular buffer
        if(parse_byte(&pstate,valueRX) == NEW_MESSAGE){       // New valid message is received
            char strbuf[25];
            // REFERENCE VALUES MESSAGE //
            if(strcmp(pstate.msg_type,"HLREF") == 0){
                if(currentState != SAFE){
                    // Reset the counter of tmr2 (TIMEOUT mode) in safe way
                    T2CONbits.TON = 0; // Disable the counter of tmr2
                    TMR2 = 0;   // Reset the counter value inside TMR2 
                    T2CONbits.TON = 1; // Begin tmr2
                    // ACK feedback
                    sprintf(strbuf, "$MCACK,REF,1*");   // SUCCESSFUL --> The current reference values are received
                    write_buffTX_string(strbuf);
                    // Change of the state
                    if(currentState == TIMEOUT){
                        currentState = CONTROLLED;
                    }
                    parse_HLREF(pstate.msg_payload);
                }
                // The catamaran is in SAFE mode
                else{ 
                    // ACK feedback
                    sprintf(strbuf, "$MCACK,REF,0*");   // UNSUCCESSFUL --> The current reference values are received but not updated 
                    write_buffTX_string(strbuf);  
                }
            }
            
            // SATURATION VALUES MESSAGE //
            if(strcmp(pstate.msg_type,"HLSAT") == 0){
                parse_HLSAT(pstate.msg_payload);
            }
            
            // ENABLE MESSAGE //
            if(strcmp(pstate.msg_type,"HLENA") == 0){
                if(currentState == SAFE){
                    currentState = CONTROLLED;
                    rpm1 = 0;
                    rpm2 = 0;
                    T2CONbits.TON = 0;      // Disable tmr2
                    TMR2 = 0;           // Reset the counter value inside TMR2
                    T2CONbits.TON = 1;      // begin tmr2
                    // ACK feedback
                    sprintf(strbuf, "$MCACK,ENA,1*");   // SUCCESSFUL --> change of status between SAFE and CONTROLLED
                    write_buffTX_string(strbuf);
                }
                // The catamaran was not in SAFE mode
                else{
                    // ACK feedback
                    sprintf(strbuf, "$MCACK,ENA,0*");   // UNSUCCESSFUL --> no change of status
                    write_buffTX_string(strbuf);
                }
            }
        }
    }
}

// SCHEDULER //
void scheduler(){
    int j;
    for (j = 1; j < MAX_TASKS+1; j++){
        schedInfo[j].n++;   // Increase the number of heartbeats that had elapsed up to now
        if (schedInfo[j].n == schedInfo[j].N){  // If it is time for the task to be executed
            switch(j){
                case 1:     
                    blink_Leds();
                    break;
                case 2:     
                    countTemp++;
                    sumTemp = get_SumTemperature();
                    if(countTemp==10){ // average of the last 10 acquired temperatures
                        averTemp = sumTemp/countTemp;
                        countTemp = 0;
                        sumTemp = 0;
                        //send_MCTEM();
                    }
                    break;
                case 3:     
                    check_button(); 
                    break;
                case 4:     
                    send_PWM();
                    break;
                case 5:     
                    LCD_display();
                    break;
                case 6:     
                    read_msg();
                    break;
                case 7:     
                    send_MCACK();
                    break;
                case 8:     
                    //send_MCFBK();
                    break;
                default:
                    break;
            }
            schedInfo[j].n = 0;
        }
    }
}
