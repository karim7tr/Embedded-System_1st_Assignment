

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>

// FOSC
#pragma config FPR = XT // Primary Oscillator Mode (XT)
#pragma config FOS = PRI // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16 // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512 // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64 // POR Timer Value (64ms)
#pragma config BODENV = BORV20 // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI // Low?side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI // High?side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)


#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4


// Global flags and counters
int S5_flag = 0; // Flag for interrupt handling
int S6_flag = 0; // Flag for interrupt handling
int charCounter = 0;    
int rowCounter = 0;
char numberBuffer[16]; // Buffer for storing numbers as characters

// Define circular buffer structure for UART communication
#define BUFFER_SIZE 64
typedef struct {
    char data[BUFFER_SIZE];
    int head;
    int tail;
    int count;
} CircularBuffer;

CircularBuffer uartBuffer;


// Timer2 interrupt handler
void __attribute__((__interrupt__, __auto_psv__))_T2Interrupt() {
    IFS0bits.T2IF = 0; // reset the flag
    T2CONbits.TON = 0; // stop timer2
    TMR2 = 0x00; // reset timer2
    LATBbits.LATB0 = 1; // switch on LED D3 for test
    if (PORTEbits.RE8 == 0x01) {
        S5_flag = 1; // flag to avoid doing too many things in the interrupt
    }
    IEC0bits.T2IE = 0x00;
}

// Timer1 interrupt handler
void __attribute__((__interrupt__, __auto_psv__))_T1Interrupt() {
    IFS0bits.T1IF = 0; // reset the flag
    T1CONbits.TON = 0; // stop timer1
    TMR1 = 0x00; // reset timer1
    if (PORTDbits.RD0 == 0x01) {
        S6_flag = 1; // flag to avoid doing too many things in the interrupt
    }
    IEC0bits.T1IE = 0x00;
}


// UART RX interrupt handler
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0; // Flag for UART RX interrupt
    char receivedData = U2RXREG;
    addToBuffer(receivedData);
}

// Initialization functions
void initSPI();
void initUART();
void initCircularBuffer();
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);
void algorithm();
// Communication functions
void sendSPI(char data);
void sendSPIBuffer(char buffer[]);
void sendUARTBuffer(char buffer[]);
void updateLCD(char cursor, char data);
void clearLCDScreen();
void addToBuffer(char data);
void processUARTData();

// Main function
int main() {
    tmr_setup_period(TIMER4, 10);

    initSPI();
    initUART();
    initCircularBuffer();
    tmr_wait_ms(TIMER1, 1000);

    TRISBbits.TRISB0 = 0; // LED D3 as output for tests
    TRISEbits.TRISE8 = 1; // S5 as input
    TRISDbits.TRISD0 = 1; // S6 as input
    IEC0bits.T2IE = 0x01; // enable interruption T2
    IEC1bits.U2RXIE = 0x01; // enable UART interruption

    // Initialize variables for character data and counters
    char dataChar = 0; // Variable to store the received character
    int rowCounter = 0; //Counter for tracking the LCD row
    int charCounter = 0; // Counter for tracking the total received characters

    while (1) { // Call the algorithm function
        algorithm();
        // Process characters in the UART buffer
        while (uartBuffer.count > 0) {
            // Clear UART Overrun Error flag to prevent communication issues
            if (U2STAbits.OERR == 0x01) {
                U2STAbits.OERR = 0;
            }
            // Retrieve the next character from the UART buffer
            dataChar = uartBuffer.data[uartBuffer.head];
            uartBuffer.head = (uartBuffer.head + 1) % BUFFER_SIZE;

            // Display the character on the LCD by sending SPI commands
            sendSPI((0x80 + rowCounter));  // Set the LCD cursor to the current row
            tmr_wait_ms(TIMER1, 1); // Wait for a short duration
            sendSPI(dataChar); // Send the character to be displayed
            rowCounter++; 

            // Update the character counter and display it on the LCD
            charCounter++;
            sendSPI(0xC0);   // Set the cursor to the second row
            sprintf(numberBuffer, "Char Recv: %d", charCounter);
            sendSPIBuffer(numberBuffer);
            
            // Check for conditions to reset the LCD row counter and clear the screen
            if ((rowCounter == 17) || (dataChar == '\n') || (dataChar == '\r')) {
                rowCounter = 0; // Reset the LCD row counter
                clearLCDScreen();  // Clear the LCD screen
                sendSPI(dataChar); // Send the character for display
            }
        
            // Disable and re-enable UART RX interrupt to handle the buffer
            IEC1bits.U2RXIE = 0x00;
            uartBuffer.count--;
            IEC1bits.U2RXIE = 0x01;
        }
            if (PORTEbits.RE8 == 0) {
        tmr_setup_period(TIMER2, (20));
        IEC0bits.T2IE = 0x01; // enable timer2 interrupt
        T2CONbits.TON = 0x01; // start the timer
    }

    if (S5_flag == 1) {// Convert the character counter to a string for UART transmission
        sprintf(numberBuffer, "%d", charCounter);
        sendUARTBuffer(numberBuffer);
        // Reset the S5 flag to avoid repetitive actions
        S5_flag = 0;
    }
        
    // Check if button S6 is pressed    
    if (PORTDbits.RD0 == 0) {
        tmr_setup_period(TIMER1, (20));
        IEC0bits.T1IE = 0x01; // enable timer1 interrupt
        T1CONbits.TON = 0x01; // start the timer
    }

    if (S6_flag == 1) {// Reset counters and clear the LCD screen when S6 is pressed
        charCounter = 0;
        rowCounter = 0;
        clearLCDScreen(); 
        sendSPI(0xC0); // Set the cursor to the second row
        sendSPIBuffer("Char Recv: 0     "); // clear the second row when S6 pressed
        sendSPI(0x80);  // Set the cursor to the first row
        S6_flag = 0;    // Reset the S6 flag to avoid repetitive actions
    }
           
    
        tmr_wait_period(TIMER4); // 100 Hz frequency
    }

    return 0;
}

void initSPI() {
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8 bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3; // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
}

void initUART() {
    U2BRG = 11; // (7372800 / 4) / (16*9600)-1
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U1TX (must be after UARTEN)
    U2STAbits.UTXISEL = 1; // 1st mode of interrupt, go to page 7
    U2STAbits.URXISEL = 0; // interrupt flag whenever a character is received   
}

void initCircularBuffer() {
    uartBuffer.head = 0;
    uartBuffer.tail = 0;
    uartBuffer.count = 0;
}

void tmr_setup_period(int timer, int ms) {
    if (timer == TIMER1) {
        PR1 = ((float) ms * ((float) 28789 / 1000.0));
        T1CONbits.TCKPS = 0x02; //set the prescaler as 64
        T1CONbits.TCS = 0x00; // Internal clock 
        T1CONbits.TON = 0x01; // set the timer ON
    } else if (timer == TIMER2) {
        PR2 = ((float) ms * ((float) 28789 / 1000.0));
        T2CONbits.TCKPS = 0x02; //set the prescaler as 64
        T2CONbits.TCS = 0x00; // Internal clock 
        T2CONbits.TON = 0x01; // set the timer ON
    } else if (timer == TIMER3) {
        PR3 = ((float) ms * ((float) 28789 / 1000.0));
        T3CONbits.TCKPS = 0x02; //set the prescaler as 64
        T3CONbits.TCS = 0x00; // Internal clock 
        T3CONbits.TON = 0x01; // set the timer ON
    } else if (timer == TIMER4) {
        PR4 = ((float) ms * ((float) 28789 / 1000.0));
        T4CONbits.TCKPS = 0x02; //set the prescaler as 64
        T4CONbits.TCS = 0x00; // Internal clock 
        T4CONbits.TON = 0x01; // set the timer ON
    }
}

void tmr_wait_period(int timer) {
    if (timer == TIMER1) {
        IFS0bits.T1IF = 0x00;
        while (IFS0bits.T1IF == 0x00);
    } else if (timer == TIMER2) {
        IFS0bits.T2IF = 0x00;
        while (IFS0bits.T2IF == 0x00);
    } else if (timer == TIMER3) {
        IFS0bits.T3IF = 0x00;
        while (IFS0bits.T3IF == 0x00);
    } else if (timer == TIMER4) {
        IFS1bits.T4IF = 0x00;
        while (IFS1bits.T4IF == 0x00);
    }
}

void tmr_wait_ms(int timer, int ms) {
    tmr_setup_period(timer, ms);
    tmr_wait_period(timer);
}

void algorithm() {
    tmr_wait_ms(TIMER3, 7);
}

void sendSPI(char data) {
    while (SPI1STATbits.SPITBF == 1); // wait until not full
    SPI1BUF = data; // send the byte containing the value data
}

void sendSPIBuffer(char buffer[]) {
    while (SPI1STATbits.SPITBF == 1); // wait until not full
    int j = 0;
    while (buffer[j] != '\0') {
        while (SPI1STATbits.SPITBF == 1); // wait until not full
        sendSPI(buffer[j]);
        j++;
    }
}

void sendUARTBuffer(char buffer[]) {
    int j = 0;
    while (buffer[j] != '\0') {
        U2TXREG = buffer[j];
        j++;
    }
}

void updateLCD(char cursor, char data) {
    sendSPI(cursor); // set the LCD cursor 1st row / 1st column
    sendSPI(data);
}

void clearLCDScreen() {
    sendSPI(0x80);
    for (int i = 0; i < 16; i++) {
        sendSPI(' ');
    }
    sendSPI(0x80);
}



void addToBuffer(char data) {
    if (uartBuffer.count < BUFFER_SIZE) {
        uartBuffer.data[uartBuffer.tail] = data;
        uartBuffer.tail = (uartBuffer.tail + 1) % BUFFER_SIZE;
        uartBuffer.count++;
    } else {
        // Buffer full --> ignore data
  
    }
}


