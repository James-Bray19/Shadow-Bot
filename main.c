/* 
 * file:        main.c 
 * authors:     James Bray
 *              Sam Davis
 *              Fabian Chrzanowski
 * project:     ShadowBots 2023/24
 * description: Navigates a two-wheel ShadowBot through obstacles towards
 *              an IR beacon using beacon and proximity sensors
 */

// INITIALISATION
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config OSC = HS 
#pragma config WDT = OFF 
#pragma config LVP = OFF 
#pragma config PWRT = ON 

// Define clock frequency for '__delay_ms()'
#define _XTAL_FREQ 10000000 

// Assign references for H-bridge pins
#define leftmotorB1 LATAbits.LATA4
#define leftmotorB2 LATAbits.LATA5
#define rightmotorA1 LATBbits.LATB0
#define rightmotorA2 LATBbits.LATB1

// Assign references for LED pins
#define LED1 LATBbits.LATB2
#define LED2 LATBbits.LATB3 
#define LED3 LATBbits.LATB4
#define LED4 LATBbits.LATB5 

// Assign references for beacon sensor pins 
#define beaconLeft PORTAbits.RA2
#define beaconRight PORTAbits.RA3

// Assign references for encoder pins
#define encoderLeft PORTCbits.RC0
#define encoderRight PORTCbits.RC5


// VARIABLES
unsigned int setpoint_distance = 300;  // Set minimum distance for obstacles
float ang2ticks = 2.6;                 // Ratio of encoder ticks to degrees
int motor_speed = 125;                 // Motor speed (0-255)

// FUNCTIONS
unsigned int readleftADC(void);  // Read left proximity sensor
unsigned int readrightADC(void); // Read right proximity sensor
void wait10ms(int del);          // Delay to program for del*10 milliseconds
void turn(int dir, int ang);     // Turns the robot
                                 //     dir: 1 for right, -1 for left
                                 //     ang: 0 for endless, value for angle
void acc(int dir, int dur);      // Accelerate the robot
                                 //     dir: 1 forward, -1 for back, 0 for stop
void flashLEDs(int dur);         // Flash all LEDs 3 times over a duration
void facebeacon(void);           // Turn robot to face

// MAIN LOOP
int main(void)
{
    TRISC = 0b11111001;   // Set ports to input/output
    TRISA = 0b00001111;
    TRISB = 0b11000000;
    PR2   = 0b11111111;   // Set period of PWM
    T2CON = 0b00000111;   // Timer 2(TMR2) on, Prescaler = 16
    CCP1CON = (0x0c);     // 0x0c enables PWM module CCP1 & CCP2
    CCP2CON = (0x0c);
    CCPR1L = motor_speed; // Load duty cycle into CCP1CON and CCP2CON
    CCPR2L = motor_speed;
    ADCON1 = 0b00001101;  // Set voltage reference and port A0 as analogue input
    ADCON2 = 0b10000010;  // Fosc/32, A/D result right justified
    LATB = 0;             // Turns motors and LEDs off
   
    // Flash LEDs 3 times
    flashLEDs(300);
    
    while(1){
        facebeacon();
        if(readleftADC() >= setpoint_distance || readrightADC() >= setpoint_distance){  // If detecting obstacle, avoid it
            acc(0, 0);
            if (beaconLeft == 1 && beaconRight == 1) { flashLEDs(300); while(1); }  // If beacon is unseen, stop code (beacon reached)
            flashLEDs(100); wait10ms(10); 
            acc(-1, 100);   
            turn(-1, 60);  
            acc(1, 120);      
            turn(1, 60);
        }
        else {                    
            acc(1, 5);
        }   
    }
}

void wait10ms(int del){     	 
    int c;
    for(c = 0; c < del; c++) { __delay_ms(10); }
    return;
}

void acc(int dir, int dur)
{
    if(dir == 1){
        rightmotorA1 = 0;     
        rightmotorA2 = 1;
        leftmotorB1 = 0;        
        leftmotorB2 = 1;
    }
    else if(dir == -1){
        rightmotorA1 = 1;
        rightmotorA2 = 0;
        leftmotorB1 = 1;
        leftmotorB2 = 0;
    }
    else if(dir == 0)
    {
        rightmotorA1 = 1;     
        rightmotorA2 = 1;
        leftmotorB1 = 1;
        leftmotorB2 = 1;
    }
    wait10ms(dur);
    return;
}

void flashLEDs(int dur){
    for(int i=0;i<3;i++){    // Flash LEDs after obstacle detection 
        LED1=1; LED2=1; LED3=1; LED4=1;
        wait10ms(dur/6);
        LED1=0; LED2=0; LED3=0; LED4=0;
        wait10ms(dur/6);
    }
}

void facebeacon()
{   
    int count = 0;
    int temp = encoderLeft;
    while(beaconLeft == 1 || beaconRight == 1){ // While beacon not detected
        turn(beaconLeft * 2 - 1, 0); // Turn towards beacon, turn right as default if unseen
        
        // Count ticks
        if (temp != encoderLeft){ count++; } 
        temp = encoderLeft;
        
        // If it can't see the beacon in a rotation, stop code (beacon reached)
        if (count > 360 * ang2ticks) { flashLEDs(300); acc(0, 0); while(1); }
    }
}

void turn(int dir, int ang){
    int ticks = ang * ang2ticks;
    int count = 0;
    if (dir == 1){ // Turn right
        rightmotorA1 = 1;
        rightmotorA2 = 1;
        leftmotorB1 = 0;
        leftmotorB2 = 1;

        // Count ticks
        int temp = encoderLeft;
        while (count < ticks){
            if (temp != encoderLeft){ count++; } 
            temp = encoderLeft;
        }
    }
    else if (dir == -1){ // Turn left
        rightmotorA1 = 0;
        rightmotorA2 = 1;
        leftmotorB1 = 1;
        leftmotorB2 = 1;

        // Count ticks
        int temp = encoderRight;
        while (count < ticks){
            if (temp != encoderRight){ count++; } 
            temp = encoderRight;
        }
    }
    
    if (ang != 0){
        rightmotorA1 = 1;
        rightmotorA2 = 1;
        leftmotorB1 = 1;
        leftmotorB2 = 1;
    }

    return;
}

// Read left proximity sensor
unsigned int readleftADC(void) {
    ADCON0 = 0b00000011;
    while (ADCON0bits.GO);
    return ((ADRESH << 8) + ADRESL);

// Read right proximity sensor
unsigned int readrightADC(void) {
    ADCON0 = 0b00000111; 
    while (ADCON0bits.GO);
    return ((ADRESH << 8) + ADRESL);
