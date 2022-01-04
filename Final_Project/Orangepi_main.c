//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);


void serialRXA(serial_t *s, char data);
void serialRXD(serial_t *s, char data); //the serialRXD function is initialized here
void setupSPIB(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint32_t numRXD = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t SendToOrange = 0;
uint16_t count = 0;
uint16_t val = 0;
float spi_val1 = 0;
float spi_val2 = 0;
float spi_val3 = 0;
float adc_val1 = 0;
float adc_val2 = 0;
int16_t dummy = 0;

int16_t accelXraw = 0;
int16_t accelYraw = 0;
int16_t accelZraw = 0;
float accelXreading = 0;
float accelYreading = 0;
float accelZreading = 0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;
float gyroXreading = 0;
float gyroYreading = 0;
float gyroZreading = 0;

float RightWheel;
float LeftWheel;
float Rightdist;
float Leftdist;

float uLeft = 0;
float uRight = 0;

float LeftWheelK = 0;
float RightWheelK = 0;
float RightWheelK_1 = 0;
float LeftWheelK_1 = 0;
float v_left = 0;
float v_right = 0;
//float Kp = 3.0;
//float Ki = 25.0;
float Vel_WhlDiff = 0;
float Err_Left;
float I_left_K;
float I_left_K_1;
float Err_LeftK_1;
float vleft;
float Err_Right;
float I_right_K;
float I_right_K_1;
float Err_RightK_1;
float turn1;
float turn;
float K_turn = 3;
float e_left;
float e_right;
float e_turn;

float w_r = 0.61;
float r_wh = 0.2125/2;
float theta_r = 0;
float theta_l = 0;
float omega_r = 0;
float omega_l = 0;
float phi_r = 0;
float x_r;
float y_r;
float theta_avg;
float omega_avg;
float xr_dot;
float yr_dot;

float xr;
float xr_1;
float xr_dot_1;
float yr;
float yr_1;
float yr_dot_1;
float e_left_1;
float e_right_1;
int16_t adcd0result = 0;
int16_t adcd1result = 0;
//int32_t count = 0;
float var0 = 0;
float var1 = 0;
int16_t adca0result = 0;
int16_t adca1result = 0;
int16_t adcb0result = 0;
int16_t adcb1result = 0;
float var2 = 0;
float var3 = 0;
float var4 = 0;
float var5 = 0;

// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset  = 0;
float gyroy_offset  = 0;
float gyroz_offset  = 0;
float accelzBalancePoint = -.77; //This was tuned using watch expressions
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value    = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value    = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001;        //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;
int32_t Spib_count;
int16_t collectdata = 0;
float accelx;
float accely;
float accelz;
float gyrox;
float gyroy;
float gyroz;

float vel_right;
float vel_left;
float vel_right_1;
float vel_left_1;

float omega_r1;
float theta_r1;
float omega_l1;
float theta_l1;

float K1 = -60;
float K2 = -4.5;
float K3 = -1.1;

float ubal = 0;
float WhlDiff;
float WhlDiff_1;
float Vel_WhlDiff;
float Vel_WhlDiff_1;
float turn;
float turn_1;
float turnrate;
float turnrate_1;
float errorDiff;
float errorDiff_1;
float intDiff;
float intDiff_1;
float errorDiff_1;
float Kp = 3;
float Kd = 0.08;
float Ki = 20;
float RightWheel_1;
float LeftWheel_1;
float DVel = 0;
float u_right;
float u_left;

#define ORANGEPIBUFFSIZE 128
char RXBuff[ORANGEPIBUFFSIZE];

float Opti_x = 0;
float Opti_y = 0;
float Opti_theta = 0;
float Opti_Body = 0;
float Opti_Stamp = 0;


void setDACA(float dacouta0) {
        if (dacouta0 >  3.0) dacouta0 =  3.0;
        if (dacouta0 < 0.0) dacouta0 = 0.0;

        DacaRegs.DACVALS.bit.DACVALS = 1365.0*dacouta0;  // perform scaling of 0-3 to 0-4095
}

void setDACB(float dacouta1) {
        if (dacouta1 >  3.0) dacouta1 =  3.0;
        if (dacouta1 < 0.0) dacouta1 = 0.0;

        DacbRegs.DACVALS.bit.DACVALS = 1365.0*dacouta1;  // perform scaling of 0-3 to 0-4095
}


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);


//pin mux for EPWM2
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); //pin mux is checked and verified to be correct
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);//pin mux is checked and verified to be correct
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1  50Mhz Clock
    EPwm5Regs.TBPRD = 50000;  // Set Period to 1ms sample.  Input clock is 50MHz.  (1/50*10^6 MHz)*x = 0.0001 s
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;

    //write configurations for all ADCs  ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    DELAY_US(1000);

    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert Channel you choose Does not have to be A1 (I chose it as A1)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared


    EDIS;

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 10000);
    ConfigCpuTimer(&CpuTimer1, 200, 4000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
    init_serial(&SerialD,1500000,serialRXD); //This needed to be enabled since the segbot communicates with the orange pi using the serialRXD, whereas the wired portion communicates using serialRXA

    setupSPIB();
    init_eQEPs();

    EALLOW;  // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    //GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    //ePWM2
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 0; //EPWM2A
    EPwm2Regs.CMPB.bit.CMPB = 0;//EPWM2B
    EPwm2Regs.TBPHS.bit.TBPHS = 0;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.AQCTLB.bit.CBU = 1;


//    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);  // Set as GPIO9 and used as DAN28027 SS
//    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO9 an Output Pin
//    GpioDataRegs.GPASET.bit.GPIO9 = 1;  //Initially Set GPIO9/SS High so DAN28027 is not selected
//
//    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);  // Set as GPIO66 and used as MPU-9250 SS
//    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO66 an Output Pin
//    GpioDataRegs.GPCSET.bit.GPIO66 = 1;  //Initially Set GPIO66/SS High so MPU-9250 is not selected
//
//    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);  //Set GPIO63 pin to SPISIMOB
//    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);  //Set GPIO64 pin to SPISOMIB
//    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);  //Set GPIO65 pin to SPICLKB

//    EALLOW;
//    GpioCtrlRegs.GPBPUD.bit.GPIO63   = 0;  // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
//    GpioCtrlRegs.GPCPUD.bit.GPIO64   = 0;
//    GpioCtrlRegs.GPCPUD.bit.GPIO65   = 0;
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
//    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
//    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
//    EDIS;

    // ---------------------------------------------------------------------------
//    SpibRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset
//
//    SpibRegs.SPICTL.bit.CLK_PHASE = 1;  //This happens to be the mode for both the DAN28027 and
//    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;  //The MPU-9250,  Mode 01.
//    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master
//    SpibRegs.SPICCR.bit.SPICHAR = 0xF;  // Set to transmit and receive 16 bits each write to SPITXBUF
//    SpibRegs.SPICTL.bit.TALK = 1;  // Enable transmission
//    SpibRegs.SPIPRI.bit.FREE = 1;  // Free run, continue SPI operation
//    SpibRegs.SPICTL.bit.SPIINTENA = 0;  // Disables the SPI interrupt
//
//    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period.  SPI base clock is
//                                                             // 50MHZ.  And this setting divides that base clock to create SCLK’s period
//    SpibRegs.SPISTS.all = 0x0000;  // Clear status flags just in case they are set for some reason
//
//    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
//    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;    // Enable SPI FIFO enhancements
//    SpibRegs.SPIFFTX.bit.TXFIFO =  0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
//    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
//
//    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
//    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;   // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL
//
//    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip  //needs to be confirmed
//
//    SpibRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset
//
//    SpibRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset.
//    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
//    SpibRegs.SPICTL.bit.SPIINTENA = 1;    // Enables SPI interrupt.  !! I don’t think this is needed.  Need to Test
//
//    SpibRegs.SPIFFRX.bit.RXFFIL = 10; //Interrupt Level to 16 words or more received into FIFO causes interrupt.  This is just the initial setting for the register.  Will be changed below

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; //group 6, column 3
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    //Group 1 Interrupt 1 (For ADCA1)
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"%s \r\n",RXBuff);
            serial_printf(&SerialA,"%.3f %.3f \r\n",turn, DVel);
            UARTPrint = 0;
        }
        if (SendToOrange == 1 ) {
            //serial_printf(&SerialD,"Testing:%ld\n",CpuTimer2.InterruptCount);
            SendToOrange = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts


    //Right-Wheel
    theta_r = RightWheel;
    omega_r = (v_right*9.5);

    omega_r = (0.6*omega_r1)+100*(theta_r-theta_r1);

    omega_r1 = omega_r;
    theta_r1 = theta_r;

    //Left-Wheel
    theta_l = LeftWheel;
    omega_l = (v_left*9.5);

    omega_l = (0.6*omega_l1)+100*(theta_l-theta_l1);

    omega_l1 = omega_l;
    theta_l1 = theta_l;

    // Insert SWI ISR Code here.......

    ubal = -K1*tilt_value - K2*gyro_value - K3*(omega_l+omega_r)/2;

    uLeft = ubal/2;
    uRight = ubal/2;


    WhlDiff = LeftWheel - RightWheel;


    Vel_WhlDiff = 0.333*Vel_WhlDiff_1 + 166.667*(WhlDiff - WhlDiff_1);
    turn = turn_1 + (((turnrate+turnrate_1)*0.004)/2); //Here the variable turnref is replaced by turn

    errorDiff = turn - WhlDiff;
    intDiff = intDiff_1 + (((errorDiff+errorDiff_1)*0.004)/2);

    turn1 = Kp*errorDiff + Ki*intDiff - Kd*Vel_WhlDiff;

    if(fabs(turn1 >= 3)){
        intDiff = intDiff_1;
    }
    if(turn1>=4){
        turn1 = 4;
    }
    if(turn1 <= -4){
        turn1 = -4;
    }

    u_right = (ubal/2.0) - turn1 + DVel;
    u_left = (ubal/2.0) + turn1 + DVel; //Here the variable FwdBackOffset is replaced by the variable DVel


    setEPWM2A(u_right);
    setEPWM2B(-u_left);

    WhlDiff_1 = WhlDiff;
    Vel_WhlDiff_1 = Vel_WhlDiff;
    RightWheel_1 = RightWheel;
    LeftWheel_1 = LeftWheel;
    intDiff_1 = intDiff;
    errorDiff_1 = errorDiff;
    turnrate_1 = turnrate;
    turn_1 = turn;


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
//    CpuTimer0.InterruptCount++;
//
//    numTimer0calls++;
//
//    if(val == 0){ //counting up
//           if(count < 4500)
//               count = count+10;
//           else
//              val = 1;
//       }else{
//           if(count > 1500)
//               count = count-10;
//           else
//               val = 0;
//       }
//
//    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
//    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
//    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00)); //
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//    SpibRegs.SPITXBUF = 0;
//
//    //Clear GPIO9 Low to act as a Slave Select.  Right now, just to scope.  Later to select DAN28027 chip
////    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
////    SpibRegs.SPIFFRX.bit.RXFFIL = 3;  // Issue the SPIB_RX_INT when two values are in the RX FIFO
////    SpibRegs.SPITXBUF = 0x00DA;  // 0x4A3B and 0xB517 have no special meaning.  Wanted to send // FROM DAN 28027 Chip
////    SpibRegs.SPITXBUF = count;  // something so you can see the pattern on the Oscilloscope // First reading
////    SpibRegs.SPITXBUF = count;//Second reading
//
////    SpibRegs.SPIRXBUF = 0x00DA;
////    SpibRegs.SPIRXBUF = count;
////    SpibRegs.SPIRXBUF = count;
//
//
//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }
//
//    if ((numTimer0calls%250) == 0) {
//        displayLEDletter(LEDdisplaynum);
//        LEDdisplaynum++;
//        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
//            LEDdisplaynum = 0;
//        }
//    }
//
//    // Blink LaunchPad Red LED
//    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    if ((CpuTimer0.InterruptCount % 10) == 0) {

        }
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
//    RightWheel = readEncRight();
//    LeftWheel = -readEncLeft();
//
//    Rightdist = RightWheel/9.5;
//    Leftdist = LeftWheel/9.5;
//
//    // calculating the position
//    LeftWheelK = Leftdist;
//    RightWheelK = Rightdist;
//    //    //calculating states:
//
//    v_right = (RightWheelK - RightWheelK_1)/0.004;
//    v_left = (LeftWheelK - LeftWheelK_1)/0.004;
//    e_turn = turn + (v_left-v_right);
//    e_left = Vel_WhlDiff - v_left-K_turn*e_turn;
//    e_right = Vel_WhlDiff - v_right+K_turn*e_turn;
//
//    //Err_Left = Vel_WhlDiff - v_left;
//    //Err_Right = Vel_WhlDiff -v_right;
//
//    I_left_K = I_left_K_1 + ((e_left+e_left_1)/2.0)*0.004;
//    I_right_K = I_right_K_1 + ((e_right+e_right_1)/2.0)*0.004;
//
//    e_left_1 = e_left;
//    e_right_1 = e_right;
//
//
//    //
//    //    //calculating the control effort
//    //
//    uLeft = Kp * e_left + Ki * I_left_K;
//    uRight = Kp * e_right + Ki * I_right_K;
//
//
//    if(fabs(uRight)>10){
//        I_right_K = I_right_K_1;
//    }
//    if(fabs(uLeft)>10){
//        I_left_K = I_left_K_1;
//    }
//    CpuTimer1.InterruptCount++;
//    setEPWM2B(-uLeft);
//    setEPWM2A(uRight);
//
//    I_left_K_1 = I_left_K;
//    I_right_K_1 = I_right_K;
//
//    Err_LeftK_1 = Err_Left;
//    Err_RightK_1 = Err_Right;
//    RightWheelK_1 = RightWheelK;
//    LeftWheelK_1 = LeftWheelK;
//
//    //new control code for exercise 5
//
//    theta_r = RightWheel;
//    theta_l = LeftWheel;
//
//    theta_avg = 0.5*(theta_r+theta_l);
//    phi_r = (r_wh/w_r)*(theta_r-theta_l);
//    omega_r = (v_right*9.5);
//    omega_l = (v_left*9.5);
//    omega_avg = 0.5*(omega_r+omega_l);
//    xr_dot = r_wh*omega_avg*cos(phi_r);
//    yr_dot = r_wh*omega_avg*sin(phi_r);
//
//
//    xr = xr_1+0.004*(xr_dot + xr_dot_1)/2; //trapezoidal rule for x
//
//    xr_dot_1 = xr_dot;
//    xr_1 = xr;
//
//    yr = yr_1 + 0.004*(yr_dot+yr_dot_1)/2; //trapezoidal rule for y
//
//    yr_1 = yr;
//    yr_dot_1 = yr_dot;
//
//    if ((CpuTimer1.InterruptCount % 10) == 0) {
//        UARTPrint = 1;
//    }

}



// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
    if ((CpuTimer2.InterruptCount % 5) == 0) {
        //      UARTPrint = 1;
        SendToOrange = 1;
        }
//    // Blink LaunchPad Blue LED
//    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
//
//    CpuTimer2.InterruptCount++;
//
//    if ((CpuTimer2.InterruptCount % 50) == 0) {
//        UARTPrint = 1;
//    }
}



int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
__interrupt void SPIB_isr(void){

    Spib_count++;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    dummy = SpibRegs.SPIRXBUF;
    accelXraw = SpibRegs.SPIRXBUF;
    accelYraw = SpibRegs.SPIRXBUF;
    accelZraw = SpibRegs.SPIRXBUF;
    accelXreading = accelXraw*4.0/32767.0;
    accelYreading = accelYraw*4.0/32767.0;
    accelZreading = accelZraw*4.0/32767.0;

    dummy = SpibRegs.SPIRXBUF;

    gyroXraw = SpibRegs.SPIRXBUF;
    gyroYraw = SpibRegs.SPIRXBUF;
    gyroZraw = SpibRegs.SPIRXBUF;
    gyroXreading = gyroXraw*250.0/32767.0;
    gyroYreading = gyroYraw*250.0/32767.0;
    gyroZreading = gyroZraw*250.0/32767.0;

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

    //    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
    //    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO.  Again probably zero
    //    spivalue3 = SpibRegs.SPIRXBUF;
    //
    //    adc_val1 = spivalue2*(3.3/4095);
    //    adc_val2 = spivalue3*(3.3/4095);
    //
    //GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO 9 high to end Slave Select.  Now to Scope. Later to deselect DAN28027
    //    // Later when actually communicating with the DAN28027 do something with the data.  Now do nothing.

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;      // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;      // Clear RX FIFO Interrupt flag so next interrupt will happen


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;  // Acknowledge INT6 PIE interrupt

    if ((Spib_count % 200) == 0) {

    }
    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelXreading;
        accely_offset+=accelYreading;
        accelz_offset+=accelZreading;
        gyrox_offset+=gyroXreading;
        gyroy_offset+=gyroYreading;
        gyroz_offset+=gyroZreading;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }

    } else if(calibration_state == 2){

        accelXreading -=(accelx_offset);
        accelYreading -=(accely_offset);
        accelZreading -=(accelz_offset-accelzBalancePoint);
        gyroXreading -= gyrox_offset;
        gyroYreading -= gyroy_offset;
        gyroZreading -= gyroz_offset;

        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyroXreading*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;

        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;

        // Update Step
        z = -accelZreading;  // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;

        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = -readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();

        if (SpibNumCalls >= 3) {  // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;

            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
        }
    }

    timecount++;

    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Always Block Red LED

          // Tell While loop to print
    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

}

void setupSPIB(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;

    //SpibRegs.SPICCR.bit.SPICHAR      = 0xF;
    //SpibRegs.SPIFFCT.bit.TXDLY       = 0;
    SpibRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1;  //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;  //The MPU-9250,  Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;  // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1;  // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1;  // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0;  // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period.  SPI base clock is
                                                             // 50MHZ.  And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000;  // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;    // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO =  0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;   // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL

    SpibRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip  //needs to be confirmed

    SpibRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1;    // Enables SPI interrupt.  !! I don’t think this is needed.  Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 10; //Interrupt Level to 16 words or more received into FIFO causes interrupt.  This is just the initial setting for the register.  Will be changed below

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);  // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1;  //Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);  // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;  //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);  //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);  //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);  //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63   = 0;  // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64   = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65   = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;


    //Step 2
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
        // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F.  Use only one SS low to high for all these writes
        // some code is given, most you have to fill you yourself.
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

        // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.

        SpibRegs.SPITXBUF = 0x1300;
        SpibRegs.SPITXBUF = 0x0000;
        SpibRegs.SPITXBUF = 0x0000;
        SpibRegs.SPITXBUF = 0x0013;
        SpibRegs.SPITXBUF = 0x0200;
        SpibRegs.SPITXBUF = 0x0806;
        SpibRegs.SPITXBUF = 0x0000;

        // To address 00x13 write 0x00

        // To address 00x14 write 0x00

        // To address 00x15 write 0x00

        // To address 00x16 write 0x00

        // To address 00x17 write 0x00

        // To address 00x18 write 0x00

        // To address 00x19 write 0x13

        // To address 00x1A write 0x02

        // To address 00x1B write 0x00

        // To address 00x1C write 0x08

        // To address 00x1D write 0x06

        // To address 00x1E write 0x00

        // To address 00x1F write 0x00

        // wait for the correct number of 16 bit values to be received into the RX FIFO
        while(SpibRegs.SPIFFRX.bit.RXFFST !=7);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
        DELAY_US(10);  // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
         // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
        // 0x27, 0x28, 0x29.  Use only one SS low to high for all these writes
        // some code is given, most you have to fill you yourself.
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

        // Perform the number of needed writes to SPITXBUF to write to all 13 registers
        // To address 00x23 write 0x00
        SpibRegs.SPITXBUF = 0x2300;
        // To address 00x24 write 0x40
        SpibRegs.SPITXBUF = 0x408C;
        // To address 00x25 write 0x8C
        SpibRegs.SPITXBUF = 0x0288;
        // To address 00x26 write 0x02
        SpibRegs.SPITXBUF = 0x0C0A;
        // To address 00x27 write 0x88

        // To address 00x28 write 0x0C

        // To address 00x29 write 0x0A

        // wait for the correct number of 16 bit values to be received into the RX FIFO
        while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
        DELAY_US(10);  // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
        // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        // Write to address 0x2A the value 0x81
        SpibRegs.SPITXBUF = (0x2A81);

        // wait for one byte to be received
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);

        // The Remainder of this code is given to you and you do not need to make any changes.
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x3800 | 0x0001);  // 0x3800
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x3A00 | 0x0001);  // 0x3A00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6400 | 0x0001);  // 0x6400
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6A00 | 0x0020);  // 0x6A00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7500 | 0x0071);  // 0x7500
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7700 | 0x0000); // 0x7700
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7800 | 0x0000); // 0x7800
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7A00 | 0x0000); // 0x7A00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7B00 | 0x0000); // 0x7B00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // 0x7D00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(50);

        // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
        SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
        SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

//lab 6 code copied here
void init_eQEPs(void) {

    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2;   // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0;    // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0;   // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0;      // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0;      // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0;        // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;   // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1;    // Enable EQep


    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;    // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;    // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;   // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0;   // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0;     // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0;     // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0;       // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF;  // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1;   // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (raw*(TWOPI/600));
}

float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (raw*(TWOPI/600));
}
void setEPWM2A(float controleffort){
    if(controleffort > 10){
        controleffort = 10;
    }
    if(controleffort < -10){
        controleffort = -10;
    }
    EPwm2Regs.CMPA.bit.CMPA = (int)(controleffort*(2500.0/20.0)+1250.0);
}
void setEPWM2B(float controleffort){
    if(controleffort > 10){
        controleffort = 10;
    }
    if(controleffort < -10){
        controleffort = -10;
    }
    EPwm2Regs.CMPB.bit.CMPB = (int)(controleffort*(2500.0/20.0)+1250.0);
}

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    if (data == 'q') {
        turnrate = turnrate - 0.1;
    } else if (data == 'r') {
        turnrate = turnrate + 0.1;
    } else if (data == '3') {
        DVel = DVel - 0.1;
    } else if (data == 's') {
        DVel = DVel + 0.1;
    } else {
        turnrate = 0;
        DVel = 0;
    }
}

typedef union optiData_s {
    uint16_t rawdata[10];
    float data[5];
} optiData_t;

optiData_t OptiRecv;

char pastRecChar[5] = {0,0,0,0,0};

int16_t newData = 0;

//void serialRXD(serial_t *s, char data) {
void serialRXD(serial_t *s, char data) {
    //    if (newData == 0) {
    //        pastRecChar[0] = data;
    //        if ( (pastRecChar[0] == ':') &&
    //             (pastRecChar[1] == 'i') &&
    //             (pastRecChar[2] == 't') &&
    //             (pastRecChar[3] == 'p') &&
    //             (pastRecChar[4] == 'O')
    //           ) {
    //            newData = 1;
    //            numRXD = 0;
    //            pastRecChar[0] = 0;
    //            pastRecChar[1] = 0;
    //            pastRecChar[2] = 0;
    //            pastRecChar[3] = 0;
    //            pastRecChar[4] = 0;
    //        }
    //        pastRecChar[4] = pastRecChar[3];
    //        pastRecChar[3] = pastRecChar[2];
    //        pastRecChar[2] = pastRecChar[1];
    //        pastRecChar[1] = pastRecChar[0];
    //
    //
    //    } else if (newData == 1) {
    //        if ((numRXD % 2) == 0) {
    //            OptiRecv.rawdata[numRXD/2] = data & 0xFF;
    //        } else {
    //            OptiRecv.rawdata[numRXD/2] = ((data << 8) & 0xFF00) | OptiRecv.rawdata[numRXD/2];
    //        }
    //        numRXD++;
    //        if (numRXD >= 20) {
    //            numRXD = 0;
    //            newData = 0;
    //            Opti_x = OptiRecv.data[0];
    //            Opti_y = OptiRecv.data[1];
    //            Opti_theta = OptiRecv.data[2];
    //            Opti_Body = OptiRecv.data[3];
    //            Opti_Stamp = OptiRecv.data[4];
    //            UARTPrint = 1;
    //        }
    //    }

    if (numRXD < ORANGEPIBUFFSIZE) {

        if (collectdata == 1) {
            if (data == '#') { //After this condition, the reading stops
                RXBuff[numRXD] = '\0';
                sscanf(RXBuff,"%f,%f", &turnrate,&DVel); //This converts the string being received from the Orange Pi into a float
                numRXD = 0;
                UARTPrint = 1;
            } else {
                if(data != '*'){


                    RXBuff[numRXD] = data;
                    numRXD ++;
                }
            }
        } else {
            if (data == '*') { //Starts reading after this variable
                collectdata = 1;
                numRXD = 0;
            }
        }
    } else {
        numRXD = 0;
        UARTPrint = 1;
    }

}


float xk[101] = {0};
//yk is the filtered value
float yk = 0;
//b is the filter coefficients

//using Bandpass filter with 10000 Hz sampling rate
float b[101]={  -4.1384865093955942e-18,
    2.4158288163475484e-05,
    -1.3320677588450310e-04,
    -2.1438469575543533e-04,
    1.1898399936030848e-04,
    5.3082205069710680e-04,
    2.1893290271722703e-04,
    -7.4768481245699380e-04,
    -9.5792023943993328e-04,
    4.6168341621969679e-04,
    1.8598657706234230e-03,
    7.0670080707196015e-04,
    -2.2492456754091747e-03,
    -2.7055293027965603e-03,
    1.2307634272343665e-03,
    4.6993269406780981e-03,
    1.6984303126684232e-03,
    -5.1577427312871921e-03,
    -5.9361687426490355e-03,
    2.5904429699616822e-03,
    9.5104864390879260e-03,
    3.3122378905612003e-03,
    -9.7118714382866452e-03,
    -1.0812123641265282e-02,
    4.5715859206989177e-03,
    1.6287321385412081e-02,
    5.5122975619431172e-03,
    -1.5726675339333283e-02,
    -1.7056081487002734e-02,
    7.0329483752597077e-03,
    2.4459678182842035e-02,
    8.0882772704277336e-03,
    -2.2565290379886044e-02,
    -2.3949227569375457e-02,
    9.6706866781569138e-03,
    3.2957303117234021e-02,
    1.0685317933349465e-02,
    -2.9243552530078470e-02,
    -3.0461077862757931e-02,
    1.2077118105660343e-02,
    4.0427773465971463e-02,
    1.2879264031643200e-02,
    -3.4645501075422983e-02,
    -3.5481182001261206e-02,
    1.3834430631479126e-02,
    4.5553192553114567e-02,
    1.4277570256188015e-02,
    -3.7792491047513456e-02,
    -3.8090059866479127e-02,
    1.4617663668474229e-02,
    4.7377897417654163e-02,
    1.4617663668474229e-02,
    -3.8090059866479127e-02,
    -3.7792491047513456e-02,
    1.4277570256188015e-02,
    4.5553192553114567e-02,
    1.3834430631479126e-02,
    -3.5481182001261206e-02,
    -3.4645501075422983e-02,
    1.2879264031643200e-02,
    4.0427773465971463e-02,
    1.2077118105660343e-02,
    -3.0461077862757931e-02,
    -2.9243552530078470e-02,
    1.0685317933349465e-02,
    3.2957303117234021e-02,
    9.6706866781569138e-03,
    -2.3949227569375457e-02,
    -2.2565290379886044e-02,
    8.0882772704277336e-03,
    2.4459678182842035e-02,
    7.0329483752597077e-03,
    -1.7056081487002734e-02,
    -1.5726675339333283e-02,
    5.5122975619431172e-03,
    1.6287321385412081e-02,
    4.5715859206989177e-03,
    -1.0812123641265282e-02,
    -9.7118714382866452e-03,
    3.3122378905612003e-03,
    9.5104864390879260e-03,
    2.5904429699616822e-03,
    -5.9361687426490355e-03,
    -5.1577427312871921e-03,
    1.6984303126684232e-03,
    4.6993269406780981e-03,
    1.2307634272343665e-03,
    -2.7055293027965603e-03,
    -2.2492456754091747e-03,
    7.0670080707196015e-04,
    1.8598657706234230e-03,
    4.6168341621969679e-04,
    -9.5792023943993328e-04,
    -7.4768481245699380e-04,
    2.1893290271722703e-04,
    5.3082205069710680e-04,
    1.1898399936030848e-04,
    -2.1438469575543533e-04,
    -1.3320677588450310e-04,
    2.4158288163475484e-05,
    -4.1384865093955942e-18};


__interrupt void ADCA_ISR(void) {
    int16_t i = 0;
    //adcd1 pie interrupt
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;

    // Here covert ADCIND0, ADCIND1 to volts
    var2 = adca0result/1365.0;
    var3 = adca1result/1365.0;

    xk[0] = var2;
    yk = 0;

    for(i = 0; i<22; i++){
    yk = yk +b[i]*xk[i];
    }
    for(i = 22; i>0 ; i--){
        xk[i] = xk[i-1];
    }
    setDACA(yk);
    setDACB(var3);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00)); //
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    // Print ADCIND0 and ADCIND1’s voltage value to TeraTerm every 100ms

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    count++;
    if ((count % 100) == 0) {

    }
}
