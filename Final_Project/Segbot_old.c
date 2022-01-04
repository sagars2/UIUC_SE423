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

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define TIMEBASE 0.005 //computed by 1/200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);
__interrupt void ecap1_isr(void);

void init_eQEPs(void);
void InitECapture(void);

float readEncLeft(void);
float readEncRight(void);

void serialRXA(serial_t *s, char data);
void serialRXD(serial_t *s, char data);
void setupSpib(void);
void hc_sr04_trigger(void);
void setup_led_GPIO(void);


void setEPWM6A(float u_left);
void setEPWM6B(float u_right);

// Count variables
int16_t adca2result = 0;
int16_t adca3result = 0;
float var1 = 0;
float var2 = 0;
int16_t A = 0;
uint16_t SendToOrange = 0;
uint32_t count = 0;
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint32_t numRXD = 0;
uint16_t UARTPrint = 0;
uint16_t rcservo = 1200;
uint16_t rcservo_flag = 0;
uint16_t valuetosend;
uint16_t rc1msb;
uint16_t rc1lsb;
uint16_t rc2lsb;
uint16_t rc2msb;
uint16_t ADC1reading;
uint16_t ADC2reading;
uint16_t ADC1msb;
uint16_t ADC1lsb;
uint16_t ADC2msb;
uint16_t ADC2lsb;
float ADC1_reading;
float ADC2_reading;
uint16_t ADC;
uint32_t SPIB_count;
// Code inside SPIB Interrupt Service Routine
int16_t dummy= 0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;
int32_t echocount = 0; //number of echos



float echoduration = 0; //duration for which the echo lasts
float echodistance = 0;//the distance the echo travels in the given duration
int16_t triggercount = 0; //number of times the trigger is being counted
int16_t triggerstate = 0;

float gyrox = 0;
float gyroy = 0;
float gyroz = 0;
float LeftWheel = 0;
float RightWheel = 0;
int16_t accelXraw = 0;
int16_t accelYraw = 0;
int16_t accelZraw = 0;

float accelx = 0.0;
float accely = 0.0;
float accelz = 0.0;

float XLeft_K = 0.0;
float XRight_K = 0.0;
float left = 9.2;
float right = 9.2;


float uLeft = 0;
float uRight = 0;
float XLeft_K_1 = 0.0;
float XRight_K_1 = 0.0;
float Kp = 3;
float Ki = 20;
float Kd = 0.08;
float err_left = 0.0;
float vref;
float err_right = 0.0;
float Ileft = 0.0;
float prev_Ileft = 0.0;
float prev_Iright = 0.0;
float prev_err_left = 0.0;
float prev_err_right = 0.0;
float Iright = 0.0;
float uleft = 0.0;
float uright = 0.0;
float WhlDiff_1 = 0;
float DVel = 0;
float DVel_1 = 0; //old wheel difference velocity value
float turn = 0; //turn angle
float FwdBckOffset = 0;
float intDiff_1 = 0;
float intDiff = 0;
float turnrate = 0;
float errorDiff_1 = 0;

// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset  = 0;
float gyroy_offset  = 0;
float gyroz_offset  = 0;
float accelzBalancePoint = -.76;
int16 IMU_data[9];
uint16_t temp = 0;
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
float turn_1 = 0;


float K1 = -30.8074;
float K2 = -2.8968;
float K3 = -1.1350;
float ubal = 0;
float u_left = 0;
float u_right = 0;
float vel_Right = 0;
float vel_Left = 0;
float left_motor_angle = 0;
float right_motor_angle = 0;
float turn;
float turn1 = 0;
float DVel;
float e_turn = 0;
float err_turn_left = 0;
float err_turn_right = 0;
float Kp_turn = 3;
float RightWheel_1 = 0;
float LeftWheel_1 = 0;
float vel_Right_1 = 0;
float vel_Left_1 = 0;
float WhlDiff = 0;
float errorDiff = 0;
float turnrate_1 = 0;

#define ORANGEPIBUFFSIZE 128
char RXBuff[ORANGEPIBUFFSIZE];

float Opti_x = 0;
float Opti_y = 0;
float Opti_theta = 0;
float Opti_Body = 0;
float Opti_Stamp = 0;

float yk = 0;
int i = 10;
//b is the filter coefficients

float xk_[10] = {0};
float b[10] = { 1.1982297073578186e-02,
                3.2593697188218529e-02,
                8.8809724362308426e-02,
                1.5903360855022139e-01,
                2.0758067282567344e-01,
                2.0758067282567344e-01,
                1.5903360855022139e-01,
                8.8809724362308426e-02,
                3.2593697188218529e-02,
                1.1982297073578186e-02};  // 0.2 is 1/5th therefore a 5 point average



void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();
    InitGpio();

    setup_led_GPIO();

    // Blue LED on LaunchPad
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
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    // LED3
    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    // LED4
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED5
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED6
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED7
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED8
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED9
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED10
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;

    // LED11
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

    // LED12
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

    // LED13
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;

    // LED14
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;

    // LED15
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;

    // LED16
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;

    // LED17
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED18
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED19
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED20
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED21
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED22
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED23
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //PushButton 1
    GPIO_SetupPinMux(122, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(122, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(123, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(123, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(124, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 1);
    //GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 1);

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB

    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;


    //code for EPWM5Regs
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;


    //Code for EPwm6Regs
    //TBCTL Setup
    EALLOW;
    EPwm6Regs.TBCTL.bit.CTRMODE = 0;
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm6Regs.TBCTL.bit.PHSEN = 0;
    EPwm6Regs.TBCTL.bit.CLKDIV = 0;
    //TBCTR Setup
    EPwm6Regs.TBCTR = 0;
    //CMPA Setup
    EPwm6Regs.CMPA.bit.CMPA = 0;
    EPwm6Regs.CMPB.bit.CMPB = 0;
    //AQCTLA Setup
    EPwm6Regs.AQCTLA.bit.ZRO = 2;
    EPwm6Regs.AQCTLA.bit.CAU = 1;
    EPwm6Regs.AQCTLB.bit.CBU = 1;
    EPwm6Regs.AQCTLB.bit.ZRO = 2;
    EPwm6Regs.TBPRD = 2500;
    //TBPHS Setup (safe practice)
    EPwm6Regs.TBPHS.bit.TBPHS =0;
    EDIS;


    DINT;

    // Trigger pin for HC-SR04
   GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
   GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
   GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;

   // Echo pin for HC-SR04
   EALLOW;
   InputXbarRegs.INPUT7SELECT = 19; // Set eCAP1 source to GPIO-pin
   EDIS;
   GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_ASYNC);

   InitECapture();
   DINT;

    setupSpib();
    init_eQEPs();

    //    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);  // Set as GPIO2 and used as DAN777 SS
    //    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO2 an Output Pin
    //    GpioDataRegs.GPASET.bit.GPIO2 = 1;  //Initially Set GPIO2/SS High so DAN777 is not selected
    //
    //    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);  // Set as GPIO66 and used as MPU-9250 SS
    //    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO66 an Output Pin
    //    GpioDataRegs.GPCSET.bit.GPIO66 = 1;  //Initially Set GPIO66/SS High so MPU-9250 is not selected
    //
    //    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);  //Set GPIO63 pin to SPISIMOB
    //    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);  //Set GPIO64 pin to SPISOMIB
    //    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);  //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63   = 0;  // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64   = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65   = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;  // Set prequalifier for SPI PINS
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;  // The prequalifier eliminates short noise spikes
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;  // by making sure the serial pin stays low for 3 clock periods.
    EDIS;


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
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.ECAP1_INT = &ecap1_isr;


    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    //ConfigCpuTimer(&CpuTimer1, 200, 4000);
    //ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    //CpuTimer1Regs.TCR.all = 0x4000;
    //CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
    //    init_serial(&SerialC,115200,serialRXC);
    init_serial(&SerialD,1500000,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT4;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;//new

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //Enable SPIB_RX in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    //Enable ADCA1 in the PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
        {
            if (UARTPrint == 1 ) {
                serial_printf(&SerialA,"%s \r\n",RXBuff);
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

    // Insert SWI ISR Code here.......

    WhlDiff = LeftWheel - RightWheel;

    DVel = 0.333*DVel_1 + 166.667*(WhlDiff - WhlDiff_1);
    //turn = turn_1 + (((turnrate+turnrate_1)*0.004)/2);
    errorDiff = turn - WhlDiff;
    intDiff = intDiff_1 + (((errorDiff+errorDiff_1)*0.004)/2);

    //turn = Kp*errorDiff + Ki*intDiff - Kd*DVel;

    if(fabs(turn1 >= 3)){
        intDiff = intDiff_1;
    }
    if(turn1>=4){
        turn1 = 4;
    }
    if(turn1 <= -4){
        turn1 = -4;
    }

    vel_Right  = 0.6*vel_Right_1 + 100*(RightWheel-RightWheel_1);
    vel_Left  = 0.6*vel_Left_1 + 100*(LeftWheel-LeftWheel_1);

    ubal = -(K1*tilt_value)-(K2*gyro_value)-(K3*((vel_Right+vel_Left)/2));

    u_left = ubal/2 + turn1 +FwdBckOffset;
    u_right = ubal/2 - turn1 +FwdBckOffset;

    setEPWM6A(u_left);
    setEPWM6B(-u_right);

    WhlDiff_1 = WhlDiff;
    DVel_1 = DVel;
    RightWheel_1 = RightWheel;
    LeftWheel_1 = LeftWheel;
    vel_Right_1 = vel_Right;
    vel_Left_1 = vel_Left;
    intDiff_1 = intDiff;
    errorDiff_1 = errorDiff;
    turnrate_1 = turnrate;
    turn_1 = turn;
 //sets condition for segbot to stop if the echo distance is less than or equal to 15 cm
    if(echodistance <= 15){
        FwdBckOffset = 0;
        turnrate = 0;
    }

    numSWIcalls++;
    //UARTPrint = 1;
    DINT;
}


__interrupt void ecap1_isr(void){
    PieCtrlRegs.PIEACK.all    = PIEACK_GROUP4;  // Must acknowledge the PIE group

    ECap1Regs.ECCLR.bit.INT   = 1;              // Clear the ECAP1 interrupt flag
    ECap1Regs.ECCLR.bit.CEVT3 = 1;              // Clear the CEVT3 flag

    // The eCAP is running at the full 200 MHz of the device.
    // Captured values reflect this time base.

    // Compute the PWM duty period (rising edge to falling edge)
    echocount = (int32)ECap1Regs.CAP2 - (int32)ECap1Regs.CAP1;

    echoduration = (float)(TIMEBASE * echocount); // in us

    echodistance = echoduration * 0.034 / 2.0; // cm

    if(echocount > 0){
        UARTPrint = 1;
    }
    if(echoduration > 0){
        UARTPrint = 1;
    }
    if(echodistance > 0){
        UARTPrint = 1;
    }
}

void setup_led_GPIO(void) {

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    numTimer0calls++;

    //To trigger the ultrasonic sensor to start

    hc_sr04_trigger();

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    //if ((CpuTimer0.InterruptCount % 200) == 0) {
    //  UARTPrint = 1;
    //}
}


int16_t spivalue1 = 0;
int16_t spivalue2 = 0;

__interrupt void SPIB_isr(void){


    dummy = SpibRegs.SPIRXBUF;
    accelXraw = SpibRegs.SPIRXBUF;
    accelYraw = SpibRegs.SPIRXBUF;
    accelZraw = SpibRegs.SPIRXBUF;
    accelx = accelXraw*4.0/32767.0;
    accely = accelYraw*4.0/32767.0;
    accelz = accelZraw*4.0/32767.0;

    dummy = SpibRegs.SPIRXBUF;

    gyroXraw = SpibRegs.SPIRXBUF;
    gyroYraw = SpibRegs.SPIRXBUF;
    gyroZraw = SpibRegs.SPIRXBUF;
    gyrox = gyroXraw*250.0/32767.0;
    gyroy = gyroYraw*250.0/32767.0;
    gyroz = gyroZraw*250.0/32767.0;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
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

        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;

        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;

        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;

        // Update Step
        z = -accelz;
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;

        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = -readEncRight();

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

        UARTPrint = 1;  // Tell While loop to print
    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}


// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    numTimer1calls++;

}
// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    CpuTimer2.InterruptCount++;

}
void hc_sr04_trigger(void) {

    if (triggercount < 2) {

        if (triggerstate == 0) {
            // last for 2ms
            GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
            triggerstate = 1;
        }
        triggercount++;

    } else if ( (triggercount >= 2) && (triggercount < 11) ) {

        // last for 10ms
        if (triggerstate == 1) {
            GpioDataRegs.GPASET.bit.GPIO0 = 1;
            triggerstate = 2;
        }
        triggercount++;

    } else {

        if (triggerstate == 2) {
            triggercount = 0;
            triggerstate = 0;
        }

    }
}

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
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep unaffected by emulation suspend in Code Composer
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

    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 18.7.
    return (raw*((TWOPI/1496)));
}

float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 20 North South magnet poles in the encoder disk so 20 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 80 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 18.7.
    return (raw*((TWOPI/1496)));
}

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    if (data == 'q') {
        turnrate = turnrate - 0.2;
    } else if (data == 'r') {
        turnrate = turnrate + 0.2;
    } else if (data == '3') {
        FwdBckOffset = FwdBckOffset - 0.2;
    } else if (data == 's') {
        FwdBckOffset = FwdBckOffset + 0.2;
    } else {
        turnrate = 0;
        FwdBckOffset = 0;
    }
}

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
        if (data == '#') {
            RXBuff[numRXD] = '\0';
            sscanf(RXBuff,"%f", & turn);
            sscanf(RXBuff,"%f",& DVel);
            numRXD = 0;
            UARTPrint = 1;
        } else {
            RXBuff[numRXD] = data;
            numRXD ++;
        }

    } else {
        numRXD = 0;
        UARTPrint = 1;
    }

}

// InitECapture - Initialize ECAP1 configurations
void InitECapture() {

    EALLOW;
    DevCfgRegs.SOFTPRES3.bit.ECAP1 = 1;     // eCAP1 is reset
    DevCfgRegs.SOFTPRES3.bit.ECAP1 = 0;     // eCAP1 is released from reset
    EDIS;

    ECap1Regs.ECEINT.all           = 0;     // Disable all eCAP interrupts
    ECap1Regs.ECCTL1.bit.CAPLDEN   = 0;     // Disabled loading of capture results
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Stop the counter

    ECap1Regs.TSCTR  = 0;                   // Clear the counter
    ECap1Regs.CTRPHS = 0;                   // Clear the counter phase register

    // ECAP control register 2
    ECap1Regs.ECCTL2.all = 0x0096; //OB0000000010010110
    // bit 15-11     00000:  reserved
    // bit 10        0:      APWMPOL, don't care
    // bit 9         0:      CAP/APWM, 0 = capture mode, 1 = APWM mode
    // bit 8         0:      SWSYNC, 0 = no action (no s/w synch)
    // bit 7-6       10:     SYNCO_SEL, 10 = disable sync out signal
    // bit 5         0:      SYNCI_EN, 0 = disable Sync-In
    // bit 4         1:      TSCTRSTOP, 1 = enable counter
    // bit 3         0:      RE-ARM, 0 = don't re-arm, 1 = re-arm
    // bit 2-1       11:     STOP_WRAP, 11 = wrap after 4 captures
    // bit 0         0:      CONT/ONESHT, 0 = continuous mode

    // ECAP control register 1
    ECap1Regs.ECCTL1.all = 0xC144; //OB1100000101000100
    // bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
    // bit 13-9      00000:  PRESCALE, 00000 = divide by 1
    // bit 8         1:      CAPLDEN, 1 = enable capture results load
    // bit 7         0:      CTRRST4, 0 = do not reset counter on CAP4 event
    // bit 6         1:      CAP4POL, 0 = rising edge, 1 = falling edge
    // bit 5         0:      CTRRST3, 0 = do not reset counter on CAP3 event
    // bit 4         0:      CAP3POL, 0 = rising edge, 1 = falling edge
    // bit 3         0:      CTRRST2, 0 = do not reset counter on CAP2 event
    // bit 2         1:      CAP2POL, 0 = rising edge, 1 = falling edge
    // bit 1         0:      CTRRST1, 0 = do not reset counter on CAP1 event
    // bit 0         0:      CAP1POL, 0 = rising edge, 1 = falling edge

    // Enable desired eCAP interrupts
    ECap1Regs.ECEINT.all = 0x0008; //000000000000100
    // bit 15-8      0's:    reserved
    // bit 7         0:      CTR=CMP, 0 = compare interrupt disabled
    // bit 6         0:      CTR=PRD, 0 = period interrupt disabled
    // bit 5         0:      CTROVF, 0 = overflow interrupt disabled
    // bit 4         0:      CEVT4, 0 = event 4 interrupt disabled
    // bit 3         1:      CEVT3, 1 = event 3 interrupt enabled
    // bit 2         0:      CEVT2, 0 = event 2 interrupt disabled
    // bit 1         0:      CEVT1, 0 = event 1 interrupt disabled
    // bit 0         0:      reserved
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3.  Change so that 16 bits are transmitted each TX FIFO write and change the delay in between each transfer to 0.
    //Also don’t forget to cut and paste the GPIO settings for GPIO2, 63, 64, 65, 66 which are also a part of the SPIB setup.

    SpibRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1;  //This happens to be the mode for both the DAN777 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;  //The MPU-9250,  Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;  // Set to transmit and receive 8 bits each write to SPITXBUF
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

    SpibRegs.SPIFFCT.bit.TXDLY = 0 ; //Set delay between transmits to 16 spi clocks. Needed by DAN777 chip

    SpibRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1;    // Enables SPI interrupt.  !! I don’t think this is needed.  Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 5; //Interrupt Level to 5 words or more received into FIFO causes interrupt

    //-----------------------------------------------------------------------------------------------------------------

    //GPIO Setup for GPIO2,66,63,64,65

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);  // Set as GPIO2 and used as DAN777 SS
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO2 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO2 = 1;  //Initially Set GPIO2/SS High so DAN777 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);  // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;  //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);  //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);  //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);  //Set GPIO65 pin to SPICLKB

    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F.  Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers

    SpibRegs.SPITXBUF = (0x1300 | 0x0000);
    SpibRegs.SPITXBUF = (0x0000);
    SpibRegs.SPITXBUF = (0x0000);
    SpibRegs.SPITXBUF = (0x0013);
    SpibRegs.SPITXBUF = (0x0200);
    SpibRegs.SPITXBUF = (0x0806);
    SpibRegs.SPITXBUF = (0x0000);

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
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

    SpibRegs.SPITXBUF = (0x2300 | 0x0040);
    SpibRegs.SPITXBUF = (0x8C02);
    SpibRegs.SPITXBUF = (0x880C);
    SpibRegs.SPITXBUF = (0x0A00);


    // To address 00x23 write 0x00
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    // To address 00x26 write 0x02
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
//This is the take home exercise.
//Two functions created for each motor. Logic stated in the lab has been applied. Void functions ensure that a return statement is not needed.

void setEPWM6A(float u_left){
    if(u_left>10.0){
        u_left = 10.0;
    }
    if(u_left<-10.0){
        u_left = -10.0;
    }
    if(u_left >= 0){
        GpioDataRegs.GPASET.bit.GPIO29 = 1;//This changes it to 1 or High
    }
    else{
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;//This changes it to 0 or Low
    }

    EPwm6Regs.CMPA.bit.CMPA = (int)(fabs(u_left)*(2500.0/10.0));//absolute value taken here and this is a float value since I added the decimals. So a float can be entered in the registry to change the CMPA Value)
}

//the same steps have been copied below except it is being applied to GPIO 32 instead of 29.
void setEPWM6B(float u_right){
    if(u_right>10.0){
        u_right = -10.0;
    }
    if(u_right<-10.0){
        u_right = -10.0;
    }
    if(u_right >= 0){
        GpioDataRegs.GPBSET.bit.GPIO32 = 1;
    }
    else{
        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
    }
    EPwm6Regs.CMPB.bit.CMPB = (int)(fabs(u_right)*(2500.0/10.0));
}

// This function is called each time a char is received over UARTA.

__interrupt void ADCA_ISR (void)
{
    adca2result = AdcaResultRegs.ADCRESULT0;
    adca3result = AdcaResultRegs.ADCRESULT1;
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    // Here covert ADCINA2, ADCINA3 to volts
    var1 = adca2result/4095.0*3.0;
    var2 = adca3result/4095.0*3.0;
    xk_[0] = var1;
    //yk = b[0]*xk_[0] + b[1]*xk_[1] + b[2]*xk_[2] + b[3]*xk_[3] + b[4]*xk_[4];

    yk = 0;
    for (i=0; i<10; i++){
        yk = yk + b[i]*xk_[i];
    }

    //Save past states before exiting from the function so that next sample they are the older state
    for (i = 9; i>0; i--){
        xk_[i] = xk_[i-1];
    }
    //    xk_4 = xk_3;
    //    xk_3 = xk_2;
    //    xk_2 = xk_1;
    //    xk_1 = xk;

//    if (count % 4 == 0) {
        //start the SPI transmission and reception of the three accelerometer readings and the 3 gyro readings
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Clear GPIO66 Low to act as a Slave Select to select MPU9250 chip
        SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when eight values are in the RX FIFO

        SpibRegs.SPITXBUF = ((0x8000) | (0x3A00));
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
        SpibRegs.SPITXBUF = 0;
    //}
    // Here write yk to DACA channel
    //setDACA(yk);
    // Print ADCINA2 and ADCINA3’s voltage value to TeraTerm every 100ms
    //if ((count % 100) == 0){
    //  UARTPrint = 1;
    //}
    count++;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1; //clearing GPIO52
}

//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu);   // DACA will now output 2.25 Volts



//void setDACA(float dacouta0) {
//    if (dacouta0 >  3.0) dacouta0 =  3.0;
//    if (dacouta0 < 0.0) dacouta0 = 0.0;
//
//    DacaRegs.DACVALS.bit.DACVALS = dacouta0*4095/3;  // perform scaling of 0-3 to 0-4095
//}
//
//void setDACB(float dacouta1) {
//    if (dacouta1 >  3.0) dacouta1 =  3.0;
//    if (dacouta1 < 0.0) dacouta1 = 0.0;
//
//    DacbRegs.DACVALS.bit.DACVALS = dacouta1*4095/3;  // perform scaling of 0-3 to 0-4095
//}


