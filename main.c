

/**
 * main.c
 */

#include "F28x_Project.h"
#include "math.h"

#define SAMPLE_FREQ 10000.f // Timer interruption frequency
#define SIGNAL_FREQ 60.f
#define CPU_FREQ    200000000.f
#define PI          3.141592

void cpu_timer0_isr(void);
void ConfigureGpio(void);
void ConfigureDac(void);
void ConfigureAdc(void);
void ConfigureTimer(void);

float PLL(float vin, float vo, float delta_time);
float notchFilter(float vin,  float tetha, float delta_time);
float ePLL(float vin, float delta_time);


void Init(void) {
    ConfigureGpio();
    ConfigureDac();
    ConfigureAdc();
    ConfigureTimer();
}

void TimerCallback(void) {
    static float time = 0, delta_time = 1.f/SAMPLE_FREQ;
    const  float signal_period = 1.f/SIGNAL_FREQ;
    Uint16 signal;
    int vadc;
    float vin;

    signal = (uint16_t)(2000*sin(2*PI*SIGNAL_FREQ*time)+2048);
    vadc = signal - 2048;
    vin  = vadc / 2048.f;
    DacaRegs.DACVALS.all = (Uint16)(2048*vin + 2048);
    DacbRegs.DACVALS.all = (Uint16)(2048.f*ePLL(vin, delta_time) + 2048);

    time += delta_time;
    if(time >= signal_period) time -= signal_period;
}

int main(void) {
	InitSysCtrl();

	Init();

	while(1) {

	}
}

void ConfigureGpio(void) {
    InitGpio();
    EALLOW;
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, 0);
    GPIO_SetupPinOptions(59, GPIO_INPUT, GPIO_PULLUP);
    EDIS;
}

void ConfigureTimer(void) {
    DINT;

    InitPieCtrl();
    IER = 0;
    IFR = 0;
    InitPieVectTable();

    // CPU frequency * interruption period (in seconds)
    CpuTimer0Regs.PRD.all = (uint32_t)(CPU_FREQ/SAMPLE_FREQ);
    CpuTimer0Regs.TCR.bit.TSS  = 1; // 1 = Stop timer, 0 = Start/Restart Timer
    CpuTimer0Regs.TCR.bit.TRB  = 1; // 1 = reload timer
    CpuTimer0Regs.TCR.bit.SOFT = 0;
    CpuTimer0Regs.TCR.bit.FREE = 0; // Timer Free Run Disabled
    CpuTimer0Regs.TCR.bit.TIE  = 1; // 0 = Disable/ 1 = Enable Timer Interrupt
    CpuTimer0Regs.TCR.bit.TSS  = 0; // 1 = Stop timer, 0 = Start/Restart Timer

    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    IER |= M_INT1;

    EINT;
    ERTM;
}

void ConfigureAdc(void) {
    EALLOW;
    // Set ADCCLK divider to /4
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //  Power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    // ADCINA0
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    // sample duration of 20 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19;
    // Timer 0
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;
    EDIS;
}

void ConfigureDac(void) {
    EALLOW;
    // Use adc references
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    // Enable DAC
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
    // Use adc references
    DacbRegs.DACCTL.bit.DACREFSEL = 1;
    // Enable DAC
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
    EDIS;
}

__interrupt void cpu_timer0_isr(void) {
    TimerCallback();
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

float PLL(float vin, float vo, float delta_time) {
    static float angle = 0, integral = 0;
    const float kp = 1, ki = 1000, wc = 2*PI*SIGNAL_FREQ;
    float Epd, Vlf, w;

    Epd = (vin - vo) * sin(angle);
    integral += Epd*delta_time;
    Vlf = kp*Epd + ki*integral;
    w   = Vlf + wc;
    angle += w*delta_time;

    if(angle >= 2*PI) angle -= 2*PI;
    return angle;
}

float notchFilter(float vin, float angle, float delta_time) {
    static float vo = 0, integral = 0;
    const  float k = 50.f;
    float cosine, ve;

    ve = vin - vo;
    cosine = -cos(angle);
    integral += k*(ve*cosine)*delta_time;
    vo = integral*cosine;
    return vo;
}

float ePLL(float vin, float delta_time) {
    static float vo = 0, angle = 0;
    vo = notchFilter(vin, angle, delta_time);
    angle = PLL(vin, vo, delta_time);
    return vo;
}
