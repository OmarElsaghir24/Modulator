#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "clock.h"
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "spi0.h"
#include "gpio.h"
#include "uart0.h"

// Defines
#define MAX_CHARS 80
#define MAX_FIELDS 6
#define PI 3.14159265358979323846
#define SIZE 16
#define ARRAY_SIZE 4096
#define DATA_SIZE 5
#define DATA_ARRAY_SIZE 1024

uint8_t dataArray[DATA_ARRAY_SIZE] = {0};
#define DAC_MAX 4095
#define VREF 4.096
#define SAMPLES 4096
#define FREQ 10000
#define TIMER_PERIOD (40000000 / (SAMPLES * FREQ))
#define PHASE_RESOLUTION 4294967295
#define SAMPLE_RATE 100000
#define CLOCK_FREQUENCY 40000000
#define FILTER 31
#define INTERP_FACTOR 4

#define BPSK 0
#define QPSK 1
#define PSK8 2
#define QAM16 3
#define SINECOSINE 4
#define CLIPPING 5

#define BPSK_SIZE 2
#define QPSK_SIZE 4
#define PSK8_SIZE 8
#define QAM16_SIZE 16

#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define LDAC PORTE, 5
#define GREEN_LED_MASK 8
#define MAX_CHARS 80
#define MAX_FIELDS 6
#define GAINI 1943
#define GAINQ 1974
#define OFFSETI 2152
#define OFFSETQ 2121
#define H_GAIN 65536

// Global Variables
uint16_t sineLUT[SAMPLES];
uint16_t cosineLUT[SAMPLES];
volatile uint16_t cosine_value;
char bitstream[1024];
int bit_len = 0;
bool running;
int symbol_rate;
volatile uint16_t tempA, tempB;
volatile uint16_t TempA, TempB;
uint32_t index;
char* input;
uint8_t length;
uint8_t qpsk_preamble[8] = {0, 1, 2, 3, 3, 2, 1, 0};
uint32_t phase_acc = 0;
uint32_t phase_step = 0;
volatile uint16_t dacA;
volatile uint16_t dacB;
uint16_t sineIndex, cosineIndex;
char str[100];
uint32_t AMPL = 0, COUNT, RRC;
uint32_t FREQ1 = 0;
uint8_t Mode;
volatile uint16_t VoutA[DAC_MAX];
volatile uint16_t VoutB[DAC_MAX];
float Volt = 0;
char Data[DATA_ARRAY_SIZE];
uint16_t DACA;
uint16_t DACB;
uint8_t phase = 0;
uint16_t x;
uint16_t i_sum = 0;
uint16_t q_sum = 0;
uint16_t dac_i;
uint16_t dac_q;
volatile bool streaming_active = false;
uint8_t symbolsPerMod[4] = {1, 2, 3, 4};
uint8_t datastream[256] = {0};
uint8_t dataSize;
bool enableRRC = false;
float circularI[FILTER] = {0};
float circularQ[FILTER] = {0};
int bufferIndex = 0;
uint16_t rate;
static uint16_t modeIndex = 0;
static bool clipping = false;
uint16_t MAX = 0;
uint16_t CLIP = 0;
uint16_t ON;
uint16_t maxclipA, maxclipB, minclipA, minclipB;

static uint16_t savedModeIndex = 0;
static uint32_t savedPhaseAcc = 0;
static bool isRunning = false;
float maxThreshold = 0;
float minThreshold = 0;

int32_t BPSK_I[2] = {GAINI, -GAINI};
int32_t BPSK_Q[2] = {0, 0};

int32_t QPSK_I[4] = {(int32_t)(GAINI * 0.707), (int32_t)(-GAINI * 0.707), (int32_t)(-GAINI * 0.707), (int32_t)(GAINI * 0.707)};

int32_t QPSK_Q[4] = {(int32_t)(GAINQ * 0.707), (int32_t)(GAINQ * 0.707), (int32_t)(-GAINQ * 0.707), (int32_t)(-GAINQ * 0.707)};

int32_t PSK8_I[8] = {GAINI, (int32_t)(GAINI * 0.707), 0, (int32_t)(-GAINI * 0.707), -GAINI, (int32_t)(-GAINI * 0.707), 0, (int32_t)(GAINI * 0.707)};

int32_t PSK8_Q[8] = {0, (int32_t)(GAINQ * 0.707), GAINQ, (int32_t)(GAINQ * 0.707), 0, (int32_t)(-GAINQ * 0.707), -GAINQ, (int32_t)(-GAINQ * 0.707)};

int32_t QAM16_I[16] = {-GAINI, -GAINI, -GAINI, -GAINI, -GAINI / 3, -GAINI / 3, -GAINI / 3, -GAINI / 3, GAINI / 3, GAINI / 3, GAINI / 3, GAINI / 3, GAINI, GAINI, GAINI, GAINI};

int32_t QAM16_Q[16] = {-GAINQ, -GAINQ / 3, GAINQ / 3, GAINQ, -GAINQ, -GAINQ / 3, GAINQ / 3, GAINQ, -GAINQ, -GAINQ / 3, GAINQ / 3, GAINQ, -GAINQ, -GAINQ / 3, GAINQ / 3, GAINQ};

float rrcCoef[FILTER] = {0.0023*H_GAIN, -0.0043*H_GAIN, -0.0102*H_GAIN, -0.0090*H_GAIN, 0.0015*H_GAIN, 0.0159*H_GAIN, 0.0230*H_GAIN, 0.0130*H_GAIN, -0.0136*H_GAIN, -0.0422*H_GAIN, -0.0493*H_GAIN, -0.0160*H_GAIN, 0.0593*H_GAIN, 0.1553*H_GAIN, 0.2357*H_GAIN, 0.2671*H_GAIN, 0.2357*H_GAIN, 0.1553*H_GAIN, 0.0593*H_GAIN, -0.0160*H_GAIN, -0.0493*H_GAIN, -0.0422*H_GAIN, -0.0136*H_GAIN, 0.0130*H_GAIN, 0.0230*H_GAIN, 0.0159*H_GAIN, 0.0015*H_GAIN, -0.0090*H_GAIN, -0.0102*H_GAIN, -0.0043*H_GAIN, 0.0023*H_GAIN};

// Subroutines
void writeSpi(uint16_t dataA, uint16_t dataB)
{
    writeSpi0Data(dataA);
    writeSpi0Data(dataB);
}

void writeSPI(uint16_t dataA)
{
    writeSpi0Data(dataA);
    setPinValue(LDAC, 0);
    setPinValue(LDAC, 1);
}

// Select which DAC to output
uint16_t ShiftBits(char* Dac, uint16_t bit)
{
    uint16_t temp = 0;
    if(strcmp(Dac,"A") == 0)
    {
        temp |= (0 << 15);
        temp |= (1 << 14);
        temp |= (1 << 13);
        temp |= (1 << 12);
        temp |= (bit);
    } else if(strcmp(Dac,"B") == 0)
    {
        temp |= (1 << 15);
        temp |= (1 << 14);
        temp |= (1 << 13);
        temp |= (1 << 12);
        temp |= (bit);
    }
    return temp;
}

// Generates our sine and cosine Lookup Tables
void generateWaveLUT()
{
    int i;
    float theta, sineValue, cosineValue;
    for (i = 0; i <= 4096; i++)
    {
        cosineLUT[i] = (Volt * cos(((i / 4096.0)*(2*PI)))*-4042) + 2152;
        sineLUT[i] = (Volt * sin(((i / 4096.0)*(2*PI)))*-4020) + 2121;
    }

}

void getRandomNumbers()
{
    uint8_t i;
    uint16_t rand;
    for(i = 0; i < 1024; i++)
    {
        rand = TIMER1_TAV_R;
        Data[i] = rand * i;
    }
}

// Apply RRC filtering to constellations
void applyRRCFilter(int32_t i_value, int32_t q_value, int32_t* filtered_i, int32_t* filtered_q)
{
    int i;
    float sumI = 0;
    float sumQ = 0;
    int readIdx = bufferIndex;
    for(i = 0; i < FILTER; i++)
    {
        readIdx = (readIdx - 1 + FILTER) % FILTER;
        sumI += circularI[readIdx] * rrcCoef[i];
        sumQ += circularQ[readIdx] * rrcCoef[i];
    }
    *filtered_i = (int32_t)(sumI / H_GAIN * INTERP_FACTOR);
    *filtered_q = (int32_t)(sumQ / H_GAIN * INTERP_FACTOR);
}

// Convert strings to bits and then to decimal to make it easier to select points to plot for the constellations
void convert_string_to_bits()
{
    memset(datastream, 0, sizeof(datastream));
    dataSize = 0;

    int p;

    // Add preamble
      uint8_t preamble[] = {
      1, 0, 2, 0, 1, 2, 1, 1,
      1, 2, 3, 0, 1, 2, 3, 0,
      1, 2, 3, 3, 0, 3, 0, 1,
      0, 3, 0, 2, 0, 3, 0, 3
    };
   for (p = 0; p < sizeof(preamble); p++)
   {
	   if (dataSize < sizeof(datastream))
	   {
		   datastream[dataSize++] = preamble[p];
	   }
   }
    uint8_t bitsPerSymbol = symbolsPerMod[Mode];
    uint8_t maxVal = (1 << bitsPerSymbol) - 1;
    uint8_t bitBuffer = 0;
    uint8_t bitCount = 0;
    int i;
    for(i = 0; i < strlen(input) && dataSize < sizeof(datastream); i++)
    {
        char c = input[i];
        int j;
        for(j = 7; j >= 0; j--)
        {
            bitBuffer = (bitBuffer << 1) | ((c >> j) & 1);
            bitCount++;
            if(bitCount == bitsPerSymbol)
            {
                if(dataSize < sizeof(datastream))
                {
                    datastream[dataSize++] = bitBuffer & maxVal;
                }
                bitBuffer = 0;
                bitCount = 0;
            }
        }
    }
    if(bitCount > 0 && dataSize < sizeof(datastream))
    {
        datastream[dataSize++] = bitBuffer & maxVal;
    }
}

// Configure Timer 1 as a periodic timer
void initTimer()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);
}

// Function to update load value for timer 1
void changeLoad(uint32_t rate)
{
    TIMER1_TAILR_R = (80000000 / rate);
}

// Generate sine and cosine waveforms with or without clipping enabled
void generateSineWave()
{
    if(!clipping)
    {
        phase_acc += phase_step;

        DACA = cosineLUT[phase_acc >> 20];

        DACB = sineLUT[phase_acc >> 20];
    }
    else
    {
        phase_acc += phase_step;

        DACA = cosineLUT[phase_acc >> 20];

        if(DACA < maxclipA)
        {
            DACA = maxclipA;
        }
        else if(DACA > minclipA)
        {
            DACA = minclipA;
        }

        DACB = sineLUT[phase_acc >> 20];

        if(DACB < maxclipB)
        {
            DACB = maxclipB;
        }
        else if(DACB > minclipB)
        {
            DACB = minclipB;
        }

    }
    TempA = ShiftBits("A", DACA);
    TempB = ShiftBits("B", DACB);
    writeSpi0Data(TempA);
    writeSpi0Data(TempB);
}

// Output constellations based on bits generated from strings entered through command prompt. Also used for applying filtering to the constellations
void generateModulationSymbol()
{
    uint8_t number = datastream[modeIndex];
    int32_t i_value = 0, q_value = 0, I, Q;
    switch (Mode)
    {
        case BPSK:
            i_value = BPSK_I[number];
            q_value = BPSK_Q[number];
            break;
        case QPSK:
            i_value = QPSK_I[number];
            q_value = QPSK_Q[number];
            break;
        case PSK8:
            i_value = PSK8_I[number];
            q_value = PSK8_Q[number];
            break;
        case QAM16:
            i_value = QAM16_I[number];
            q_value = QAM16_Q[number];
            break;
        default:
            break;
    }
    if (enableRRC)
    {
        int32_t filtered_i, filtered_q;
        static int bufferIndex = 0;
	    int i, n;
	    // Store the current value and add zeros for interpolation
	    for(n = 0; n < INTERP_FACTOR; n++)
	    {
		    if(n == 0)
		    {
			  // Store actual sample
			  circularI[bufferIndex] = (float)i_value;
			  circularQ[bufferIndex] = (float)q_value;
		    } else
		    {
			  // Insert zeros for interpolation
			  circularI[bufferIndex] = 0.0f;
			  circularQ[bufferIndex] = 0.0f;
		    }

		    // Move to next position, wrapping around if needed
		    bufferIndex = (bufferIndex + 1) % FILTER;

		    // If we've filled the buffer, stop
		    if(bufferIndex >= FILTER)
		    {
			   bufferIndex = 0;
		    }
	   }
	   applyRRCFilter(circularI[bufferIndex], circularQ[bufferIndex], &filtered_i, &filtered_q);
        I = ((uint16_t)filtered_i + OFFSETI);
        Q = ((uint16_t)filtered_q + OFFSETQ);
    } else
    {
        I = i_value + OFFSETI;
        Q = q_value + OFFSETQ;
    }
    tempA = ShiftBits("A", I);
    tempB = ShiftBits("B", Q);
    writeSpi0Data(tempA);
    writeSpi0Data(tempB);
    modeIndex = (modeIndex + 1) % dataSize;
}

void startGeneration()
{
    if (!isRunning)
    {
        isRunning = true;
        modeIndex = savedModeIndex;
        phase_acc = savedPhaseAcc;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        putsUart0("Generation started\n");
    } else
    {
        putsUart0("Generation is already running\n");
    }
}

void stopGeneration()
{
    if (isRunning)
    {
        isRunning = false;
        savedModeIndex = modeIndex;
        savedPhaseAcc = phase_acc;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        putsUart0("Generation stopped\n");
    } else
    {
        putsUart0("Generation is already stopped\n");
    }
}

// Timer 1 ISR to either output constellations for each modulation type or plot waveforms
void timer1Isr(void)
{
    setPinValue(LDAC, 0);
    setPinValue(LDAC, 1);
    if (isRunning)
    {
        if (Mode == SINECOSINE)
        {
            generateSineWave();
        } else
        {
            generateModulationSymbol();
        }
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void initHw(void)
{
    initSystemClockTo80Mhz();
    enablePort(PORTE);
    _delay_cycles(3);
}

int main(void)
{
    USER_DATA data;
    initHw();
    initUart0();
    setUart0BaudRate(115200, 80e6);
    initSpi0(USE_SSI0_FSS);
    setSpi0BaudRate(40e6, 80e6);
    setSpi0Mode(1,1);
    initTimer();


    while(true) {
        if(kbhitUart0()) {
            bool valid = false;
            data.fieldCount = 0;
            getsUart0(&data);
            parseFields(&data);

            // Write a DAC value (0-4095) to either DAC A or B
            if(isCommand(&data, "Write DAC DATA", 2))
            {
                valid = true;
                char* DAC = getFieldString(&data, 1);
                uint32_t DATA = getFieldInteger(&data, 2);
                putsUart0("Writing data\n");
                uint16_t bits = ShiftBits(DAC, DATA);
                writeSPI(bits);
                putsUart0("\n");
            }

            // This command is used for deciding which modulation type to select or to plot our waveforms
            if(isCommand(&data, "Mode: ", 1))
            {
                valid = true;
                Mode = getFieldInteger(&data, 1);

                if(Mode == 0)
                {
                	putsUart0("Plotting BPSK constellation\n");
                }
                else if(Mode == 1)
                {
                	putsUart0("Plotting QPSK constellation\n");
                }
                else if(Mode == 2)
                {
                	putsUart0("Plotting 8PSK constellation\n");
                }
                else if(Mode == 3)
                {
                	putsUart0("Plotting 16QAM constellation\n");
                }
                else if(Mode == 4)
                {
                	putsUart0("Plotting sine and cosine waveforms\n");
                }
            }

            // This command is used for inputting our amplitude and frequency for plotting our sine and cosine waveforms.
            // We also input our clipping voltage.
            if(isCommand(&data, "Plot ", 3))
            {
                valid = true;
                changeLoad(100000);
                AMPL = getFieldInteger(&data, 1);
                FREQ1 = getFieldInteger(&data, 2);
                CLIP = getFieldInteger(&data, 3);

                phase_step = (uint32_t)(((uint64_t)FREQ1 * PHASE_RESOLUTION) / SAMPLE_RATE);
                Volt = ((float)AMPL / 1000);
                maxThreshold = ((float)CLIP / 1000);
                minThreshold = -maxThreshold;
                maxclipA = -4041.5 * (maxThreshold)+2152.2;
                maxclipB = -4020.3 * (maxThreshold)+2121.1;
                minclipA = -4041.5 * (minThreshold)+2152.2;
                minclipB = -4020.3 * (minThreshold)+2121.1;
                generateWaveLUT();
                snprintf(str, sizeof(str), "Frequency = %dHz\n", FREQ1);
                putsUart0(str);
                snprintf(str, sizeof(str), "Volts = %.3fmV\n", Volt);
                putsUart0(str);
                snprintf(str, sizeof(str), "Max Clip = %f\n", maxThreshold);
                putsUart0(str);
                snprintf(str, sizeof(str), "Min Clip = %f\n", minThreshold);
                putsUart0(str);
                putsUart0("\n");
            }

            // This command enables and disables clipping on our waveforms
            if(isCommand(&data, "Clipping ON|OFF", 1))
            {
                valid = true;
                ON = getFieldInteger(&data, 1);
                if(ON == 1)
                {
                    clipping = true;
                }
                else
                {
                    clipping = false;
                }
                putsUart0("\n");
            }

            // This command is used to input strings to convert to bits and plot our constellations for each modulation type
            if(isCommand(&data, "Data ", 1))
            {
                valid = true;
                input = getFieldString(&data, 1);
                convert_string_to_bits();
                snprintf(str, sizeof(str), "String = %s\n", input);
                putsUart0(str);
            }

            // This command reads a frequency value to be used as the load for the timer to generate interrupts at diferent rates
            if(isCommand(&data, "Rate ", 1))
            {
                valid = true;
                rate = getFieldInteger(&data, 1);
                snprintf(str, sizeof(str), "Symbol rate set to %d symbols/sec\n", rate);
                putsUart0(str);
                changeLoad(rate);
                putsUart0("\n");
            }

            // Enable streaming
            if(isCommand(&data, "Start", 0))
            {
                valid = true;
                startGeneration();
            }

            // Disable Streaming
            if (isCommand(&data, "Stop", 0))
            {
                valid = true;
                stopGeneration();
            }

            // Enable or disable filtering
            if(isCommand(&data, "rrc ", 1))
            {
                valid = true;
                RRC = getFieldInteger(&data, 1);
                if(RRC == 1)
                {
                    enableRRC = true;
                } else
                {
                    enableRRC = false;
                }
                putsUart0("\n");
            }

            if(!valid)
            {
                putsUart0("Invalid Command\n");
                putsUart0("\n");
            }
        }
    }
}
