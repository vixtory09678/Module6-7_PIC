#ifndef INTERRUPT_H
#define INTERRUPT_H

extern volatile long pulseX;
extern volatile long pulseY;

void initTimer()
{
    setup_timer1(TMR_INTERNAL | TMR_DIV_BY_8, 20000);
    enable_interrupts(INT_TIMER1);
    setup_timer3(TMR_INTERNAL | TMR_DIV_BY_256, 1250);
    enable_interrupts(INT_TIMER3);
    setup_timer4(TMR_INTERNAL | TMR_DIV_BY_256);
    set_timer4(0);
}

void setupEncoderInterrupt()
{
    enable_interrupts(INT_EXT1);
    ext_int_edge(1, H_TO_L);
    enable_interrupts(INT_EXT0);
    ext_int_edge(0, H_TO_L);
}

void setSerialInterrupt()
{
    enable_interrupts(INT_RDA);
}

// PIN_B4
#INT_EXT1
void INT_EXT_INPUT1(void)
{
    if (input(PIN_B5) == 1)
    {
        pulseX++;
    }
    else
    {
        pulseX--;
    }
}

// PIN_B7
#INT_EXT0
void INT_EXT_INPUT2(void)
{
    if (input(PIN_B6) == 0)
    {
        pulseY++;
    }
    else
    {
        pulseY--;
    }
}

#endif