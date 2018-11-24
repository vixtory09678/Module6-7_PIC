#ifndef MATH_CAL_H
#define MATH_CAL_H

// #define RESOLUTION 600.0
// #define LEAD 8.0 // Lead length

float chirpSineFreq(float time, float freq0, float freq1)
{
    // sin(2(pi)(f)(t^2))
    float o1 = 2 * pi;
    float o2 = freq0 * time;
    float o3 = (time * time) / 2.0f;
    float o4 = (freq1 - freq0) / TIME_OUT;
    float signal = sin(o1 * (o2 + (o3 * o4))) * INPUT_VOLTAGE;
    return signal;
}

double distance(int pulse)
{
    double dis = ((float)pulse / RESOLUTION) * LEAD;
    return dis;
}

float convertVoltToPwm(float volt)
{
    return (volt * 100f) / 12f;
}

#endif