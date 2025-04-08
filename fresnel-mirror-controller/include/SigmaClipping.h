#ifndef SIGMA_CLIPPING_H
#define SIGMA_CLIPPING_H

#include <Arduino.h>
#include <math.h>

class SigmaClipping {
public:
    static float filter(float *data, int &size, float sigma = 2.0);
    static float mean(const float *data, int size);
    static float stdDev(const float *data, int size, float mean);
};

#endif // SIGMA_CLIPPING_H