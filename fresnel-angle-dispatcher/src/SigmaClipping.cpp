#include "SigmaClipping.h"

float SigmaClipping::filter(float *data, int &size, float sigma) {
    while (true) {
        float avg = mean(data, size);
        float dev = stdDev(data, size, avg);
        float minThreshold = avg - sigma * dev;
        float maxThreshold = avg + sigma * dev;

        float temp[size];
        int newSize = 0;

        for (int i = 0; i < size; i++) {
            if (data[i] >= minThreshold && data[i] <= maxThreshold) {
                temp[newSize++] = data[i];
            }
        }

        if (newSize == size) break;  // No more outliers to remove

        for (int i = 0; i < newSize; i++) {
            data[i] = temp[i];
        }
        size = newSize;
    }
    return mean(data, size);  // Return the mean of the filtered data
}

float SigmaClipping::mean(const float *data, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

float SigmaClipping::stdDev(const float *data, int size, float mean) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += pow(data[i] - mean, 2);
    }
    return sqrt(sum / size);
}
