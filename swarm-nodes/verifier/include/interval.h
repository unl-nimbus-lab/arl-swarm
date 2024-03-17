#ifndef INTERVAL
#define INTERVAL

#include <iostream>



class interval {
    /* Interval class defines a domain representing a hyper-rectangle face */
    public:
        interval(float minValue, float maxValue) {
            min = minValue;
            max = maxValue;
        }
        float getIntervalMin();
        float getIntervalMax();
        void setIntervalMin(float min_value);
        void setIntervalMax(float max_value);
        void disp();

        interval(const interval& input) {
            /* Deep copy constructor */
            min = input.min;
            max = input.max;
        }
        
        float min;
        float max;
};

float interval::getIntervalMin() {
    return min;
}
float interval::getIntervalMax() {
    return max;
}
void interval::setIntervalMin(float min_value) {
    min = min_value;
}
void interval::setIntervalMax(float max_value) {
    max = max_value;
}
void interval::disp() {
    std::cout << "Interval: " << min << ", " << max << std::endl;
}


#endif