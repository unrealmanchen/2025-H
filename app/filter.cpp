

const float PI = 3.14159265358979323846f;

class LowPassFilter_1_Order{
public:

    LowPassFilter_1_Order(const float Ts, const float Fc) :
        Ts(Ts), Fc(Fc), value(0),a(2 * PI * Fc * Ts / (2 * PI * Fc * Ts + 1))  {}

    LowPassFilter_1_Order(const float a) :
        Ts(0), Fc(0),value(0), a(a){}

    float operator()(const float value) {
        this->value = a * value + (1 - a) * this->value;
        return this->value;
    }

private:
    const float Ts; // Low pass filter time constant, unit s
    const float Fc; // Low pass filter cut-off frequency, unit Hz
    float value;
    const float a; // filter coefficient,default 1(no filter)
};


extern "C" {

void* LowPassFilter_Create_A(float a) {
    return new LowPassFilter_1_Order(a);
}

float LowPassFilter_Process(void* handle, float value) {
    if (handle == 0) return value;
    LowPassFilter_1_Order* filter = static_cast<LowPassFilter_1_Order*>(handle);
    return (*filter)(value);  
}

}