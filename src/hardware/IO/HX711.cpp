#include "HX711.h"


#define HIGH true
#define LOW false

HX711::HX711(Eigen::Matrix<int, Eigen::Dynamic, 2> inputPins, Eigen::Vector2i clockPin) {
    iolib_init();
    iolib_setdir(clockPin[0], clockPin[1], BBBIO_DIR_OUT);
    spdlog::info("Clock Pin P{}.{}", clockPin[0], clockPin[1]);

    GAIN = 0;                                      // amplification factor
    OFFSET = Eigen::VectorXi::Zero(inputPins.rows());  //= 0;     // used for tare weight
    SCALE = Eigen::VectorXd::Zero(inputPins.rows());   //= 1;     // used to return weight in grams, kg, ounces, whateverc
    force = Eigen::VectorXd::Zero(inputPins.rows());
    rawData = Eigen::VectorXi::Zero(inputPins.rows());

    for (int i = 0; i < inputPins.rows(); i++) {
        iolib_setdir(inputPins(i,0), inputPins(i,1), BBBIO_DIR_IN);
        SCALE(i) = 1;
        OFFSET(i) = 0;
        spdlog::info("Input Pin P{}.{}", inputPins(i, 0), inputPins(i, 1));
    }

    this->inputPins = inputPins;
    this->clockPin = clockPin;


    clockDigitalWrite(HIGH);
    usleep(10000);
    clockDigitalWrite(LOW);
}

HX711::~HX711() {
}

//half
void HX711::begin(int gain) {
    setGain(gain);
}

void HX711::updateInput() {
    spdlog::trace("HX711::updateInput(), {}.{}", clockPin[0],clockPin[1]);
    // Wait for the chip to become ready.
    timespec startTime;
    timespec currTime;
    // Wait for the chip to become ready.
    waitReady(1000);

    // Define structures for reading data into.
    unsigned long data[inputPins.rows()] = {0};

    double elapsedMS =0;

    bool goodReading = true;

    for (int i = 0; i < 24; i++) {
        clock_gettime(CLOCK_MONOTONIC, &startTime);
        clockDigitalWrite(HIGH);
       // usleep(1);
        clockDigitalWrite(LOW);
        clock_gettime(CLOCK_MONOTONIC, &currTime);
        elapsedMS = (currTime.tv_sec - startTime.tv_sec) * 1e6 + (currTime.tv_nsec - startTime.tv_nsec) / 1e3;
        if (elapsedMS > 50){
            spdlog::warn("Possible Mistime, skipping reading");
            goodReading = true;
        }

        for (int j = 0; j < inputPins.rows(); j++){
            data[j] |= digitalRead(j) << (24 - i);
        }

        usleep(1);
    }

    // First Pulse
    clockDigitalWrite(HIGH);
    //usleep(1);
    clockDigitalWrite(LOW);
    usleep(1);

    // At this point, everything should be high
    for (int j = 0; j < inputPins.rows(); j++){
        if(digitalRead(j) != 1){spdlog::warn("Problems with {}", j);}
    }

    // Set the channel and the gain factor for the next reading using the clock pin.
    for (int i = 0; i < GAIN-1; i++) {
        clockDigitalWrite(HIGH);
        //usleep(2);
        clockDigitalWrite(LOW);
        usleep(2);
    }
    // Replicate the most significant bit to pad out a 32-bit signed integer
    if(goodReading){
        for (int j = 0; j < inputPins.rows(); j++) {
            if (data[j] & 0x800000) {
                data[j] |= 0xFF000000;
            }

            // Sometimes errorneous data comes in. Usually 0xFFFFFFFE or 0xFFFFE. Ignore if this is the case
            // TODO: Better comparison might be if all of them are equal
            if ((data[j] & 0xFFE) == 0xFFE) {
                spdlog::warn("Possible Error on {0}, {1:x}", j, data[j]);
                // Do not update values
            } else {
                rawData(j) = static_cast<int32_t>(data[j]);
                force(j) = (rawData(j) - OFFSET(j))*SCALE(j);
            }
        }
    }
}



void HX711::clockDigitalWrite(bool value) {
    if (value) {
        pin_high(clockPin[0], clockPin[1]);
    } else {
        pin_low(clockPin[0], clockPin[1]);
    }
}

uint8_t HX711::digitalRead(int sensorNum) {

    if (is_high(inputPins(sensorNum, 0), inputPins(sensorNum, 1))) {
        //printf("1");
        return true;
    } else {
        //printf("0");
        return false;
    }
}

//no
bool HX711::isReady() {
    bool returnValue = true;

    for (int j =0; j< inputPins.rows(); j++){
        returnValue &= (digitalRead(j)==LOW);
    }
    return returnValue;
}

//go
void HX711::setGain(uint8_t gain) {
    switch (gain) {
        case 128:  // channel A, gain factor 128
            GAIN = 1;
            break;
        case 64:  // channel A, gain factor 64
            GAIN = 3;
            break;
        case 32:  // channel B, gain factor 32
            GAIN = 2;
            break;
    }
}


//go
//TODO: How compatible is this with RT requirement? Is that necessary to be 1ms ? Can we change it?
void HX711::waitReady(unsigned long delay_ns) {
    while (!isReady()) {
        usleep(delay_ns);
    }
}

//go
Eigen::VectorXi HX711::getAllRawData() {
    return rawData;
}

//go
double HX711::getRawData(int sensorNum) {
    return rawData(sensorNum);
}

Eigen::VectorXd& HX711::getAllForces() {
    return force;
}

//go
double HX711::getForce(int sensorNum) {
    return force(sensorNum);
}

//go
void HX711::setScale(int sensorNum,double scale) {
    SCALE[sensorNum] = scale;
}

//go
double HX711::getScale(int sensorNum) {
    return SCALE(sensorNum);
}

//go
void HX711::setOffset(int sensorNum, int32_t offset) {
    OFFSET(sensorNum) = offset;
    spdlog::debug("OffsetSet {}, {}", sensorNum, OFFSET(sensorNum));
}

//go
int32_t HX711::getOffset(int sensorNum) {
    return OFFSET(sensorNum);
}

//no
void HX711::powerDown() {
    clockDigitalWrite(LOW);
    clockDigitalWrite(HIGH);
}

//no
void HX711::powerUp() {
    clockDigitalWrite(LOW);
}
