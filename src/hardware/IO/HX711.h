/**
 * \file HX711.h
 * \author Justin Fong
 * \brief  Class representing HX711 load cells amplifiers able to read up load cells and provide their measurments via a
 * two-wire interface (Clock and Data), typically connected on digital pins of the BeagleBone or other SBC.
 * NOTE that each object is able to read up to 4 HX711 (so 4 load cells) with a shared CLOCK pin.
 * \version 0.1
 * \date 2021-01-12
 * \version 0.1
 * \copyright Copyright (c) 2021
 *
 */

#ifndef HX711_h
#define HX711_h

#include <time.h>
#include <unistd.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

#include "InputDevice.h"
#include "iobb.h"

//TODO: Make more generic to handle 1 to N sensors (and maybe change class name accordingly).

class HX711 : public InputDevice {
   private:
    int GAIN;      // amplification factor. Can only have one for all sensors
    Eigen::VectorXi OFFSET;  //= 0;     // used for tare weight
    Eigen::VectorXd SCALE;   //= 1;     // used to return weight in grams, kg, ounces, whatever

    Eigen::Matrix<int, Eigen::Dynamic, 2> inputPins;
    Eigen::Vector2i clockPin;

    Eigen::VectorXd force;  //= 0;
    Eigen::VectorXi rawData;  //= 0;

    // Write a value to the clock pin
    void clockDigitalWrite(bool value);

    // Read a value from a data pin
    uint8_t digitalRead(int sensorNum);

   public:
    /**
    * \brief Create an HX711 object to read up to 4 HX711 boards.
    *
    * \param inputPins a Nx2 Matrix containing the DATA pins of the HX711, each in format {PORT, PIN}
    * \param clockPin a vector of two elements containing the CLOCK pin of the HX711 in format {PORT, PIN}
    */
    HX711(Eigen::Matrix<int, Eigen::Dynamic, 2> inputPins, Eigen::Vector2i clockPin);
    ~HX711();

    void updateInput();

    /**
    * \brief Initialize library with data output pin, clock input pin and gain factor.
    * Channel selection is made by passing the appropriate gain:
    * - With a gain factor of 64 or 128, channel A is selected
    * - With a gain factor of 32, channel B is selected
    * The library default is "128" (Channel A).
    */
    void begin(int gain = 128);

    /**
    * \brief Check if HX711 is ready
    * from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
    * input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
    */
    bool isReady();

    /**
    * \brief Wait for the HX711 to become ready
    */
    void waitReady(unsigned long delay_ns = 0);

    /**
    * \brief Set the gain factor; takes effect only after a call to read()
    * channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
    * depending on the parameter, the channel is also set to either A or B
    */
    void setGain(uint8_t gain = 128);

    /**
    * \brief Get the latest Raw Data (int32_t) from all sensors
    */
    Eigen::VectorXi getAllRawData();

    /**
    * \brief Get the latest Raw Data (int32_t) from a single sensor
    */
    double getRawData(int sensorNum);

    /**
    * \brief Get the latest force measurement from all sensors
    */
    Eigen::VectorXd& getAllForces();

    /**
    * \brief Get the latest force measurement from a single
    */
    double getForce(int sensorNum);

    /**
    * \brief set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
    */
    void setScale(int sensorNum,double scale = 1);

    /**
    * \brief get the current SCALE
    */
    double getScale(int sensorNum);

    /**
    * \brief set OFFSET, the value that's subtracted from the actual reading (tare weight)
    */
    void setOffset(int sensorNum, int32_t offset = 0);

    /**
    * \brief get the current OFFSET
    */
    int32_t getOffset(int sensorNum);

    /**
    * \brief puts the chip into power down mode
    */
    void powerDown();

    /**
    * \brief wakes up the chip after power down mode
    */
    void powerUp();

    /**
    * Return NB of HX711 (strain gauges) configured
    *
    */
    uint32_t nbGauges() { return force.size(); };
};

#endif /* HX711_h */
