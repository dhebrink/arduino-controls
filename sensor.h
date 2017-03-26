#ifndef SENSOR_H
#define SENSOR_H

class Sensor {

    private:
        int triggerPin;
        int echoPin;

    public:
        void setUp();
        void initializePins();
        int getDistanceInches();
        Sensor(int, int);
        ~Sensor();
};

#endif
