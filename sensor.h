#ifndef SENSOR_H
#define SENSOR_H

class Sensor {

    private:
        int triggerPin;
        int echoPin;

    public:
        void setUp();
        void initializePins();
        int getDistance();
        Sensor(int, int);
        ~Sensor();
};

#endif
