#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int pinEnable;
    int pinDrive1;
    int pinDrive2;
    int encoderTickCount;
    void setPinSpeed();
  
  public:
    int motorSpeed;
    bool isMovingForward = false;
    bool isMovingBackward = false;
    void setUp();
    void forward(int);
    void backward(int);
    void step(int);
    void stop();
    int getEncoderTickCount();
    void resetEncoderTickCount();
    void incrementEncoderTickCount();
    void decrementEncoderTickCount();
    Motor(int, int, int);
    ~Motor();
};

#endif
