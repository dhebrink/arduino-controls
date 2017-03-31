#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int pinEnable;
    int pinDrive1;
    int pinDrive2;
    int encoderTickCount;
    int previousEncoderTickCount;
    int getPreviousEncoderTickCount();
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
    int getEncoderTickCountDelta();
    void resetEncoderTickCount();
    void incrementEncoderTickCount();
    void decrementEncoderTickCount();
    Motor(int, int, int);
    ~Motor();
};

#endif
