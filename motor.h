#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int pinEnable;
    int pinDrive1;
    int pinDrive2;
    long long encoderTickCount;
    void setPinSpeed();
  
  public:
    int motorSpeed;
    bool isMovingForward = false;
    bool isMovingBackward = false;
    void setUp();
    void forward(int);
    void backward(int);
    void stop();
    long long getEncoderTickCount();
    void incrementEncoderTickCount();
    void decrementEncoderTickCount();
    Motor(int, int, int);
    ~Motor();
};

#endif
