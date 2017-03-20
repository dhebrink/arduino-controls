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
    void setUp();
    void forward(int);
    void backward(int);
    void stop();
    void incrementEncoderTickCount();
    Motor(int, int, int);
    ~Motor();
};

#endif
