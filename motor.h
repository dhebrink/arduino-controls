#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int pinEnable;
    int pinDrive1;
    int pinDrive2;
    void setPinSpeed();
  
  public:
    int motorSpeed;
    void setUp();
    void forward(int);
    void backward(int);
    void stop();
    Motor(int, int, int);
    ~Motor();
};

#endif
