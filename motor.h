#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int pinEnable;
    int pinDrive1;
    int pinDrive2;
  
  public:
    int motorSpeed;
    void setUp();
    void forward();
    void backward();
    void stop();
    Motor(int, int, int);
    ~Motor();
};

#endif
