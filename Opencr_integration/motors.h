#ifndef MOTORS_H_
#define MOTORS_H_

#include "Arduino.h"

template <typename T>
T limit(T val, T min, T max) {
    if (val > max) {
        val = max;
    } else if (val < min) {
        val = min;
    }
    return val;
}

enum Motors {
    BASE_MOTOR,
    PWM_MOTOR,
    PAIRED_BASE_MOTOR,
    PAIRED_PWM_MOTOR,
    ENCODER_MOTOR,
    PULSE_CONTROLLED_MOTOR,
    PAIRED_PULSE_CONTROLLED_MOTOR
};

enum Direction {
    REVERSE = -1,
    STOP,
    FORWARD,
};

class BaseMotor {
   protected:
    const uint8_t pin1_;
    const uint8_t pin2_;
    Motors motor_type_;

   public:
    BaseMotor() = delete;
    BaseMotor(const uint8_t, const uint8_t);
    virtual Motors motorType() const;
    virtual void control(const Direction);
};

class PWMMotor : public BaseMotor {
   private:
    const uint8_t pwm_pin_;
    int max_pwm_;

   public:
    PWMMotor() = delete;
    PWMMotor(const uint8_t, const uint8_t, const uint8_t);
    virtual void control(const Direction, int);
    PWMMotor &setMaxPWM(const int);
};

//Paired 모터는 두개의 모터가 동시에 같은 신호로 제어되는 모터이다. Paired Base motor는 리니어 액추에이터 두개를 제어한다.
class PairedBaseMotor : public BaseMotor {
   private:
    BaseMotor motor2_;

   public:
    PairedBaseMotor() = delete;
    PairedBaseMotor(const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    void control(const Direction) override;
};

// Paired Pwm 모터는 두개의 DC 모터를 같은 신호로 동일 하게 제어할 때 사용된다.
class PairedPWMMotor : public PWMMotor {
   private:
    PWMMotor motor2_;

   public:
    PairedPWMMotor() = delete;
    PairedPWMMotor(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    void control(const Direction, int) override;
};

class Encoder {
   protected:
    const uint8_t interrupt_pin_;
    const uint8_t digital_pin_;
    long cnt_;
    bool state_;
    bool direction_;
    int resolution_;

   public:
    Encoder() = delete;
    Encoder(const uint8_t, const uint8_t);
    Encoder &counter();
    Encoder &setResolution(const int);
    long getCnt() const;
};

// Encoder motor는 말그대로 encoder가 달린 DC 모터를 정밀 제어 할때 사용한다.
class EncoderMotor : public PWMMotor, public Encoder {
   private:
    double angular_velocity_;
    double target_velocity_;
    int pwm_;
    long prev_cnt_;
    uint8_t control_period_;  // ms 제어주기
    double kp_;

   public:
    EncoderMotor() = delete;
    EncoderMotor(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    EncoderMotor &setControlPeriod(const uint8_t);
    EncoderMotor &setKp(const double);
    EncoderMotor &setTarget(const double);
    EncoderMotor &run();
    double getAngularVelocity() const;
    uint8_t getPWM() const;
};

// Pulse Controlled motor는 현재 캐리어에 사용되는 스텝모터에 사용한다. 분해능에 맞게 펄스를 주면 헤드가 이동한다.
class PulseControlledMotor : public BaseMotor {
   private:
    const uint8_t enable_;
    int resolution_;
    double wheel_radius_;
    double position_;
    double target_position_;
    double tick_dist_;
    bool state_;

   public:
    PulseControlledMotor() = delete;
    PulseControlledMotor(const uint8_t, const uint8_t, const uint8_t);
    void control(const Direction) override;
    PulseControlledMotor &setResolution(const int);
    PulseControlledMotor &setWheelRadius(const double);
    PulseControlledMotor &setTarget(const double);
    int setVelocity(const double);
    void run();
    double getPosition() const;
    bool isArrived() const;
};

class PairedPulseControlledMotor : public PulseControlledMotor {
   private:
    PulseControlledMotor motor2_;

   public:
    PairedPulseControlledMotor() = delete;
    PairedPulseControlledMotor(const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t, const uint8_t);
    void control(const Direction) override;
};

class MotorController {
   protected:
    uint8_t alloc_motor_;
    uint8_t current_motor_;
    BaseMotor **motors_;

   public:
    MotorController(const uint8_t = 5);
    virtual ~MotorController();
    MotorController &addMotor(BaseMotor *motor);
    int currentMotorNum() const;
    MotorController &control(const int, Direction, const int pwm = 255);
    MotorController &setTarget(const int, const double);
    bool getState(const int);
    double getPosition(const int);
};

class Vehicle : public MotorController {
   private:
    double wheel_radius_;
    double width_between_wheels_;
    double linear_x_;
    double angular_z_;
    double pose_x_;
    double pose_y_;
    double theta_;

   public:
    Vehicle(const uint8_t = 2);
    ~Vehicle();
    Vehicle &setWheelRadius(const double);
    Vehicle &setWidth(const double);
    Vehicle &run();
    Vehicle &setVelocity(double, double);
};

enum Command {
    SET_RESOLUTION,
    SET_WHEEL_RADIUS
};

class Carrier : public MotorController {
   private:
    int max_row_;
    int max_column_;
    double container_width_;
    double container_height_;

   public:
    Carrier(const uint8_t = 2);
    ~Carrier();
    Carrier &goHome();
    Carrier &goTo(const int, const int);
    Carrier &run();
    Carrier &setMaxRow(const int);
    Carrier &setMaxColumn(const int);
    Carrier &setContainerWidth(const double);
    Carrier &setContainerHeight(const double);
    Carrier &execute(const Command, const double);
    bool isArrived();
    int setVelocity(const double);
};

#endif
