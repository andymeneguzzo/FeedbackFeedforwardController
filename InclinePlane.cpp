#include <iostream>
#include <cmath>
#include <iomanip>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class InterfaceController {
    public:
        virtual double compute(double setpoint, double measurement, double dt) = 0;
        virtual ~InterfaceController() {}
};

// FEEDBACK CONTROLLER PID
class FeedbackController : public InterfaceController {
    private:
        double Kp, Ki, Kd;
        double integral;
        double prev_error;
        bool first_run;
    public:
        FeedbackController(double Kp, double Ki, double Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd), integral(0.0), prev_error(0.0), first_run(true) {}

        double compute(double setpoint, double measurement, double dt) override {
            double error = setpoint - measurement;
            integral += error * dt;

            double derivative = first_run ? 0.0 : (error - prev_error)/dt;
            first_run = false;
            prev_error = error;

            return Kp*error + Ki*integral + Kd*derivative;
        }
};

// FEEDFORWARD CONTROLLER
class FeedforwardController : public InterfaceController {
    private:
        double m;
        double g;
        double theta;
    public:
        FeedforwardController(double m, double g, double theta) : m(m), g(g), theta(theta) {}

        double compute(double /*setpoint*/, double /*measurement*/, double /*dt*/) {
            return m*g*sin(theta);
        }
};

// COMBINED CONTROLLER
class CombinedController : public InterfaceController {
    private:
        InterfaceController* feedback;
        InterfaceController* feedforward;
    public:
        CombinedController(InterfaceController* feedback, InterfaceController* feedforward)
        : feedback(feedback), feedforward(feedforward) {}

        double compute(double setpoint, double measurement, double dt) override {
            double u_ff = feedforward->compute(setpoint, measurement, dt);
            double u_fb = feedback->compute(setpoint, measurement, dt);
            
            return u_ff + u_fb;
        }
};

// DYNAMIC MODEL OF A CAR ON AN INCLINE PLANE
// EQ OF MOTION -> Mx'' = -u - Mgsin(theta)
class Car {
    private:
        double position;
        double velocity;
        double m;
        double theta;
    
    public:
    Car(double m, double theta, double initial_position = 0.0, double initial_velocity = 0.0)
    : m(m), theta(theta), position(initial_position), velocity(initial_velocity) {}

    void update(double u, double dt) {
        double g = 9.81;

        double acceleration = (-u - m*g*sin(theta))/m;
        position += velocity*dt;
        velocity += acceleration*dt;
    }

    double getPosition() const {
        return position;
    }
    double getVelocity() const {
        return velocity;
    }
};

class Simulation {};

int main() {}