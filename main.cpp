#include <iostream>

/**
 * 
 * INTERFACCIA PER CONTROLLORE
 * 
 */
class InterfaceController {
    public:
        // VIRTUAL METHOD TO COMPUTE CONTROL SIGNAL
        virtual double compute(double setpoint, double measurement, double dt) = 0;
        virtual ~InterfaceController() {}
};

// IMPLEMENT A FEEDBACK CONTROLLER
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

            // SUMMING ALL TERMS
            return Kp*error + Ki*integral + Kd*derivative;
        }
};