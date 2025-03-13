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

// IMPLEMENT A FEEDFORWARD CONTROLLER
class FeedforwardController : public InterfaceController {
    private:
        // ASSUME I° ORDER DYNAMICS -> dx/dt = -ax + bu
        // FOR A STATIONARY STATE -> u_ff = (a/b) * setpoint
        double a, b;
    public:
    FeedforwardController(double a, double b) : a(a), b(b) {}

    // WE DON'T HAVE ANY PAST MEASUREMENT TO ACCOUNT FOR IN THIS CASE
    double compute(double setpoint, double /*measurement*/, double /*dt*/) {
        return (a/b)*setpoint;
    }
};

// IMPLEMENT A COMBINATION OF FEEDBACK AND FEEDFORWWARD CONTROLLER
class CombinedController: public InterfaceController {
    
}