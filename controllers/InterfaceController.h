#ifndef INTERFACECONTROLLER_H
#define INTERFACECONTROLLER_H

class InterfaceController {
    public:
        // Virtual method to compute the control signal
        virtual double compute(double setpoint, double measurement, double time_deriv) = 0;
        virtual ~InterfaceController() {}
};

#endif