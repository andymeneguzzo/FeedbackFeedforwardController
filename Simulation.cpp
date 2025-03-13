
#include <iomanip> // formatting in file
#include <iostream>
#include <fstream> // for file handling operations

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
    private:
        // POINTERS TO THE TWO CONTROLLERS
        InterfaceController* feedback;
        InterfaceController* feedforward;
    public:
        CombinedController(InterfaceController* feedback, InterfaceController* forward)
        : feedback(feedback), feedforward(forward) {}

        // COMPUTE CONTROL SIGNAL
        double compute(double setpoint, double measurement, double dt) override {
            double u_ff = feedforward->compute(setpoint, measurement, dt);
            double u_fb = feedback->compute(setpoint, measurement, dt);

            // SUMMING THE TWO CONTROL SIGNALS
            return u_ff + u_fb;
        }
};

// IMPLEMENTATION OF THE SYSTEM TO CONTROL
class System {
    private:
        double state;
        double a, b;
    public:
        // INITIALIZE SYSTEM WITH PARAMETERS a,b AND an initial state
        System(double a, double b, double intial_state = 0.0)
        : a(a), b(b), state(intial_state) {}

        // USE EULER'S METHOD TO INTEGRATE AND UPDATE THE SYSTEM STATE
        void update(double u, double dt) {
            double dx = -a*state + b*u;
            state += dx*dt;
        }

        double getState() const {
            return state;
        }
};

// IMPLEMENTATION TO HANDLE SIMULATION OF THE SYSTEM
class Simulation {
    private:
        System system; // object system to run simulation on
        InterfaceController* controller; // selected controller to use

        double dt; // sampling interval
        double totalTime; // time of simulation
    
    public:
        Simulation(const System& system, InterfaceController* controller, double dt, double totalTime) 
        : system(system), controller(controller), dt(dt), totalTime(totalTime) {}

        // RUN SIMULATION FOR A CERTAIN SETPOINT FIXED CONSTANT
        void run(double setpoint) {
            // CSV FILE HANDLING
            std::ofstream file("simulation.csv");
            if(!file) {
                std::cerr << "Error opening file" << std::endl;
                return;
            }
            
            // column width
            const int width = 15;

            file << "FEEDBACK CONTROLLER - simulation \n\n";
            
            // Titles of the columns formatted to make nicer output
            file << std::left << std::setw(width) << "Time" << ","
                 << std::left << std::setw(width) << "Setpoint" << ","
                 << std::left << std::setw(width) << "State" << ","
                 << std::left << std::setw(width) << "ControlSignal" << "\n";


            int steps = static_cast<int>(totalTime/dt); // number of "samples" to take from simulation

            for(int i=0; i<steps; ++i) {
                double t = i*dt; // time at the current step
                double measurement = system.getState(); // get the current state
                double u = controller->compute(setpoint, measurement, dt); // for the certain controller used, we compute the u signal

                system.update(u, dt); // update the system state after every computed u signal

                // visualize the results for the step and go to the next step
                std::cout << "t = " << t
                          << " s, setpoint = " << setpoint
                          << ", state = " << measurement
                          << ", control signal u = " << u << std::endl;
                
                // file output, formatted to make nicer output
                file << std::left << std::setw(width) << t << ","
                          << std::left << std::setw(width) << setpoint << ","
                          << std::left << std::setw(width) << measurement << ","
                          << std::left << std::setw(width) << u << "\n";
            }
            file.close();
        }
};

// MAIN -> INITIALIZE CONTROLLERS, RUN SIMULATION
int main() {
    // SYSTEM PARAMETERS
    double a = 1.0;
    double b = 1.0;
    System system(a, b, 0.0); // intitial state 0.0

    // PARAMETERS FOR PID (FEEDBACK CONTROLLER)
    double Kp = 2.0, Ki = 0.5, Kd = 0.1;
    FeedbackController feedback(Kp, Ki, Kd);

    // FEEDFORWARD CONTROLLER PARAMTERS ARE THE SYSTEM'S PARAMETERS
    FeedforwardController feedforward(a, b);

    // COMBINED CONTROLLER TAKES THEM BOTH, PASSED BY REFERENCE
    CombinedController combined (&feedback, &feedforward);

    // Simulation parameters
    double dt = 0.01; // sampling interval
    double totalTime = 10.0; // total time of simulation
    double setpoint = 1.0; // reference value that we expect

    // R U N   S I M U L A T I O N
    Simulation simulation(system, &combined, dt, totalTime);
    simulation.run(setpoint);

    return 0;
}