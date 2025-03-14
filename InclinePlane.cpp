#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

// Definizione di PI se non presente
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// INTERFACCIA PER IL CONTROLLORE
class InterfaceController {
public:
    virtual double compute(double setpoint, double measurement, double dt) = 0;
    virtual ~InterfaceController() {}
};

// CONTROLLORE FEEDBACK (PID)
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
        double derivative = first_run ? 0.0 : (error - prev_error) / dt;
        first_run = false;
        prev_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }
};

// CONTROLLORE FEEDFORWARD
// Per compensare la componente gravitazionale lungo il piano inclinato:
// u_ff = M * g * sin(theta)
class FeedforwardController : public InterfaceController {
private:
    double m;      // massa dell'auto
    double g;      // accelerazione di gravità
    double theta;  // inclinazione del piano (radianti)
public:
    FeedforwardController(double m, double g, double theta)
        : m(m), g(g), theta(theta) {}

    double compute(double /*setpoint*/, double /*measurement*/, double /*dt*/) override {
        return m * g * sin(theta);
    }
};

// COMBINAZIONE DEI CONTROLLORI (FEEDBACK + FEEDFORWARD)
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

// MODELLO DINAMICO DELL'AUTO SU PIANO INCLINATO
// Equazione del moto: M * x'' = u - M * g * sin(theta)
class Car {
private:
    double position;  // posizione lungo il piano (m)
    double velocity;  // velocità lungo il piano (m/s)
    double m;         // massa dell'auto (kg)
    double theta;     // inclinazione del piano (radianti)
public:
    Car(double m, double theta, double initial_position = 0.0, double initial_velocity = 0.0)
        : m(m), theta(theta), position(initial_position), velocity(initial_velocity) {}

    // Integrazione della dinamica tramite il metodo di Eulero
    void update(double u, double dt) {
        double g = 9.81; // accelerazione di gravità (m/s²)
        double acceleration = (u - m * g * sin(theta)) / m;
        position += velocity * dt;
        velocity += acceleration * dt;
    }

    double getPosition() const { return position; }
    double getVelocity() const { return velocity; }
};

// CLASSE PER GESTIRE LA SIMULAZIONE E L'OUTPUT SU FILE CSV
class Simulation {
private:
    Car car;
    InterfaceController* controller;
    double dt;
    double totalTime;
public:
    Simulation(Car car, InterfaceController* controller, double dt, double totalTime)
        : car(car), controller(controller), dt(dt), totalTime(totalTime) {}

    void run(double setpoint) {
        std::ofstream file("InclinePlane.csv");
        if (!file) {
            std::cerr << "Errore nell'apertura del file InclinePlane.csv" << std::endl;
            return;
        }

        // Larghezza fissa per una formattazione visiva chiara
        const int width = 15;
        // Scrittura dell'intestazione nel file CSV
        file << std::left << std::setw(width) << "Time" << ","
             << std::left << std::setw(width) << "Setpoint" << ","
             << std::left << std::setw(width) << "Position" << ","
             << std::left << std::setw(width) << "Velocity" << ","
             << std::left << std::setw(width) << "ControlSignal" << "\n";

        int steps = static_cast<int>(totalTime / dt);
        for (int i = 0; i < steps; i++) {
            double t = i * dt;
            double measurement = car.getPosition();
            double u = controller->compute(setpoint, measurement, dt);
            car.update(u, dt);

            // Stampa a video formattata in colonne
            std::cout << std::left << std::setw(width) << t
                      << std::left << std::setw(width) << setpoint
                      << std::left << std::setw(width) << measurement
                      << std::left << std::setw(width) << car.getVelocity()
                      << std::left << std::setw(width) << u << std::endl;

            // Scrittura dei dati nel file CSV
            file << std::left << std::setw(width) << t << ","
                 << std::left << std::setw(width) << setpoint << ","
                 << std::left << std::setw(width) << measurement << ","
                 << std::left << std::setw(width) << car.getVelocity() << ","
                 << std::left << std::setw(width) << u << "\n";
        }
        file.close();
    }
};

int main() {
    // Parametri del sistema
    double mass = 1000.0;                     // massa dell'auto (kg)
    double theta = 10.0 * M_PI / 180.0;         // inclinazione del piano (10° in radianti)
    Car car(mass, theta, 0.0, 0.0);

    // Parametri per il controllore feedback (PID)
    double Kp = 500.0, Ki = 10.0, Kd = 50.0;
    FeedbackController feedback(Kp, Ki, Kd);

    // Controllore feedforward per compensare la gravità: u_ff = M * g * sin(theta)
    FeedforwardController feedforward(mass, 9.81, theta);

    // Combinazione dei due controlli
    CombinedController combined(&feedback, &feedforward);

    // Parametri della simulazione
    double dt = 0.01;       // intervallo di campionamento (s)
    double totalTime = 10.0; // tempo totale di simulazione (s)
    double setpoint = 0.0;  // obiettivo: mantenere l'equilibrio (posizione 0)

    Simulation sim(car, &combined, dt, totalTime);
    sim.run(setpoint);

    return 0;
}
