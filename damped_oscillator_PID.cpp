// controlled_oscillator.cpp
// Mass-spring-damper system with PID controller for position/velocity tracking
// Controlled equation: ẍ + 2ζω₀ẋ + ω₀²x = u(t)/m
// So original system (without PID) does this: Mass bounces freely, gradually stops due to damping, you just watch it happen
// Controlled system: you tell the mass where you want it, controller calculates force needed, applies force to make it go there,
// continuously adjusts to minimize error
// Control Law: u(t) = Kp*e(t) + Ki * integral(e(t)dt) + Kd*de/dt
// e(t) = error = where you wanna be - where you are
// u(t) = force to apply
// kp,ki,kd = "knobs" you tune, integral = keeps running of total error, prev_error = remember last error to calculate change,
// max_output = prevents requesting impossible forces

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>
#include <limits>
#include <sstream>

// Define M_PI if not already defined
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

struct State {
    double x;  // position
    double v;  // velocity
};

// PID Controller class
class PIDController {
private:
    double kp;           // Proportional gain - react to current error
    double ki;           // Integral gain - fix accumulated error
    double kd;           // Derivative gain - anticipate future error
    double integral;     // Integral of error
    double prev_error;   // Previous error for derivative
    double max_output;   // Output saturation limit
    double integral_limit; // Anti-windup limit
    
public:
    PIDController(double p, double i, double d, double max_out = 100.0) 
        : kp(p), ki(i), kd(d), integral(0), prev_error(0), 
          max_output(max_out), integral_limit(max_out/2.0) {}
    
    double compute(double setpoint, double measurement, double dt) {
        // Calculate error
        double error = setpoint - measurement;
        
        // Proportional term
        double p_term = kp * error;
        
        // Integral term with anti-windup
        integral += error * dt;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;
        double i_term = ki * integral;
        
        // Derivative term (on error)
        double derivative = (dt > 0) ? (error - prev_error) / dt : 0;
        double d_term = kd * derivative;
        
        // Calculate total output
        double output = p_term + i_term + d_term;
        
        // Apply saturation
        if (output > max_output) output = max_output;
        if (output < -max_output) output = -max_output;
        
        // Store error for next iteration
        prev_error = error;
        
        return output;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    void setGains(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
    }
};

// Trajectory generator for desired position/velocity
class TrajectoryGenerator {
public:
    enum TrajectoryType {  // enum lets you give names to a set of integer constants, making code more readable
        // for example enum Color { Red, Green, Blue}; first item so Red = 0, Green = 1, Blue = 2, or can assign values
        CONSTANT,      // Constant setpoint
        STEP,          // Step changes
        SINUSOIDAL,    // Sine wave tracking
        SQUARE_WAVE,   // Square wave
        RAMP,          // Ramp input
        CUSTOM_SEQUENCE // Multiple waypoints
    };
    
    static double getDesiredPosition(double t, TrajectoryType type, double amplitude = 1.0, double frequency = 0.1) {
        switch(type) {  // switch is cleaner way of writing if/else if/else when checking same variable for different values
            case CONSTANT:
                return amplitude;
                
            case STEP:
                if (t < 5.0) return 0;
                else if (t < 10.0) return amplitude;
                else if (t < 15.0) return -amplitude;
                else return 0;
                
            case SINUSOIDAL:
                return amplitude * sin(2 * M_PI * frequency * t);
                
            case SQUARE_WAVE:
                return amplitude * (sin(2 * M_PI * frequency * t) > 0 ? 1 : -1);
                
            case RAMP:
                return amplitude * t * 0.1; // Slow ramp
                
            case CUSTOM_SEQUENCE:
                // Example: move between waypoints
                if (t < 5.0) return 1.0;
                else if (t < 10.0) return 2.0;
                else if (t < 15.0) return 0.5;
                else return 1.5;
                
            default:
                return 0;
        }
    }
    
    // static: utility function that generates trajectories, doesn't need to know about any specific oscillator
    // like a formula - same for everyone
    static double getDesiredVelocity(double t, TrajectoryType type, double amplitude = 1.0, double frequency = 0.1) {
        double dt = 0.001;
        return (getDesiredPosition(t + dt, type, amplitude, frequency) - 
                getDesiredPosition(t, type, amplitude, frequency)) / dt;
    }
};

class ControlledOscillator {
private:
    double omega0;       // Natural frequency
    double zeta;         // Damping ratio
    double mass;         // System mass (normalized, usually 1)
    PIDController* controller;
    bool use_velocity_control;
    TrajectoryGenerator::TrajectoryType trajectory_type;
    double trajectory_amplitude;
    double trajectory_frequency;
    
public:
    ControlledOscillator(double naturalFreq, double dampingRatio, double m = 1.0) 
        : omega0(naturalFreq), zeta(dampingRatio), mass(m), 
          controller(nullptr), use_velocity_control(false),
          trajectory_type(TrajectoryGenerator::STEP),
          trajectory_amplitude(1.0), trajectory_frequency(0.1) {}
    
    ~ControlledOscillator() {
        if (controller) delete controller;
    }
    
    void setController(double kp, double ki, double kd, double max_force) {
        if (controller) delete controller;
        controller = new PIDController(kp, ki, kd, max_force);
    }
    
    void setTrajectory(TrajectoryGenerator::TrajectoryType type, double amp, double freq) {
        trajectory_type = type;
        trajectory_amplitude = amp;
        trajectory_frequency = freq;
    }
    
    void setControlMode(bool velocity_control) {
        use_velocity_control = velocity_control;
    }
    
    // System dynamics with control input
    State derivatives(double t, const State& state, double control_force) {
        State deriv;
        deriv.x = state.v;
        // ẍ = -2ζω₀ẋ - ω₀²x + u/m
        deriv.v = -2.0 * zeta * omega0 * state.v - omega0 * omega0 * state.x + control_force / mass;
        return deriv;
    }
    
    // 4th-order Runge-Kutta with control
    State rungeKutta4(double t, const State& state, double dt, double control_force) {
        State k1, k2, k3, k4;
        State temp;
        
        k1 = derivatives(t, state, control_force);
        
        temp.x = state.x + k1.x * dt / 2.0;
        temp.v = state.v + k1.v * dt / 2.0;
        k2 = derivatives(t + dt / 2.0, temp, control_force);
        
        temp.x = state.x + k2.x * dt / 2.0;
        temp.v = state.v + k2.v * dt / 2.0;
        k3 = derivatives(t + dt / 2.0, temp, control_force);
        
        temp.x = state.x + k3.x * dt;
        temp.v = state.v + k3.v * dt;
        k4 = derivatives(t + dt, temp, control_force);
        
        State newState;
        newState.x = state.x + (dt / 6.0) * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
        newState.v = state.v + (dt / 6.0) * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
        
        return newState;
    }
    
    void solve(double x0, double v0, double tEnd, double dt, const std::string& filename) {
        std::vector<double> time;
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> energy;
        std::vector<double> control_force;
        std::vector<double> desired_position;
        std::vector<double> desired_velocity;
        std::vector<double> position_error;
        
        State state = {x0, v0};
        double t = 0.0;
        
        // Reset controller
        if (controller) controller->reset();
        
        // Time integration loop
        while (t <= tEnd) {
            // Get desired trajectory
            double xd = TrajectoryGenerator::getDesiredPosition(t, trajectory_type, 
                                                               trajectory_amplitude, 
                                                               trajectory_frequency);
            double vd = TrajectoryGenerator::getDesiredVelocity(t, trajectory_type,
                                                               trajectory_amplitude,
                                                               trajectory_frequency);
            
            // Calculate control force
            double u = 0;
            if (controller) {
                if (use_velocity_control) {
                    u = controller->compute(vd, state.v, dt);
                } else {
                    u = controller->compute(xd, state.x, dt);
                }
            }
            
            // Store data
            time.push_back(t);
            position.push_back(state.x);
            velocity.push_back(state.v);
            desired_position.push_back(xd);
            desired_velocity.push_back(vd);
            control_force.push_back(u);
            position_error.push_back(xd - state.x);
            
            // Calculate energy
            double KE = 0.5 * mass * state.v * state.v;
            double PE = 0.5 * mass * omega0 * omega0 * state.x * state.x;
            energy.push_back(KE + PE);
            
            // Advance one time step with control
            state = rungeKutta4(t, state, dt, u);
            
            // Check for numerical instability
            if (std::isnan(state.x) || std::isinf(state.x) || 
                std::isnan(state.v) || std::isinf(state.v) ||
                std::abs(state.x) > 1e10 || std::abs(state.v) > 1e10) {
                std::cerr << "\nERROR: Simulation became unstable at t = " << t << std::endl;
                std::cerr << "Try reducing gains or time step." << std::endl;
                return;
            }
            
            t += dt;
        }
        
        // Save results to CSV file
        saveToCSV(filename, time, position, velocity, energy, control_force, 
                 desired_position, desired_velocity, position_error);
        
        // Calculate performance metrics
        double max_error = 0;
        double rms_error = 0;
        for (size_t i = 0; i < position_error.size(); ++i) {
            double abs_error = std::abs(position_error[i]);
            if (abs_error > max_error) max_error = abs_error;
            rms_error += position_error[i] * position_error[i];
        }
        rms_error = sqrt(rms_error / position_error.size());
        
        // Print summary
        std::cout << "\nSimulation completed successfully!" << std::endl;
        std::cout << "Results saved to: " << filename << std::endl;
        std::cout << "\nPerformance Metrics:" << std::endl;
        std::cout << "  Maximum position error: " << max_error << " m" << std::endl;
        std::cout << "  RMS position error: " << rms_error << " m" << std::endl;
        std::cout << "  Final position: " << state.x << " m" << std::endl;
        std::cout << "  Final velocity: " << state.v << " m/s" << std::endl;
    }
    
private:
    void saveToCSV(const std::string& filename, 
                   const std::vector<double>& time,
                   const std::vector<double>& position,
                   const std::vector<double>& velocity,
                   const std::vector<double>& energy,
                   const std::vector<double>& control_force,
                   const std::vector<double>& desired_position,
                   const std::vector<double>& desired_velocity,
                   const std::vector<double>& position_error) {
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }
        
        // Write header
        file << "Time(s),Position(m),Velocity(m/s),Energy(J),ControlForce(N),"
             << "DesiredPosition(m),DesiredVelocity(m/s),PositionError(m)" << std::endl;
        
        // Write data
        file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < time.size(); ++i) {
            file << time[i] << "," 
                 << position[i] << "," 
                 << velocity[i] << "," 
                 << energy[i] << ","
                 << control_force[i] << ","
                 << desired_position[i] << ","
                 << desired_velocity[i] << ","
                 << position_error[i] << std::endl;
        }
        
        file.close();
    }
};

// Helper function for input
double getValidDouble(const std::string& prompt, double min, double max) {
    double value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && value >= min && value <= max) {
            return value;
        }
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input. Please enter a number between " << min << " and " << max << std::endl;
    }
}

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "   CONTROLLED MASS-SPRING-DAMPER SYSTEM  " << std::endl;
    std::cout << "==========================================" << std::endl;
    
    // System parameters
    std::cout << "\n=== SYSTEM PARAMETERS ===" << std::endl;
    double omega0 = getValidDouble("Natural frequency ω₀ (rad/s) [0.5-10]: ", 0.5, 10.0);
    double zeta = getValidDouble("Damping ratio ζ [0-2]: ", 0.0, 2.0);
    double mass = getValidDouble("Mass (kg) [0.1-10]: ", 0.1, 10.0);
    
    // Controller parameters
    std::cout << "\n=== PID CONTROLLER PARAMETERS ===" << std::endl;
    std::cout << "Suggested starting values:" << std::endl;
    std::cout << "  Kp = " << 10.0 * mass * omega0 * omega0 << " (stiff response)" << std::endl;
    std::cout << "  Ki = " << 2.0 * mass * omega0 * omega0 << " (eliminate steady-state error)" << std::endl;
    std::cout << "  Kd = " << 4.0 * mass * omega0 * zeta << " (add damping)" << std::endl;
    
    double kp = getValidDouble("Proportional gain Kp [0-1000]: ", 0.0, 1000.0);
    double ki = getValidDouble("Integral gain Ki [0-100]: ", 0.0, 100.0);
    double kd = getValidDouble("Derivative gain Kd [0-100]: ", 0.0, 100.0);
    double max_force = getValidDouble("Maximum control force (N) [1-1000]: ", 1.0, 1000.0);
    
    // Control mode
    std::cout << "\n=== CONTROL MODE ===" << std::endl;
    std::cout << "1. Position control (track desired position)" << std::endl;
    std::cout << "2. Velocity control (track desired velocity)" << std::endl;
    int mode = static_cast<int>(getValidDouble("Select mode [1-2]: ", 1, 2));
    bool velocity_control = (mode == 2);
    
    // Trajectory type
    std::cout << "\n=== REFERENCE TRAJECTORY ===" << std::endl;
    std::cout << "1. Constant setpoint" << std::endl;
    std::cout << "2. Step changes" << std::endl;
    std::cout << "3. Sinusoidal tracking" << std::endl;
    std::cout << "4. Square wave" << std::endl;
    std::cout << "5. Ramp input" << std::endl;
    std::cout << "6. Custom waypoints" << std::endl;
    int traj = static_cast<int>(getValidDouble("Select trajectory [1-6]: ", 1, 6));
    
    double amplitude = 1.0;
    double frequency = 0.1;
    
    if (traj == 3 || traj == 4) {  // Sinusoidal or square wave
        amplitude = getValidDouble("Amplitude [0.1-5]: ", 0.1, 5.0);
        frequency = getValidDouble("Frequency (Hz) [0.01-1]: ", 0.01, 1.0);
    } else if (traj == 1) {  // Constant
        amplitude = getValidDouble("Setpoint value [-5 to 5]: ", -5.0, 5.0);
    }
    
    // Initial conditions
    std::cout << "\n=== INITIAL CONDITIONS ===" << std::endl;
    double x0 = getValidDouble("Initial position (m) [-5 to 5]: ", -5.0, 5.0);
    double v0 = getValidDouble("Initial velocity (m/s) [-5 to 5]: ", -5.0, 5.0);
    
    // Simulation parameters
    std::cout << "\n=== SIMULATION PARAMETERS ===" << std::endl;
    double tEnd = getValidDouble("End time (s) [1-100]: ", 1.0, 100.0);
    double dt = getValidDouble("Time step (s) [0.001-0.1]: ", 0.001, 0.1);
    
    // Create and configure system
    ControlledOscillator system(omega0, zeta, mass);
    system.setController(kp, ki, kd, max_force);
    system.setControlMode(velocity_control);
    system.setTrajectory(static_cast<TrajectoryGenerator::TrajectoryType>(traj - 1), 
                        amplitude, frequency);
    
    // Run simulation
    std::cout << "\n=== RUNNING SIMULATION ===" << std::endl;
    system.solve(x0, v0, tEnd, dt, "controlled_oscillator.csv");
    
    std::cout << "\nYou can now run the MATLAB visualization script!" << std::endl;
    std::cout << "The CSV file contains:" << std::endl;
    std::cout << "  - Actual position and velocity" << std::endl;
    std::cout << "  - Desired position and velocity" << std::endl;
    std::cout << "  - Control force applied" << std::endl;
    std::cout << "  - Tracking error" << std::endl;
    
    return 0;
}