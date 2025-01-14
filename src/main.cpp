#include <vex.h>
#include <iostream>

#define DEBUG_FUNCTION
#define DEBUG_OUT
#ifdef DEBUG_OUT
#define FILE_OUT
#endif

// Define the port numbers for the motors and gears
const int32_t LM1PORT = vex::PORT1;
const int32_t LM2PORT = vex::PORT2;
const int32_t RM1PORT = vex::PORT3;
const int32_t RM2PORT = vex::PORT4; 
const int32_t GEARPORT = vex::PORT5;

// Set the gear ratio for the motors
const vex::gearSetting GEARS = vex::gearSetting::ratio18_1;

// Set the motor direction
const bool REVERSE = false;

// Create motor objects for the left and right motors
vex::motor left_motor_1(LM1PORT, GEARS, REVERSE), left_motor_2(LM2PORT, GEARS, REVERSE);
vex::motor right_motor_1(RM1PORT, GEARS, !REVERSE), right_motor_2(RM2PORT, GEARS, !REVERSE);

// Create a controller object
vex::controller controller(vex::controllerType::primary);
// Create a brain object
vex::brain brain;
// Create a GPS (Global Positioning System) object
vex::gps gps(GEARPORT, 0.0, vex::turnType::left);

#ifdef DEBUG_FUNCTION
// Declare a function for debugging the GPS functionality
inline bool functional_gps();
#endif

// Declare a function for toggling a button state
inline bool toggle(vex::controller::button button, bool & toggle_state, bool & last_toggle_state);

// Declare functions for controlling the robot
void control(vex::controller::button precision_toggle);
void move(double x, double y, double heading);
void turn(double delta_heading);
void set_heading(double heading);

// Declare functions for autonomous and driver control periods
void autonomous_period();
void control_period();

// Main function
int main() {
    // Set the competition functions
    vex::competition competition;
    competition.autonomous(autonomous_period);
    competition.drivercontrol(control_period);
}


// Autonomous period function
void autonomous_period() {

}

// Autonomous period function
void control_period() {

}

// Declare variables for driver
namespace dtvar {

    // Declare acceptable error range for distance and rotation
    const double distance_tolerance = 5.0;
    const double rotation_tolerance = 1.0;

    // Declares PID gains for positional control (Adjusts the motor power based on position from target)
    const double kp = 1.0; // Proportional Value
    const double ki = 0.0; // Integeral Value
    const double kd = 0.0; // Distance Value

    // Declares PID gains for rotational control (Adjusts the motor power (for rotation) based on position and angle from target)
    const double kpa = 0.1; // Proportional Value
    const double kia = 0.0; // Integeral Value
    const double kda = 0.0; // Distance Value

    // Constants used for percision and smoothing control
    const double forget_constant = 0.99; // How much previous motor control affects current motor control
    const double precision_constant = 0.2; // Percision of robot's movements

    // Variables control and store the 4 motor speeds in the XDrive config (Updated based on calculated motor power)
    double left_motor_1_speed, left_motor_2_speed, right_motor_1_speed, right_motor_2_speed;

    // Variables store the trig calculations during XDrive implementation. Theta refers to the angle, and sine and cosine referrs
    // To the trig functions.
    double theta, sine, cosine;

    // Variables store the current x, y, and rotation of the XDrive
    double current_x, current_y, current_rotation;

    // Variables store the relative x, y, and rotation of the XDrive in respect to the target location
    double relative_x, relative_y, relative_rotation;

    // Variables store the difference between the desired and current x, y, and rotational values.
    double error_axis_1, error_axis_2, error_rotation;

    // Variables store the "porportional" part of PID, calculated by multiplying positional and rotation errors by their respective
    // Porportional gains (Variables kp, kpa)
    double proportional_axis_1, proportional_axis_2, proportional_rotation;

    // Variables store the "Intergral" part of PID, calculated by accuminlating the positional and integeral values over time and
    // multiplying them by their repective intergral gains (Variables ki, kia)
    double integral_axis_1, integral_axis_2, integral_rotation;

    // Variables store the "Intergral" part of PID, calculated by calculating the rate of change for positional and rotational errors
    // and multiplying them by their repective derivitive gains (Variables kd, kda)
    double derivative_axis_1, derivative_axis_2, derivative_rotation;

    // These variables store the last positional and rotational errors for calculation the rate of change for deriviatve and integeral 
    // variables used for PID (above two)
    double last_error_axis_1, last_error_axis_2, last_error_rotation;

    // These variables store the final positional and rotational power calculations for the XDrive (PID System), used to set speed
    // For XDrive configs.
    double power_axis_1, power_axis_2, power_rotation;

}

// Define namespace for the controller
namespace cntrlvar {
    // Declare variables for controller axis inputs
    double controller_axis_1, controller_axis_2, controller_axis_3, controller_axis_4;
    // Declare a variable for precision control constant
    double precision_constant;
}

#ifdef DEBUG_FUNCTION
inline bool functional_gps() {
    // Check if GPS position and heading is all zero
    if (gps.xPosition() == 0.0 and gps.yPosition() == 0.0 and gps.heading() == 0.0) {
        return false;
    }
    return true;
}
#endif

// Function for toggling boolean with button
inline bool toggle(vex::controller::button button, bool & toggle_state, bool & past_state) {
    if (button.pressing() and past_state == false) {
        return toggle_state = not toggle_state;
    }
    return toggle_state;
}

// 
void control(vex::controller::button precision_toggle) {

    // Precision state and helper variable as the while loop goes quickly so two variables are needed to make sure it doesn't rapidly toggle
    bool precision_state = false, last_precision_state = false;
    
    while (true) {
        // Read controller axis positions
        cntrlvar::controller_axis_1 = controller.Axis1.position();
        cntrlvar::controller_axis_2 = controller.Axis2.position();
        cntrlvar::controller_axis_3 = controller.Axis3.position();
        cntrlvar::controller_axis_4 = controller.Axis4.position();
        // Calculate mmotor speeds based on controller inputs
        dtvar::left_motor_1_speed = cntrlvar::controller_axis_3 + cntrlvar::controller_axis_4 + cntrlvar::controller_axis_1;
        dtvar::left_motor_2_speed = cntrlvar::controller_axis_3 - cntrlvar::controller_axis_4 + cntrlvar::controller_axis_1;
        dtvar::right_motor_1_speed = cntrlvar::controller_axis_3 - cntrlvar::controller_axis_4 - cntrlvar::controller_axis_1;
        dtvar::right_motor_2_speed = cntrlvar::controller_axis_3 + cntrlvar::controller_axis_4 - cntrlvar::controller_axis_1;

        // Check precision state to go to precision mode
        if (toggle(precision_toggle, precision_state, last_precision_state)) {
            dtvar::left_motor_1_speed *= dtvar::precision_constant;
            dtvar::left_motor_2_speed *= dtvar::precision_constant;
            dtvar::right_motor_1_speed *= dtvar::precision_constant;
            dtvar::right_motor_2_speed *= dtvar::precision_constant;
        }

        // Spin the motors based on the calculated speeds
        left_motor_1.spin(vex::directionType::fwd, dtvar::left_motor_1_speed, vex::percentUnits::pct);
        left_motor_2.spin(vex::directionType::fwd, dtvar::left_motor_2_speed, vex::percentUnits::pct);
        right_motor_1.spin(vex::directionType::fwd, dtvar::right_motor_1_speed, vex::percentUnits::pct);
        right_motor_2.spin(vex::directionType::fwd, dtvar::right_motor_2_speed, vex::percentUnits::pct);
    }
}

void move(double x, double y, double heading) {
    dtvar::integral_axis_1 = 0.0;
    dtvar::integral_axis_2 = 0.0;
    dtvar::integral_rotation = 0.0;
    dtvar::current_x = gps.xPosition();
    dtvar::current_y = gps.yPosition();
    dtvar::current_rotation = gps.heading();
    dtvar::relative_x = x - dtvar::current_x;
    dtvar::relative_y = y - dtvar::current_y;
    dtvar::relative_rotation = heading - dtvar::current_rotation;
    dtvar::theta = dtvar::current_rotation + 45;
    dtvar::sine = sin(dtvar::theta / 180.0 * M_PI);
    dtvar::cosine = cos(dtvar::theta / 180.0 * M_PI);
    dtvar::last_error_axis_1 = dtvar::cosine * dtvar::current_x + dtvar::sine * dtvar::current_y;
    dtvar::last_error_axis_2 = dtvar::cosine * dtvar::current_y - dtvar::sine * dtvar::current_x;
    dtvar::error_rotation = dtvar::relative_rotation;
    dtvar::last_error_rotation = dtvar::error_rotation;

    #ifdef DEBUG_OUT
    #ifdef FILE_OUT
    freopen("debug_out", "w", stdout);
    #endif
    std::cout << "INITIAL READINGS\n";
    std::cout << "Current (x, y, heading): (" << dtvar::current_x << ", " << dtvar::current_y << ", " << dtvar::current_rotation << ")\n";
    std::cout << "Relative (x, y, heading): (" << dtvar::relative_x << ", " << dtvar::relative_y << ", " << dtvar::relative_rotation << ")\n";
    std::cout << "Change of basis (x, y) with i-hat on axis 1, j-hat on axis 2: " << dtvar::last_error_axis_1 << ", " << dtvar::last_error_axis_2 << ")\n";
    brain.Screen.print("INITIAL READINGS");
    brain.Screen.newLine();
    brain.Screen.print("Current x=%.2lf, y=%.2lf, rot=%.2lf)", dtvar::current_x, dtvar::current_y, dtvar::current_rotation);
    brain.Screen.newLine();
    brain.Screen.print("Relative x=%.2lf, y=%.2lf, rot=%.2lf", dtvar::relative_x, dtvar::relative_y, dtvar::relative_rotation);
    brain.Screen.newLine();
    brain.Screen.print("Basis x=%.2lf, y=%.2lf", dtvar::last_error_axis_1, dtvar::last_error_axis_2);
    brain.Screen.newLine();
    #endif


    // Creates a while loop until the desired position and rotation is reached
    while (fabs(dtvar::relative_x) > dtvar::distance_tolerance or fabs(dtvar::relative_y) > dtvar::distance_tolerance or fabs(dtvar::relative_rotation) > dtvar::rotation_tolerance) {
        // Update correct position and rotation
        dtvar::current_x = gps.xPosition();
        dtvar::current_y = gps.yPosition();
        dtvar::current_rotation = gps.heading();

        // Calculates relative position and rotation
        dtvar::relative_x = x - dtvar::current_x;
        dtvar::relative_y = y - dtvar::current_y;
        dtvar::relative_rotation = heading - dtvar::current_rotation;

        // Calculates the rotational values (thetha = angle, sine and consine being the trig functions respectivley)
        // Used for coordinate transformation
        dtvar::theta = dtvar::current_rotation + 45;
        dtvar::sine = sin(dtvar::theta / 180.0 * M_PI);
        dtvar::cosine = cos(dtvar::theta / 180.0 * M_PI);

        // Calculate erros in coordinate transformation system mentioned above
        dtvar::error_axis_1 = dtvar::cosine * dtvar::current_x + dtvar::sine * dtvar::current_y;
        dtvar::error_axis_2 = dtvar::cosine * dtvar::current_y - dtvar::sine * dtvar::current_x;
        dtvar::error_rotation = dtvar::relative_rotation;

        // Calculate positional, integral and derivative values for control
        // Format: Porportional value is equal to error value * constant (k_ for positiona, k_a for angular)
        dtvar::proportional_axis_1 = dtvar::error_axis_1 * dtvar::kp;
        dtvar::proportional_axis_2 = dtvar::error_axis_2 * dtvar::kp;
        dtvar::proportional_rotation = dtvar::error_rotation * dtvar::kpa;
        dtvar::integral_axis_1 += dtvar::error_axis_1 * dtvar::ki;
        dtvar::integral_axis_2 += dtvar::error_axis_2 * dtvar::ki;
        dtvar::integral_rotation += dtvar::error_rotation * dtvar::kia;
        dtvar::integral_axis_1 *= dtvar::forget_constant;
        dtvar::integral_axis_2 *= dtvar::forget_constant;
        dtvar::integral_rotation *= dtvar::forget_constant;
        dtvar::derivative_axis_1 = (dtvar::error_axis_1 - dtvar::last_error_axis_1) * dtvar::kd;
        dtvar::derivative_axis_2 = (dtvar::error_axis_2 - dtvar::last_error_axis_2) * dtvar::kd;
        dtvar::derivative_rotation = (dtvar::error_rotation - dtvar::last_error_rotation) * dtvar::kda;

        // Calculated to aid derivative calculations
        dtvar::last_error_axis_1 = dtvar::error_axis_1;
        dtvar::last_error_axis_2 = dtvar::error_axis_2;
        dtvar::last_error_rotation = dtvar::error_rotation;

        // Calculates motor power based on PID values
        dtvar::power_axis_1 = dtvar::proportional_axis_1 + dtvar::integral_axis_1 + dtvar::derivative_axis_1;
        dtvar::power_axis_2 = dtvar::proportional_axis_2 + dtvar::integral_axis_2 + dtvar::derivative_axis_2;
        dtvar::power_rotation = dtvar::proportional_rotation + dtvar::integral_rotation + dtvar::derivative_rotation;

        // Calculates motor speed based on the power and rotational values calculated above
        dtvar::left_motor_1_speed  = dtvar::power_axis_1 - dtvar::power_rotation;
        dtvar::left_motor_2_speed  = dtvar::power_axis_2 - dtvar::power_rotation;
        dtvar::right_motor_1_speed = dtvar::power_axis_2 + dtvar::power_rotation;
        dtvar::right_motor_2_speed = dtvar::power_axis_1 + dtvar::power_rotation;

        // Sets motor speed for each wheel
        left_motor_1.spin(vex::directionType::fwd, dtvar::left_motor_1_speed, vex::percentUnits::pct);
        left_motor_2.spin(vex::directionType::fwd, dtvar::left_motor_2_speed, vex::percentUnits::pct);
        right_motor_2.spin(vex::directionType::fwd, dtvar::right_motor_1_speed, vex::percentUnits::pct);
        right_motor_2.spin(vex::directionType::fwd, dtvar::right_motor_2_speed, vex::percentUnits::pct);


        // Debug Code
        #ifdef DEBUG_OUT
        std::cout << "\n\n\n";
        std::cout << "Current (x, y, heading): (" << dtvar::current_x << ", " << dtvar::current_y << ", " << dtvar::current_rotation << ")\n";
        std::cout << "Relative (x, y, heading): (" << dtvar::relative_x << ", " << dtvar::relative_y << ", " << dtvar::relative_rotation << ")\n";
        std::cout << "Change of basis (x, y) with i-hat on axis 1, j-hat on axis 2: (" << dtvar::last_error_axis_1 << ", " << dtvar::last_error_axis_2 << ")\n";
        std::cout << "(P1, I1, D1), (P2, I2, D2): (" << dtvar::proportional_axis_1 << ", " << dtvar::integral_axis_1 << ", " << dtvar::derivative_axis_1 << "), (" << dtvar::proportional_axis_2 << ", " << dtvar::integral_axis_2 << ", " << dtvar::derivative_axis_2 << ")\n";
        std::cout << "(Pa, Ia, Da): (" << dtvar::proportional_rotation << ", " << dtvar::integral_rotation << ", " << dtvar::derivative_rotation << ")\n";
        brain.Screen.clearScreen();
        brain.Screen.print("Current x=%.2lf, y=%.2lf, rot=%.2lf)", dtvar::current_x, dtvar::current_y, dtvar::current_rotation);
        brain.Screen.newLine();
        brain.Screen.print("Relative x=%.2lf, y=%.2lf, rot=%.2lf", dtvar::relative_x, dtvar::relative_y, dtvar::relative_rotation);
        brain.Screen.newLine();
        brain.Screen.print("Basis x=%.2lf, y=%.2lf", dtvar::last_error_axis_1, dtvar::last_error_axis_2);
        brain.Screen.newLine();
        brain.Screen.print("P1=%.2lf, I1=%.2lf, D1=%.2lf, P2=%.2lf, I2=%.2lf, D2=%.2lf", dtvar::proportional_axis_1, dtvar::integral_axis_1, dtvar::derivative_axis_1, dtvar::proportional_axis_2, dtvar::integral_axis_2, dtvar::derivative_axis_2);
        brain.Screen.newLine();
        brain.Screen.print("Pa=%.2lf, Ia=%.2lf, Da=%.2lf", dtvar::proportional_rotation, dtvar::integral_rotation, dtvar::derivative_rotation);
        #endif

        /*
        Breakdown of code:

        1. The control loop runs as long as the difference between the current position/rotation and the desired position/rotation is above certain tolerance values.

        2. The current position and rotation of the robot are obtained from a GPS system (represented by `gps.xPosition()`, `gps.yPosition()`, and `gps.heading()`).

        3. The relative position and rotation errors are calculated by subtracting the current values from the desired values.

        4. A coordinate transformation is performed using the current rotation angle (`dtvar::current_rotation`) to align the error calculation with a specific axis.

        5. Proportional, integral, and derivative terms are computed for each axis of control (axis 1, axis 2, and rotation). These terms are multiplied by corresponding gain constants (kp, ki, kd, kpa, kia, kda) to adjust the control response.

        6. Integral and derivative terms are accumulated and updated for each iteration of the loop.

        7. Motor power values are calculated based on the control terms for each axis.

        8. The motor speeds for each wheel are calculated based on the power values and the rotational direction.

        9. The calculated speeds are applied to the corresponding motors using the VEX Robotics API (`left_motor_1.spin()`, `left_motor_2.spin()`, etc.).

        10. Optional debug information can be printed.
        */

    }
}

void turn(double delta_heading) {
    move(gps.xPosition(), gps.yPosition(), gps.heading() + delta_heading);
}

void set_heading(double heading) {
    move(gps.xPosition(), gps.yPosition(), heading);
}
