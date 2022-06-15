#include <initializer_list>
#include "pros\motors.hpp"
#include <vector>
#include <memory>
class MotorGroup
{
public:
    MotorGroup(const std::initializer_list<int> _motors);
    MotorGroup(const std::initializer_list<int> _motors, const std::initializer_list<bool> _reversed);
    int move(const int voltage);
    int move_absolute(const int voltage);
    int move_velocity(int velocity);
    int brake(void);
    int move_voltage(int voltage);
    int modify_profiled_velocity(int voltage);
    int set_brake_mode(pros::motor_brake_mode_e_t mode);
    int set_current_limit(int limit);
    int set_encoder_units(pros::motor_encoder_units_e_t units);
    int set_voltage_limit(int limit);
    int set_zero_position(double position);
    int tare_position(void);
    double get_target_position(void);
    int get_target_velocity(void);
    double get_actual_velocity(void);
    int get_current_draw(void);
    int get_direction(void);
    int get_efficiency(void);
    int get_faults(void);
    int get_flags(void);
    int get_position(void);
    double get_power(void);
    int get_raw_position(void);
    double get_temperature(void);
    double get_torque(void);
    int get_voltage(void);
    int get_zero_position_flag(void);
    int motor_is_stopped(void);
    int is_over_current(void);
    int is_over_temp(void);
    pros::motor_brake_mode_e_t get_brake_mode(void);
    int get_current_limit(void);
    pros::motor_encoder_units_e_t get_encoder_units(void);
    int get_voltage_limit(void);
private:
    std::vector<std::shared_ptr<pros::Motor>> motors;
};