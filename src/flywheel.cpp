#include "main.h"
#include <numeric>
Flywheel::Flywheel(pros::Motor *motor, VelPID *pid, EMAFilter *filter, double gearRatio, double motorSlew)
{
    this->flywheelMotor = motor;
    this->flywheelMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    this->flywheelMotor->set_reversed(false);

    this->pid = pid;
    this->rpmFilter = filter;
    this->gearRatio = gearRatio;
    this->motorSlew = motorSlew;
    this->task = new pros::Task(this->taskFn, (void *)this);
}

void Flywheel::run()
{
    int i = 0;
    int averageLength = 5;
    double averageArray[averageLength] = {};
    while (true)
    {
        currentRPM = rpmFilter->filter(flywheelMotor->get_actual_velocity() * gearRatio);
        averageArray[i] = currentRPM;
        output = pid->calculate(targetRPM, currentRPM);

        // if (output > lastOutput && lastOutput < 3000 && output > 3000)
        //     lastOutput = 3000;

        // double change = output - lastOutput;
        // if (change > motorSlew)
        //     output = lastOutput + motorSlew;
        // else if (change < -motorSlew)
        //     output = lastOutput - motorSlew;
        // lastOutput = output;
        if (isRecovering && pros::millis() - lastShotTime < 1000)
        {
            // output += 1000;
            // printf("Output: %f Boosting!\n", output);
        }
        flywheelMotor->move_voltage(output);
        // printf("Target: %f Current: %f Output: %f\n", targetRPM, currentRPM, output);
        acceleration = (currentRPM - oldRPM) / ((pros::millis() - oldTime));
        isShot = acceleration <= -3;
        if (isShot && !isRecovering)
        {
            lastShotTime = pros::millis();
            lastShotSpeed = oldRPM;
            isRecovering = true;
            pros::lcd::clear_line(4);
            pros::lcd::set_text(4, "Recovering...");
        }

        if (isRecovering)
        {
            if (currentRPM >= lastShotSpeed || targetRPM != oldTarget)
            {
                isRecovering = false;
                pros::lcd::clear_line(4);
                pros::lcd::set_text(4, "Recovery Time: " + std::to_string((pros::millis() - lastShotTime) / 1000.0));
            }
        }

        i++;
        i %= averageLength;

        double sum = 0;
        for (int i = 0; i < averageLength; i++)
        {
            sum += averageArray[i];
        }
        this->averageRPM = sum / averageLength;
        // printf("%f,%f,%f\n", output, currentRPM, averageRPM);
        pros::lcd::clear_line(1);
        // PRINTF1, "Average: " + std::to_string(averageRPM));

        oldRPM = currentRPM;
        oldTarget = targetRPM;
        oldTime = pros::millis();
        pros::delay(20);
    }
}

void Flywheel::taskFn(void *param)
{
    ((Flywheel *)param)->run();
}

void Flywheel::setTargetRPM(double rpm)
{
    targetRPM = rpm;
}

double Flywheel::getTargetRPM()
{
    return targetRPM;
}
double Flywheel::getCurrentRPM()
{
    return currentRPM;
}
double Flywheel::getAverageRPM()
{
    return averageRPM;
}

void Flywheel::waitUntilReady()
{
    while (abs(averageRPM - targetRPM) > 10 || averageRPM < targetRPM)
    {
        pros::delay(20);
    }
    return;
    // return abs(currentRPM - targetRPM) < 5;
}