#include "main.h"
#include <numeric>
Flywheel::Flywheel(pros::Motor *motor, VelPID *pid, EMAFilter *filter, double gearRatio, double motorSlew)
{
    this->flywheelMotor = motor;
    this->flywheelMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    this->flywheelMotor->set_reversed(false);
    // this->flywheelMotor->set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

    this->pid = pid;
    this->rpmFilter = new EMAFilter(0.1);
    this->smaFilter = new SMAFilter(3);
    this->gearRatio = gearRatio;
    this->motorSlew = motorSlew;
    this->task = new pros::Task(this->taskFn, (void *)this);
}

// void vPortEnterCritical();
// void vPortExitCritical();

void Flywheel::run()
{
    int i = 0;
    int averageLength = 5;
    double averageArray[averageLength] = {};
    now = pros::millis();
    printf("time,vexVelocity,filteredVelocity");
    while (true)
    {
        // vPortEnterCritical();
        this->currentTime = pros::millis();
        this->currentTicks = flywheelMotor->get_position();
        // flywheelMotorTimestamp = vexDeviceGetTimestampByIndex(flywheelMotor->get_port());
        // systemTime = vexSystemTimeGet();
        this->rawTicks = flywheelMotor->get_raw_position(&this->motorTime);
        // vPortExitCritical();

        this->timestampDiff = this->motorTime - this->prev_flywheelMotorTimestamp;
        this->prev_flywheelMotorTimestamp = this->motorTime;
        this->internalVelocityMeasure = this->flywheelMotor->get_actual_velocity();
        this->dP = this->currentTicks - this->oldTicks;
        this->oldTicks = this->currentTicks;
        this->realVelocity = ((double)this->dP / (double)this->timestampDiff) * 1000;

        currentRPM = this->rpmFilter->filter(this->smaFilter->filter(this->realVelocity));

        // averageArray[i] = currentRPM;
        output = pid->calculate(targetRPM, currentRPM);

        if (isRecovering && pros::millis() - lastShotTime < 1000)
        {
            // output += 1000;
            // printf("Output: %f Boosting!\n", output);
        }
        flywheelMotor->move_voltage(12000);
        // printf("Target: %f Current: %f Output: %f\n", targetRPM, currentRPM, output);
        acceleration = (currentRPM - oldRPM) / ((pros::millis() - oldTime));
        isShot = acceleration <= -3;
        if (isShot && !isRecovering)
        {
            lastShotTime = pros::millis();
            lastShotSpeed = oldRPM;
            isRecovering = true;
        }

        if (isRecovering)
        {
            if (currentRPM >= lastShotSpeed || targetRPM != oldTarget)
            {
                isRecovering = false;
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
        printf("\n%d,%f,%f", pros::millis(), this->internalVelocityMeasure * 15, this->realVelocity);
        // pros::lcd::clear_line(1);
        // PRINTF1, "Average: " + std::to_string(averageRPM));

        oldRPM = currentRPM;
        oldTarget = targetRPM;
        oldTime = pros::millis();
        pros::Task::delay_until(&now, 20);
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
void Flywheel::updatePID(VelPID *pid)
{
    this->pid = pid;
}