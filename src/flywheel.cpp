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
    this->accelEMAFilter = new EMAFilter(0.1);
    this->accelFilter = new SMAFilter(3);
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
    printf("time,vexVelocity,filteredVelocity,rawVelocity,acceleration,emaGain");
    while (true)
    {
        this->currentTime = pros::millis();
        this->currentTicks = flywheelMotor->get_position();
        this->rawTicks = flywheelMotor->get_raw_position(&this->motorTime);

        this->timestampDiff = this->motorTime - this->prev_flywheelMotorTimestamp;
        // Round timestampDiff to nearest 5ms
        this->timestampDiff = floor((this->timestampDiff + 3) / 5) * 5;

        this->prev_flywheelMotorTimestamp = this->motorTime;
        this->internalVelocityMeasure = this->flywheelMotor->get_actual_velocity();
        this->dP = this->currentTicks - this->oldTicks;
        this->oldTicks = this->currentTicks;
        this->realVelocity = ((double)this->dP / (double)this->timestampDiff) * 1000;

        this->currentRPM = this->rpmFilter->filter(this->smaFilter->filter(this->realVelocity));
        this->currentTime = pros::millis();

        this->currentAccel = (this->currentRPM - this->lastRPM) / (this->currentTime - this->prevTime);
        // scale acceleration to ema between 0.1 and 0.3
        this->rpmFilter->setGains(0.1 + (0.15 * (this->currentAccel)));
        this->filteredAccel = this->accelEMAFilter->filter(this->currentAccel);
        // this->filteredAccel = this->accelEMAFilter->filter(this->accelFilter->filter(this->currentAccel));

        this->lastRPM = this->currentRPM;
        this->prevTime = this->currentTime;

        // averageArray[i] = this->currentRPM;
        output = pid->calculate(targetRPM, this->currentRPM);

        if (isRecovering && pros::millis() - lastShotTime < 1000)
        {
            // output += 1000;
            // printf("Output: %f Boosting!\n", output);
        }
        flywheelMotor->move_voltage(12000);
        // printf("Target: %f Current: %f Output: %f\n", targetRPM, this->currentRPM, output);
        acceleration = (this->currentRPM - oldRPM) / ((pros::millis() - oldTime));
        isShot = acceleration <= -3;
        if (isShot && !isRecovering)
        {
            lastShotTime = pros::millis();
            lastShotSpeed = oldRPM;
            isRecovering = true;
        }

        if (isRecovering)
        {
            if (this->currentRPM >= lastShotSpeed || targetRPM != oldTarget)
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
        printf("\n%d,%f,%f,%f,%f,%f", pros::millis(), this->internalVelocityMeasure * 15, this->currentRPM, this->realVelocity, this->filteredAccel, 0.1 + (0.15 * (this->currentAccel)));
        // pros::lcd::clear_line(1);
        // PRINTF1, "Average: " + std::to_string(averageRPM));

        oldRPM = this->currentRPM;
        oldTarget = targetRPM;
        oldTime = pros::millis();
        // pros::Task::delay_until(&now, 20);
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
    return this->currentRPM;
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
    // return abs(this->currentRPM - targetRPM) < 5;
}
void Flywheel::updatePID(VelPID *pid)
{
    this->pid = pid;
}