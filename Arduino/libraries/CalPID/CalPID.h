#ifndef CALPID_H
#define CALPID_H

class CalPID
{
private:
    double kp, ki, kd;
    double delta_t;
    double deviation_old;
    double integral;
    double value_PID;
    double max_pid;

public:
    CalPID(double kp_, double ki_, double kd_,double dt, double max); // 3 PID coefficients (even for PD, set I=0), control cycle, max value.
    CalPID();
    void setParameter(double kp_, double ki_, double kd_);
    void setMaxValue(double max);
    void setDELTA_T(double delta_time);
    double calPID(double devia_present);
    double calPI(double devia_present);
    double calPD(double devia_present);
    double calPI_D(double devia_present,double diff_value); // PI-D (Derivative on Measurement) control.
    double calP_D(double devia_present,double diff_value); // P-D (Derivative on Measurement) control.
    void resetIntegral();
};

#endif