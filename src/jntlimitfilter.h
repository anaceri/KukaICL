#ifndef JNTLIMITFILTER_H
#define JNTLIMITFILTER_H



class JntLimitFilter
{
public:
    JntLimitFilter(double t);
    void get_filtered_value(double *v_in, double *v_out);
private:
    double jerk_limits[7];
    double accel_limits[7];
    double velocity_limits[7];
    double vel_out[7];
    double lastCorr[7];
    double lastAccel[7];
    double lastJerk[7];
    double cycle_time;
    double speedlimit;

};

#endif // JNTLIMITFILTER_H
