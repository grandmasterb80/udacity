#include <iostream>
#include <cmath>

int main(int argc, char** argv)
{
    double s0 = 25;      // distance to lead in meter
    double v0 = -30/3.6;  // relative speed to lead in meter per second
    double a0 = -5;      // maximal deceleration of ego in meter / seconds^2
    
    double ttc1 = - ( v0 / (2.0 * a0) ) + sqrt( s0 + v0 * v0 / ( 4.0 * a0 ) );
    double ttc2 = - ( v0 / (2.0 * a0) ) - sqrt( s0 + v0 * v0 / ( 4.0 * a0 ) );
    double ttc3 = s0 / v0;
    std::cout << "TTC CAM 1 = " << ttc1 << std::endl;
    std::cout << "TTC CAM 2 = " << ttc2 << std::endl;
    std::cout << "*******************************************************" << std::endl;
    std::cout << "TTC CVM   = " << ttc3 << std::endl;
    std::cout << "*******************************************************" << std::endl;
    double s;
    double v;
    double dt;
    double t;

    s = s0;
    v = v0;
    dt = 0.00001;
    t = 0.0;
    while(s > 0.0 && t < 10.0)
    {
        s+=v*dt;
        t+=dt;
    }
    std::cout << "TTC CVM = " << t << std::endl;
    
    s = s0;
    v = v0;
    dt = 0.00001;
    t = 0.0;
    while(s > 0.0 && t < 10.0)
    {
        s+=v*dt;
        v+=a0*dt;
        t+=dt;
    }
    std::cout << "TTC CAM = " << t << std::endl;
    
    return 0;
}
