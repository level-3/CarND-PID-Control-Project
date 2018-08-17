

#include <uWS/uWS.h>
#include "PID.h"
#include <OGRE/OgreMath.h>

#include <iostream>
#include <cmath>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    
    PID::p = {Kp, Ki, Kd};
    
    PID::dp = {0.01,0.0,0.1};
    

    PID::tolerance = 0.0001;

    PID::best_error = numeric_limits<double>::max();
    PID::err = 0.0;

    t_step = 0;
}

void PID::UpdateError(double cte) {

    //Proportional
    p_error = cte;

    //Integral
    i_error += cte;

    //derivative
    d_error = cte - p_error;

    err += pow(TotalError(),2);
    //std::cout << "\t error" << pid.err;
   

}

double PID::TotalError() {


    return  - ( p[0] * p_error) - (p[1] * i_error) - (p[2] * d_error);
    
}


double PID::AverageError()
{
    //pparam();
        return PID::err / PID::t_step;
   
}


double PID::SumIncrements()
{ 
    return std::fabs( accumulate(PID::dp.begin(), PID::dp.end(), 0.0f) ) ;
}


void PID::ResetSettings()
{

    twiddle = false;
    err = 0.0;
    t_step = 0; 
    
}


void PID::perror()
{
    std::cout << t_step << "\tBest Error : " << best_error << "\tAvg: " << pow(TotalError(),2) << endl;
}

void PID::pparam()
{
    cout << "p : " <<  "\t" <<  p[0] <<  "\t" << p[1] <<  "\t" <<  p[2] << endl ;
}

void PID::Twiddle(int i)
{   



  
    cout << t_step << "\t" <<  best_error << endl;

    cout << setprecision(4) ;

        for (unsigned int i=0; i < p.size(); i++)
        {
            cout << i << "\t>>>>>\t" << p[i] << "\t>>>>>\t";
            double val = dp[i];
            p[i] += val;
            cout << p[i] << endl;
            perror();
            if ( pow(TotalError(),2) < best_error)
            {
                cout << t_step << "\tIncrease param\t";
                best_error = pow(TotalError(),2);
                dp[i] *= 1.1;

                cout << t_step << "\tNew best error " <<  best_error << "\t"  << endl;
            }

            else 
            {
                cout << t_step << "\tDecrease param" << "\t";

                val = dp[i];
                p[i] -= 2 * val;
                cout << p[i] << endl;
                perror();
                if ( pow(TotalError(),2) < best_error)
                {
                    best_error = pow(TotalError(),2);
                    dp[i] = val * 1.1;
                }
                else
                {   
                    p[i] += val;
                    dp[i] = val * 0.9;
                };

                cout << t_step << "\tNew best error " <<  best_error << "\t"  << endl;

            };

        
            cout << t_step << "\t p[i] \t" << p[i] << "\t dp[i] \t" << dp[i] << endl << endl;

        };

    pparam();


    //err = 0.0;
    //Init(p[0],p[1],p[2]);


}