#include "Bar_integrator2_2d_dynamics.hpp"
#include <cmath>
// Auto generated file
// Created at: 2024-06-21--11-01-40

namespace dynobench {
void calcV_Bar_integrator2_2d(double* ff,
                    double l,
                    const double *x, const double *u)
{

    ff[0] = x[3];
    ff[1] = x[4];
    ff[2] = x[5];
    ff[3] = u[0] + u[2];
    ff[4] = u[1] + u[3];
    ff[5] = -1.0/2.0*l*u[0]*sin(x[2]) + (1.0/2.0)*l*u[1]*cos(x[2]) + (1.0/2.0)*l*u[2]*sin(x[2]) - 1.0/2.0*l*u[3]*cos(x[2]);

}
void calcStep_Bar_integrator2_2d(double* xnext, 
            double l, 
            const double *x, const double *u, double dt)
{

    xnext[0] = dt*x[3] + x[0];
    xnext[1] = dt*x[4] + x[1];
    xnext[2] = dt*x[5] + x[2];
    xnext[3] = dt*(u[0] + u[2]) + x[3];
    xnext[4] = dt*(u[1] + u[3]) + x[4];
    xnext[5] = dt*(-1.0/2.0*l*u[0]*sin(x[2]) + (1.0/2.0)*l*u[1]*cos(x[2]) + (1.0/2.0)*l*u[2]*sin(x[2]) - 1.0/2.0*l*u[3]*cos(x[2])) + x[5];

}
void calcJ_Bar_integrator2_2d(
                double* Jx, 
                double* Ju, 
                double l,
                    const double *x, const double *u)
{
    Jx[18] = 1;
    Jx[25] = 1;
    Jx[32] = 1;
    Jx[17] = -1.0/2.0*l*u[0]*cos(x[2]) - 1.0/2.0*l*u[1]*sin(x[2]) + (1.0/2.0)*l*u[2]*cos(x[2]) + (1.0/2.0)*l*u[3]*sin(x[2]);
    Ju[3] = 1;
    Ju[15] = 1;
    Ju[10] = 1;
    Ju[22] = 1;
    Ju[5] = -1.0/2.0*l*sin(x[2]);
    Ju[11] = (1.0/2.0)*l*cos(x[2]);
    Ju[17] = (1.0/2.0)*l*sin(x[2]);
    Ju[23] = -1.0/2.0*l*cos(x[2]);

}
void calcF_Bar_integrator2_2d(
            double* Fx, 
            double* Fu, 
            double l,
            const double *x, const double *u, double dt)
{
    Fx[0] = 1;
    Fx[18] = dt;
    Fx[7] = 1;
    Fx[25] = dt;
    Fx[14] = 1;
    Fx[32] = dt;
    Fx[21] = 1;
    Fx[28] = 1;
    Fx[17] = dt*(-1.0/2.0*l*u[0]*cos(x[2]) - 1.0/2.0*l*u[1]*sin(x[2]) + (1.0/2.0)*l*u[2]*cos(x[2]) + (1.0/2.0)*l*u[3]*sin(x[2]));
    Fx[35] = 1;
    Fu[3] = dt;
    Fu[15] = dt;
    Fu[10] = dt;
    Fu[22] = dt;
    Fu[5] = -1.0/2.0*dt*l*sin(x[2]);
    Fu[11] = (1.0/2.0)*dt*l*cos(x[2]);
    Fu[17] = (1.0/2.0)*dt*l*sin(x[2]);
    Fu[23] = -1.0/2.0*dt*l*cos(x[2]);

}

}
