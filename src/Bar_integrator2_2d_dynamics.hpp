#pragma once

// Auto generated file
// Created at: 2024-06-21--11-01-40


namespace dynobench {

void calcV_Bar_integrator2_2d(double* ff,
                    double l,
                    const double *x, const double *u);

void calcStep_Bar_integrator2_2d(double* xnext, 
            double l, 
            const double *x, const double *u, double dt);

void calcJ_Bar_integrator2_2d(
                double* Jx, 
                double* Ju, 
                double l,
                    const double *x, const double *u);

void calcF_Bar_integrator2_2d(
            double* Fx, 
            double* Fu, 
            double l,
            const double *x, const double *u, double dt);

}
