// ==================================================================================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ==================================================================================================================================================
// Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
// ==================================================================================================================================================

#ifndef CHELECTRONICMOTOR_H
#define CHELECTRONICMOTOR_H

#include "ChElectronicCircuit.h"
#include "chrono/physics/ChBody.h"


using namespace ::chrono;

class ChElectronicMotor : public ChElectronicCircuit {
public:

    std::shared_ptr<ChBody> spindle = nullptr;
    
    double VbackemfCVAR;
    double VgenPWMVAR;

    double kt_motor = 0.1105; //[Nm/A] 150
    double ke_motor = -0.953e-1;//-0.9e-4; //[Nm/A]
    double L_coil = 0.002f;//0.002f;//33.4e-5f; //[H]
    double R_coil = 69.1f; //[Ohm] Resistance of the equivalent coil 

    double shaft_angvel;
    ChVector3d torque;
    
    ChElectronicMotor(std::shared_ptr<ChBody> spindle, double t_step, double t_end)
        : ChElectronicCircuit(
            "../data/Circuit/MotorControl/Circuit_Netlist.cir", t_step, t_end) {
            this->spindle = spindle;
        }

    ChElectronicMotor(double t_step, double t_end) : ChElectronicCircuit(
            "../data/Circuit/MotorControl/Circuit_Netlist.cir", t_step, t_end) 
    {
                
    }


    void PreInitialize() override {

        /* Initialize the keys and initial values of the PWL sources
        */
        this->SetInitialPWLIn({
           {"VgenVAR", 15. },
           {"VbackemfCVAR", 0.0},
           {"VSW1VAR", 1.},
           {"VgenPWMVAR", 5.200000e+00}
        });

        /*
        These can be any parameter you want to use as a parameter {}
        */
        this->SetInitialFlowInICs({
           {"LaC", L_coil},
           {"RaC", R_coil},
        });

        /*
        this->SetInitialCurrents({
           {"LaC", 0.}
        })
        */

    }

    void SetShaftAngVel(double angvel) {
        this->shaft_angvel = angvel;
    }

    ChVector3d GetOutputTorque() {
        return this->spindle_torque;
    }

    void SetPWM(double PWM) {
        this->VgenPWMVAR = PWM;
    }

    void PostInitialize() override {
    }

    void PreAdvance() override {
        this->flow_in["LaC"] = L_coil;
        this->flow_in["RaC"] = R_coil;

        this->pwl_in["VgenVAR"] = 15.;
        this->pwl_in["VbackemfCVAR"] = this->VbackemfCVAR;
        this->pwl_in["VSW1VAR"] = 1.;
        this->pwl_in["VgenPWMVAR"] = this->VgenPWMVAR;
    }

    
    void PostAdvance() override {

        auto res = this->GetResult();

        double IVprobe1 = res["vprobe1"][res["vprobe1"].size() - 1];

        std::cout << IVprobe1 << std::endl;

        // IVprobe1 = 0.4; // Override value for now but keep the last to make sure it's being accessed correclty

        ChVector3d torque_vec_norm(0, 0, 1); // IMPORTANT!! the direction vertex need to be normalized  
        double spindle_torque_mag = this->kt_motor * IVprobe1 * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])   
        ChVector3d spindle_torque = spindle_torque_mag * torque_vec_norm;

        double ang_vel = this->shaft_angvel;

        if(this->spindle != nullptr) {  // If motor was initialized with spindle, use it as ang vel source
            ang_vel = spindle->GetAngVelLocal()[2];
        }
    
        // Electro-mechanical coupling
        this->VbackemfCVAR = ke_motor * ang_vel;

        if(this->spindle != nullptr) {
            spindle->EmptyAccumulators(); // Clean the body from the previous force/torque 
            spindle->AccumulateTorque(spindle_torque, false); // Apply to the body the force
        }

        this->torque = spindle_torque;

    }


};

#endif // CHELECTRONICMOTOR_H
