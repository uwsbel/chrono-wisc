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
#include "../utils/CircuitParser.h"


using namespace ::chrono;

class ChElectronicMotor : public ChElectronicCircuit {
public:

    std::shared_ptr<ChBody> spindle;
    
    std::vector<double> VgenVAR;
    std::vector<double> VbackemfCVAR;
    std::vector<double> LaC;
    std::vector<double> RaC;
    std::vector<double> VSW1VAR;
    std::vector<double> VgenPWMVAR;

    double kt_motor = 0.1105; //[Nm/A] 150
    double ke_motor = -0.953e-1;//-0.9e-4; //[Nm/A]
    double L_coil = 0.002f;//0.002f;//33.4e-5f; //[H]
    double R_coil = 69.1f; //[Ohm] Resistance of the equivalent coil 

    
    ChElectronicMotor(std::shared_ptr<ChBody> spindle, double t_step, double t_end)
        : ChElectronicCircuit(
            CircuitParserIO::CircuitDirs {        
                "data/Circuit/MotorControl/Circuit_Netlist.cir",     // netlist path
                "data/Circuit/MotorControl/INPUT_Netlist.txt",       // input path
                "data/Circuit/MotorControl/OUTPUT_Netlist.txt",     // output path
                "data/Circuit/MotorControl/PWL_sources.txt",// pwl_sources path
                "data/Circuit/MotorControl/Param_IC.txt"     // param_ic path
            }, t_step, t_end) {
            this->spindle = spindle;
        }

    void PreInitialize() override {
        this->VgenVAR = { 15.0 };
        this->VbackemfCVAR = { 0.0 };
        this->LaC = { L_coil };
        this->RaC = { R_coil };
        this->VSW1VAR = { 1.0 };
        this->VgenPWMVAR = { 0.0 };
    }


    void SetPWM(double PWM) {
        this->VgenPWMVAR = { PWM };
    }

    void SetVbackEMF() {
        ChVector3d angvel_euler = spindle->GetAngVelLocal(); 
        this->VbackemfCVAR = { ke_motor * angvel_euler[2] };
    }

    void PostInitialize() override {
    }

    void PreAdvance() override {
        this->flow_in.clear();
        this->flow_in.push_back(VgenVAR);
        this->flow_in.push_back(VbackemfCVAR);
        this->flow_in.push_back(LaC);
        this->flow_in.push_back(RaC);
        this->flow_in.push_back(VSW1VAR);
        this->flow_in.push_back(VgenPWMVAR);
    }

    void PostAdvance() override {

        auto res = this->GetResult();

        double IVprobe1 = res["vprobe1"][res["vprobe1"].size() - 1];

        IVprobe1 = 0.4; // Override value for now but keep the last to make sure it's being accessed correclty

        ChVector3d torque_vec_norm(0, 0, 1); // IMPORTANT!! the direction vertex need to be normalized  
        double spindle_torque_mag = this->kt_motor * IVprobe1 * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])   
        ChVector3d spindle_torque = spindle_torque_mag * torque_vec_norm;

        spindle->EmptyAccumulators(); // Clean the body from the previous force/torque 
        spindle->AccumulateTorque(spindle_torque, false); // Apply to the body the force

        // Electro-mechanical coupling
        SetVbackEMF();

    }


};

#endif // CHELECTRONICMOTOR_H
