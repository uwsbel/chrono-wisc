// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antonio Recuero, Michael Taylor
// =============================================================================
//
// ANCF airless tire example.
// This is a customizable ANCF tire class which creates an example of an airless
// style tires using ANCF Shell 3443B elements.
//
// =============================================================================

#ifndef ANCF_AIRLESS_TIRE_3443B_H
#define ANCF_AIRLESS_TIRE_3443B_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ANCFAirlessTire3443B : public ChANCFTire {
  public:
    ANCFAirlessTire3443B(const std::string& name);
    ~ANCFAirlessTire3443B() {}

    virtual double GetRadius() const override { return m_rim_radius + m_height; }
    virtual double GetRimRadius() const override { return m_rim_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetDefaultPressure() const override { return 0; }
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const override;

    void SetRimRadius(double rim_radius) { m_rim_radius = rim_radius; }
    void SetHeight(double height) { m_height = height; }
    void SetWidth(double width) { m_width = width; }

    void SetOuterRingThickness(double thickness) { m_t_outer_ring = thickness; }
    void SetSpokeThickness(double thickness) { m_t_spoke = thickness; }
    void SetNumberSpokes(int number) { m_num_spoke = number; }
    void SetTSpokeFraction(double frac) { m_spoke_t_frac = frac;  };
    void SetHubRelativeRotation(double ang) { m_hub_rel_ang = ang; };
    void SetSpokeCurvatureXPoint(double x) { m_spoke_curv_pnt_x = x; };
    void SetSpokeCurvatureZPoint(double z) { m_spoke_curv_pnt_z = z; };

    void SetDivWidth(int div_width) { m_div_width = div_width; }
    void SetDivSpokeLength(int div_spoke_len) { m_div_spoke_len = div_spoke_len; }
    void SetDivOuterRingPerSpoke(int div_ring) { m_div_ring_per_spoke = div_ring; }

    void SetContactMaterial(std::shared_ptr<ChContactMaterialSMC> mat) { m_mat = mat; }

    void SetYoungsModulus(double E) { m_E = E; }
    void SetPoissonsRatio(double nu) { m_nu = nu; }
    void SetDensity(double rho) { m_rho = rho; }
    void SetAlpha(double alpha) { m_alpha = alpha; }
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) override;

  private:
    virtual void CreateContactMaterial() override;

    double m_rim_radius;
    double m_height;
    double m_width;
    double m_t_outer_ring;
    double m_t_spoke;
    double m_spoke_t_frac;
    double m_hub_rel_ang;
    double m_spoke_curv_pnt_x;
    double m_spoke_curv_pnt_z;
    int m_num_spoke;

    int m_div_width;
    int m_div_spoke_len;
    int m_div_ring_per_spoke;

    double m_E;
    double m_nu;
    double m_rho;
    double m_alpha;

    std::shared_ptr<ChContactMaterialSMC> m_mat;

    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> m_hub_nodes;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
