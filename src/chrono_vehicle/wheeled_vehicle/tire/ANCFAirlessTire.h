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
// style tires using ANCF Shell 3423 elements.
//
// =============================================================================

#ifndef ANCF_AIRLESS_TIRE_H
#define ANCF_AIRLESS_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ANCFAirlessTire : public ChANCFTire {
  public:
    ANCFAirlessTire(const std::string& name);
    ~ANCFAirlessTire() {}

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

    void SetDivWidth(int div_width) { m_div_width = div_width; }
    void SetDivSpokeLength(int div_spoke_len) { m_div_spoke_len = div_spoke_len; }
    void SetDivOuterRingPerSpoke(int div_ring) { m_div_ring_per_spoke = div_ring; }

    void SetContactMaterialSpokes(std::shared_ptr<ChContactMaterialSMC> mat) { m_matSpokes = mat; }
    void SetContactMaterialOuterRing(std::shared_ptr<ChContactMaterialSMC> mat) { m_matOuterRing = mat; }

    void SetContactMaterial(std::shared_ptr<ChContactMaterialSMC> mat) {
        m_matSpokes = mat;
        m_matOuterRing = mat;
    }

    // Set Youngs Modulus for Spokes and Outer Ring
    void SetYoungsModulusSpokes(double E) { m_ESpokes = E; }
    void SetYoungsModulusOuterRing(double E) { m_EOuterRing = E; }

    void SetYoungsModulus(double E) {
        m_ESpokes = E;
        m_EOuterRing = E;
    }

    // Set Poissons Ratio
    void SetPoissonsRatio(double nu) { m_nu = nu; }

    // Set Density
    void SetDensity(double rho) { m_rho = rho; }

    // Set Alpha
    void SetAlpha(double alpha) { m_alpha = alpha; }
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) override;

  private:
    virtual void CreateContactMaterial() override;

    double m_rim_radius;
    double m_height;
    double m_width;
    double m_t_outer_ring;
    double m_t_spoke;
    int m_num_spoke;

    int m_div_width;
    int m_div_spoke_len;
    int m_div_ring_per_spoke;

    double m_ESpokes;
    double m_EOuterRing;
    double m_nu;
    double m_rho;
    double m_alpha;

    std::shared_ptr<ChContactMaterialSMC> m_matSpokes;
    std::shared_ptr<ChContactMaterialSMC> m_matOuterRing;
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> m_hub_nodes;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
