// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: bgwitt
// =============================================================================
//
// Rigid planetary terrain: a collision patch of a ChPlanetSurface in a site
// frame, rebuilt to follow a moving vehicle.
//
// =============================================================================

#ifndef PLANET_TERRAIN_H
#define PLANET_TERRAIN_H

#include <functional>
#include <memory>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

#include "chrono_planet/ChPlanetSurface.h"
#include "chrono_planet/ChSiteFrame.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Rigid terrain over a ChPlanetSurface, as a square collision patch that follows the vehicle.
/// Heights and normals come from the surface itself, so they are exact anywhere; the collision
/// mesh covers only the patch and is rebuilt when the tracked position nears its edge. A rebuild
/// swaps the ground body, which stalls the step for the mesh construction.
class CH_VEHICLE_API PlanetTerrain : public ChTerrain {
  public:
    /// Construct a terrain over the surface in the given site frame.
    PlanetTerrain(ChSystem* system,
                  std::shared_ptr<const planet::ChPlanetSurface> surface,
                  const planet::ChSiteFrame& site);
    ~PlanetTerrain();

    /// Set the patch side length (default: 30 m). Must be called before Initialize.
    void SetPatchSize(double size);

    /// Set the patch grid spacing (default: 0.25 m). Must be called before Initialize.
    void SetPatchResolution(double resolution);

    /// Set the distance from the patch edge at which a rebuild triggers (default: 4 m).
    /// Clamped to a quarter of the patch size.
    void SetRebuildMargin(double margin);

    /// Set the coefficient of friction (default: 0.8), used by the default contact material
    /// and reported by GetCoefficientFriction.
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set the contact material of the patch (default: the system's default material with SetContactFrictionCoefficient).
    void SetContactMaterial(std::shared_ptr<ChContactMaterial> material) { m_material = material; }

    /// Build the collision mesh as a triangle soup instead of a connected mesh (default: false).
    void UseTriangleSoup(bool val) { m_soup = val; }

    /// Leave a rectangle of site x/y out of the collision mesh, cutting the faces that cross its edges, where other
    /// ground takes over, such as a work site modeled on its own with its own collision shapes. Height queries inside
    /// it go to `height` (site x/y to height, m), if given, such as that ground's top height. Applies from the next patch rebuild; call before Initialize.
    void AddHole(const planet::ChSiteRegion& hole, std::function<double(double x, double y)> height = nullptr);

    /// Keep the patch from colliding with a collision family, such as the fixed bodies of other ground, which it would
    /// otherwise be tested against. Applies from the next patch rebuild; call before Initialize.
    void DisallowCollisionsWith(int family) { m_disallowed_families.push_back(family); }

    /// Build the first patch, centered at the given site x/y.
    void Initialize(const ChVector2d& center = ChVector2d(0, 0));

    /// Rebuild the patch if loc is within the rebuild margin of its edge. Returns true if rebuilt.
    bool UpdatePatch(const ChVector3d& loc);

    /// Rebuild the patch centered at the given site x/y.
    void RebuildPatch(const ChVector2d& center);

    /// Get the terrain height below the specified location.
    virtual double GetHeight(const ChVector3d& loc) const override;

    /// Get the terrain point below the specified location.
    virtual ChVector3d GetPoint(const ChVector3d& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual ChVector3d GetNormal(const ChVector3d& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// Defers to a registered FrictionFunctor, otherwise returns the constant value.
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override;

    std::shared_ptr<ChBody> GetGroundBody() const { return m_ground; }               ///< current patch body
    ChVector2d GetPatchCenter() const { return m_center; }                            ///< current patch center
    double GetPatchSize() const { return m_size; }                                    ///< patch side length
    const planet::ChSiteFrame& GetSiteFrame() const { return m_site; }                ///< site frame
    std::shared_ptr<const planet::ChPlanetSurface> GetSurface() const { return m_surface; }  ///< height source

  private:
    double SurfaceHeight(double x, double y) const;
    // The height function of the hole holding site x/y, if any
    const std::function<double(double, double)>* HoleHeight(double x, double y) const;

    ChSystem* m_system;
    std::shared_ptr<const planet::ChPlanetSurface> m_surface;
    planet::ChSiteFrame m_site;

    double m_size;
    double m_resolution;
    double m_margin;
    float m_friction;
    bool m_soup;
    std::shared_ptr<ChContactMaterial> m_material;

    ChVector2d m_center;
    std::shared_ptr<ChBody> m_ground;

    std::vector<planet::ChSiteRegion> m_holes;
    std::vector<std::function<double(double, double)>> m_hole_heights;
    std::vector<int> m_disallowed_families;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
