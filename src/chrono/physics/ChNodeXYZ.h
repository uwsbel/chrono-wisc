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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHNODEXYZ_H
#define CHNODEXYZ_H

#include "chrono/physics/ChLoadable.h"
#include "chrono/physics/ChNodeBase.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

/// Class for a single 'point' node, that has 3 DOF degrees of freedom and a mass.
class ChApi ChNodeXYZ : public virtual ChNodeBase, public ChLoadableUVW {
  public:
    ChNodeXYZ();
    ChNodeXYZ(const ChVector3d& initial_pos);
    ChNodeXYZ(const ChNodeXYZ& other);
    virtual ~ChNodeXYZ() {}

    ChNodeXYZ& operator=(const ChNodeXYZ& other);

    // Access the xyz 'variables' of the node
    virtual ChVariablesNode& Variables() = 0;

    // Position of the node - in absolute csys.
    const ChVector3d& GetPos() const { return pos; }

    // Position of the node - in absolute csys.
    void SetPos(const ChVector3d& mpos) { pos = mpos; }

    // Velocity of the node - in absolute csys.
    const ChVector3d& GetPosDt() const { return pos_dt; }

    // Velocity of the node - in absolute csys.
    void SetPosDt(const ChVector3d& mposdt) { pos_dt = mposdt; }

    // Acceleration of the node - in absolute csys.
    const ChVector3d& GetPosDt2() const { return pos_dtdt; }

    // Acceleration of the node - in absolute csys.
    void SetPosDt2(const ChVector3d& mposdtdt) { pos_dtdt = mposdtdt; }

    // Get mass of the node. To be implemented in children classes
    virtual double GetMass() const = 0;

    // Set mass of the node. To be implemented in children classes
    virtual void SetMass(double mm) = 0;

    /// Get the number of degrees of freedom
    virtual unsigned int GetNumCoordsPosLevel() const override { return 3; }

    // INTERFACE to ChLoadable

    /// Gets the number of DOFs affected by this element (position part)
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, etc. Here is 6: xyz displ + xyz rots
    virtual unsigned int GetNumFieldCoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 1; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override { return NodeGetOffsetVelLevel(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return true; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate Q=N'*F , for Q generalized lagrangian load, where N is some type of matrix
    /// evaluated at point P(U,V,W) assumed in absolute coordinates, and
    /// F is a load assumed in absolute coordinates.
    /// The det[J] is unused.
    virtual void ComputeNF(
        const double U,              ///< x coordinate of application point in absolute space
        const double V,              ///< y coordinate of application point in absolute space
        const double W,              ///< z coordinate of application point in absolute space
        ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
        double& detJ,                ///< Return det[J] here
        const ChVectorDynamic<>& F,  ///< Input F vector, size is 3, it is Force x,y,z in absolute coords.
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
        ) override;

    /// This is not needed because not used in quadrature.
    virtual double GetDensity() override { return 1; }

    // SERIALIZATION
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // DATA
    ChVector3d pos;
    ChVector3d pos_dt;
    ChVector3d pos_dtdt;
};

CH_CLASS_VERSION(ChNodeXYZ, 0)

}  // end namespace chrono

#endif
