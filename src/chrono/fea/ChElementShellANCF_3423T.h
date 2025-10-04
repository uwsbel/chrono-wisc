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
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero, Radu Serban, Michael Taylor
// =============================================================================
// ANCF laminated shell element with four nodes.
// =============================================================================

#ifndef CHELEMENTSHELLANCF3423T_H
#define CHELEMENTSHELLANCF3423T_H

#include <vector>

#include "chrono/fea/ChElementANCF.h"
#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChMaterialShellANCF.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// ANCF laminated shell element with four nodes.
/// This class implements composite material elastic force formulations.
///
/// The node numbering is in ccw fashion as in the following scheme: (TODO: NEED TO UPDATE)
/// <pre>
///         v
///         ^
///         |
/// D o-----+-----o C
///   |     |     |
/// --+-----+-----+----> u
///   |     |     |
/// A o-----+-----o B
/// </pre>
class ChApi ChElementShellANCF_3423T : public ChElementANCF,
                                      public ChElementShell,
                                      public ChLoadableUV,
                                      public ChLoadableUVW {
  public:
    static const int NSF = 12;  ///< number of shape functions

    using ShapeVector = ChMatrixNM<double, 1, NSF>;
    using VectorN = ChVectorN<double, NSF>;
    using MatrixNx3 = ChMatrixNM<double, NSF, 3>;

    ChElementShellANCF_3423T();
    ~ChElementShellANCF_3423T() {}

    /// Definition of a layer
    class Layer {
      public:
        /// Return the layer thickness.
        double GetThickness() const { return m_thickness; }

        /// Return the fiber angle.
        double GetFiberAngle() const { return m_theta; }

        /// Return the layer material.
        std::shared_ptr<ChMaterialShellANCF> GetMaterial() const { return m_material; }

      private:
        /// Private constructor (a layer can be created only by adding it to an element)
        Layer(ChElementShellANCF_3423T* element,              ///< containing element
              double thickness,                              ///< layer thickness
              double theta,                                  ///< fiber angle
              std::shared_ptr<ChMaterialShellANCF> material  ///< layer material
        );

        double Get_detJ0C() const { return m_detJ0C; }
        const ChMatrix66d& Get_T0() const { return m_T0; }

        /// Initial setup for this layer: calculate T0 and detJ0 at the element center.
        void SetupInitial();

        ChElementShellANCF_3423T* m_element;               ///< containing ANCF shell element
        std::shared_ptr<ChMaterialShellANCF> m_material;  ///< layer material
        double m_thickness;                               ///< layer thickness
        double m_theta;                                   ///< fiber angle

        double m_detJ0C;
        ChMatrix66d m_T0;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        friend class ChElementShellANCF_3423T;
        friend class ShellANCF_3423T_Force;
        friend class ShellANCF_3423T_Jacobian;
    };

    /// Get the number of nodes used by this element.
    virtual unsigned int GetNumNodes() override { return 6; }

    /// Get the number of coordinates in the field used by the referenced nodes.
    virtual unsigned int GetNumCoordsPosLevel() override { return 6 * 6; }

    /// Get the number of active coordinates in the field used by the referenced nodes.
    virtual unsigned int GetNumCoordsPosLevelActive() override { return m_element_dof; }

    /// Get the number of coordinates from the n-th node used by this element.
    virtual unsigned int GetNodeNumCoordsPosLevel(unsigned int n) override {
        return m_nodes[n]->GetNumCoordsPosLevel();
    }

    /// Get the number of active coordinates from the n-th node used by this element.
    virtual unsigned int GetNodeNumCoordsPosLevelActive(unsigned int n) override {
        return m_nodes[n]->GetNumCoordsPosLevelActive();
    }

    /// Specify the nodes of this element.
    void SetNodes(std::shared_ptr<ChNodeFEAxyzD> node1,
                  std::shared_ptr<ChNodeFEAxyzD> node2,
                  std::shared_ptr<ChNodeFEAxyzD> nodeB,
                  std::shared_ptr<ChNodeFEAxyzD> nodeC,
                  std::shared_ptr<ChNodeFEAxyzD> node3,
                  std::shared_ptr<ChNodeFEAxyzD> node4);

    /// Specify the element dimensions.
    void SetDimensions(double lenX, double lenY, double lenXT, double Toffset, double TAng) {
        m_lenX = lenX;
        m_lenY = lenY;
        m_lenXT = lenXT;
        m_Toffset = Toffset;
        m_TAng = TAng;
    }

    /// Access the n-th node of this element.
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return m_nodes[n]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNode1() const { return m_nodes[0]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNode2() const { return m_nodes[1]; }

    /// Get a handle to the third node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNodeB() const { return m_nodes[2]; }

    /// Get a handle to the fourth node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNodeC() const { return m_nodes[3]; }

    /// Get a handle to the first node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNode3() const { return m_nodes[4]; }

    /// Get a handle to the second node of this element.
    std::shared_ptr<ChNodeFEAxyzD> GetNode4() const { return m_nodes[5]; }


    /// Add a layer.
    void AddLayer(double thickness,                              ///< layer thickness
                  double theta,                                  ///< fiber angle (radians)
                  std::shared_ptr<ChMaterialShellANCF> material  ///< layer material
    );

    /// Get the number of layers.
    size_t GetNumLayers() const { return m_numLayers; }

    /// Get a handle to the specified layer.
    const Layer& GetLayer(size_t i) const { return m_layers[i]; }

    /// Set the structural damping.
    void SetAlphaDamp(double a) { m_Alpha = a; }

    /// Get the element length in the X direction.
    double GetLengthX() const { return m_lenX; }
    /// Get the element length in the Y direction.
    double GetLengthY() const { return m_lenY; }
    /// Get the total thickness of the shell element.
    double GetThickness() { return m_thickness; }
    /// Get the element length in the X direction for the T element.
    double GetLengthXT() const { return m_lenXT; }
    /// Get the normalized offset of the T connection [-1 = Node 1 to 4 edge, 0 = center, 1 = Node 2 to 3 edge]
    double GetTOffset() const { return m_Toffset; }
    /// Get the angle at the T connection [90deg = T-Down, 0 = in-line, -90 = T-Up]
    double GetTAng() const { return m_TAng; }

    // Shape functions
    // ---------------

    /// Fills the N shape function matrix.
    /// NOTE! actually N should be a 3row, 36 column sparse matrix,
    /// as  N = [s1*eye(3) s2*eye(3) s3*eye(3) s4*eye(3)...]; ,
    /// but to avoid wasting zero and repeated elements, here
    /// it stores only the s1 through s12 values in a 1 row, 12 columns matrix!
    void ShapeFunctions(ShapeVector& N, double x, double y, double z);

    /// Fills the Nx shape function derivative matrix with respect to X.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 12 columns matrix!
    void ShapeFunctionsDerivativeX(ShapeVector& Nx, double x, double y, double z);

    /// Fills the Ny shape function derivative matrix with respect to Y.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 12 columns matrix!
    void ShapeFunctionsDerivativeY(ShapeVector& Ny, double x, double y, double z);

    /// Fills the Nz shape function derivative matrix with respect to Z.
    /// NOTE! to avoid wasting zero and repeated elements, here
    /// it stores only the four values in a 1 row, 12 columns matrix!
    void ShapeFunctionsDerivativeZ(ShapeVector& Nz, double x, double y, double z);

    /// Return a struct with 6-component strain and stress vectors evaluated at a
    /// given quadrature point and layer number.
    ChStrainStress3D EvaluateSectionStrainStress(const ChVector3d& loc, int layer_id);
    void EvaluateDeflection(double& defVec);

  public:
    // Interface to ChElementBase base class
    // -------------------------------------

    /// Fill the D vector with the current field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNumCoordsPosLevel(), it will be resized.
    ///  {x_a y_a z_a Dx_a Dx_a Dx_a x_b y_b z_b Dx_b Dy_b Dz_b}
    virtual void GetStateBlock(ChVectorDynamic<>& mD) override;

    // Set H as a linear combination of M, K, and R.
    //   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R],
    // where [M] is the mass matrix, [K] is the stiffness matrix, and [R] is the damping matrix.
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                          double Kfactor,
                                          double Rfactor = 0,
                                          double Mfactor = 0) override;

    /// Set M as the global mass matrix.
    virtual void ComputeMmatrixGlobal(ChMatrixRef M) override;

    /// Add contribution of element inertia to total nodal masses
    virtual void ComputeNodalMass() override;

    /// Compute the generalized force vector due to gravity using the efficient ANCF specific method
    virtual void ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector3d& G_acc) override;

    /// Computes the internal forces.
    /// (E.g. the actual position of nodes is not in relaxed reference position) and set values in the Fi vector.
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) override;

    /// Update the state of this element.
    virtual void Update() override;

    // Interface to ChElementShell base class
    // --------------------------------------

    virtual void EvaluateSectionDisplacement(const double u,
                                             const double v,
                                             ChVector3d& u_displ,
                                             ChVector3d& u_rotaz) override;

    virtual void EvaluateSectionFrame(const double u, const double v, ChVector3d& point, ChQuaternion<>& rot) override;

    virtual void EvaluateSectionPoint(const double u, const double v, ChVector3d& point) override;

    // Internal computations
    // ---------------------

    /// Compute Jacobians of the internal forces.
    /// This function calculates a linear combination of the stiffness (K) and damping (R) matrices,
    ///     J = Kfactor * K + Rfactor * R
    /// for given coefficients Kfactor and Rfactor.
    /// This Jacobian will be further combined with the global mass matrix M and included in the global
    /// stiffness matrix H in the function ComputeKRMmatricesGlobal().
    void ComputeInternalJacobians(double Kfactor, double Rfactor);

    /// Compute the mass matrix of the element.
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed
    void ComputeMassMatrix();

    /// Compute the matrix to scale gravity by to get the generalized gravitational force.
    void ComputeGravityForceScale();

    // [ANS] Shape function for Assumed Naturals Strain (Interpolation of strain and strainD in a thickness direction)
    void ShapeFunctionANSbilinearShell(ChMatrixNM<double, 1, 4>& S_ANS, double x, double y);

    // [ANS] Calculate the ANS strain and strain derivatives.
    void CalcStrainANSbilinearShell();

    // [EAS] Basis function of M for Enhanced Assumed Strain.
    void Basis_M(ChMatrixNM<double, 6, 5>& M, double x, double y, double z);

    // Calculate the determinant of the initial configuration position vector gradient matrix
    // at the specified point.
    double Calc_detJ0(double x, double y, double z);

    // Same as above, but also return the dense shape function vector derivatives.
    double Calc_detJ0(double x,
                      double y,
                      double z,
                      ShapeVector& Nx,
                      ShapeVector& Ny,
                      ShapeVector& Nz,
                      ChMatrixNM<double, 1, 3>& Nx_d0,
                      ChMatrixNM<double, 1, 3>& Ny_d0,
                      ChMatrixNM<double, 1, 3>& Nz_d0);

    // Calculate the current 12x3 matrix of nodal coordinates.
    void CalcCoordMatrix(ChMatrixNM<double, 12, 3>& d);

    // Calculate the current 36x1 matrix of nodal coordinate derivatives.
    void CalcCoordDtMatrix(ChVectorN<double, 36>& dt);

    // Functions for ChLoadable interface
    // ----------------------------------

    /// Gets the number of DOFs affected by this element (position part).
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 6 * 6; }

    /// Gets the number of DOFs affected by this element (velocity part).
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 6 * 6; }

    /// Gets all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override;

    /// Gets all the DOFs packed in a single vector (velocity part).
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override;

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override;

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual unsigned int GetNumFieldCoords() override { return 6; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 6; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return m_nodes[nblock]->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 6; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !m_nodes[nblock]->IsFixed(); }

    virtual void EvaluateSectionVelNorm(double U, double V, ChVector3d& Result) override;

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    /// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
    /// the applied force in global coordinates and the second 3 entries is the applied moment in global space.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    /// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
    /// the applied force in global coordinates and the second 3 entries is the applied moment in global space.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override;

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity.
    /// Density is mass per unit surface.
    virtual double GetDensity() override;

    /// Gets the normal to the surface at the parametric coordinate U,V.
    /// Each coordinate ranging in -1..+1.
    virtual ChVector3d ComputeNormal(const double U, const double V) override;

  private:
    /// Initial setup. This is used to precompute matrices that do not change during the simulation, such as the local
    /// stiffness of each element (if any), the mass, etc.
    virtual void SetupInitial(ChSystem* system) override;

    //// RADU
    //// Why is m_d_dt inconsistent with m_d?  Why not keep it as an 8x3 matrix?

    std::vector<std::shared_ptr<ChNodeFEAxyzD>> m_nodes;           ///< element nodes
    std::vector<Layer, Eigen::aligned_allocator<Layer>> m_layers;  ///< element layers
    size_t m_numLayers;                                            ///< number of layers for this element
    double m_lenX;                                                 ///< element length in X direction
    double m_lenY;                                                 ///< element length in Y direction
    double m_thickness;                                            ///< total element thickness
    double m_lenXT;                                                ///< length in X direction of the element that forms the flat part of the T
    double m_Toffset;                                              ///< normalized offset for the connection to the T between the edge defined by nodes 1 & 4 [-1] to the edge defined by nodes 2 & 3 [1], value in the range of [-1 to 1 with 0 in the middle]
    double m_TAng;                                                 ///< Angle of the T connection [90deg = T-Down at a right angle, 0deg = in-line with the connecting shell, -90 = T-Up at a right angle
    std::vector<double> m_GaussZ;                                  ///< layer separation z values (scaled to [-1,1])
    double m_GaussScaling;     ///< scaling factor due to change of integration intervals
    double m_Alpha;            ///< structural damping
    VectorN m_GravForceScale;  ///< Gravity scaling matrix used to get the generalized force due to gravity
    ChMatrixNM<double, 36, 36> m_MassMatrix;            ///< mass matrix
    ChMatrixNM<double, 36, 36> m_JacobianMatrix;        ///< Jacobian matrix (Kfactor*[K] + Rfactor*[R])
    ChMatrixNM<double, 12, 3> m_d0;                     ///< initial nodal coordinates
    ChMatrixNM<double, 12, 12> m_d0d0T;                 ///< matrix m_d0 * m_d0^T
    ChMatrixNM<double, 12, 3> m_d;                      ///< current nodal coordinates
    ChMatrixNM<double, 12, 12> m_ddT;                   ///< matrix m_d * m_d^T
    ChVectorN<double, 36> m_d_dt;                       ///< current nodal velocities
    ChVectorN<double, 8> m_strainANS;                   ///< ANS strain
    ChMatrixNM<double, 8, 36> m_strainANS_D;            ///< ANS strain derivatives
    std::vector<ChVectorN<double, 5>> m_alphaEAS;       ///< EAS parameters (5 per layer)
    std::vector<ChMatrixNM<double, 5, 5>> m_KalphaEAS;  ///< EAS Jacobians (a 5x5 matrix per layer)
    static const double m_toleranceEAS;                 ///< tolerance for nonlinear EAS solver (on residual)
    static const int m_maxIterationsEAS;                ///< maximum number of nonlinear EAS iterations

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend class ShellANCF_3423T_Mass;
    friend class ShellANCF_3423T_Gravity;
    friend class ShellANCF_3423T_Force;
    friend class ShellANCF_3423T_Jacobian;
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
