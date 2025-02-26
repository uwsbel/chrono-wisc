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
// Authors: Michael Taylor, Antonio Recuero, Radu Serban
// =============================================================================
// Fully Parameterized ANCF shell element with 4 nodes (48DOF). A Description of this element can be found in: Aki M
// Mikkola and Ahmed A Shabana. A non-incremental finite element procedure for the analysis of large deformation of
// plates and shells in mechanical system applications. Multibody System Dynamics, 9(3) : 283–309, 2003.
// =============================================================================
//
// =============================================================================
// Global Nodal Coordinate version of the Shell 3443 element based on:
// TR08 = Vectorized Continuous Integration without Data Caching for the Jacobian
// =============================================================================
// Mass Matrix = Compact Upper Triangular
// Reduced Number of GQ Points
// Nodal Coordinates in Matrix Form
// PK1 Stress
// Precomputed Adjusted Shape Function Derivatives and minus Element Jacobians time the corresponding GQ Weight
// Analytic Jacobian without calculations cached from the internal force calculation
// =============================================================================

#include "chrono/fea/ChElementShellANCF_3443B.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementShellANCF_3443B::ChElementShellANCF_3443B()
    : m_numLayers(0), m_lenX(0), m_lenY(0), m_thicknessZ(0), m_midsurfoffset(0), m_Alpha(0), m_damping_enabled(false) {
    m_nodes.resize(4);
    m_Tij.setIdentity();
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementShellANCF_3443B::SetNodes(std::shared_ptr<ChNodeFEAxyzDDD> nodeA,
                                            std::shared_ptr<ChNodeFEAxyzDDD> nodeB,
                                            std::shared_ptr<ChNodeFEAxyzDDD> nodeC,
                                            std::shared_ptr<ChNodeFEAxyzDDD> nodeD) {
    assert(nodeA);
    assert(nodeB);
    assert(nodeC);
    assert(nodeD);

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    m_nodes[2] = nodeC;
    m_nodes[3] = nodeD;

    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[0]->VariablesSlope1());
    mvars.push_back(&m_nodes[0]->VariablesSlope2());
    mvars.push_back(&m_nodes[0]->VariablesSlope3());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[1]->VariablesSlope1());
    mvars.push_back(&m_nodes[1]->VariablesSlope2());
    mvars.push_back(&m_nodes[1]->VariablesSlope3());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[2]->VariablesSlope1());
    mvars.push_back(&m_nodes[2]->VariablesSlope2());
    mvars.push_back(&m_nodes[2]->VariablesSlope3());
    mvars.push_back(&m_nodes[3]->Variables());
    mvars.push_back(&m_nodes[3]->VariablesSlope1());
    mvars.push_back(&m_nodes[3]->VariablesSlope2());
    mvars.push_back(&m_nodes[3]->VariablesSlope3());

    Kmatr.SetVariables(mvars);

    // Initial positions and slopes of the element nodes
    // These values define the reference configuration of the element
    // CalcCoordMatrix(m_ebar0);
}

// -----------------------------------------------------------------------------
// Add a layer.
// -----------------------------------------------------------------------------

void ChElementShellANCF_3443B::AddLayer(double thickness,
                                            double theta,
                                            std::shared_ptr<ChMaterialShellANCF> material) {
    m_layers.push_back(Layer(thickness, theta, material));
    m_layer_zoffsets.push_back(m_thicknessZ);
    m_numLayers += 1;
    m_thicknessZ += thickness;
}

// -----------------------------------------------------------------------------
// Element Settings
// -----------------------------------------------------------------------------

// Specify the element dimensions.

void ChElementShellANCF_3443B::SetDimensions(double lenX, double lenY) {
    m_lenX = lenX;
    m_lenY = lenY;
}

// Offset the midsurface of the composite shell element.  A positive value shifts the element's midsurface upward
// along the elements zeta direction.  The offset should be provided in model units.

void ChElementShellANCF_3443B::SetMidsurfaceOffset(const double offset) {
    m_midsurfoffset = offset;
}

// Set the value for the single term structural damping coefficient.

void ChElementShellANCF_3443B::SetAlphaDamp(double a) {
    m_Alpha = a;
    if (std::abs(m_Alpha) > 1e-10)
        m_damping_enabled = true;
    else
        m_damping_enabled = false;
}

// Set the values of the reference local element coordinates.

void ChElementShellANCF_3443B::SetLocalElRefCoords(const ChVector3d NodeA_Pos,
    const ChVector3d NodeA_Slope1,
    const ChVector3d NodeA_Slope2,
    const ChVector3d NodeA_Slope3,
    const ChVector3d NodeB_Pos,
    const ChVector3d NodeB_Slope1,
    const ChVector3d NodeB_Slope2,
    const ChVector3d NodeB_Slope3,
    const ChVector3d NodeC_Pos,
    const ChVector3d NodeC_Slope1,
    const ChVector3d NodeC_Slope2,
    const ChVector3d NodeC_Slope3,
    const ChVector3d NodeD_Pos,
    const ChVector3d NodeD_Slope1,
    const ChVector3d NodeD_Slope2,
    const ChVector3d NodeD_Slope3) {

    m_ebar0.col(0) = NodeA_Pos.eigen();
    m_ebar0.col(1) = NodeA_Slope1.eigen();
    m_ebar0.col(2) = NodeA_Slope2.eigen();
    m_ebar0.col(3) = NodeA_Slope3.eigen();

    m_ebar0.col(4) = NodeB_Pos.eigen();
    m_ebar0.col(5) = NodeB_Slope1.eigen();
    m_ebar0.col(6) = NodeB_Slope2.eigen();
    m_ebar0.col(7) = NodeB_Slope3.eigen();

    m_ebar0.col(8) = NodeC_Pos.eigen();
    m_ebar0.col(9) = NodeC_Slope1.eigen();
    m_ebar0.col(10) = NodeC_Slope2.eigen();
    m_ebar0.col(11) = NodeC_Slope3.eigen();

    m_ebar0.col(12) = NodeD_Pos.eigen();
    m_ebar0.col(13) = NodeD_Slope1.eigen();
    m_ebar0.col(14) = NodeD_Slope2.eigen();
    m_ebar0.col(15) = NodeD_Slope3.eigen();

    ChVector3d dir1(1, 0, 0);
    ChVector3d dir2(0, 1, 0);
    ChVector3d dir3(0, 0, 1);

    m_pbar0.col(0) = NodeA_Pos.eigen();
    m_pbar0.col(1) = dir1.eigen();
    m_pbar0.col(2) = dir2.eigen();
    m_pbar0.col(3) = dir3.eigen();

    m_pbar0.col(4) = NodeB_Pos.eigen();
    m_pbar0.col(5) = dir1.eigen();
    m_pbar0.col(6) = dir2.eigen();
    m_pbar0.col(7) = dir3.eigen();

    m_pbar0.col(8)  = NodeC_Pos.eigen();
    m_pbar0.col(9)  = dir1.eigen();
    m_pbar0.col(10) = dir2.eigen();
    m_pbar0.col(11) = dir3.eigen();

    m_pbar0.col(12) = NodeD_Pos.eigen();
    m_pbar0.col(13) = dir1.eigen();
    m_pbar0.col(14) = dir2.eigen();
    m_pbar0.col(15) = dir3.eigen();


    ComputeNodalCoordTransform();
}

// -----------------------------------------------------------------------------
// Evaluate Strains and Stresses
// -----------------------------------------------------------------------------
// These functions are designed for single function calls.  If these values are needed at the same points in the element
// through out the simulation, then the adjusted normalized shape function derivative matrix (Sxi_D) for each query
// point should be cached and saved to increase the execution speed
// -----------------------------------------------------------------------------

// Get the Green-Lagrange strain tensor at the normalized element coordinates (xi, eta, zeta) [-1...1]

ChMatrix33d ChElementShellANCF_3443B::GetGreenLagrangeStrain(const double xi,
                                                                  const double eta,
                                                                  const double zeta) {
    MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives
    Calc_Sxi_D(Sxi_D, xi, eta, zeta, m_thicknessZ, m_midsurfoffset);

    ChMatrix33d J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
    J_0xi.noalias() = m_pbar0 * Sxi_D;

    Sxi_D = Sxi_D * J_0xi.inverse();  // Adjust the shape function derivative matrix to account for the potentially
                                      // distorted reference configuration

    Matrix3xN e_bar;  // Element coordinates in matrix form
    CalcCoordMatrix(e_bar);

    // Calculate the Deformation Gradient at the current point
    ChMatrixNM_col<double, 3, 3> F = e_bar * Sxi_D;

    ChMatrix33d I3x3;
    I3x3.setIdentity();
    return 0.5 * (F.transpose() * F - I3x3);
}

// Get the 2nd Piola-Kirchoff stress tensor at the normalized **layer** coordinates (xi, eta, layer_zeta) at the current
// state of the element for the specified layer number (0 indexed) since the stress can be discontinuous at the layer
// boundary.   "layer_zeta" spans -1 to 1 from the bottom surface to the top surface

ChMatrix33d ChElementShellANCF_3443B::GetPK2Stress(const double layer,
                                                        const double xi,
                                                        const double eta,
                                                        const double layer_zeta) {
    MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives
    double layer_midsurface_offset =
        -m_thicknessZ / 2 + m_layer_zoffsets[layer] + m_layers[layer].Get_thickness() / 2 + m_midsurfoffset;
    Calc_Sxi_D(Sxi_D, xi, eta, layer_zeta, m_layers[layer].Get_thickness(), layer_midsurface_offset);

    ChMatrix33d J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
    J_0xi.noalias() = m_pbar0 * Sxi_D;

    Sxi_D = Sxi_D * J_0xi.inverse();  // Adjust the shape function derivative matrix to account for the potentially
                                      // distorted reference configuration

    Matrix3xN e_bar;  // Element coordinates in matrix form
    CalcCoordMatrix(e_bar);

    // Calculate the Deformation Gradient at the current point
    ChMatrixNM_col<double, 3, 3> F = e_bar * Sxi_D;

    // Calculate the Green-Lagrange strain tensor at the current point in Voigt notation
    ChVectorN<double, 6> epsilon_combined;
    epsilon_combined(0) = 0.5 * (F.col(0).dot(F.col(0)) - 1);
    epsilon_combined(1) = 0.5 * (F.col(1).dot(F.col(1)) - 1);
    epsilon_combined(2) = 0.5 * (F.col(2).dot(F.col(2)) - 1);
    epsilon_combined(3) = F.col(1).dot(F.col(2));
    epsilon_combined(4) = F.col(0).dot(F.col(2));
    epsilon_combined(5) = F.col(0).dot(F.col(1));

    if (m_damping_enabled) {
        Matrix3xN ebardot;  // Element coordinate time derivatives in matrix form
        CalcCoordDtMatrix(ebardot);

        // Calculate the time derivative of the Deformation Gradient at the current point
        ChMatrixNM_col<double, 3, 3> Fdot = ebardot * Sxi_D;

        // Calculate the time derivative of the Green-Lagrange strain tensor in Voigt notation
        // and combine it with epsilon assuming a Linear Kelvin-Voigt Viscoelastic material model
        epsilon_combined(0) += m_Alpha * F.col(0).dot(Fdot.col(0));
        epsilon_combined(1) += m_Alpha * F.col(1).dot(Fdot.col(1));
        epsilon_combined(2) += m_Alpha * F.col(2).dot(Fdot.col(2));
        epsilon_combined(3) += m_Alpha * (F.col(1).dot(Fdot.col(2)) + Fdot.col(1).dot(F.col(2)));
        epsilon_combined(4) += m_Alpha * (F.col(0).dot(Fdot.col(2)) + Fdot.col(0).dot(F.col(2)));
        epsilon_combined(5) += m_Alpha * (F.col(0).dot(Fdot.col(1)) + Fdot.col(0).dot(F.col(1)));
    }

    // Get the stiffness tensor in 6x6 matrix form for the current layer and rotate it in the midsurface according
    // to the user specified angle.  Note that the matrix is reordered as well to match the Voigt notation used in
    // this element compared to what is used in ChMaterialShellANCF

    ChMatrix66d D = m_layers[layer].GetMaterial()->Get_E_eps();
    RotateReorderStiffnessMatrix(D, m_layers[layer].Get_theta());

    ChVectorN<double, 6> sigmaPK2 = D * epsilon_combined;  // 2nd Piola Kirchhoff Stress tensor in Voigt notation

    ChMatrix33d SPK2;
    SPK2(0, 0) = sigmaPK2(0);
    SPK2(1, 1) = sigmaPK2(1);
    SPK2(2, 2) = sigmaPK2(2);
    SPK2(1, 2) = sigmaPK2(3);
    SPK2(2, 1) = sigmaPK2(3);
    SPK2(0, 2) = sigmaPK2(4);
    SPK2(2, 0) = sigmaPK2(4);
    SPK2(0, 1) = sigmaPK2(5);
    SPK2(1, 0) = sigmaPK2(5);

    return SPK2;
}

// Get the von Mises stress value at the normalized **layer** coordinates (xi, eta, layer_zeta) at the current state
// of the element for the specified layer number (0 indexed) since the stress can be discontinuous at the layer
// boundary.  "layer_zeta" spans -1 to 1 from the bottom surface to the top surface

double ChElementShellANCF_3443B::GetVonMissesStress(const double layer,
                                                        const double xi,
                                                        const double eta,
                                                        const double layer_zeta) {
    MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives
    double layer_midsurface_offset =
        -m_thicknessZ / 2 + m_layer_zoffsets[layer] + m_layers[layer].Get_thickness() / 2 + m_midsurfoffset;
    Calc_Sxi_D(Sxi_D, xi, eta, layer_zeta, m_layers[layer].Get_thickness(), layer_midsurface_offset);

    ChMatrix33d J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
    J_0xi.noalias() = m_pbar0 * Sxi_D;

    Sxi_D = Sxi_D * J_0xi.inverse();  // Adjust the shape function derivative matrix to account for the potentially
                                      // distorted reference configuration

    Matrix3xN e_bar;  // Element coordinates in matrix form
    CalcCoordMatrix(e_bar);

    // Calculate the Deformation Gradient at the current point
    ChMatrixNM_col<double, 3, 3> F = e_bar * Sxi_D;

    // Calculate the Green-Lagrange strain tensor at the current point in Voigt notation
    ChVectorN<double, 6> epsilon_combined;
    epsilon_combined(0) = 0.5 * (F.col(0).dot(F.col(0)) - 1);
    epsilon_combined(1) = 0.5 * (F.col(1).dot(F.col(1)) - 1);
    epsilon_combined(2) = 0.5 * (F.col(2).dot(F.col(2)) - 1);
    epsilon_combined(3) = F.col(1).dot(F.col(2));
    epsilon_combined(4) = F.col(0).dot(F.col(2));
    epsilon_combined(5) = F.col(0).dot(F.col(1));

    if (m_damping_enabled) {
        Matrix3xN ebardot;  // Element coordinate time derivatives in matrix form
        CalcCoordDtMatrix(ebardot);

        // Calculate the time derivative of the Deformation Gradient at the current point
        ChMatrixNM_col<double, 3, 3> Fdot = ebardot * Sxi_D;

        // Calculate the time derivative of the Green-Lagrange strain tensor in Voigt notation
        // and combine it with epsilon assuming a Linear Kelvin-Voigt Viscoelastic material model
        epsilon_combined(0) += m_Alpha * F.col(0).dot(Fdot.col(0));
        epsilon_combined(1) += m_Alpha * F.col(1).dot(Fdot.col(1));
        epsilon_combined(2) += m_Alpha * F.col(2).dot(Fdot.col(2));
        epsilon_combined(3) += m_Alpha * (F.col(1).dot(Fdot.col(2)) + Fdot.col(1).dot(F.col(2)));
        epsilon_combined(4) += m_Alpha * (F.col(0).dot(Fdot.col(2)) + Fdot.col(0).dot(F.col(2)));
        epsilon_combined(5) += m_Alpha * (F.col(0).dot(Fdot.col(1)) + Fdot.col(0).dot(F.col(1)));
    }

    // Get the stiffness tensor in 6x6 matrix form for the current layer and rotate it in the midsurface according
    // to the user specified angle.  Note that the matrix is reordered as well to match the Voigt notation used in
    // this element compared to what is used in ChMaterialShellANCF

    ChMatrix66d D = m_layers[layer].GetMaterial()->Get_E_eps();
    RotateReorderStiffnessMatrix(D, m_layers[layer].Get_theta());

    ChVectorN<double, 6> sigmaPK2 = D * epsilon_combined;  // 2nd Piola Kirchhoff Stress tensor in Voigt notation

    ChMatrixNM<double, 3, 3> SPK2;  // 2nd Piola Kirchhoff Stress tensor
    SPK2(0, 0) = sigmaPK2(0);
    SPK2(1, 1) = sigmaPK2(1);
    SPK2(2, 2) = sigmaPK2(2);
    SPK2(1, 2) = sigmaPK2(3);
    SPK2(2, 1) = sigmaPK2(3);
    SPK2(0, 2) = sigmaPK2(4);
    SPK2(2, 0) = sigmaPK2(4);
    SPK2(0, 1) = sigmaPK2(5);
    SPK2(1, 0) = sigmaPK2(5);

    // Convert from 2ndPK Stress to Cauchy Stress
    ChMatrix33d S = (F * SPK2 * F.transpose()) / F.determinant();
    double SVonMises =
        sqrt(0.5 * ((S(0, 0) - S(1, 1)) * (S(0, 0) - S(1, 1)) + (S(1, 1) - S(2, 2)) * (S(1, 1) - S(2, 2)) +
                    (S(2, 2) - S(0, 0)) * (S(2, 2) - S(0, 0))) +
             3 * (S(1, 2) * S(1, 2) + S(2, 0) * S(2, 0) + S(0, 1) * S(0, 1)));

    return (SVonMises);
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.

void ChElementShellANCF_3443B::SetupInitial(ChSystem* system) {
    // Store the initial nodal coordinates. These values define the reference configuration of the element.
    //CalcCoordMatrix(m_ebar0);

    // Compute the transform between global nodal coordiantes and local nodal coordiantes
    ComputeNodalCoordTransform();

    // Compute and store the constant mass matrix and the matrix used to multiply the acceleration due to gravity to get
    // the generalized gravitational force vector for the element
    ComputeMassMatrixAndGravityForce();

    // Compute Pre-computed matrices and vectors for the generalized internal force calcualtions
    PrecomputeInternalForceMatricesWeights();
}

// Fill the D vector with the current field values at the element nodes.

void ChElementShellANCF_3443B::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = m_nodes[0]->GetSlope1().eigen();
    mD.segment(6, 3) = m_nodes[0]->GetSlope2().eigen();
    mD.segment(9, 3) = m_nodes[0]->GetSlope3().eigen();

    mD.segment(12, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(15, 3) = m_nodes[1]->GetSlope1().eigen();
    mD.segment(18, 3) = m_nodes[1]->GetSlope2().eigen();
    mD.segment(21, 3) = m_nodes[1]->GetSlope3().eigen();

    mD.segment(24, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(27, 3) = m_nodes[2]->GetSlope1().eigen();
    mD.segment(30, 3) = m_nodes[2]->GetSlope2().eigen();
    mD.segment(33, 3) = m_nodes[2]->GetSlope3().eigen();

    mD.segment(36, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(39, 3) = m_nodes[3]->GetSlope1().eigen();
    mD.segment(42, 3) = m_nodes[3]->GetSlope2().eigen();
    mD.segment(45, 3) = m_nodes[3]->GetSlope3().eigen();
}

// State update.

void ChElementShellANCF_3443B::Update() {
    ChElementGeneric::Update();
}

// Return the mass matrix in full sparse form.

void ChElementShellANCF_3443B::ComputeMmatrixGlobal(ChMatrixRef M) {
    M.setZero();

    // Mass Matrix is Stored in Compact Upper Triangular Form
    // Expand it out into its Full Sparse Symmetric Form
    unsigned int idx = 0;
    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = i; j < NSF; j++) {
            M(3 * i, 3 * j) = m_MassMatrix(idx);
            M(3 * i + 1, 3 * j + 1) = m_MassMatrix(idx);
            M(3 * i + 2, 3 * j + 2) = m_MassMatrix(idx);
            if (i != j) {
                M(3 * j, 3 * i) = m_MassMatrix(idx);
                M(3 * j + 1, 3 * i + 1) = m_MassMatrix(idx);
                M(3 * j + 2, 3 * i + 2) = m_MassMatrix(idx);
            }
            idx++;
        }
    }
}

// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass

void ChElementShellANCF_3443B::ComputeNodalMass() {
    m_nodes[0]->m_TotalMass += m_MassMatrix(0) + m_MassMatrix(4) + m_MassMatrix(8) + m_MassMatrix(12);
    m_nodes[1]->m_TotalMass += m_MassMatrix(4) + m_MassMatrix(58) + m_MassMatrix(62) + m_MassMatrix(66);
    m_nodes[2]->m_TotalMass += m_MassMatrix(8) + m_MassMatrix(62) + m_MassMatrix(100) + m_MassMatrix(104);
    m_nodes[3]->m_TotalMass += m_MassMatrix(12) + m_MassMatrix(66) + m_MassMatrix(104) + m_MassMatrix(126);
}

// Compute the generalized internal force vector for the current nodal coordinates and set the value in the Fi vector.

void ChElementShellANCF_3443B::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert((unsigned int)Fi.size() == GetNumCoordsPosLevel());

    // Retrieve the nodal coordinates and nodal coordinate time derivatives
    Matrix6xN ebar_ebardot;
    CalcCombinedCoordMatrix(ebar_ebardot);

    // Zero out the QiCompact since the contribution from each GQ point will be added to it
    MatrixNx3 QiCompact;
    QiCompact.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        // =============================================================================
        // Calculate the deformation gradient and time derivative of the deformation gradient for all Gauss quadrature
        // points in a single matrix multiplication.  Note that since the shape function derivative matrix is ordered by
        // columns, the resulting deformation gradient will be ordered by block matrix (column vectors) components
        //      [F11     F12     F13    ]
        //      [F21     F22     F23    ]
        // FC = [F31     F32     F33    ]
        //      [Fdot11  Fdot12  Fdot13 ]
        //      [Fdot21  Fdot22  Fdot23 ]
        //      [Fdot31  Fdot32  Fdot33 ]
        // =============================================================================
        ChMatrixNM<double, 6, 3 * NIP> FC = ebar_ebardot * m_SD.block<NSF, 3 * NIP>(kl * NSF, 0);

        Eigen::Map<ArrayNIP> F11(FC.block<1, NIP>(0, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F12(FC.block<1, NIP>(0, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F13(FC.block<1, NIP>(0, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> F21(FC.block<1, NIP>(1, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F22(FC.block<1, NIP>(1, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F23(FC.block<1, NIP>(1, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> F31(FC.block<1, NIP>(2, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F32(FC.block<1, NIP>(2, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F33(FC.block<1, NIP>(2, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> Fdot11(FC.block<1, NIP>(3, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot12(FC.block<1, NIP>(3, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot13(FC.block<1, NIP>(3, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> Fdot21(FC.block<1, NIP>(4, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot22(FC.block<1, NIP>(4, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot23(FC.block<1, NIP>(4, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> Fdot31(FC.block<1, NIP>(5, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot32(FC.block<1, NIP>(5, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot33(FC.block<1, NIP>(5, 2 * NIP).data(), 1, NIP);

        // =============================================================================
        // Calculate the 2nd Piola-Kirchhoff Stresses in Voigt notation across all of the GQ Points at the same time
        // =============================================================================

        ChMatrix66d D = m_layers[kl].GetMaterial()->Get_E_eps();
        RotateReorderStiffnessMatrix(D, m_layers[kl].Get_theta());

        ArrayNIP epsilon_1 =
            m_kGQ.block<1, NIP>(0, kl * NIP) * ((0.5 * (F11 * F11 + F21 * F21 + F31 * F31) - 0.5) +
                                                m_Alpha * (F11 * Fdot11 + F21 * Fdot21 + F31 * Fdot31));
        ArrayNIP epsilon_2 =
            m_kGQ.block<1, NIP>(0, kl * NIP) * ((0.5 * (F12 * F12 + F22 * F22 + F32 * F32) - 0.5) +
                                                m_Alpha * (F12 * Fdot12 + F22 * Fdot22 + F32 * Fdot32));
        ArrayNIP epsilon_3 =
            m_kGQ.block<1, NIP>(0, kl * NIP) * ((0.5 * (F13 * F13 + F23 * F23 + F33 * F33) - 0.5) +
                                                m_Alpha * (F13 * Fdot13 + F23 * Fdot23 + F33 * Fdot33));
        ArrayNIP epsilon_4 =
            m_kGQ.block<1, NIP>(0, kl * NIP) *
            ((F12 * F13 + F22 * F23 + F32 * F33) +
             m_Alpha * (F12 * Fdot13 + F22 * Fdot23 + F32 * Fdot33 + Fdot12 * F13 + Fdot22 * F23 + Fdot32 * F33));
        ArrayNIP epsilon_5 =
            m_kGQ.block<1, NIP>(0, kl * NIP) *
            ((F11 * F13 + F21 * F23 + F31 * F33) +
             m_Alpha * (F11 * Fdot13 + F21 * Fdot23 + F31 * Fdot33 + Fdot11 * F13 + Fdot21 * F23 + Fdot31 * F33));
        ArrayNIP epsilon_6 =
            m_kGQ.block<1, NIP>(0, kl * NIP) *
            ((F11 * F12 + F21 * F22 + F31 * F32) +
             m_Alpha * (F11 * Fdot12 + F21 * Fdot22 + F31 * Fdot32 + Fdot11 * F12 + Fdot21 * F22 + Fdot31 * F32));

        ArrayNIP SPK2_1 = D(0, 0) * epsilon_1 + D(0, 1) * epsilon_2 + D(0, 2) * epsilon_3 + D(0, 3) * epsilon_4 +
                          D(0, 4) * epsilon_5 + D(0, 5) * epsilon_6;
        ArrayNIP SPK2_2 = D(1, 0) * epsilon_1 + D(1, 1) * epsilon_2 + D(1, 2) * epsilon_3 + D(1, 3) * epsilon_4 +
                          D(1, 4) * epsilon_5 + D(1, 5) * epsilon_6;
        ArrayNIP SPK2_3 = D(2, 0) * epsilon_1 + D(2, 1) * epsilon_2 + D(2, 2) * epsilon_3 + D(2, 3) * epsilon_4 +
                          D(2, 4) * epsilon_5 + D(2, 5) * epsilon_6;
        ArrayNIP SPK2_4 = D(3, 0) * epsilon_1 + D(3, 1) * epsilon_2 + D(3, 2) * epsilon_3 + D(3, 3) * epsilon_4 +
                          D(3, 4) * epsilon_5 + D(3, 5) * epsilon_6;
        ArrayNIP SPK2_5 = D(4, 0) * epsilon_1 + D(4, 1) * epsilon_2 + D(4, 2) * epsilon_3 + D(4, 3) * epsilon_4 +
                          D(4, 4) * epsilon_5 + D(4, 5) * epsilon_6;
        ArrayNIP SPK2_6 = D(5, 0) * epsilon_1 + D(5, 1) * epsilon_2 + D(5, 2) * epsilon_3 + D(5, 3) * epsilon_4 +
                          D(5, 4) * epsilon_5 + D(5, 5) * epsilon_6;

        // =============================================================================
        // Calculate the transpose of the 1st Piola-Kirchoff stresses in block tensor form whose entries have been
        // scaled by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point.
        // The entries are grouped by component in block matrices (column vectors)
        // P_Block = kGQ*P_transpose = kGQ*SPK2*F_transpose
        //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ]
        //         = [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ]
        //           [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ]
        // =============================================================================

        ChMatrixNM_col<double, 3 * NIP, 3> P_Block;

        P_Block.block<NIP, 1>(0 * NIP, 0).array().transpose() = F11 * SPK2_1 + F12 * SPK2_6 + F13 * SPK2_5;  // PT11
        P_Block.block<NIP, 1>(1 * NIP, 0).array().transpose() = F11 * SPK2_6 + F12 * SPK2_2 + F13 * SPK2_4;  // PT21
        P_Block.block<NIP, 1>(2 * NIP, 0).array().transpose() = F11 * SPK2_5 + F12 * SPK2_4 + F13 * SPK2_3;  // PT31

        P_Block.block<NIP, 1>(0 * NIP, 1).array().transpose() = F21 * SPK2_1 + F22 * SPK2_6 + F23 * SPK2_5;  // PT12
        P_Block.block<NIP, 1>(1 * NIP, 1).array().transpose() = F21 * SPK2_6 + F22 * SPK2_2 + F23 * SPK2_4;  // PT22
        P_Block.block<NIP, 1>(2 * NIP, 1).array().transpose() = F21 * SPK2_5 + F22 * SPK2_4 + F23 * SPK2_3;  // PT32

        P_Block.block<NIP, 1>(0 * NIP, 2).array().transpose() = F31 * SPK2_1 + F32 * SPK2_6 + F33 * SPK2_5;  // PT13
        P_Block.block<NIP, 1>(1 * NIP, 2).array().transpose() = F31 * SPK2_6 + F32 * SPK2_2 + F33 * SPK2_4;  // PT23
        P_Block.block<NIP, 1>(2 * NIP, 2).array().transpose() = F31 * SPK2_5 + F32 * SPK2_4 + F33 * SPK2_3;  // PT33

        // =============================================================================
        // Multiply the scaled first Piola-Kirchoff stresses by the shape function derivative matrix to get the
        // generalized force vector in matrix form (in the correct order if its calculated in row-major memory layout)
        // =============================================================================

        QiCompact.noalias() += m_SD.block<NSF, 3 * NIP>(kl * NSF, 0) * P_Block;
    }

    Eigen::Map<ChVectorN<double, 3 * NSF>> QiReshaped(QiCompact.data(), QiCompact.size());
    Fi.noalias() = QiReshaped;
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]

void ChElementShellANCF_3443B::ComputeKRMmatricesGlobal(ChMatrixRef H,
                                                            double Kfactor,
                                                            double Rfactor,
                                                            double Mfactor) {
    assert((H.rows() == 3 * NSF) && (H.cols() == 3 * NSF));

    // Retrieve the nodal coordinates and nodal coordinate time derivatives
    Matrix6xN ebar_ebardot;
    CalcCombinedCoordMatrix(ebar_ebardot);

    // Zero out the Jacobian matrix since the contribution from each GQ point will be added to it
    Matrix3Nx3N Jac;
    Jac.setZero();

    ChVectorN<double, (NSF * (NSF + 1)) / 2> Jac_CompactPart = Mfactor * m_MassMatrix;

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        // =============================================================================
        // Calculate the deformation gradient and time derivative of the deformation gradient for all Gauss quadrature
        // points in a single matrix multiplication.  Note that since the shape function derivative matrix is ordered by
        // columns, the resulting deformation gradient will be ordered by block matrix (column vectors) components
        //      [F11     F12     F13    ]
        //      [F21     F22     F23    ]
        // FC = [F31     F32     F33    ]
        //      [Fdot11  Fdot12  Fdot13 ]
        //      [Fdot21  Fdot22  Fdot23 ]
        //      [Fdot31  Fdot32  Fdot33 ]
        // =============================================================================
        ChMatrixNM<double, 6, 3 * NIP> FC = ebar_ebardot * m_SD.block<NSF, 3 * NIP>(kl * NSF, 0);

        Eigen::Map<ArrayNIP> F11(FC.block<1, NIP>(0, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F12(FC.block<1, NIP>(0, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F13(FC.block<1, NIP>(0, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> F21(FC.block<1, NIP>(1, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F22(FC.block<1, NIP>(1, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F23(FC.block<1, NIP>(1, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> F31(FC.block<1, NIP>(2, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F32(FC.block<1, NIP>(2, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> F33(FC.block<1, NIP>(2, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> Fdot11(FC.block<1, NIP>(3, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot12(FC.block<1, NIP>(3, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot13(FC.block<1, NIP>(3, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> Fdot21(FC.block<1, NIP>(4, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot22(FC.block<1, NIP>(4, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot23(FC.block<1, NIP>(4, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> Fdot31(FC.block<1, NIP>(5, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot32(FC.block<1, NIP>(5, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> Fdot33(FC.block<1, NIP>(5, 2 * NIP).data(), 1, NIP);

        ChMatrixNM<double, 3, 3 * NIP> FCS = (-Kfactor - m_Alpha * Rfactor) * FC.block<3, 3 * NIP>(0, 0) -
                                             (m_Alpha * Kfactor) * FC.block<3, 3 * NIP>(3, 0);
        for (auto i = 0; i < 3; i++) {
            FCS.block<1, NIP>(i, 0 * NIP).array() *= m_kGQ.block<1, NIP>(0, kl * NIP);
            FCS.block<1, NIP>(i, 1 * NIP).array() *= m_kGQ.block<1, NIP>(0, kl * NIP);
            FCS.block<1, NIP>(i, 2 * NIP).array() *= m_kGQ.block<1, NIP>(0, kl * NIP);
        }

        Eigen::Map<ArrayNIP> FS11(FCS.block<1, NIP>(0, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> FS12(FCS.block<1, NIP>(0, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> FS13(FCS.block<1, NIP>(0, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> FS21(FCS.block<1, NIP>(1, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> FS22(FCS.block<1, NIP>(1, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> FS23(FCS.block<1, NIP>(1, 2 * NIP).data(), 1, NIP);

        Eigen::Map<ArrayNIP> FS31(FCS.block<1, NIP>(2, 0 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> FS32(FCS.block<1, NIP>(2, 1 * NIP).data(), 1, NIP);
        Eigen::Map<ArrayNIP> FS33(FCS.block<1, NIP>(2, 2 * NIP).data(), 1, NIP);

        // =============================================================================
        // Get the 6x6 stiffness tensor for the current layer
        // =============================================================================

        ChMatrix66d D = m_layers[kl].GetMaterial()->Get_E_eps();
        RotateReorderStiffnessMatrix(D, m_layers[kl].Get_theta());

        // Calculate the depsilon'/de * D * depsilon/de terms in chunks rather than as one very large matrix
        // multiplication (outer product)
        ChMatrixNM<double, 3 * NSF, NIP> PE;
        ChMatrixNM<double, 3 * NSF, NIP> DPE;

        // Calculate the Contribution from the depsilon1/de terms
        {
            ArrayNIP S1A = D(0, 0) * FS11 + D(0, 5) * FS12 + D(0, 4) * FS13;
            ArrayNIP S2A = D(0, 5) * FS11 + D(0, 1) * FS12 + D(0, 3) * FS13;
            ArrayNIP S3A = D(0, 4) * FS11 + D(0, 3) * FS12 + D(0, 2) * FS13;

            ArrayNIP S1B = D(0, 0) * FS21 + D(0, 5) * FS22 + D(0, 4) * FS23;
            ArrayNIP S2B = D(0, 5) * FS21 + D(0, 1) * FS22 + D(0, 3) * FS23;
            ArrayNIP S3B = D(0, 4) * FS21 + D(0, 3) * FS22 + D(0, 2) * FS23;

            ArrayNIP S1C = D(0, 0) * FS31 + D(0, 5) * FS32 + D(0, 4) * FS33;
            ArrayNIP S2C = D(0, 5) * FS31 + D(0, 1) * FS32 + D(0, 3) * FS33;
            ArrayNIP S3C = D(0, 4) * FS31 + D(0, 3) * FS32 + D(0, 2) * FS33;

            for (auto i = 0; i < NSF; i++) {
                PE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F11;
                PE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F21;
                PE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F31;
                DPE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3A;
                DPE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3B;
                DPE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3C;
            }
            Jac.noalias() += PE * DPE.transpose();
        }
        // Calculate the Contribution from the depsilon2/de terms
        {
            ArrayNIP S1A = D(1, 0) * FS11 + D(1, 5) * FS12 + D(1, 4) * FS13;
            ArrayNIP S2A = D(1, 5) * FS11 + D(1, 1) * FS12 + D(1, 3) * FS13;
            ArrayNIP S3A = D(1, 4) * FS11 + D(1, 3) * FS12 + D(1, 2) * FS13;

            ArrayNIP S1B = D(1, 0) * FS21 + D(1, 5) * FS22 + D(1, 4) * FS23;
            ArrayNIP S2B = D(1, 5) * FS21 + D(1, 1) * FS22 + D(1, 3) * FS23;
            ArrayNIP S3B = D(1, 4) * FS21 + D(1, 3) * FS22 + D(1, 2) * FS23;

            ArrayNIP S1C = D(1, 0) * FS31 + D(1, 5) * FS32 + D(1, 4) * FS33;
            ArrayNIP S2C = D(1, 5) * FS31 + D(1, 1) * FS32 + D(1, 3) * FS33;
            ArrayNIP S3C = D(1, 4) * FS31 + D(1, 3) * FS32 + D(1, 2) * FS33;

            for (auto i = 0; i < NSF; i++) {
                PE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F12;
                PE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F22;
                PE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F32;
                DPE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3A;
                DPE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3B;
                DPE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3C;
            }
            Jac.noalias() += PE * DPE.transpose();
        }
        // Calculate the Contribution from the depsilon3/de terms
        {
            ArrayNIP S1A = D(2, 0) * FS11 + D(2, 5) * FS12 + D(2, 4) * FS13;
            ArrayNIP S2A = D(2, 5) * FS11 + D(2, 1) * FS12 + D(2, 3) * FS13;
            ArrayNIP S3A = D(2, 4) * FS11 + D(2, 3) * FS12 + D(2, 2) * FS13;

            ArrayNIP S1B = D(2, 0) * FS21 + D(2, 5) * FS22 + D(2, 4) * FS23;
            ArrayNIP S2B = D(2, 5) * FS21 + D(2, 1) * FS22 + D(2, 3) * FS23;
            ArrayNIP S3B = D(2, 4) * FS21 + D(2, 3) * FS22 + D(2, 2) * FS23;

            ArrayNIP S1C = D(2, 0) * FS31 + D(2, 5) * FS32 + D(2, 4) * FS33;
            ArrayNIP S2C = D(2, 5) * FS31 + D(2, 1) * FS32 + D(2, 3) * FS33;
            ArrayNIP S3C = D(2, 4) * FS31 + D(2, 3) * FS32 + D(2, 2) * FS33;

            for (auto i = 0; i < NSF; i++) {
                PE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F13;
                PE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F23;
                PE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F33;
                DPE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3A;
                DPE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3B;
                DPE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3C;
            }
            Jac.noalias() += PE * DPE.transpose();
        }
        // Calculate the Contribution from the depsilon4/de terms
        {
            ArrayNIP S1A = D(3, 0) * FS11 + D(3, 5) * FS12 + D(3, 4) * FS13;
            ArrayNIP S2A = D(3, 5) * FS11 + D(3, 1) * FS12 + D(3, 3) * FS13;
            ArrayNIP S3A = D(3, 4) * FS11 + D(3, 3) * FS12 + D(3, 2) * FS13;

            ArrayNIP S1B = D(3, 0) * FS21 + D(3, 5) * FS22 + D(3, 4) * FS23;
            ArrayNIP S2B = D(3, 5) * FS21 + D(3, 1) * FS22 + D(3, 3) * FS23;
            ArrayNIP S3B = D(3, 4) * FS21 + D(3, 3) * FS22 + D(3, 2) * FS23;

            ArrayNIP S1C = D(3, 0) * FS31 + D(3, 5) * FS32 + D(3, 4) * FS33;
            ArrayNIP S2C = D(3, 5) * FS31 + D(3, 1) * FS32 + D(3, 3) * FS33;
            ArrayNIP S3C = D(3, 4) * FS31 + D(3, 3) * FS32 + D(3, 2) * FS33;

            for (auto i = 0; i < NSF; i++) {
                PE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F13 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F12;
                PE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F23 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F22;
                PE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F33 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F32;
                DPE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3A;
                DPE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3B;
                DPE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3C;
            }
            Jac.noalias() += PE * DPE.transpose();
        }
        // Calculate the Contribution from the depsilon5/de terms
        {
            ArrayNIP S1A = D(4, 0) * FS11 + D(4, 5) * FS12 + D(4, 4) * FS13;
            ArrayNIP S2A = D(4, 5) * FS11 + D(4, 1) * FS12 + D(4, 3) * FS13;
            ArrayNIP S3A = D(4, 4) * FS11 + D(4, 3) * FS12 + D(4, 2) * FS13;

            ArrayNIP S1B = D(4, 0) * FS21 + D(4, 5) * FS22 + D(4, 4) * FS23;
            ArrayNIP S2B = D(4, 5) * FS21 + D(4, 1) * FS22 + D(4, 3) * FS23;
            ArrayNIP S3B = D(4, 4) * FS21 + D(4, 3) * FS22 + D(4, 2) * FS23;

            ArrayNIP S1C = D(4, 0) * FS31 + D(4, 5) * FS32 + D(4, 4) * FS33;
            ArrayNIP S2C = D(4, 5) * FS31 + D(4, 1) * FS32 + D(4, 3) * FS33;
            ArrayNIP S3C = D(4, 4) * FS31 + D(4, 3) * FS32 + D(4, 2) * FS33;

            for (auto i = 0; i < NSF; i++) {
                PE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F13 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F11;
                PE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F23 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F21;
                PE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F33 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * F31;
                DPE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3A;
                DPE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3B;
                DPE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3C;
            }
            Jac.noalias() += PE * DPE.transpose();
        }
        // Calculate the Contribution from the depsilon6/de terms
        {
            ArrayNIP S1A = D(5, 0) * FS11 + D(5, 5) * FS12 + D(5, 4) * FS13;
            ArrayNIP S2A = D(5, 5) * FS11 + D(5, 1) * FS12 + D(5, 3) * FS13;
            ArrayNIP S3A = D(5, 4) * FS11 + D(5, 3) * FS12 + D(5, 2) * FS13;

            ArrayNIP S1B = D(5, 0) * FS21 + D(5, 5) * FS22 + D(5, 4) * FS23;
            ArrayNIP S2B = D(5, 5) * FS21 + D(5, 1) * FS22 + D(5, 3) * FS23;
            ArrayNIP S3B = D(5, 4) * FS21 + D(5, 3) * FS22 + D(5, 2) * FS23;

            ArrayNIP S1C = D(5, 0) * FS31 + D(5, 5) * FS32 + D(5, 4) * FS33;
            ArrayNIP S2C = D(5, 5) * FS31 + D(5, 1) * FS32 + D(5, 3) * FS33;
            ArrayNIP S3C = D(5, 4) * FS31 + D(5, 3) * FS32 + D(5, 2) * FS33;

            for (auto i = 0; i < NSF; i++) {
                PE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F12 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F11;
                PE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F22 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F21;
                PE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * F32 +
                                                         m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * F31;
                DPE.block<1, NIP>(3 * i + 0, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2A +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3A;
                DPE.block<1, NIP>(3 * i + 1, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2B +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3B;
                DPE.block<1, NIP>(3 * i + 2, 0).array() = m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() * S1C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() * S2C +
                                                          m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array() * S3C;
            }
            Jac.noalias() += PE * DPE.transpose();
        }

        ArrayNIP epsilon_1 =
            m_kGQ.block<1, NIP>(0, kl * NIP) * ((0.5 * (F11 * F11 + F21 * F21 + F31 * F31) - 0.5) +
                                                m_Alpha * (F11 * Fdot11 + F21 * Fdot21 + F31 * Fdot31));
        ArrayNIP epsilon_2 =
            m_kGQ.block<1, NIP>(0, kl * NIP) * ((0.5 * (F12 * F12 + F22 * F22 + F32 * F32) - 0.5) +
                                                m_Alpha * (F12 * Fdot12 + F22 * Fdot22 + F32 * Fdot32));
        ArrayNIP epsilon_3 =
            m_kGQ.block<1, NIP>(0, kl * NIP) * ((0.5 * (F13 * F13 + F23 * F23 + F33 * F33) - 0.5) +
                                                m_Alpha * (F13 * Fdot13 + F23 * Fdot23 + F33 * Fdot33));
        ArrayNIP epsilon_4 =
            m_kGQ.block<1, NIP>(0, kl * NIP) *
            ((F12 * F13 + F22 * F23 + F32 * F33) +
             m_Alpha * (F12 * Fdot13 + F22 * Fdot23 + F32 * Fdot33 + Fdot12 * F13 + Fdot22 * F23 + Fdot32 * F33));
        ArrayNIP epsilon_5 =
            m_kGQ.block<1, NIP>(0, kl * NIP) *
            ((F11 * F13 + F21 * F23 + F31 * F33) +
             m_Alpha * (F11 * Fdot13 + F21 * Fdot23 + F31 * Fdot33 + Fdot11 * F13 + Fdot21 * F23 + Fdot31 * F33));
        ArrayNIP epsilon_6 =
            m_kGQ.block<1, NIP>(0, kl * NIP) *
            ((F11 * F12 + F21 * F22 + F31 * F32) +
             m_Alpha * (F11 * Fdot12 + F21 * Fdot22 + F31 * Fdot32 + Fdot11 * F12 + Fdot21 * F22 + Fdot31 * F32));

        ArrayNIP SPK2_1 = D(0, 0) * epsilon_1 + D(0, 1) * epsilon_2 + D(0, 2) * epsilon_3 + D(0, 3) * epsilon_4 +
                          D(0, 4) * epsilon_5 + D(0, 5) * epsilon_6;
        ArrayNIP SPK2_2 = D(1, 0) * epsilon_1 + D(1, 1) * epsilon_2 + D(1, 2) * epsilon_3 + D(1, 3) * epsilon_4 +
                          D(1, 4) * epsilon_5 + D(1, 5) * epsilon_6;
        ArrayNIP SPK2_3 = D(2, 0) * epsilon_1 + D(2, 1) * epsilon_2 + D(2, 2) * epsilon_3 + D(2, 3) * epsilon_4 +
                          D(2, 4) * epsilon_5 + D(2, 5) * epsilon_6;
        ArrayNIP SPK2_4 = D(3, 0) * epsilon_1 + D(3, 1) * epsilon_2 + D(3, 2) * epsilon_3 + D(3, 3) * epsilon_4 +
                          D(3, 4) * epsilon_5 + D(3, 5) * epsilon_6;
        ArrayNIP SPK2_5 = D(4, 0) * epsilon_1 + D(4, 1) * epsilon_2 + D(4, 2) * epsilon_3 + D(4, 3) * epsilon_4 +
                          D(4, 4) * epsilon_5 + D(4, 5) * epsilon_6;
        ArrayNIP SPK2_6 = D(5, 0) * epsilon_1 + D(5, 1) * epsilon_2 + D(5, 2) * epsilon_3 + D(5, 3) * epsilon_4 +
                          D(5, 4) * epsilon_5 + D(5, 5) * epsilon_6;

        // Calculate the contribution from the Mass Matrix and expand(SD*SPK2*SD')
        // Since both a symmetric and in compact form, only the upper triangular entries need to be calculated and then
        // added to the correct corresponding locations in the Jacobian matrix
        unsigned int idx = 0;
        for (unsigned int i = 0; i < NSF; i++) {
            ChVectorN<double, 3 * NIP> S_scaled_SD_row_i;
            S_scaled_SD_row_i.segment(0 * NIP, NIP).array() =
                SPK2_1 * m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() +
                SPK2_6 * m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() +
                SPK2_5 * m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array();
            S_scaled_SD_row_i.segment(1 * NIP, NIP).array() =
                SPK2_6 * m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() +
                SPK2_2 * m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() +
                SPK2_4 * m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array();
            S_scaled_SD_row_i.segment(2 * NIP, NIP).array() =
                SPK2_5 * m_SD.block<1, NIP>(i + kl * NSF, 0 * NIP).array() +
                SPK2_4 * m_SD.block<1, NIP>(i + kl * NSF, 1 * NIP).array() +
                SPK2_3 * m_SD.block<1, NIP>(i + kl * NSF, 2 * NIP).array();

            for (unsigned int j = i; j < NSF; j++) {
                Jac_CompactPart(idx) -= Kfactor * (S_scaled_SD_row_i.dot(m_SD.row(j + kl * NSF)));
                idx++;
            }
        }
    }

    unsigned int idx = 0;
    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = i; j < NSF; j++) {
            Jac(3 * i, 3 * j) += Jac_CompactPart(idx);
            Jac(3 * i + 1, 3 * j + 1) += Jac_CompactPart(idx);
            Jac(3 * i + 2, 3 * j + 2) += Jac_CompactPart(idx);
            if (i != j) {
                Jac(3 * j, 3 * i) += Jac_CompactPart(idx);
                Jac(3 * j + 1, 3 * i + 1) += Jac_CompactPart(idx);
                Jac(3 * j + 2, 3 * i + 2) += Jac_CompactPart(idx);
            }
            idx++;
        }
    }

    H.noalias() = Jac;
}

// Compute the generalized force vector due to gravity using the efficient ANCF specific method
void ChElementShellANCF_3443B::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector3d& G_acc) {
    assert((unsigned int)Fg.size() == GetNumCoordsPosLevel());

    // Calculate and add the generalized force due to gravity to the generalized internal force vector for the element.
    // The generalized force due to gravity could be computed once prior to the start of the simulation if gravity was
    // assumed constant throughout the entire simulation.  However, this implementation assumes that the acceleration
    // due to gravity, while a constant for the entire system, can change from step to step which could be useful for
    // gravity loaded units tests as an example.  The generalized force due to gravity is calculated in compact matrix
    // form and is pre-mapped to the desired vector format
    Eigen::Map<MatrixNx3> GravForceCompact(Fg.data(), NSF, 3);
    GravForceCompact = m_GravForceScale * G_acc.eigen().transpose();
}

// -----------------------------------------------------------------------------
// Interface to ChElementShell base class
// -----------------------------------------------------------------------------

void ChElementShellANCF_3443B::EvaluateSectionFrame(const double xi,
                                                        const double eta,
                                                        ChVector3d& point,
                                                        ChQuaternion<>& rot) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, 0, m_thicknessZ, m_midsurfoffset);
    VectorN Sxi_xi_compact;
    Calc_Sxi_xi_compact(Sxi_xi_compact, xi, eta, 0, m_thicknessZ, m_midsurfoffset);
    VectorN Sxi_eta_compact;
    Calc_Sxi_eta_compact(Sxi_eta_compact, xi, eta, 0, m_thicknessZ, m_midsurfoffset);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // r = S*e written in compact form
    point = e_bar * Sxi_compact;

    // Since ANCF does not use rotations, calculate an approximate
    // rotation based off the position vector gradients
    ChVector3d MidsurfaceX = e_bar * Sxi_xi_compact * 2 / m_lenX;
    ChVector3d MidsurfaceY = e_bar * Sxi_eta_compact * 2 / m_lenY;

    // Since the position vector gradients are not in general orthogonal,
    // set the Dx direction tangent to the shell xi axis and
    // compute the Dy and Dz directions by using a
    // Gram-Schmidt orthonormalization, guided by the shell eta axis
    ChMatrix33d msect;
    msect.SetFromAxisX(MidsurfaceX, MidsurfaceY);

    rot = msect.GetQuaternion();
}

void ChElementShellANCF_3443B::EvaluateSectionPoint(const double xi, const double eta, ChVector3d& point) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, 0, m_thicknessZ, m_midsurfoffset);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // r = S*e written in compact form
    point = e_bar * Sxi_compact;
}

void ChElementShellANCF_3443B::EvaluateSectionVelNorm(const double xi, const double eta, ChVector3d& Result) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, 0, m_thicknessZ, m_midsurfoffset);

    Matrix3xN e_bardot;
    CalcCoordDtMatrix(e_bardot);

    // rdot = S*edot written in compact form
    Result = e_bardot * Sxi_compact;
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).

void ChElementShellANCF_3443B::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetSlope1().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetSlope2().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[0]->GetSlope3().eigen();

    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetSlope1().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[1]->GetSlope2().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[1]->GetSlope3().eigen();

    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 27, 3) = m_nodes[2]->GetSlope1().eigen();
    mD.segment(block_offset + 30, 3) = m_nodes[2]->GetSlope2().eigen();
    mD.segment(block_offset + 33, 3) = m_nodes[2]->GetSlope3().eigen();

    mD.segment(block_offset + 36, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 39, 3) = m_nodes[3]->GetSlope1().eigen();
    mD.segment(block_offset + 42, 3) = m_nodes[3]->GetSlope2().eigen();
    mD.segment(block_offset + 45, 3) = m_nodes[3]->GetSlope3().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).

void ChElementShellANCF_3443B::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPosDt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetSlope1Dt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetSlope2Dt().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[0]->GetSlope3Dt().eigen();

    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetPosDt().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetSlope1Dt().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[1]->GetSlope2Dt().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[1]->GetSlope3Dt().eigen();

    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetPosDt().eigen();
    mD.segment(block_offset + 27, 3) = m_nodes[2]->GetSlope1Dt().eigen();
    mD.segment(block_offset + 30, 3) = m_nodes[2]->GetSlope2Dt().eigen();
    mD.segment(block_offset + 33, 3) = m_nodes[2]->GetSlope3Dt().eigen();

    mD.segment(block_offset + 36, 3) = m_nodes[3]->GetPosDt().eigen();
    mD.segment(block_offset + 39, 3) = m_nodes[3]->GetSlope1Dt().eigen();
    mD.segment(block_offset + 42, 3) = m_nodes[3]->GetSlope2Dt().eigen();
    mD.segment(block_offset + 45, 3) = m_nodes[3]->GetSlope3Dt().eigen();
}

/// Increment all DOFs using a delta.

void ChElementShellANCF_3443B::LoadableStateIncrement(const unsigned int off_x,
                                                          ChState& x_new,
                                                          const ChState& x,
                                                          const unsigned int off_v,
                                                          const ChStateDelta& Dv) {
    m_nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    m_nodes[1]->NodeIntStateIncrement(off_x + 12, x_new, x, off_v + 12, Dv);
    m_nodes[2]->NodeIntStateIncrement(off_x + 24, x_new, x, off_v + 24, Dv);
    m_nodes[3]->NodeIntStateIncrement(off_x + 36, x_new, x, off_v + 36, Dv);
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.

void ChElementShellANCF_3443B::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
        mvars.push_back(&m_nodes[i]->VariablesSlope1());
        mvars.push_back(&m_nodes[i]->VariablesSlope2());
        mvars.push_back(&m_nodes[i]->VariablesSlope3());
    }
}

// Evaluate N'*F, which is the projection of the applied point force and moment at the midsurface coordinates (xi,eta,0)
// This calculation takes a slightly different form for ANCF elements
// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
// the applied force in global coordinates and the second 3 entries is the applied moment in global space.

void ChElementShellANCF_3443B::ComputeNF(
    const double xi,             // parametric coordinate in surface
    const double eta,            // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    // Compute the generalized force vector for the applied force component using the compact form of the shape
    // functions.  This requires a reshaping of the calculated matrix to get it into the correct vector order (just a
    // reinterpretation of the data since the matrix is in row-major format)
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, 0, m_thicknessZ, m_midsurfoffset);
    MatrixNx3 QiCompact;

    QiCompact = Sxi_compact * F.segment(0, 3).transpose();

    Eigen::Map<Vector3N> QiReshaped(QiCompact.data(), QiCompact.size());
    Qi = QiReshaped;

    // Compute the generalized force vector for the applied moment component
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    MatrixNx3c Sxi_D;
    Calc_Sxi_D(Sxi_D, xi, eta, 0, m_thicknessZ, m_midsurfoffset);

    ChMatrix33d J_Cxi;
    ChMatrix33d J_Cxi_Inv;

    J_Cxi.noalias() = e_bar * Sxi_D;
    J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    VectorN G_A = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 0) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 0) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 0);
    VectorN G_B = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 1) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 1) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 1);
    VectorN G_C = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 2) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 2) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 2);

    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    for (unsigned int i = 0; i < NSF; i++) {
        Qi(3 * i) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi((3 * i) + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi((3 * i) + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }

    // Compute the element Jacobian between the current configuration and the normalized configuration
    // This is different than the element Jacobian between the reference configuration and the normalized
    //  configuration used in the internal force calculations.  For this calculation, this is the ratio between the
    //  actual differential area and the normalized differential area.  The cross product is
    //  used to calculate this area ratio for potential use in Gauss-Quadrature or similar numeric integration.
    detJ = J_Cxi.col(0).cross(J_Cxi.col(1)).norm();
}

// Evaluate N'*F, which is the projection of the applied point force and moment at the coordinates (xi,eta,zeta)
// This calculation takes a slightly different form for ANCF elements
// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
// the applied force in global coordinates and the second 3 entries is the applied moment in global space.

void ChElementShellANCF_3443B::ComputeNF(
    const double xi,             // parametric coordinate in volume
    const double eta,            // parametric coordinate in volume
    const double zeta,           // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    // Compute the generalized force vector for the applied force component using the compact form of the shape
    // functions.  This requires a reshaping of the calculated matrix to get it into the correct vector order (just a
    // reinterpretation of the data since the matrix is in row-major format)
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, zeta, m_thicknessZ, m_midsurfoffset);
    MatrixNx3 QiCompact;

    QiCompact = Sxi_compact * F.segment(0, 3).transpose();

    Eigen::Map<Vector3N> QiReshaped(QiCompact.data(), QiCompact.size());
    Qi = QiReshaped;

    // Compute the generalized force vector for the applied moment component
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    MatrixNx3c Sxi_D;
    Calc_Sxi_D(Sxi_D, xi, eta, zeta, m_thicknessZ, m_midsurfoffset);

    ChMatrix33d J_Cxi;
    ChMatrix33d J_Cxi_Inv;

    J_Cxi.noalias() = e_bar * Sxi_D;
    J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    VectorN G_A = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 0) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 0) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 0);
    VectorN G_B = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 1) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 1) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 1);
    VectorN G_C = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 2) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 2) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 2);

    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    for (unsigned int i = 0; i < NSF; i++) {
        Qi(3 * i) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi((3 * i) + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi((3 * i) + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }

    // Compute the element Jacobian between the current configuration and the normalized configuration
    // This is different than the element Jacobian between the reference configuration and the normalized
    //  configuration used in the internal force calculations.  For this calculation, this is the ratio between the
    //  actual differential volume and the normalized differential volume.  The determinate of the element Jacobian is
    //  used to calculate this volume ratio for potential use in Gauss-Quadrature or similar numeric integration.
    detJ = J_Cxi.determinant();
}

// Calculate the average element density (needed for ChLoaderVolumeGravity).

double ChElementShellANCF_3443B::GetDensity() {
    double tot_density = 0;
    for (int kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->GetDensity();
        double layerthick = m_layers[kl].Get_thickness();
        tot_density += rho * layerthick;
    }
    return tot_density / m_thicknessZ;
}

// Calculate normal to the midsurface at coordinates (xi, eta).

ChVector3d ChElementShellANCF_3443B::ComputeNormal(const double xi, const double eta) {
    VectorN Sxi_zeta_compact;
    Calc_Sxi_zeta_compact(Sxi_zeta_compact, xi, eta, 0, m_thicknessZ, m_midsurfoffset);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // Calculate the position vector gradient with respect to zeta at the current point (whose length may not equal 1)
    ChVector3d r_zeta = e_bar * Sxi_zeta_compact;

    return r_zeta.GetNormalized();
}

// -----------------------------------------------------------------------------
// Transform from global nodal coordinates to local nodal coordinates calculation
// -----------------------------------------------------------------------------

void ChElementShellANCF_3443B::ComputeNodalCoordTransform() {
    m_Tij.setIdentity();

    MatrixNx3c Sxi_D;   // Matrix of normalized shape function derivatives

    // NodeA
    double xi = -1.0;
    double eta = -1.0;
    Calc_Sxi_D(Sxi_D, xi, eta, 0, m_thicknessZ, m_midsurfoffset);
    Sxi_D.col(0) = Sxi_D.col(0) * 2.0 / m_lenX;
    Sxi_D.col(1) = Sxi_D.col(1) * 2.0 / m_lenY;
    Sxi_D.col(2) = Sxi_D.col(2) * 2.0 / m_thicknessZ;
    ChMatrix33d TijNodeA = m_ebar0 * Sxi_D;
    
    // NodeB
    xi = 1.0;
    eta = -1.0;
    Calc_Sxi_D(Sxi_D, xi, eta, 0, m_thicknessZ, m_midsurfoffset);
    Sxi_D.col(0) = Sxi_D.col(0) * 2.0 / m_lenX;
    Sxi_D.col(1) = Sxi_D.col(1) * 2.0 / m_lenY;
    Sxi_D.col(2) = Sxi_D.col(2) * 2.0 / m_thicknessZ;
    ChMatrix33d TijNodeB = m_ebar0 * Sxi_D;

    // NodeC
    xi = 1.0;
    eta = 1.0;
    Calc_Sxi_D(Sxi_D, xi, eta, 0, m_thicknessZ, m_midsurfoffset);
    Sxi_D.col(0) = Sxi_D.col(0) * 2.0 / m_lenX;
    Sxi_D.col(1) = Sxi_D.col(1) * 2.0 / m_lenY;
    Sxi_D.col(2) = Sxi_D.col(2) * 2.0 / m_thicknessZ;
    ChMatrix33d TijNodeC = m_ebar0 * Sxi_D;

    // NodeD
    xi = -1.0;
    eta = 1.0;
    Calc_Sxi_D(Sxi_D, xi, eta, 0, m_thicknessZ, m_midsurfoffset);
    Sxi_D.col(0) = Sxi_D.col(0) * 2.0 / m_lenX;
    Sxi_D.col(1) = Sxi_D.col(1) * 2.0 / m_lenY;
    Sxi_D.col(2) = Sxi_D.col(2) * 2.0 / m_thicknessZ;
    ChMatrix33d TijNodeD = m_ebar0 * Sxi_D;

    // Update the transform only at the end since this transform is used within the functions that calculate the shape functions
    // It is originally set to identity so that the transform has no effect during this calculation phase
    m_Tij.block<3, 3>(1 + 0 * 4, 1 + 0 * 4).noalias() = TijNodeA;
    m_Tij.block<3, 3>(1 + 1 * 4, 1 + 1 * 4).noalias() = TijNodeB;
    m_Tij.block<3, 3>(1 + 2 * 4, 1 + 2 * 4).noalias() = TijNodeC;
    m_Tij.block<3, 3>(1 + 3 * 4, 1 + 3 * 4).noalias() = TijNodeD;
}

// -----------------------------------------------------------------------------
// Mass Matrix & Generalized Force Due to Gravity Calculation
// -----------------------------------------------------------------------------

void ChElementShellANCF_3443B::ComputeMassMatrixAndGravityForce() {
    // For this element, the mass matrix integrand is of order 12 in xi, 12 in eta, and 4 in zeta.
    // 7 GQ Points are needed in the xi & eta directions and 3 GQ Points are needed in the zeta direction for
    // exact integration of the element's mass matrix, even if the reference configuration is not straight. Since the
    // major pieces of the generalized force due to gravity can also be used to calculate the mass matrix, these
    // calculations are performed at the same time.  Only the matrix that scales the acceleration due to gravity is
    // calculated at this time so that any changes to the acceleration due to gravity in the system are correctly
    // accounted for in the generalized internal force calculation.

    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi_eta = 6;  // 7 Point Gauss-Quadrature;
    unsigned int GQ_idx_zeta = 2;    // 3 Point Gauss-Quadrature;

    // Mass Matrix in its compact matrix form.  Since the mass matrix is symmetric, just the upper diagonal entries will
    // be stored.
    MatrixNxN MassMatrixCompactSquare;

    // Set these to zeros since they will be incremented as the vector/matrix is calculated
    MassMatrixCompactSquare.setZero();
    m_GravForceScale.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->GetDensity();  // Density of the material for the current layer
        double thickness = m_layers[kl].Get_thickness();
        double zoffset = m_layer_zoffsets[kl];
        double layer_midsurface_offset =
            -m_thicknessZ / 2 + m_layer_zoffsets[kl] + m_layers[kl].Get_thickness() / 2 + m_midsurfoffset;

        // Sum the contribution to the mass matrix and generalized force due to gravity at the current point
        for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi_eta].size(); it_xi++) {
            for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_xi_eta].size(); it_eta++) {
                for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_zeta].size(); it_zeta++) {
                    double GQ_weight = GQTable->Weight[GQ_idx_xi_eta][it_xi] * GQTable->Weight[GQ_idx_xi_eta][it_eta] *
                                       GQTable->Weight[GQ_idx_zeta][it_zeta];
                    double xi = GQTable->Lroots[GQ_idx_xi_eta][it_xi];
                    double eta = GQTable->Lroots[GQ_idx_xi_eta][it_eta];
                    double zeta = GQTable->Lroots[GQ_idx_zeta][it_zeta];
                    double det_J_0xi = Calc_det_J_0xi(xi, eta, zeta, thickness,
                                                      zoffset);  // determinant of the element Jacobian (volume ratio)

                    VectorN Sxi_compact;  // Vector of the Unique Normalized Shape Functions
                    Calc_Sxi_compact(Sxi_compact, xi, eta, zeta, thickness, layer_midsurface_offset);

                    m_GravForceScale += (GQ_weight * rho * det_J_0xi) * Sxi_compact;
                    MassMatrixCompactSquare += (GQ_weight * rho * det_J_0xi) * Sxi_compact * Sxi_compact.transpose();
                }
            }
        }
    }

    // Store just the unique entries in the Mass Matrix in Compact Upper Triangular Form
    // since the full Mass Matrix is both sparse and symmetric
    unsigned int idx = 0;
    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = i; j < NSF; j++) {
            m_MassMatrix(idx) = MassMatrixCompactSquare(i, j);
            idx++;
        }
    }
}

// Precalculate constant matrices and scalars for the internal force calculations
void ChElementShellANCF_3443B::PrecomputeInternalForceMatricesWeights() {
    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi_eta = NP - 1;  // Gauss-Quadrature table index for xi and eta
    unsigned int GQ_idx_zeta = NT - 1;    // Gauss-Quadrature table index for zeta

    m_SD.resize(m_numLayers * NSF, 3 * NIP);
    m_kGQ.resize(1, m_numLayers * NIP);

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double thickness = m_layers[kl].Get_thickness();
        double layer_midsurface_offset = -m_thicknessZ / 2 + m_layer_zoffsets[kl] + thickness / 2 + m_midsurfoffset;

        for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi_eta].size(); it_xi++) {
            for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_xi_eta].size(); it_eta++) {
                for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_zeta].size(); it_zeta++) {
                    double GQ_weight = GQTable->Weight[GQ_idx_xi_eta][it_xi] * GQTable->Weight[GQ_idx_xi_eta][it_eta] *
                                       GQTable->Weight[GQ_idx_zeta][it_zeta];
                    double xi = GQTable->Lroots[GQ_idx_xi_eta][it_xi];
                    double eta = GQTable->Lroots[GQ_idx_xi_eta][it_eta];
                    double zeta = GQTable->Lroots[GQ_idx_zeta][it_zeta];
                    auto index = it_zeta + it_eta * GQTable->Lroots[GQ_idx_zeta].size() +
                                 it_xi * GQTable->Lroots[GQ_idx_zeta].size() * GQTable->Lroots[GQ_idx_xi_eta].size();
                    ChMatrix33d
                        J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
                    MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives

                    Calc_Sxi_D(Sxi_D, xi, eta, zeta, thickness, layer_midsurface_offset);
                    J_0xi.noalias() = m_pbar0 * Sxi_D;

                    // Adjust the shape function derivative matrix to account for a potentially deformed reference state
                    ChMatrixNM<double, NSF, 3> SD_precompute_D = Sxi_D * J_0xi.inverse();
                    m_kGQ(index + NIP * kl) = -J_0xi.determinant() * GQ_weight;

                    // Group all of the columns together in blocks, and then layer by layer across the entire element
                    m_SD.block<NSF, 1>(kl * NSF, index + 0 * NIP) = SD_precompute_D.col(0);
                    m_SD.block<NSF, 1>(kl * NSF, index + 1 * NIP) = SD_precompute_D.col(1);
                    m_SD.block<NSF, 1>(kl * NSF, index + 2 * NIP) = SD_precompute_D.col(2);
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

// Nx1 Vector Form of the Normalized Shape Functions
// [s1; s2; s3; ...]

void ChElementShellANCF_3443B::Calc_Sxi_compact(VectorN& Sxi_compact,
                                                    double xi,
                                                    double eta,
                                                    double zeta,
                                                    double thickness,
                                                    double zoffset) {
    Sxi_compact(0) = -0.125 * (xi - 1) * (eta - 1) * (eta * eta + eta + xi * xi + xi - 2);
    Sxi_compact(1) = -0.0625 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (eta - 1);
    Sxi_compact(2) = -0.0625 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (xi - 1);
    Sxi_compact(3) = -0.125 * (xi - 1) * (eta - 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_compact(4) = 0.125 * (xi + 1) * (eta - 1) * (eta * eta + eta + xi * xi - xi - 2);
    Sxi_compact(5) = -0.0625 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (eta - 1);
    Sxi_compact(6) = 0.0625 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (xi + 1);
    Sxi_compact(7) = 0.125 * (xi + 1) * (eta - 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_compact(8) = -0.125 * (xi + 1) * (eta + 1) * (eta * eta - eta + xi * xi - xi - 2);
    Sxi_compact(9) = 0.0625 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (eta + 1);
    Sxi_compact(10) = 0.0625 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (xi + 1);
    Sxi_compact(11) = -0.125 * (xi + 1) * (eta + 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_compact(12) = 0.125 * (xi - 1) * (eta + 1) * (eta * eta - eta + xi * xi + xi - 2);
    Sxi_compact(13) = 0.0625 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (eta + 1);
    Sxi_compact(14) = -0.0625 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (xi - 1);
    Sxi_compact(15) = 0.125 * (xi - 1) * (eta + 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);
    Sxi_compact = m_Tij * Sxi_compact;
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to xi
// [s1; s2; s3; ...]

void ChElementShellANCF_3443B::Calc_Sxi_xi_compact(VectorN& Sxi_xi_compact,
                                                       double xi,
                                                       double eta,
                                                       double zeta,
                                                       double thickness,
                                                       double zoffset) {
    Sxi_xi_compact(0) = -0.125 * (eta - 1) * (eta * eta + eta + 3 * xi * xi - 3);
    Sxi_xi_compact(1) = -0.0625 * m_lenX * (3 * xi + 1) * (xi - 1) * (eta - 1);
    Sxi_xi_compact(2) = -0.0625 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1);
    Sxi_xi_compact(3) = -0.125 * (eta - 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_xi_compact(4) = 0.125 * (eta - 1) * (eta * eta + eta + 3 * xi * xi - 3);
    Sxi_xi_compact(5) = -0.0625 * m_lenX * (xi + 1) * (3 * xi - 1) * (eta - 1);
    Sxi_xi_compact(6) = 0.0625 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1);
    Sxi_xi_compact(7) = 0.125 * (eta - 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_xi_compact(8) = -0.125 * (eta + 1) * (eta * eta - eta + 3 * xi * xi - 3);
    Sxi_xi_compact(9) = 0.0625 * m_lenX * (xi + 1) * (3 * xi - 1) * (eta + 1);
    Sxi_xi_compact(10) = 0.0625 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1);
    Sxi_xi_compact(11) = -0.125 * (eta + 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_xi_compact(12) = 0.125 * (eta + 1) * (eta * eta - eta + 3 * xi * xi - 3);
    Sxi_xi_compact(13) = 0.0625 * m_lenX * (3 * xi + 1) * (xi - 1) * (eta + 1);
    Sxi_xi_compact(14) = -0.0625 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1);
    Sxi_xi_compact(15) = 0.125 * (eta + 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);
    Sxi_xi_compact = m_Tij * Sxi_xi_compact;
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to eta
// [s1; s2; s3; ...]

void ChElementShellANCF_3443B::Calc_Sxi_eta_compact(VectorN& Sxi_eta_compact,
                                                        double xi,
                                                        double eta,
                                                        double zeta,
                                                        double thickness,
                                                        double zoffset) {
    Sxi_eta_compact(0) = -0.125 * (xi - 1) * (3 * eta * eta + xi * xi + xi - 3);
    Sxi_eta_compact(1) = -0.0625 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1);
    Sxi_eta_compact(2) = -0.0625 * m_lenY * (3 * eta + 1) * (eta - 1) * (xi - 1);
    Sxi_eta_compact(3) = -0.125 * (xi - 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_eta_compact(4) = 0.125 * (xi + 1) * (3 * eta * eta + xi * xi - xi - 3);
    Sxi_eta_compact(5) = -0.0625 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1);
    Sxi_eta_compact(6) = 0.0625 * m_lenY * (3 * eta + 1) * (eta - 1) * (xi + 1);
    Sxi_eta_compact(7) = 0.125 * (xi + 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_eta_compact(8) = -0.125 * (xi + 1) * (3 * eta * eta + xi * xi - xi - 3);
    Sxi_eta_compact(9) = 0.0625 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1);
    Sxi_eta_compact(10) = 0.0625 * m_lenY * (eta + 1) * (3 * eta - 1) * (xi + 1);
    Sxi_eta_compact(11) = -0.125 * (xi + 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);

    Sxi_eta_compact(12) = 0.125 * (xi - 1) * (3 * eta * eta + xi * xi + xi - 3);
    Sxi_eta_compact(13) = 0.0625 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1);
    Sxi_eta_compact(14) = -0.0625 * m_lenY * (eta + 1) * (3 * eta - 1) * (xi - 1);
    Sxi_eta_compact(15) = 0.125 * (xi - 1) * (m_thicknessZ - 2 * zoffset - thickness - thickness * zeta);
    Sxi_eta_compact = m_Tij * Sxi_eta_compact;
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to zeta
// [s1; s2; s3; ...]

void ChElementShellANCF_3443B::Calc_Sxi_zeta_compact(VectorN& Sxi_zeta_compact,
                                                         double xi,
                                                         double eta,
                                                         double zeta,
                                                         double thickness,
                                                         double zoffset) {
    Sxi_zeta_compact(0) = 0.0;
    Sxi_zeta_compact(1) = 0.0;
    Sxi_zeta_compact(2) = 0.0;
    Sxi_zeta_compact(3) = 0.125 * thickness * (xi - 1) * (eta - 1);

    Sxi_zeta_compact(4) = 0.0;
    Sxi_zeta_compact(5) = 0.0;
    Sxi_zeta_compact(6) = 0.0;
    Sxi_zeta_compact(7) = -0.125 * thickness * (xi + 1) * (eta - 1);

    Sxi_zeta_compact(8) = 0.0;
    Sxi_zeta_compact(9) = 0.0;
    Sxi_zeta_compact(10) = 0.0;
    Sxi_zeta_compact(11) = 0.125 * thickness * (xi + 1) * (eta + 1);

    Sxi_zeta_compact(12) = 0.0;
    Sxi_zeta_compact(13) = 0.0;
    Sxi_zeta_compact(14) = 0.0;
    Sxi_zeta_compact(15) = -0.125 * thickness * (xi - 1) * (eta + 1);
    Sxi_zeta_compact = m_Tij * Sxi_zeta_compact;
}

// Nx3 compact form of the partial derivatives of Normalized Shape Functions with respect to xi, eta, and zeta by
// columns

void ChElementShellANCF_3443B::Calc_Sxi_D(MatrixNx3c& Sxi_D,
                                              double xi,
                                              double eta,
                                              double zeta,
                                              double thickness,
                                              double zoffset) {
    VectorN Sxi_D_col;
    Calc_Sxi_xi_compact(Sxi_D_col, xi, eta, zeta, thickness, zoffset);
    Sxi_D.col(0) = Sxi_D_col;

    Calc_Sxi_eta_compact(Sxi_D_col, xi, eta, zeta, thickness, zoffset);
    Sxi_D.col(1) = Sxi_D_col;

    Calc_Sxi_zeta_compact(Sxi_D_col, xi, eta, zeta, thickness, zoffset);
    Sxi_D.col(2) = Sxi_D_col;
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

void ChElementShellANCF_3443B::CalcCoordVector(Vector3N& e) {
    e.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    e.segment(3, 3) = m_nodes[0]->GetSlope1().eigen();
    e.segment(6, 3) = m_nodes[0]->GetSlope2().eigen();
    e.segment(9, 3) = m_nodes[0]->GetSlope3().eigen();

    e.segment(12, 3) = m_nodes[1]->GetPos().eigen();
    e.segment(15, 3) = m_nodes[1]->GetSlope1().eigen();
    e.segment(18, 3) = m_nodes[1]->GetSlope2().eigen();
    e.segment(21, 3) = m_nodes[1]->GetSlope3().eigen();

    e.segment(24, 3) = m_nodes[2]->GetPos().eigen();
    e.segment(27, 3) = m_nodes[2]->GetSlope1().eigen();
    e.segment(30, 3) = m_nodes[2]->GetSlope2().eigen();
    e.segment(33, 3) = m_nodes[2]->GetSlope3().eigen();

    e.segment(36, 3) = m_nodes[3]->GetPos().eigen();
    e.segment(39, 3) = m_nodes[3]->GetSlope1().eigen();
    e.segment(42, 3) = m_nodes[3]->GetSlope2().eigen();
    e.segment(45, 3) = m_nodes[3]->GetSlope3().eigen();
}

void ChElementShellANCF_3443B::CalcCoordMatrix(Matrix3xN& ebar) {
    ebar.col(0) = m_nodes[0]->GetPos().eigen();
    ebar.col(1) = m_nodes[0]->GetSlope1().eigen();
    ebar.col(2) = m_nodes[0]->GetSlope2().eigen();
    ebar.col(3) = m_nodes[0]->GetSlope3().eigen();

    ebar.col(4) = m_nodes[1]->GetPos().eigen();
    ebar.col(5) = m_nodes[1]->GetSlope1().eigen();
    ebar.col(6) = m_nodes[1]->GetSlope2().eigen();
    ebar.col(7) = m_nodes[1]->GetSlope3().eigen();

    ebar.col(8) = m_nodes[2]->GetPos().eigen();
    ebar.col(9) = m_nodes[2]->GetSlope1().eigen();
    ebar.col(10) = m_nodes[2]->GetSlope2().eigen();
    ebar.col(11) = m_nodes[2]->GetSlope3().eigen();

    ebar.col(12) = m_nodes[3]->GetPos().eigen();
    ebar.col(13) = m_nodes[3]->GetSlope1().eigen();
    ebar.col(14) = m_nodes[3]->GetSlope2().eigen();
    ebar.col(15) = m_nodes[3]->GetSlope3().eigen();
}

void ChElementShellANCF_3443B::CalcCoordDtVector(Vector3N& edot) {
    edot.segment(0, 3) = m_nodes[0]->GetPosDt().eigen();
    edot.segment(3, 3) = m_nodes[0]->GetSlope1Dt().eigen();
    edot.segment(6, 3) = m_nodes[0]->GetSlope2Dt().eigen();
    edot.segment(9, 3) = m_nodes[0]->GetSlope3Dt().eigen();

    edot.segment(12, 3) = m_nodes[1]->GetPosDt().eigen();
    edot.segment(15, 3) = m_nodes[1]->GetSlope1Dt().eigen();
    edot.segment(18, 3) = m_nodes[1]->GetSlope2Dt().eigen();
    edot.segment(21, 3) = m_nodes[1]->GetSlope3Dt().eigen();

    edot.segment(24, 3) = m_nodes[2]->GetPosDt().eigen();
    edot.segment(27, 3) = m_nodes[2]->GetSlope1Dt().eigen();
    edot.segment(30, 3) = m_nodes[2]->GetSlope2Dt().eigen();
    edot.segment(33, 3) = m_nodes[2]->GetSlope3Dt().eigen();

    edot.segment(36, 3) = m_nodes[3]->GetPosDt().eigen();
    edot.segment(39, 3) = m_nodes[3]->GetSlope1Dt().eigen();
    edot.segment(42, 3) = m_nodes[3]->GetSlope2Dt().eigen();
    edot.segment(45, 3) = m_nodes[3]->GetSlope3Dt().eigen();
}

void ChElementShellANCF_3443B::CalcCoordDtMatrix(Matrix3xN& ebardot) {
    ebardot.col(0) = m_nodes[0]->GetPosDt().eigen();
    ebardot.col(1) = m_nodes[0]->GetSlope1Dt().eigen();
    ebardot.col(2) = m_nodes[0]->GetSlope2Dt().eigen();
    ebardot.col(3) = m_nodes[0]->GetSlope3Dt().eigen();

    ebardot.col(4) = m_nodes[1]->GetPosDt().eigen();
    ebardot.col(5) = m_nodes[1]->GetSlope1Dt().eigen();
    ebardot.col(6) = m_nodes[1]->GetSlope2Dt().eigen();
    ebardot.col(7) = m_nodes[1]->GetSlope3Dt().eigen();

    ebardot.col(8) = m_nodes[2]->GetPosDt().eigen();
    ebardot.col(9) = m_nodes[2]->GetSlope1Dt().eigen();
    ebardot.col(10) = m_nodes[2]->GetSlope2Dt().eigen();
    ebardot.col(11) = m_nodes[2]->GetSlope3Dt().eigen();

    ebardot.col(12) = m_nodes[3]->GetPosDt().eigen();
    ebardot.col(13) = m_nodes[3]->GetSlope1Dt().eigen();
    ebardot.col(14) = m_nodes[3]->GetSlope2Dt().eigen();
    ebardot.col(15) = m_nodes[3]->GetSlope3Dt().eigen();
}

void ChElementShellANCF_3443B::CalcCombinedCoordMatrix(Matrix6xN& ebar_ebardot) {
    ebar_ebardot.block<3, 1>(0, 0) = m_nodes[0]->GetPos().eigen();
    ebar_ebardot.block<3, 1>(3, 0) = m_nodes[0]->GetPosDt().eigen();
    ebar_ebardot.block<3, 1>(0, 1) = m_nodes[0]->GetSlope1().eigen();
    ebar_ebardot.block<3, 1>(3, 1) = m_nodes[0]->GetSlope1Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 2) = m_nodes[0]->GetSlope2().eigen();
    ebar_ebardot.block<3, 1>(3, 2) = m_nodes[0]->GetSlope2Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 3) = m_nodes[0]->GetSlope3().eigen();
    ebar_ebardot.block<3, 1>(3, 3) = m_nodes[0]->GetSlope3Dt().eigen();

    ebar_ebardot.block<3, 1>(0, 4) = m_nodes[1]->GetPos().eigen();
    ebar_ebardot.block<3, 1>(3, 4) = m_nodes[1]->GetPosDt().eigen();
    ebar_ebardot.block<3, 1>(0, 5) = m_nodes[1]->GetSlope1().eigen();
    ebar_ebardot.block<3, 1>(3, 5) = m_nodes[1]->GetSlope1Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 6) = m_nodes[1]->GetSlope2().eigen();
    ebar_ebardot.block<3, 1>(3, 6) = m_nodes[1]->GetSlope2Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 7) = m_nodes[1]->GetSlope3().eigen();
    ebar_ebardot.block<3, 1>(3, 7) = m_nodes[1]->GetSlope3Dt().eigen();

    ebar_ebardot.block<3, 1>(0, 8) = m_nodes[2]->GetPos().eigen();
    ebar_ebardot.block<3, 1>(3, 8) = m_nodes[2]->GetPosDt().eigen();
    ebar_ebardot.block<3, 1>(0, 9) = m_nodes[2]->GetSlope1().eigen();
    ebar_ebardot.block<3, 1>(3, 9) = m_nodes[2]->GetSlope1Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 10) = m_nodes[2]->GetSlope2().eigen();
    ebar_ebardot.block<3, 1>(3, 10) = m_nodes[2]->GetSlope2Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 11) = m_nodes[2]->GetSlope3().eigen();
    ebar_ebardot.block<3, 1>(3, 11) = m_nodes[2]->GetSlope3Dt().eigen();

    ebar_ebardot.block<3, 1>(0, 12) = m_nodes[3]->GetPos().eigen();
    ebar_ebardot.block<3, 1>(3, 12) = m_nodes[3]->GetPosDt().eigen();
    ebar_ebardot.block<3, 1>(0, 13) = m_nodes[3]->GetSlope1().eigen();
    ebar_ebardot.block<3, 1>(3, 13) = m_nodes[3]->GetSlope1Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 14) = m_nodes[3]->GetSlope2().eigen();
    ebar_ebardot.block<3, 1>(3, 14) = m_nodes[3]->GetSlope2Dt().eigen();
    ebar_ebardot.block<3, 1>(0, 15) = m_nodes[3]->GetSlope3().eigen();
    ebar_ebardot.block<3, 1>(3, 15) = m_nodes[3]->GetSlope3Dt().eigen();
}

// Calculate the 3x3 Element Jacobian at the given point (xi,eta,zeta) in the element

void ChElementShellANCF_3443B::Calc_J_0xi(ChMatrix33d& J_0xi,
                                              double xi,
                                              double eta,
                                              double zeta,
                                              double thickness,
                                              double zoffset) {
    MatrixNx3c Sxi_D;
    Calc_Sxi_D(Sxi_D, xi, eta, zeta, thickness, zoffset);

    J_0xi = m_pbar0 * Sxi_D;
}

// Calculate the determinant of the 3x3 Element Jacobian at the given point (xi,eta,zeta) in the element

double ChElementShellANCF_3443B::Calc_det_J_0xi(double xi,
                                                    double eta,
                                                    double zeta,
                                                    double thickness,
                                                    double zoffset) {
    ChMatrix33d J_0xi;
    Calc_J_0xi(J_0xi, xi, eta, zeta, thickness, zoffset);

    return (J_0xi.determinant());
}

void ChElementShellANCF_3443B::RotateReorderStiffnessMatrix(ChMatrix66d& D, double theta) {
    // Reorder the stiffness matrix from the order assumed in ChMaterialShellANCF.h
    //  E = [E11,E22,2*E12,E33,2*E13,2*E23]
    // to the order assumed in this element formulation
    //  E = [E11,E22,E33,2*E23,2*E13,2*E12]
    // Note that the 6x6 stiffness matrix is symmetric

    ChMatrix66d D_Reordered;
    D_Reordered << D(0, 0), D(0, 1), D(0, 3), D(0, 5), D(0, 4), D(0, 2), D(1, 0), D(1, 1), D(1, 3), D(1, 5), D(1, 4),
        D(1, 2), D(3, 0), D(3, 1), D(3, 3), D(3, 5), D(3, 4), D(3, 2), D(5, 0), D(5, 1), D(5, 3), D(5, 5), D(5, 4),
        D(5, 2), D(4, 0), D(4, 1), D(4, 3), D(4, 5), D(4, 4), D(4, 2), D(2, 0), D(2, 1), D(2, 3), D(2, 5), D(2, 4),
        D(2, 2);

    // Stiffness Tensor Rotation Matrix From:
    // http://solidmechanics.org/text/Chapter3_2/Chapter3_2.htm

    ChMatrix66d K;
    K << std::cos(theta) * std::cos(theta), std::sin(theta) * std::sin(theta), 0, 0, 0,
        2 * std::cos(theta) * std::sin(theta), std::sin(theta) * std::sin(theta), std::cos(theta) * std::cos(theta), 0,
        0, 0, -2 * std::cos(theta) * std::sin(theta), 0, 0, 1, 0, 0, 0, 0, 0, 0, std::cos(theta), std::sin(theta), 0, 0,
        0, 0, -std::sin(theta), std::cos(theta), 0, -std::cos(theta) * std::sin(theta),
        std::cos(theta) * std::sin(theta), 0, 0, 0,
        std::cos(theta) * std::cos(theta) - std::sin(theta) * std::sin(theta);

    D = K * D_Reordered * K.transpose();
}

////////////////////////////////////////////////////////////////

//#ifndef CH_QUADRATURE_STATIC_TABLES
#define CH_QUADRATURE_STATIC_TABLES 10
ChQuadratureTables static_tables_3443B(1, CH_QUADRATURE_STATIC_TABLES);
//#endif // !CH_QUADRATURE_STATIC_TABLES

ChQuadratureTables* ChElementShellANCF_3443B::GetStaticGQTables() {
    return &static_tables_3443B;
}

////////////////////////////////////////////////////////////////

// ============================================================================
// Implementation of ChElementShellANCF_3443B::Layer methods
// ============================================================================

// Private constructor (a layer can be created only by adding it to an element)

ChElementShellANCF_3443B::Layer::Layer(double thickness,
                                           double theta,
                                           std::shared_ptr<ChMaterialShellANCF> material)
    : m_thickness(thickness), m_theta(theta), m_material(material) {}

}  // namespace fea
}  // namespace chrono
