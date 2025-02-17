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

#include <cmath>

#include "chrono/fea/ChElementShellANCF_3443B.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire3443B.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

ANCFAirlessTire3443B::ANCFAirlessTire3443B(const std::string& name)
    : ChANCFTire(name),
      m_rim_radius(0.225),
      m_height(0.225),
      m_width(0.4),
      m_t_outer_ring(0.005),
      m_t_spoke(0.003),
      m_spoke_t_frac(1.0/3.0),
      m_hub_rel_ang(10.0 * CH_DEG_TO_RAD),
      m_spoke_curv_pnt_x(0.5 * 0.225),
      m_spoke_curv_pnt_z(0.25 * 0.225),
      m_num_spoke(16),
      m_div_width(3),
      m_div_spoke_len(5),
      m_div_ring_per_spoke(3),
      m_E(76e9),
      m_nu(0.2),
      m_rho(2580),
      m_alpha(0.05) {
      // default contact material
      m_mat = chrono_types::make_shared<ChContactMaterialSMC>();
}

void ANCFAirlessTire3443B::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    // Create an isotropic material (shared by all elements)
    //  The default values approximate fiberglass
    //  Density and Young's modulus were taken from: https://en.wikipedia.org/wiki/Glass_fiber
    //  Poisson's ratio was taken from:
    //  https://matweb.com/search/DataSheet.aspx?MatGUID=d9c18047c49147a2a7c0b0bb1743e812
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(m_rho, m_E, m_nu);
    double alpha = m_alpha;

    //-------------------------------------------------
    // Define the geometry
    //-------------------------------------------------

    int numSpokes = m_num_spoke;  // Number of spokes in the wheel
    int divElPerSpokeLen =
        m_div_spoke_len;  // Number of elements along the length of the spoke, including the T element
    int divOuterElPerSpoke =
        m_div_ring_per_spoke;  // Number of elements along the circumference of the tire / numSpokes
    int divW = m_div_width;    // Number of elements in the width direction

    double rOut = m_rim_radius + m_rim_radius;  // meters; Radius of the midsurface of the outer ring of elements
    double rHub = m_rim_radius;                 // meters; Radius of the rigid hub
    double width = m_width;                     // meters; Width of the Tire
    double tOut = m_t_outer_ring;               // meters; thickness of the outer ring of elements
    double tSpoke = m_t_spoke;                  // meters; thickness of the spoke elements
    double spokeTFrac = m_spoke_t_frac;  // Faction of the spoke modeled by the Shell 3423 T element (0 to 1 exclusion)
    double angRelHub =
        m_hub_rel_ang;  // relative rotation of the hub with respect to the outer ring ranging from -\pi to \pi
    double spokeCurPntX = m_spoke_curv_pnt_x;  // X coordinate in the local spoke coordinate system for the
                                                // third point on a circle defining the arc shape of the spoke.
    double spokeCurPntZ =
        m_spoke_curv_pnt_z;  // Z coordinate in the local spoke coordinate system for the third point on a circle
                               // defining the arc shape of the spoke. A value of 0 means a straight spoke.
    
    //-------------------------------------------------
    // Derived geometry
    //-------------------------------------------------
    int numNodesWidth = divW + 1;  // Number of nodes across the width of the tire
    int numNodesOuterRingSlice =
        numSpokes * divOuterElPerSpoke;  // number of nodes in a slice normal to the width of the tire, also equals the
                                         // number of elements forming a circle in the outer ring
    int numNodesOuterRingTotal =
        numNodesOuterRingSlice * numNodesWidth;  // total number of nodes in the outer band/ring of the tire

    double widthEl = width / divW;                            // width of each element
    double angOuterRingEl = CH_2PI / numNodesOuterRingSlice;  // arc spanned by a single element in the outer ring
    double lenOuterRingEl = 2 * rOut * std::sin(0.5 * angOuterRingEl);  // Chord length spanned by a single element

    int numNodesPerSpoke = divElPerSpokeLen * numNodesWidth;  // number of nodes added by each spoke

    int numElsOutCir = numSpokes * divOuterElPerSpoke;  // number of elements forming a circle in the outer ring

    //-------------------------------------------------
    // Create the nodes for the outer ring.
    //-------------------------------------------------
    for (int i = 0; i < numNodesOuterRingTotal; i++) {
        // Node location
        double loc_x = rOut * std::sin(i * angOuterRingEl);
        double loc_y = widthEl * (i / numNodesOuterRingSlice) - 0.5 * width;
        double loc_z = rOut * std::cos(i * angOuterRingEl);
        ChVector3d loc = wheel_frame.TransformPointLocalToParent(ChVector3d(loc_x, loc_y, loc_z));

        // Global Nodal Coordinate directions - initally aligned with the inertia reference frame
        auto global_dr_du = ChVector3d(1.0, 0, 0);
        auto global_dr_dv = ChVector3d(0, 1.0, 0);
        auto global_dr_dw = ChVector3d(0, 0, 1.0);

        // Create the node
        auto node = chrono_types::make_shared<ChNodeFEAxyzDDD>(loc, global_dr_du, global_dr_dv, global_dr_dw);
        node->SetMass(0);

        // Add node to mesh
        m_mesh->AddNode(node);
    }

    //-------------------------------------------------
    // Create the elements for the outer ring.
    //-------------------------------------------------
    for (int slice = 0; slice < divW; slice++) {
        for (int i = 0; i < numElsOutCir; i++) {
            // Adjacent nodes
            int idx_nodeA = i + numNodesOuterRingSlice * (0 + slice);
            int idx_nodeB = (i + 1) % numNodesOuterRingSlice + numNodesOuterRingSlice * (0 + slice);
            int idx_nodeC = (i + 1) % numNodesOuterRingSlice + numNodesOuterRingSlice * (1 + slice);
            int idx_nodeD = i + numNodesOuterRingSlice * (1 + slice);

            // Get the Nodes
            auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeA));
            auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeB));
            auto nodeC = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeC));
            auto nodeD = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeD));

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3443B>();
            element->SetNodes(nodeA, nodeB, nodeC, nodeD);

            // Set element dimensions
            element->SetDimensions(lenOuterRingEl, widthEl);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(tOut, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(alpha);  // Structural damping for this element

            // Local element nodal position vector gradient coordinates
            auto nodeA_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
            auto nodeA_dr_dw = wheel_frame.TransformDirectionLocalToParent(
                ChVector3d(std::sin(idx_nodeA * angOuterRingEl), 0.0, std::cos(idx_nodeA * angOuterRingEl)));
            auto nodeA_dr_du = nodeA_dr_dv.Cross(nodeA_dr_dw);

            auto nodeB_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
            auto nodeB_dr_dw = wheel_frame.TransformDirectionLocalToParent(
                ChVector3d(std::sin(idx_nodeB * angOuterRingEl), 0.0, std::cos(idx_nodeB * angOuterRingEl)));
            auto nodeB_dr_du = nodeB_dr_dv.Cross(nodeB_dr_dw);

            auto nodeC_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
            auto nodeC_dr_dw = wheel_frame.TransformDirectionLocalToParent(
                ChVector3d(std::sin(idx_nodeC * angOuterRingEl), 0.0, std::cos(idx_nodeC * angOuterRingEl)));
            auto nodeC_dr_du = nodeC_dr_dv.Cross(nodeC_dr_dw);

            auto nodeD_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
            auto nodeD_dr_dw = wheel_frame.TransformDirectionLocalToParent(
                ChVector3d(std::sin(idx_nodeD * angOuterRingEl), 0.0, std::cos(idx_nodeD * angOuterRingEl)));
            auto nodeD_dr_du = nodeD_dr_dv.Cross(nodeD_dr_dw);

            // Set the values of the local element coordinates
            element->SetLocalElRefCoords(nodeA->GetPos(), nodeA_dr_du, nodeA_dr_dv, nodeA_dr_dw, 
                                         nodeB->GetPos(), nodeB_dr_du, nodeB_dr_dv, nodeB_dr_dw, 
                                         nodeC->GetPos(), nodeC_dr_du, nodeC_dr_dv, nodeC_dr_dw, 
                                         nodeD->GetPos(), nodeD_dr_du, nodeD_dr_dv, nodeD_dr_dw);

            // Add element to mesh
            m_mesh->AddElement(element);
        }
    }

    //-------------------------------------------------
    // Create the spokes, one by one
    //-------------------------------------------------
    for (int spoke = 0; spoke < numSpokes; spoke++) {
        // Connection to the outer ring occurs at the Node A & D edge of an outer ring element, so take the x and z
        // coordinates for that element's starting nodes
        ChVector3d CSYSOriginSpoke = ChVector3d(rOut * std::sin(spoke * divOuterElPerSpoke * angOuterRingEl), 0,
                                                rOut * std::cos(spoke * divOuterElPerSpoke * angOuterRingEl));

        auto SpokeHubConnection =
            ChVector3d(rHub * std::sin(spoke * divOuterElPerSpoke * angOuterRingEl + angRelHub), 0,
                       rHub * std::cos(spoke * divOuterElPerSpoke * angOuterRingEl + angRelHub));

        auto xAxisSpoke = SpokeHubConnection - CSYSOriginSpoke;
        double spokeChordLen = xAxisSpoke.Length();
        xAxisSpoke.Normalize();
        ChVector3d yAxisSpoke = ChVector3d(0, 1, 0);
        ChVector3d zAxisSpoke = xAxisSpoke.Cross(yAxisSpoke);

        auto xAxisSpoke_whl = wheel_frame.TransformDirectionLocalToParent(xAxisSpoke);
        auto yAxisSpoke_whl = wheel_frame.TransformDirectionLocalToParent(yAxisSpoke);
        auto zAxisSpoke_whl = wheel_frame.TransformDirectionLocalToParent(zAxisSpoke);

        auto global_dr_du = ChVector3d(1.0, 0, 0);
        auto global_dr_dv = ChVector3d(0, 1.0, 0);
        auto global_dr_dw = ChVector3d(0, 0, 1.0);

        if (spokeCurPntZ == 0.0) {
            // Spoke nodes are colinear
            double spokeChordLenTEl = spokeTFrac * spokeChordLen;
            double spokeChordLenRegEl = (1 - spokeTFrac) * spokeChordLen / (divElPerSpokeLen - 1);

            // Create the nodes first
            for (int slice = 0; slice < numNodesWidth; slice++) {
                auto ySlice = ChVector3d(0, widthEl * slice - 0.5 * width, 0);
                auto loc = CSYSOriginSpoke + ySlice;

                for (int k = 0; k < divElPerSpokeLen; k++) {
                    if (k == 0) {
                        // Create the ending T node as the first node in each slice
                        loc += spokeChordLenTEl * xAxisSpoke;
                    } else {
                        // Create the ending node for a regular Shell 3423 element
                        loc += spokeChordLenRegEl * xAxisSpoke;
                    }

                    ChVector3d loc_whl = wheel_frame.TransformPointLocalToParent(loc);

                    // Create the node
                    auto node = chrono_types::make_shared<ChNodeFEAxyzDDD>(loc_whl, global_dr_du, global_dr_dv, global_dr_dw);
                    node->SetMass(0);
                    // node->SetFixed(true);

                    // Add node to mesh
                    m_mesh->AddNode(node);

                    // Store the nodes on the hub
                    if (k == (divElPerSpokeLen - 1)) {
                        m_hub_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(node));
                    }
                }
            }

            // Create the elements
            for (int slice = 0; slice < divW; slice++) {
                // The first element in the spoke connects to the outer hub (forms a T shape).  For the
                // ChElementShellANCF_3443B mesh, there is nothing special about this element.

                // Determine the indices of the connecting nodes:
                int idx_nodeA = spoke * divOuterElPerSpoke + numNodesOuterRingSlice * (0 + slice);
                int idx_nodeD = spoke * divOuterElPerSpoke + numNodesOuterRingSlice * (1 + slice);

                int idx_nodeB = numNodesOuterRingTotal + spoke * numNodesPerSpoke + (slice + 0) * divElPerSpokeLen;
                int idx_nodeC = numNodesOuterRingTotal + spoke * numNodesPerSpoke + (slice + 1) * divElPerSpokeLen;

                // Get the Nodes
                auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeA));
                auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeB));
                auto nodeC = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeC));
                auto nodeD = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeD));

                // Create the element and set its nodes.
                auto element = chrono_types::make_shared<ChElementShellANCF_3443B>();
                element->SetNodes(nodeA, nodeB, nodeC, nodeD);

                // Set element dimensions
                element->SetDimensions(spokeChordLenTEl, widthEl);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(tOut, 0 * CH_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(alpha);  // Structural damping for this element

                // Set the values of the local element coordinates
                element->SetLocalElRefCoords(nodeA->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl, 
                                             nodeB->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl,
                                             nodeC->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl,
                                             nodeD->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl);

                // Add element to mesh
                m_mesh->AddElement(element);

                // The remaining elements only use the nodes that were created for this spoke
                for (int i = 1; i < divElPerSpokeLen; i++) {
                    // Adjacent nodes
                    int idx_nodeA =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (0 + slice);
                    int idx_nodeB =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (0 + slice);
                    int idx_nodeC =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (1 + slice);
                    int idx_nodeD =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (1 + slice);

                    // Get the Nodes
                    auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeA));
                    auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeB));
                    auto nodeC = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeC));
                    auto nodeD = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeD));

                    // Create the element and set its nodes.
                    auto element = chrono_types::make_shared<ChElementShellANCF_3443B>();
                    element->SetNodes(nodeA, nodeB, nodeC, nodeD);

                    auto edgeElSpk = nodeB->GetPos() - nodeA->GetPos();

                    // Set element dimensions
                    element->SetDimensions(edgeElSpk.Length(), widthEl);

                    // Add a single layers with a fiber angle of 0 degrees.
                    element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

                    // Set other element properties
                    element->SetAlphaDamp(alpha);  // Structural damping for this element

                    // Set the values of the local element coordinates
                    element->SetLocalElRefCoords(nodeA->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl,
                                                 nodeB->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl,
                                                 nodeC->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl,
                                                 nodeD->GetPos(), xAxisSpoke_whl, yAxisSpoke_whl, zAxisSpoke_whl);

                    // Add element to mesh
                    m_mesh->AddElement(element);
                }
            }
        } else {
            // Spokes are curved

            // Calculate the curve center and radius
            // based on https://www.johndcook.com/blog/2023/06/18/circle-through-three-points/
            double x1 = 0.0;
            double z1 = 0.0;
            double x2 = spokeChordLen;
            double z2 = 0.0;
            double x3 = spokeCurPntX;  // 0.5 * spokeChordLen;  // spokeCurPntX;
            double z3 = spokeCurPntZ;  // 0.5 * spokeChordLen;  // spokeCurPntZ;

            double s1 = z1 * z1 + x1 * x1;
            double s2 = z2 * z2 + x2 * x2;
            double s3 = z3 * z3 + x3 * x3;
            double M11 = z1 * x2 + z2 * x3 + z3 * x1 - (z2 * x1 + z3 * x2 + z1 * x3);
            double M12 = s1 * x2 + s2 * x3 + s3 * x1 - (s2 * x1 + s3 * x2 + s1 * x3);
            double M13 = s1 * z2 + s2 * z3 + s3 * z1 - (s2 * z1 + s3 * z2 + s1 * z3);
            double z0 = 0.5 * M12 / M11;   // Z location of the center of the circle
            double x0 = -0.5 * M13 / M11;  // X location of the center of the circle
            double r0 = std::sqrt((z1 - z0) * (z1 - z0) + (x1 - x0) * (x1 - x0));  // Radius of the circle

            double angSpokeArcStart = std::atan2(-x0, -z0);
            double angSpokeArcEnd = std::atan2(spokeChordLen - x0, -z0);
            if ((spokeCurPntZ > 0) && (angSpokeArcEnd < angSpokeArcStart)) {
                angSpokeArcEnd += CH_2PI;
            }
            if ((spokeCurPntZ < 0) && (angSpokeArcEnd > angSpokeArcStart)) {
                angSpokeArcEnd -= CH_2PI;
            }
            double angSpokeArc = angSpokeArcEnd - angSpokeArcStart;

            // Create the nodes first
            for (int slice = 0; slice < numNodesWidth; slice++) {
                auto ySlice = ChVector3d(0, widthEl * slice - 0.5 * width, 0);
                auto origin = CSYSOriginSpoke + ySlice + x0 * xAxisSpoke + z0 * zAxisSpoke;

                auto loc = CSYSOriginSpoke + ySlice;

                for (int k = 0; k < divElPerSpokeLen; k++) {
                    double ang = angSpokeArcStart + spokeTFrac * angSpokeArc +
                                 k * (1 - spokeTFrac) * angSpokeArc / (divElPerSpokeLen - 1);
                    auto dir = std::sin(ang) * xAxisSpoke + std::cos(ang) * zAxisSpoke;
                    loc = origin + r0 * dir;

                    ChVector3d loc_whl = wheel_frame.TransformPointLocalToParent(loc);

                    // Create the node
                    auto node = chrono_types::make_shared<ChNodeFEAxyzDDD>(loc_whl, global_dr_du, global_dr_dv, global_dr_dw);
                    node->SetMass(0);
                    // node->SetFixed(true);

                    // Add node to mesh
                    m_mesh->AddNode(node);

                    // Store the nodes on the hub
                    if (k == divElPerSpokeLen - 1) {
                        m_hub_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(node));
                    }
                }
            }

            // Create the elements
            for (int slice = 0; slice < divW; slice++) {
                // The first element in the spoke connects to the outer hub (forms a T shape).  For the
                // ChElementShellANCF_3443B mesh, there is nothing special about this element.

                // Determine the indices of the connecting nodes:
                int idx_nodeA = spoke * divOuterElPerSpoke + numNodesOuterRingSlice * (0 + slice);
                int idx_nodeD = spoke * divOuterElPerSpoke + numNodesOuterRingSlice * (1 + slice);

                int idx_nodeB = numNodesOuterRingTotal + spoke * numNodesPerSpoke + (slice + 0) * divElPerSpokeLen;
                int idx_nodeC = numNodesOuterRingTotal + spoke * numNodesPerSpoke + (slice + 1) * divElPerSpokeLen;

                // Get the Nodes
                auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeA));
                auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeB));
                auto nodeC = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeC));
                auto nodeD = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeD));

                // Create the element and set its nodes.
                auto element = chrono_types::make_shared<ChElementShellANCF_3443B>();
                element->SetNodes(nodeA, nodeB, nodeC, nodeD);

                // Set element dimensions
                auto edgeElSpk = nodeB->GetPos() - nodeA->GetPos();
                element->SetDimensions(edgeElSpk.Length(), widthEl);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(tOut, 0 * CH_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(alpha);  // Structural damping for this element

                // Local element nodal position vector gradient coordinates
                auto dirAD = std::sin(angSpokeArcStart) * xAxisSpoke + std::cos(angSpokeArcStart) * zAxisSpoke;
                auto ang = angSpokeArcStart + spokeTFrac * angSpokeArc;
                auto dirBC = std::sin(ang) * xAxisSpoke + std::cos(ang) * zAxisSpoke;
                if (spokeCurPntZ < 0) {
                    dirAD *= -1;
                    dirBC *= -1;
                }

                auto nodeA_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                auto nodeA_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirAD);
                auto nodeA_dr_du = nodeA_dr_dv.Cross(nodeA_dr_dw);

                auto nodeB_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                auto nodeB_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirBC);
                auto nodeB_dr_du = nodeB_dr_dv.Cross(nodeB_dr_dw);

                auto nodeC_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                auto nodeC_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirBC);
                auto nodeC_dr_du = nodeC_dr_dv.Cross(nodeC_dr_dw);

                auto nodeD_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                auto nodeD_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirAD);
                auto nodeD_dr_du = nodeD_dr_dv.Cross(nodeD_dr_dw);

                // Set the values of the local element coordinates
                element->SetLocalElRefCoords(nodeA->GetPos(), nodeA_dr_du, nodeA_dr_dv, nodeA_dr_dw,
                                             nodeB->GetPos(), nodeB_dr_du, nodeB_dr_dv, nodeB_dr_dw,
                                             nodeC->GetPos(), nodeC_dr_du, nodeC_dr_dv, nodeC_dr_dw,
                                             nodeD->GetPos(), nodeD_dr_du, nodeD_dr_dv, nodeD_dr_dw);

                // Add element to mesh
                m_mesh->AddElement(element);

                // The remaining elements only use the nodes that were created for this spoke
                for (int i = 1; i < divElPerSpokeLen; i++) {
                    // Adjacent nodes
                    int idx_nodeA =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (0 + slice);
                    int idx_nodeB =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (0 + slice);
                    int idx_nodeC =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (1 + slice);
                    int idx_nodeD =
                        numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (1 + slice);

                    // Get the Nodes
                    auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeA));
                    auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeB));
                    auto nodeC = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeC));
                    auto nodeD = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(idx_nodeD));

                    // Create the element and set its nodes.
                    auto element = chrono_types::make_shared<ChElementShellANCF_3443B>();
                    element->SetNodes(nodeA, nodeB, nodeC, nodeD);

                    // Set element dimensions
                    auto edgeElSpk = nodeB->GetPos() - nodeA->GetPos();
                    element->SetDimensions(edgeElSpk.Length(), widthEl);

                    // Add a single layers with a fiber angle of 0 degrees.
                    element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

                    // Set other element properties
                    element->SetAlphaDamp(alpha);  // Structural damping for this element

                    // Local element nodal position vector gradient coordinates
                    auto ang = angSpokeArcStart + spokeTFrac * angSpokeArc +
                               (i - 1) * (1 - spokeTFrac) * angSpokeArc / (divElPerSpokeLen - 1);
                    auto dirAD = std::sin(ang) * xAxisSpoke + std::cos(ang) * zAxisSpoke;
                    ang = angSpokeArcStart + spokeTFrac * angSpokeArc +
                          i * (1 - spokeTFrac) * angSpokeArc / (divElPerSpokeLen - 1);
                    auto dirBC = std::sin(ang) * xAxisSpoke + std::cos(ang) * zAxisSpoke;
                    if (spokeCurPntZ < 0) {
                        dirAD *= -1;
                        dirBC *= -1;
                    }

                    auto nodeA_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                    auto nodeA_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirAD);
                    auto nodeA_dr_du = nodeA_dr_dv.Cross(nodeA_dr_dw);

                    auto nodeB_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                    auto nodeB_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirBC);
                    auto nodeB_dr_du = nodeB_dr_dv.Cross(nodeB_dr_dw);

                    auto nodeC_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                    auto nodeC_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirBC);
                    auto nodeC_dr_du = nodeC_dr_dv.Cross(nodeC_dr_dw);

                    auto nodeD_dr_dv = wheel_frame.TransformDirectionLocalToParent(ChVector3d(0.0, 1.0, 0.0));
                    auto nodeD_dr_dw = wheel_frame.TransformDirectionLocalToParent(dirAD);
                    auto nodeD_dr_du = nodeD_dr_dv.Cross(nodeD_dr_dw);

                    // Set the values of the local element coordinates
                    element->SetLocalElRefCoords(nodeA->GetPos(), nodeA_dr_du, nodeA_dr_dv, nodeA_dr_dw,
                                                 nodeB->GetPos(), nodeB_dr_du, nodeB_dr_dv, nodeB_dr_dw,
                                                 nodeC->GetPos(), nodeC_dr_du, nodeC_dr_dv, nodeC_dr_dw,
                                                 nodeD->GetPos(), nodeD_dr_du, nodeD_dr_dv, nodeD_dr_dw);

                    // Add element to mesh
                    m_mesh->AddElement(element);
                }
            }
        }
    }
}

std::vector<std::shared_ptr<ChNodeFEAbase>> ANCFAirlessTire3443B::GetConnectedNodes() const {
    return m_hub_nodes;
}

void ANCFAirlessTire3443B::CreateContactMaterial() {
    m_contact_mat = m_mat;
}

}  // end namespace vehicle
}  // end namespace chrono
