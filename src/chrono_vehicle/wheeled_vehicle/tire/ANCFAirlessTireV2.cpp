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

#include <cmath>

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3423T.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTireV2.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

ANCFAirlessTireV2::ANCFAirlessTireV2(const std::string& name)
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
      m_mid_el_spokes(false),
      m_E(76e9),
      m_nu(0.2),
      m_rho(2580),
      m_alpha(0.05) {
      // default contact material
      m_mat = chrono_types::make_shared<ChContactMaterialSMC>();
}

void ANCFAirlessTireV2::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
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

    bool midElSpokes = m_mid_el_spokes;  // Spokes intersect in the middle of an outer ring element.  If false, it
                                         // intersects at
                               // the starting edge (-u edge) of the outer ring element
    
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

        // Node direction
        double dir_x = std::sin(i * angOuterRingEl);
        double dir_y = 0;
        double dir_z = std::cos(i * angOuterRingEl);
        ChVector3d dir = wheel_frame.TransformDirectionLocalToParent(ChVector3d(dir_x, dir_y, dir_z));

        // Create the node
        auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
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
            int nodeA = i + numNodesOuterRingSlice * (0 + slice);
            int nodeB = (i + 1) % numNodesOuterRingSlice + numNodesOuterRingSlice * (0 + slice);
            int nodeC = (i + 1) % numNodesOuterRingSlice + numNodesOuterRingSlice * (1 + slice);
            int nodeD = i + numNodesOuterRingSlice * (1 + slice);

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeA)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeB)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeC)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeD)));

            // Set element dimensions
            element->SetDimensions(lenOuterRingEl, widthEl);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(tOut, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(alpha);  // Structural damping for this element

            // Add element to mesh
            m_mesh->AddElement(element);
        }
    }

    //-------------------------------------------------
    // Create the spokes, one by one
    //-------------------------------------------------
    if (midElSpokes) {
        angRelHub +=
            0.5 * angOuterRingEl;  // Rotate the hub connection to account for the spoke intersecting in the middle of
                                   // an outer ring element rather than its starting edge (NodeA & D).
    }

    for (int spoke = 0; spoke < numSpokes; spoke++) {
        ChVector3d CSYSOriginSpoke;
        if (midElSpokes) {
            // Connection to the outer ring occurs in the middle of an outer ring element, so average the x and z
            // coordinates for that element's bounding nodes
            CSYSOriginSpoke = 0.5 * ChVector3d(rOut * std::sin(spoke * divOuterElPerSpoke * angOuterRingEl), 0,
                                               rOut * std::cos(spoke * divOuterElPerSpoke * angOuterRingEl)) +
                              0.5 * ChVector3d(rOut * std::sin((1 + spoke * divOuterElPerSpoke) * angOuterRingEl), 0,
                                               rOut * std::cos((1 + spoke * divOuterElPerSpoke) * angOuterRingEl));
        } else {
            // Connection to the outer ring occurs at the Node A & D edge of an outer ring element, so take the x and z
            // coordinates for that element's starting nodes
            CSYSOriginSpoke = ChVector3d(rOut * std::sin(spoke * divOuterElPerSpoke * angOuterRingEl), 0,
                                         rOut * std::cos(spoke * divOuterElPerSpoke * angOuterRingEl));
        }

        auto SpokeHubConnection =
            ChVector3d(rHub * std::sin(spoke * divOuterElPerSpoke * angOuterRingEl + angRelHub), 0,
                       rHub * std::cos(spoke * divOuterElPerSpoke * angOuterRingEl + angRelHub));

        auto xAxisSpoke = SpokeHubConnection - CSYSOriginSpoke;
        double spokeChordLen = xAxisSpoke.Length();
        xAxisSpoke.Normalize();
        ChVector3d yAxisSpoke = ChVector3d(0, 1, 0);
        ChVector3d zAxisSpoke = xAxisSpoke.Cross(yAxisSpoke);

        // Create the Nodes first
        for (int slice = 0; slice < numNodesWidth; slice++) {
            auto ySlice = ChVector3d(0, widthEl * slice - 0.5 * width, 0);
            auto loc = CSYSOriginSpoke + ySlice;
            auto dir = zAxisSpoke;

            // Create the nodes
            for (int k = 0; k < divElPerSpokeLen; k++) {
                if (spokeCurPntZ == 0.0) {
                    // Spoke nodes are colinear
                    if (k == 0) {
                        // Create the ending T node as the first node in each slice
                        loc += spokeTFrac * spokeChordLen * xAxisSpoke;
                    } else {
                        // Create the ending node for a regular Shell 3423 element
                        double spokeChordLenRegEl = (1 - spokeTFrac) * spokeChordLen / (divElPerSpokeLen - 1);
                        loc += spokeChordLenRegEl * xAxisSpoke;
                    }
                } else {
                    // Spoke nodes are curved
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

                    auto origin = CSYSOriginSpoke + ySlice + x0 * xAxisSpoke + z0 * zAxisSpoke;
                    double angSpokeArcStart = std::atan2(-x0, -z0);
                    double angSpokeArcEnd = std::atan2(spokeChordLen - x0, -z0);
                    if ((spokeCurPntZ > 0) && (angSpokeArcEnd < angSpokeArcStart)) {
                        angSpokeArcEnd += CH_2PI;
                    }
                    if ((spokeCurPntZ < 0) && (angSpokeArcEnd > angSpokeArcStart)) {
                        angSpokeArcEnd -= CH_2PI;
                    }
                    double angSpokeArc = angSpokeArcEnd - angSpokeArcStart;

                    if (k == 0) {
                        // Create the ending T node as the first node in each slice
                        double ang = angSpokeArcStart + spokeTFrac * angSpokeArc;
                        dir = std::sin(ang) * xAxisSpoke + std::cos(ang) * zAxisSpoke;
                        loc = origin + r0 * dir;

                    } else {
                        // Create the ending node for a regular Shell 3423 element
                        double ang = angSpokeArcStart + spokeTFrac * angSpokeArc +
                                     k * (1 - spokeTFrac) * angSpokeArc / (divElPerSpokeLen - 1);
                        dir = std::sin(ang) * xAxisSpoke + std::cos(ang) * zAxisSpoke;
                        loc = origin + r0 * dir;
                    }
                    if (spokeCurPntZ < 0) {
                        dir *= -1;
                    }
                }
                ChVector3d loc_whl = wheel_frame.TransformPointLocalToParent(loc);
                ChVector3d dir_whl = wheel_frame.TransformDirectionLocalToParent(dir);

                // Create the node
                auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc_whl, dir_whl);
                node->SetMass(0);

                // Add node to mesh
                m_mesh->AddNode(node);

                // Store the nodes on the hub
                if (k == (divElPerSpokeLen - 1)) {
                    m_hub_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(node));
                }
            }
        }

        // Create the T element
        // The first element in the spoke is a "T" element
        for (int slice = 0; slice < divW; slice++) {
            // Determine the connecting nodes:
            int nodeT0 = spoke * divOuterElPerSpoke + numNodesOuterRingSlice * (0 + slice);
            int nodeT1 =
                (spoke * divOuterElPerSpoke + 1) % numNodesOuterRingSlice + numNodesOuterRingSlice * (0 + slice);
            int nodeT2 =
                (spoke * divOuterElPerSpoke + 1) % numNodesOuterRingSlice + numNodesOuterRingSlice * (1 + slice);
            int nodeT3 = spoke * divOuterElPerSpoke + numNodesOuterRingSlice * (1 + slice);

            int nodeB = numNodesOuterRingTotal + spoke * numNodesPerSpoke + (slice + 0) * divElPerSpokeLen;
            int nodeC = numNodesOuterRingTotal + spoke * numNodesPerSpoke + (slice + 1) * divElPerSpokeLen;

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423T>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeB)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeC)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT3)));

            // Calculate the intersection angle
            auto nodeT0_loc = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT0))->GetPos();
            auto nodeT1_loc = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT1))->GetPos();
            auto nodeB_loc = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeB))->GetPos();
            auto OuterEdge = nodeT1_loc - nodeT0_loc;

            if (midElSpokes) {
                //Spoke intersects the outer edge element in its middle
                auto origin = 0.5 * nodeT0_loc + 0.5 * nodeT1_loc;
                auto vOut = nodeT1_loc - origin;
                auto vEl = nodeB_loc - origin;
                vOut.Normalize();
                vEl.Normalize();
                double angT = std::acos(vOut.Dot(vEl));
                auto edgeElSpk = nodeB_loc - origin;

                element->SetDimensions(edgeElSpk.Length(), widthEl, OuterEdge.Length(), 0, angT);
            } else {
                // Spoke intersects the outer edge element at its starting edge
                auto origin = nodeT0_loc;
                auto vOut = nodeT1_loc - origin;
                auto vEl = nodeB_loc - origin;
                vOut.Normalize();
                vEl.Normalize();
                double angT = std::acos(vOut.Dot(vEl));
                auto edgeElSpk = nodeB_loc - origin;

                element->SetDimensions(edgeElSpk.Length(), widthEl, OuterEdge.Length(), -1, angT);
            }

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(alpha);  // Structural damping for this element

            // Add element to mesh
            m_mesh->AddElement(element);

            // The remaining elements are regular 3423 shell elements
            for (int i = 1; i < divElPerSpokeLen; i++) {
                // Adjacent nodes
                int node0 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (0 + slice);
                int node1 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (0 + slice);
                int node2 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (1 + slice);
                int node3 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (1 + slice);

                // Create the element and set its nodes.
                auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node0)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node2)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node3)));

                auto edgeElSpk = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node1))->GetPos() -
                                 std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node0))->GetPos();

                // Set element dimensions
                element->SetDimensions(edgeElSpk.Length(), widthEl);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(alpha);  // Structural damping for this element

                // Add element to mesh
                m_mesh->AddElement(element);
            }
        }
    }
}

std::vector<std::shared_ptr<ChNodeFEAbase>> ANCFAirlessTireV2::GetConnectedNodes() const {
    return m_hub_nodes;
}

void ANCFAirlessTireV2::CreateContactMaterial() {
    m_contact_mat = m_mat;
}

}  // end namespace vehicle
}  // end namespace chrono
