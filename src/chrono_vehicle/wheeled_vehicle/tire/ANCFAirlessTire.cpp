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
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFAirlessTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

ANCFAirlessTire::ANCFAirlessTire(const std::string& name)
    : ChANCFTire(name),
      m_rim_radius(0.225),
      m_height(0.225),
      m_width(0.4),
      m_t_outer_ring(0.05),
      m_t_spoke(0.003),
      m_num_spoke(20),
      m_div_width(3),
      m_div_spoke_len(3),
      m_div_ring_per_spoke(3),
      m_E(76e9),
      m_nu(0.2),
      m_rho(800),
      m_alpha(0.05) {
    // default contact material
    m_mat = chrono_types::make_shared<ChContactMaterialSMC>();
}

void ANCFAirlessTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    // Create an isotropic material (shared by all elements)
    //  The default values approximate fiberglass
    //  Density and Young's modulus were taken from: https://en.wikipedia.org/wiki/Glass_fiber
    //  Poisson's ratio was taken from:
    //  https://matweb.com/search/DataSheet.aspx?MatGUID=d9c18047c49147a2a7c0b0bb1743e812
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(m_rho, m_E, m_nu);

    //-------------------------------------------------
    // Derived geometry
    //-------------------------------------------------
    double rOut = m_rim_radius + m_height;
    int numNodesWidth = m_div_width + 1;                    // Number of nodes across the width of the tire
    int numElsOutCir = m_num_spoke * m_div_ring_per_spoke;  // number of elements forming a circle in the outer ring
    int numElsOutCirTot = numElsOutCir * m_div_width;       // number of elements forming the complete outer ring
    int numNodesOutRingSlice = numElsOutCir;  // number of nodes in a slice normal to the width of the tire
    int numNodesOutRingTotal =
        numElsOutCir * (m_div_width + 1);  // total number of nodes in the outer band/ring of the tire

    double widthEl = m_width / m_div_width;         // width of each element
    double radiansElOuter = CH_2PI / numElsOutCir;  // arc spand by a single element in the outer ring
    double lenElOuter =
        2.0 * rOut * std::sin(0.5 * radiansElOuter);  // Corresponding length of each element in the outer ring

    double lenElSpk = (rOut - m_rim_radius) / m_div_spoke_len;
    int numNodesSpokeSlice = m_div_spoke_len;
    int numNodesSpokeTotal = numNodesSpokeSlice * (m_div_width + 1);
    int TElOffset = m_div_ring_per_spoke / 2 + m_div_ring_per_spoke % 2 - 1;
    double Toffset = (m_div_ring_per_spoke + 1) % 2;
    double radiansSpkOff = (0.5 + 0.5 * Toffset + TElOffset) * radiansElOuter;

    //-------------------------------------------------
    // Create the nodes for the outer ring.
    //-------------------------------------------------

    for (int i = 0; i < numNodesOutRingTotal; i++) {
        // Node location
        double loc_x = rOut * std::sin(radiansElOuter * (i % numElsOutCir));
        double loc_y = ((i / numElsOutCir) % numNodesWidth) * widthEl - 0.5 * m_width;
        double loc_z = rOut * std::cos(radiansElOuter * (i % numElsOutCir));
        ChVector3d loc = wheel_frame.TransformPointLocalToParent(ChVector3d(loc_x, loc_y, loc_z));

        // Node direction
        double dir_x = std::sin(radiansElOuter * (i % numElsOutCir));
        double dir_y = 0;
        double dir_z = std::cos(radiansElOuter * (i % numElsOutCir));
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
    for (int slice = 0; slice < m_div_width; slice++) {
        for (int i = 0; i < numElsOutCir; i++) {
            // Adjacent nodes
            int node3 = i + numNodesOutRingSlice * (1 + slice);
            int node2 = (i + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (1 + slice);
            int node1 = (i + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (0 + slice);
            int node0 = i + numNodesOutRingSlice * (0 + slice);

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node3)));

            // Set element dimensions
            element->SetDimensions(lenElOuter, widthEl);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(m_t_outer_ring, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(m_alpha);  // Structural damping for this element

            // Add element to mesh
            m_mesh->AddElement(element);
        }
    }

    //-------------------------------------------------
    // Create the spokes, one by one
    //-------------------------------------------------
    for (int spoke = 0; spoke < m_num_spoke; spoke++) {
        for (int i = 0; i < numNodesSpokeTotal; i++) {
            // Node location
            double loc_x = (rOut - lenElSpk * (1 + i % m_div_spoke_len)) *
                           std::sin(radiansSpkOff + radiansElOuter * m_div_ring_per_spoke * spoke);
            double loc_y = ((i / m_div_spoke_len) % numNodesWidth) * widthEl - 0.5 * m_width;
            double loc_z = (rOut - lenElSpk * (1 + i % m_div_spoke_len)) *
                           std::cos(radiansSpkOff + radiansElOuter * m_div_ring_per_spoke * spoke);
            ChVector3d loc = wheel_frame.TransformPointLocalToParent(ChVector3d(loc_x, loc_y, loc_z));

            // Node direction
            double dir_x = std::sin(radiansSpkOff + radiansElOuter * m_div_ring_per_spoke * spoke + CH_PI_2);
            double dir_y = 0;
            double dir_z = std::cos(radiansSpkOff + radiansElOuter * m_div_ring_per_spoke * spoke + CH_PI_2);
            ChVector3d dir = wheel_frame.TransformDirectionLocalToParent(ChVector3d(dir_x, dir_y, dir_z));

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
            node->SetMass(0);

            // Store the nodes on the hub
            if ((i % numNodesSpokeSlice) == (numNodesSpokeSlice - 1)) {
                m_hub_nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(node));
            }

            // Add node to mesh
            m_mesh->AddNode(node);
        }

        // The first element in the spoke is a "T" element
        for (int slice = 0; slice < m_div_width; slice++) {
            // Determine the connecting element:
            int ElIdTopSlice = TElOffset + spoke * m_div_ring_per_spoke;

            int nodeT0 = ElIdTopSlice + numNodesOutRingSlice * (0 + slice);
            int nodeT1 = (ElIdTopSlice + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (0 + slice);
            int nodeT2 = (ElIdTopSlice + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (1 + slice);
            int nodeT3 = ElIdTopSlice + numNodesOutRingSlice * (1 + slice);

            int nodeB = numNodesOutRingTotal + spoke * numNodesSpokeTotal + (slice + 0) * m_div_spoke_len;
            int nodeC = numNodesOutRingTotal + spoke * numNodesSpokeTotal + (slice + 1) * m_div_spoke_len;

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423T>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeB)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeC)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(nodeT3)));

            // Set element dimensions
            element->SetDimensions(lenElSpk, widthEl, lenElOuter, Toffset, CH_PI_2);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(m_t_spoke, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(m_alpha);  // Structural damping for this element

            // Add element to mesh
            m_mesh->AddElement(element);

            // The remaining elements are regular 3423 shell elements
            for (int i = 1; i < m_div_spoke_len; i++) {
                // Adjacent nodes
                int node0 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i - 1 + m_div_spoke_len * (0 + slice);
                int node1 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i + m_div_spoke_len * (0 + slice);
                int node2 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i + m_div_spoke_len * (1 + slice);
                int node3 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i - 1 + m_div_spoke_len * (1 + slice);

                // Create the element and set its nodes.
                auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node0)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node2)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_mesh->GetNode(node3)));

                // Set element dimensions
                element->SetDimensions(lenElOuter, widthEl);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(m_t_spoke, 0 * CH_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(m_alpha);  // Structural damping for this element

                // Add element to mesh
                m_mesh->AddElement(element);
            }
        }
    }
}

std::vector<std::shared_ptr<ChNodeFEAbase>> ANCFAirlessTire::GetConnectedNodes() const {
    return m_hub_nodes;
}

void ANCFAirlessTire::CreateContactMaterial() {
    m_contact_mat = m_mat;
}

}  // end namespace vehicle
}  // end namespace chrono
