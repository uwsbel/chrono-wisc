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
// ANCF toroidal tire.
// This is a concrete ANCF tire class which uses a semi-toroidal tire mesh
// composed of single-layer 4-node ANCF 3443 shell elements.
//
// =============================================================================

#include <cmath>

#include "chrono/fea/ChElementShellANCF_3443.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire3443.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

ANCFToroidalTire3443::ANCFToroidalTire3443(const std::string& name)
    : ChANCFTire(name),
      m_rim_radius(0.35),
      m_height(0.195),
      m_thickness(0.014),
      m_div_circumference(60),
      m_div_width(12),
      m_default_pressure(320.0e3),
      m_alpha(0.15) {
    // default contact material
    m_mat = chrono_types::make_shared<ChContactMaterialSMC>();
}

void ANCFToroidalTire3443::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    // Create an isotropic material (shared by all elements)
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(500, 9.0e7, 0.3);

    // Create the mesh nodes.
    // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
    // and are then transformed to the global frame.
    for (int i = 0; i < m_div_circumference; i++) {
        double phi = (CH_2PI * i) / m_div_circumference;

        for (int j = 0; j <= m_div_width; j++) {
            double theta = -CH_PI_2 + (CH_PI * j) / m_div_width;

            double x = (m_rim_radius + m_height * std::cos(theta)) * std::cos(phi);
            double y = m_height * std::sin(theta);
            double z = (m_rim_radius + m_height * std::cos(theta)) * std::sin(phi);
            ChVector3d loc = wheel_frame.TransformPointLocalToParent(ChVector3d(x, y, z));

            double dr_dw_x = std::cos(theta) * std::cos(phi);
            double dr_dw_y = std::sin(theta);
            double dr_dw_z = std::cos(theta) * std::sin(phi);
            ChVector3d loc_dr_dw = ChVector3d(dr_dw_x, dr_dw_y, dr_dw_z);

            double dr_dv_x = -std::sin(theta) * std::cos(phi);
            double dr_dv_y = std::cos(theta);
            double dr_dv_z = -std::sin(theta) * std::sin(phi);
            ChVector3d loc_dr_dv = ChVector3d(dr_dv_x, dr_dv_y, dr_dv_z);

            ChVector3d loc_dr_du = loc_dr_dv.Cross(loc_dr_dw);



            ChVector3d dr_du = wheel_frame.TransformDirectionLocalToParent(loc_dr_du);
            ChVector3d dr_dv = wheel_frame.TransformDirectionLocalToParent(loc_dr_dv);
            ChVector3d dr_dw = wheel_frame.TransformDirectionLocalToParent(loc_dr_dw);

            auto node = chrono_types::make_shared<ChNodeFEAxyzDDD>(loc, dr_du, dr_dv, dr_dw);
            node->SetMass(0);
            m_mesh->AddNode(node);
        }
    }

    // Element thickness
    double dz = m_thickness;

    // Create the ANCF shell elements
    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j < m_div_width; j++) {
            // Adjacent nodes
            int inode0, inode1, inode2, inode3;
            inode1 = j + i * (m_div_width + 1);
            inode2 = j + 1 + i * (m_div_width + 1);
            if (i == m_div_circumference - 1) {
                inode0 = j;
                inode3 = j + 1;
            } else {
                inode0 = j + (i + 1) * (m_div_width + 1);
                inode3 = j + 1 + (i + 1) * (m_div_width + 1);
            }

            auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(inode0));
            auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(inode1));
            auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(inode2));
            auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(m_mesh->GetNode(inode3));

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3443>();
            element->SetNodes(node0, node1, node2, node3);

            // Element dimensions
            double dx =
                0.5 * ((node1->GetPos() - node0->GetPos()).Length() + (node3->GetPos() - node2->GetPos()).Length());
            double dy =
                0.5 * ((node2->GetPos() - node1->GetPos()).Length() + (node3->GetPos() - node0->GetPos()).Length());

            // Set element dimensions
            element->SetDimensions(dx, dy);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(m_alpha);

            // Add element to mesh
            m_mesh->AddElement(element);

            ////Check the position vector gradients for a specific element to make sure the nodal gradient calculations are correct:
            //if ((i == 2) && (j == 2)) {
            //    ChVector3d el_x1 = node1->GetPos() - node0->GetPos();
            //    ChVector3d el_x2 = node2->GetPos() - node3->GetPos();
            //    ChVector3d el_y1 = node3->GetPos() - node0->GetPos();
            //    ChVector3d el_y2 = node2->GetPos() - node1->GetPos();
            //    ChVector3d el_z1 = el_x1.Cross(el_y1);
            //    ChVector3d el_z2 = el_x2.Cross(el_y2);

            //    el_x1.Normalize();
            //    el_x2.Normalize();
            //    el_y1.Normalize();
            //    el_y2.Normalize();
            //    el_z1.Normalize();
            //    el_z2.Normalize();

            //    std::cout << std::endl;
            //    std::cout << "X1: " << el_x1 << ",  X2:" << el_x2 << std::endl;
            //    std::cout << "Node0: " << node0->GetSlope1() <<std::endl;
            //    std::cout << "Node1: " << node1->GetSlope1() << std::endl;
            //    std::cout << "Node2: " << node2->GetSlope1() << std::endl;
            //    std::cout << "Node3: " << node3->GetSlope1() << std::endl;

            //    std::cout << std::endl;
            //    std::cout << "Y1: " << el_y1 << ",  Y2:" << el_y2 << std::endl;
            //    std::cout << "Node0: " << node0->GetSlope2() << std::endl;
            //    std::cout << "Node1: " << node1->GetSlope2() << std::endl;
            //    std::cout << "Node2: " << node2->GetSlope2() << std::endl;
            //    std::cout << "Node3: " << node3->GetSlope2() << std::endl;

            //    std::cout << std::endl;
            //    std::cout << "Z1: " << el_z1 << ",  Z2:" << el_z2 << std::endl;
            //    std::cout << "Node0: " << node0->GetSlope3() << std::endl;
            //    std::cout << "Node1: " << node1->GetSlope3() << std::endl;
            //    std::cout << "Node2: " << node2->GetSlope3() << std::endl;
            //    std::cout << "Node3: " << node3->GetSlope3() << std::endl;

            //    std::cout << std::endl;
            //}
        }
    }
}

std::vector<std::shared_ptr<ChNodeFEAbase>> ANCFToroidalTire3443::GetConnectedNodes() const {
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> nodes;

    for (int i = 0; i < m_div_circumference; i++) {
        for (int j = 0; j <= m_div_width; j++) {
            int index = j + i * (m_div_width + 1);
            if (index % (m_div_width + 1) == 0) {
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(m_mesh->GetNode(index)));
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(m_mesh->GetNode(index + m_div_width)));
            }
        }
    }

    return nodes;
}

void ANCFToroidalTire3443::CreateContactMaterial() {
    m_contact_mat = m_mat;
}

}  // end namespace vehicle
}  // end namespace chrono
