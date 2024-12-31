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
// Authors: Milad Rakhsha, Radu Serban, Michael Taylor
// =============================================================================
//
// Demo on using ANCF shell elements
//
// =============================================================================

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3423T.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/physics/ChLoadContainer.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -1.62));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << " ANCF Shell Element Airless tire demo with implicit integration \n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";

    bool debugItems = false;
    double debugScale = 0.02;

    //-------------------------------------------------
    // Define the geometry
    //-------------------------------------------------

    int numSpokes = 16;          // Number of spokes in the wheel
    int divElPerSpokeLen = 5;    // Number of elements along the length of the spoke, including the T element
    int divOuterElPerSpoke = 3;  // Number of elements along the circumference of the tire / numSpokes
    int divW = 2;                // Number of elements in the width direction

    double rOut = 0.45;     // meters; Radius of the midsurface of the outer ring of elements
    double rHub = 0.225;    // meters; Radius of the rigid hub
    double width = 0.4;     // meters; Width of the Tire
    double tOut = 0.005;    // meters; thickness of the outer ring of elements
    double tSpoke = 0.003;  // meters; thickness of the spoke elements
    double spokeTFrac =
        1.0 / divElPerSpokeLen;  // Faction of the spoke modeled by the Shell 3423 T element (0 to 1 exclusion)
    double angRelHub =
        20.0 * CH_DEG_TO_RAD;  // relative rotation of the hub with respect to the outer ring ranging from -\pi to \pi
    double spokeCurPntX = 0.5 * (rOut - rHub);  // X coordinate in the local spoke coordinate system for the
                                                // third point on a circle defining the arc shape of the spoke.
    double spokeCurPntZ =
        0.25 * (rOut - rHub);  // Z coordinate in the local spoke coordinate system for the third point on a circle
                               // defining the arc shape of the spoke. A value of 0 means a straight spoke.

    bool midElSpokes = false;  // Spokes intersect in the middle of an outer ring element.  If false, it intersects at
                               // the starting edge (-u edge) of the outer ring element

    //-------------------------------------------------
    // Define the mesh material properties
    //-------------------------------------------------
    // Create an orthotropic material.
    // All layers for all elements share the same material.
    //  Approximate fiberglass
    //  Density and Young's modulus were taken from: https://en.wikipedia.org/wiki/Glass_fiber
    //  Poisson's ratio was taken from:
    //  https://matweb.com/search/DataSheet.aspx?MatGUID=d9c18047c49147a2a7c0b0bb1743e812
    double rho = 2580;              // kg/m^3
    double E = 76e9;                // Pa
    double nu = 0.2;                // Poisson's ratio
    double G = E / (2 * (1 + nu));  // Shear Modulus Pa
    ChVector3d Evec(E, E, E);
    ChVector3d nuVec(nu, nu, nu);
    ChVector3d Gvec(G, G, G);
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, Evec, nuVec, Gvec);
    double alpha = 0.02;  // Damping

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
    int numElsOutCirTot = numElsOutCir * divW;          // number of elements forming the complete outer ring

    std::cout << " Number of Nodes: " << numNodesOuterRingTotal + numSpokes * numNodesPerSpoke << "\n";
    std::cout << " Number of Elements: " << numElsOutCirTot + numSpokes * divElPerSpokeLen * divW << "\n";
    std::cout << "-----------------------------------------------------------\n";

    //-------------------------------------------------
    // Create the rigid hub
    //-------------------------------------------------
    auto hub = chrono_types::make_shared<ChBody>();
    sys.AddBody(hub);
    hub->SetFixed(true);
    hub->EnableCollision(false);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(rHub, width);
    hub->AddVisualShape(cyl,
                        ChFrame<>(ChVector3d(0, 0, 0), ChQuaternion(std::cos(CH_PI_4), std::sin(CH_PI_4), 0.0, 0.0)));

    //-------------------------------------------------
    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    //-------------------------------------------------
    auto mesh = chrono_types::make_shared<ChMesh>();

    //-------------------------------------------------
    // Create the nodes for the outer ring.
    //-------------------------------------------------

    for (int i = 0; i < numNodesOuterRingTotal; i++) {
        // Node location
        double loc_x = rOut * std::sin(i * angOuterRingEl);
        double loc_y = widthEl * (i / numNodesOuterRingSlice) - 0.5 * width;
        double loc_z = rOut * std::cos(i * angOuterRingEl);

        // Node direction
        double dir_x = std::sin(i * angOuterRingEl);
        double dir_y = 0;
        double dir_z = std::cos(i * angOuterRingEl);

        // Create the node
        auto node =
            chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(loc_x, loc_y, loc_z), ChVector3d(dir_x, dir_y, dir_z));

        node->SetMass(0);

        // node->SetFixed(true);

        // Add node to mesh
        mesh->AddNode(node);

        if (debugItems) {
            std::cout << "Node# " << i << ": (" << loc_x << ", " << loc_y << ", " << loc_z << ") (" << dir_x << ", "
                      << dir_y << ", " << dir_z << ")" << std::endl;

            auto sph_p = chrono_types::make_shared<ChVisualShapeSphere>(0.004);
            sph_p->SetColor(ChColor(0.6f, 0, 0));
            hub->AddVisualShape(sph_p, ChFrame<>(ChVector3d(loc_x + debugScale * dir_x, loc_y + debugScale * dir_y,
                                                            loc_z + debugScale * dir_z),
                                                 QUNIT));

            if (i == 0) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(sph, ChFrame<>(ChVector3d(loc_x, loc_y, loc_z), QUNIT));
            }

            if (i == 1) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.03);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(sph, ChFrame<>(ChVector3d(loc_x, loc_y, loc_z), QUNIT));
            }

            if (i == numNodesOuterRingSlice + 1) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(sph, ChFrame<>(ChVector3d(loc_x, loc_y, loc_z), QUNIT));
            }

            if (i == numNodesOuterRingSlice) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.01);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(sph, ChFrame<>(ChVector3d(loc_x, loc_y, loc_z), QUNIT));
            }
        }
    }

    if (debugItems) {
        std::cout << std::endl << std::endl;
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

            if (debugItems) {
                std::cout << "El# " << i + slice * numElsOutCir << ": A=" << nodeA << ", B=" << nodeB << ", C=" << nodeC
                          << ", D=" << nodeD << ")" << std::endl;
            }

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeA)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeD)));

            // Set element dimensions
            element->SetDimensions(lenOuterRingEl, widthEl);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(tOut, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(alpha);  // Structural damping for this element

            // Add element to mesh
            mesh->AddElement(element);
        }
    }
    if (debugItems) {
        std::cout << std::endl << std::endl;
    }

    //-------------------------------------------------
    // Create the spokes, one by one
    //-------------------------------------------------
    if (midElSpokes) {
        angRelHub +=
            0.5 * angOuterRingEl;  // Rotate the hub connection to account for the spoke intersecting in the middle of
                                   // an outer ring element rather than its starting edge (NodeA & D).
    }

    // int spoke = 0; {
    for (int spoke = 0; spoke < numSpokes; spoke++) {
        if (debugItems) {
            std::cout << "Spoke # " << spoke << std::endl << std::endl;
        }

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

                    if (debugItems) {
                        std::cout << "x0: " << x0 << "   z0: " << z0 << "   r0: " << r0 << std::endl;
                        std::cout << "angSpokeArcStart: " << angSpokeArcStart * CH_RAD_TO_DEG
                                  << "   angSpokeArcEnd: " << angSpokeArcEnd * CH_RAD_TO_DEG
                                  << "   angSpokeArc: " << angSpokeArc * CH_RAD_TO_DEG << std::endl;
                    }
                }

                // Create the node
                auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
                node->SetMass(0);
                // node->SetFixed(true);

                // Add node to mesh
                mesh->AddNode(node);

                if (k == divElPerSpokeLen - 1) {
                    auto constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
                    constraintxyz->Initialize(node, hub);
                    sys.Add(constraintxyz);

                    auto constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
                    constraintD->Initialize(node, hub);
                    sys.Add(constraintD);
                }

                if (debugItems) {
                    std::cout << "Node# "
                              << numNodesOuterRingTotal + spoke * numNodesPerSpoke + slice * divElPerSpokeLen + k
                              << ": (" << loc.x() << ", " << loc.y() << ", " << loc.z() << ") (" << dir.x() << ", "
                              << dir.y() << ", " << dir.z() << ")" << std::endl;

                    auto sph_p = chrono_types::make_shared<ChVisualShapeSphere>(0.004);
                    sph_p->SetColor(ChColor(0, 0.6f, 0));
                    hub->AddVisualShape(sph_p, ChFrame<>(loc + debugScale * dir, QUNIT));
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

            if (debugItems) {
                std::cout << "El# " << numElsOutCirTot + slice * divElPerSpokeLen << ": N0=" << nodeT0
                          << ", N1=" << nodeT1 << ", B = " << nodeB << ", C = " << nodeC << ", N2=" << nodeT2
                          << ", N3=" << nodeT3 << ")" << std::endl;

                auto nodeT0_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT0));
                auto nodeT1_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT1));
                auto nodeB_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB));
                auto nodeC_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC));
                auto nodeT2_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT2));
                auto nodeT3_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT3));

                auto edge1 = nodeT1_xyzD->GetPos() - nodeT0_xyzD->GetPos();
                auto edge2 = nodeT2_xyzD->GetPos() - nodeT3_xyzD->GetPos();
                std::cout << "T Check:" << std::endl;
                std::cout << "T edge 1 dir: " << edge1.GetNormalized() << std::endl;
                std::cout << "T B grad dir: " << nodeB_xyzD->GetSlope1() << std::endl;
                std::cout << "T edge 2 dir: " << edge2.GetNormalized() << std::endl;
                std::cout << "T C grad dir: " << nodeB_xyzD->GetSlope1() << std::endl;
            }

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423T>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT3)));

            // Calculate the intersection angle
            auto nodeT0_loc = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT0))->GetPos();
            auto nodeT1_loc = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT1))->GetPos();
            auto nodeB_loc = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB))->GetPos();
            auto OuterEdge = nodeT1_loc - nodeT0_loc;

            if (midElSpokes) {
                auto origin = 0.5 * nodeT0_loc + 0.5 * nodeT1_loc;
                auto vOut = nodeT1_loc - origin;
                auto vEl = nodeB_loc - origin;
                vOut.Normalize();
                vEl.Normalize();
                double angT = std::acos(vOut.Dot(vEl));
                auto edgeElSpk = nodeB_loc - origin;

                element->SetDimensions(edgeElSpk.Length(), widthEl, OuterEdge.Length(), 0, angT);
                if (debugItems) {
                    std::cout << "angT: " << angT << " rad = " << angT * CH_RAD_TO_DEG << "deg \n";
                }
            } else {
                auto origin = nodeT0_loc;
                auto vOut = nodeT1_loc - origin;
                auto vEl = nodeB_loc - origin;
                vOut.Normalize();
                vEl.Normalize();
                double angT = std::acos(vOut.Dot(vEl));
                auto edgeElSpk = nodeB_loc - origin;

                element->SetDimensions(edgeElSpk.Length(), widthEl, OuterEdge.Length(), -1, angT);
                if (debugItems) {
                    std::cout << "angT: " << angT << " rad = " << angT * CH_RAD_TO_DEG << "deg \n";
                }
            }

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(alpha);  // Structural damping for this element

            // Add element to mesh
            mesh->AddElement(element);

            // The remaining elements are regular 3423 shell elements
            for (int i = 1; i < divElPerSpokeLen; i++) {
                // Adjacent nodes
                int node0 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (0 + slice);
                int node1 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (0 + slice);
                int node2 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i + divElPerSpokeLen * (1 + slice);
                int node3 = numNodesOuterRingTotal + spoke * numNodesPerSpoke + i - 1 + divElPerSpokeLen * (1 + slice);

                if (debugItems) {
                    std::cout << "El# " << i + numElsOutCirTot + slice * divElPerSpokeLen << ": A=" << node0
                              << ", B=" << node1 << ", C=" << node2 << ", D=" << node3 << ")" << std::endl;
                }

                // Create the element and set its nodes.
                auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

                auto edgeElSpk = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1))->GetPos() -
                                 std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0))->GetPos();

                // Set element dimensions
                element->SetDimensions(edgeElSpk.Length(), widthEl);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(alpha);  // Structural damping for this element

                // Add element to mesh
                mesh->AddElement(element);
            }
        }
    }

    //-------------------------------------------------
    // Add the mesh to the system
    //-------------------------------------------------
    sys.Add(mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto visualizemeshA = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizemeshA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    visualizemeshA->SetColorscaleMinMax(0.0, 5.50);
    visualizemeshA->SetShrinkElements(true, 0.85);
    visualizemeshA->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(visualizemeshA);

    auto visualizemeshB = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizemeshB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    visualizemeshB->SetWireframe(true);
    visualizemeshB->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(visualizemeshB);

    auto visualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    visualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshC->SetSymbolsThickness(0.004);
    mesh->AddVisualShapeFEA(visualizemeshC);

    auto visualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>();
    visualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    visualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshD->SetSymbolsScale(1);
    visualizemeshD->SetColorscaleMinMax(-0.5, 5);
    visualizemeshD->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(visualizemeshD);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "ANCF Shell 3423 Airless Tire",
                                         ChVector3d(0.2, -1.0, 0.2), ChVector3d(0.0, 0.0, 0.0));

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Set up integrator
    auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    sys.SetTimestepper(stepper);
    // Alternative way of changing the integrator:
    ////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    ////auto stepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());

    stepper->SetAlpha(-0.2);
    stepper->SetMaxIters(50);
    stepper->SetAbsTolerances(1e-5);
    // stepper->SetStepControl(true);
    // stepper->SetMinStepSize(1e-4);
    stepper->SetModifiedNewton(true);
    ////stepper->SetVerbose(true);

    // Simulation loop
    double time = 0;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.001);
        // std::cout << "Simulation Time: " << time << "s \n";
        time += 0.001;
    }

    return 0;
}
