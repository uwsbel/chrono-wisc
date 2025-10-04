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
// Authors: Milad Rakhsha, Radu Serban
// =============================================================================
//
// Demo on using ANCF shell elements
//
// =============================================================================
#include <algorithm>
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3423T.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/physics/ChLoadContainer.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;
double timestep = 0.001;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
/// Some functions for various motions of the wheels
class BaseFunction {
  protected:
    BaseFunction(double speed) : m_speed(speed) {}
    double calc(double t) const {
        double delay = 0.25;
        double ramp = 0.5;
        if (t <= delay)
            return 0;
        double tt = t - delay;
        if (tt >= ramp)
            return m_speed;
        return m_speed * tt / ramp;
    }
    double m_speed;
};
class RotSpeedFunction : public BaseFunction, public ChFunction {
  public:
    RotSpeedFunction(double slip, double speed, double radius) : BaseFunction(speed), m_slip(slip), m_radius(radius) {}
    virtual double GetVal(double t) const override {
        double v = calc(t);
        return (1 + m_slip) * v / m_radius;
    }
    virtual RotSpeedFunction* Clone() const override { return new RotSpeedFunction(*this); }

    double m_slip;
    double m_radius;
};
struct DelayedFun : public ChFunction {
    DelayedFun() : m_fun(nullptr), m_delay(0) {}
    DelayedFun(std::shared_ptr<ChFunction> fun, double delay) : m_fun(fun), m_delay(delay) {}
    virtual DelayedFun* Clone() const override { return new DelayedFun(); }
    virtual double GetVal(double x) const override {
        if (x < m_delay)
            return 0;
        return m_fun->GetVal(x - m_delay);
    }
    std::shared_ptr<ChFunction> m_fun;
    double m_delay;
};

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -1.62));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << " ANCF Shell Element Airless tire demo with implicit integration \n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";

    bool debugItems = false;

    //-------------------------------------------------
    // Define the geometry
    //-------------------------------------------------
    double rOut = 0.45;     // meters; Radius of the midsurface of the outer ring of elements
    double rIn = 0.225;     // meters; Radius of the rigid hub
    double width = 0.4;     // meters; Width of the Tire
    double tOut = 0.005;    // meters; thickness of the outer ring of elements
    double tSpoke = 0.003;  // meters; thickness of the spoke elements
    int numSpokes = 16;     // Number of spokes in the wheel

    int numElWidth = 3;        // Number of elements in the width direction
    int numElSpokeLen = 3;     // Number of elements along the length of the spoke
    int numElOutPerSpoke = 3;  // Number of elements along the length of the spoke

    double scale = 0.01;

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
    int numNodesWidth = numElWidth + 1;               // Number of nodes across the width of the tire
    int numElsOutCir = numSpokes * numElOutPerSpoke;  // number of elements forming a circle in the outer ring
    int numElsOutCirTot = numElsOutCir * numElWidth;  // number of elements forming the complete outer ring
    int numNodesOutRingSlice = numElsOutCir;          // number of nodes in a slice normal to the width of the tire
    int numNodesOutRingTotal =
        numElsOutCir * (numElWidth + 1);  // total number of nodes in the outer band/ring of the tire

    double widthEl = width / numElWidth;            // width of each element
    double radiansElOuter = CH_2PI / numElsOutCir;  // arc spand by a single element in the outer ring
    double lenElOuter =
        2.0 * rOut * std::sin(0.5 * radiansElOuter);  // Corresponding length of each element in the outer ring

    double lenElSpk = (rOut - rIn) / numElSpokeLen;
    int numNodesSpokeSlice = numElSpokeLen;
    int numNodesSpokeTotal = numNodesSpokeSlice * (numElWidth + 1);
    int TElOffset = numElOutPerSpoke / 2 + numElOutPerSpoke % 2 - 1;
    double Toffset = (numElOutPerSpoke + 1) % 2;
    double radiansSpkOff = (0.5 + 0.5 * Toffset + TElOffset) * radiansElOuter;

    std::cout << " Number of Nodes: " << numNodesOutRingTotal + numSpokes * numNodesSpokeTotal << "\n";
    std::cout << " Number of Elements: " << numElsOutCirTot + numSpokes * numElSpokeLen * numElWidth << "\n";
    std::cout << "-----------------------------------------------------------\n";

    //-------------------------------------------------
    // Create the rigid hub
    //-------------------------------------------------
    auto hub = chrono_types::make_shared<ChBody>();
    sys.AddBody(hub);
    hub->SetFixed(false);
    hub->EnableCollision(false);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(rIn, width);
    hub->AddVisualShape(cyl,
                        ChFrame<>(ChVector3d(0, 0, 0), ChQuaternion(std::cos(CH_PI_4), std::sin(CH_PI_4), 0.0, 0.0)));

    ChContactMaterialData mat_data;
    mat_data.mu = 0.9f;
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetPos(VNULL);
    floor->SetFixed(true);
    floor->EnableCollision(true);
    utils::AddBoxGeometry(floor.get(), mat_data.CreateMaterial(ChContactMethod::SMC),
                          ChVector3d(20 * rOut, 4 * rOut, 0.1), ChVector3d(-8 * rOut, 0, -rOut - 0.1));
    sys.AddBody(floor);

    // -------------------------------------------------
    // Create the chassis
    // -------------------------------------------------
    auto chassis_body = chrono_types::make_shared<ChBody>();
    sys.AddBody(chassis_body);
    chassis_body->SetName("rig_chassis");
    chassis_body->SetPos(ChVector3d(0, 0, 0));
    chassis_body->SetFixed(false);
    chassis_body->SetMass(125);
    // Slide along x axis
    auto slide = chrono_types::make_shared<ChLinkLockPrismatic>();
    slide->Initialize(floor, chassis_body, ChFrame<>(chassis_body->GetPos(), QuatFromAngleY(CH_PI_2)));
    sys.AddLink(slide);

    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(125);
    axle->SetPos(ChVector3d(0, 0, 0));
    axle->EnableCollision(false);
    axle->SetFixed(false);
    sys.AddBody(axle);

    // Axle can fall down along z axis
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis_body, axle, ChFrame<>(chassis_body->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sys.AddLink(prismatic2);

    // Connect the hub to the chassis using a motor
    ChQuaternion<> z2y;
    z2y.SetFromAngleX(-CH_PI_2);
    auto rot_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    sys.AddLink(rot_motor);
    rot_motor->Initialize(hub, axle, ChFrame<>(ChVector3d(0, 0, 0), z2y));
    // Give the motor a constant speed of 20 RPM
    std::shared_ptr<ChFunction> rs_fun =
        chrono_types::make_shared<ChFunctionConst>(-20 * CH_RPM_TO_RAD_S);  ///< angular speed function of time
    rot_motor->SetSpeedFunction(chrono_types::make_shared<DelayedFun>(rs_fun, 1));

    //-------------------------------------------------
    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    //-------------------------------------------------
    auto mesh = chrono_types::make_shared<ChMesh>();

    //-------------------------------------------------
    // Create the nodes for the outer ring.
    //-------------------------------------------------

    for (int i = 0; i < numNodesOutRingTotal; i++) {
        // Node location with offset
        double loc_x = rOut * std::sin(radiansElOuter * (i % numElsOutCir));
        double loc_y = ((i / numElsOutCir) % numNodesWidth) * widthEl - 0.5 * width;
        double loc_z = rOut * std::cos(radiansElOuter * (i % numElsOutCir));

        // Node direction
        double dir_x = std::sin(radiansElOuter * (i % numElsOutCir));
        double dir_y = 0;
        double dir_z = std::cos(radiansElOuter * (i % numElsOutCir));

        // Create the node
        auto node =
            chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(loc_x, loc_y, loc_z), ChVector3d(dir_x, dir_y, dir_z));

        node->SetMass(0);

        // Fix all nodes along the axis X=0
        // if (i % (numElsOutCir + 1) == 0)
        //    node->SetFixed(true);
        // node->SetFixed(true);
        // if (i == 0)
        //    node->SetFixed(true);

        // Add node to mesh
        mesh->AddNode(node);

        if (debugItems) {
            std::cout << "Node# " << i << ": (" << loc_x << ", " << loc_y << ", " << loc_z << ") (" << dir_x << ", "
                      << dir_y << ", " << dir_z << ")" << std::endl;

            auto sph_p = chrono_types::make_shared<ChVisualShapeSphere>(0.004);
            sph_p->SetColor(ChColor(0.6f, 0, 0));
            hub->AddVisualShape(
                sph_p,
                ChFrame<>(ChVector3d(loc_x + scale * dir_x, loc_y + scale * dir_y, loc_z + scale * dir_z), QUNIT));

            if (i == 0) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(
                    sph,
                    ChFrame<>(ChVector3d(loc_x + scale * dir_x, loc_y + scale * dir_y, loc_z + scale * dir_z), QUNIT));
            }

            if (i == 1) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.03);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(
                    sph,
                    ChFrame<>(ChVector3d(loc_x + scale * dir_x, loc_y + scale * dir_y, loc_z + scale * dir_z), QUNIT));
            }

            if (i == numNodesOutRingSlice + 1) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(
                    sph,
                    ChFrame<>(ChVector3d(loc_x + scale * dir_x, loc_y + scale * dir_y, loc_z + scale * dir_z), QUNIT));
            }

            if (i == numNodesOutRingSlice) {
                auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.01);
                sph->SetColor(ChColor(0, 0, 0.6f));
                hub->AddVisualShape(
                    sph,
                    ChFrame<>(ChVector3d(loc_x + scale * dir_x, loc_y + scale * dir_y, loc_z + scale * dir_z), QUNIT));
            }
        }
    }

    if (debugItems) {
        std::cout << std::endl << std::endl;
    }

    //-------------------------------------------------
    // Create the elements for the outer ring.
    //-------------------------------------------------
    for (int slice = 0; slice < numElWidth; slice++) {
        for (int i = 0; i < numElsOutCir; i++) {
            // Adjacent nodes
            int node3 = i + numNodesOutRingSlice * (1 + slice);
            int node2 = (i + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (1 + slice);
            int node1 = (i + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (0 + slice);
            int node0 = i + numNodesOutRingSlice * (0 + slice);

            if (debugItems) {
                std::cout << "El# " << i + slice * numElsOutCir << ": A=" << node0 << ", B=" << node1 << ", C=" << node2
                          << ", D=" << node3 << ")" << std::endl;
            }

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

            // Set element dimensions
            element->SetDimensions(lenElOuter, widthEl);

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
    // int spoke = 0;{
    for (int spoke = 0; spoke < numSpokes; spoke++) {
        if (debugItems) {
            std::cout << "Spoke # " << spoke << std::endl << std::endl;
        }

        for (int i = 0; i < numNodesSpokeTotal; i++) {
            // Node location with offset
            double loc_x = (rOut - lenElSpk * (1 + i % numElSpokeLen)) *
                           std::sin(radiansSpkOff + radiansElOuter * numElOutPerSpoke * spoke);
            double loc_y = ((i / numElSpokeLen) % numNodesWidth) * widthEl - 0.5 * width;
            double loc_z = (rOut - lenElSpk * (1 + i % numElSpokeLen)) *
                           std::cos(radiansSpkOff + radiansElOuter * numElOutPerSpoke * spoke);

            // Node direction
            double dir_x = std::sin(radiansSpkOff + radiansElOuter * numElOutPerSpoke * spoke + CH_PI_2);
            double dir_y = 0;
            double dir_z = std::cos(radiansSpkOff + radiansElOuter * numElOutPerSpoke * spoke + CH_PI_2);

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(loc_x, loc_y, loc_z),
                                                                 ChVector3d(dir_x, dir_y, dir_z));

            node->SetMass(0);

            //// Fix all nodes along the axis X=0
            // if (i % (numDiv_x + 1) == 0)
            //     node->SetFixed(true);
            // node->SetFixed(true);

            // Constrain the nodes on the hub to the hub
            if ((i % numNodesSpokeSlice) == (numNodesSpokeSlice - 1)) {
                auto constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
                constraintxyz->Initialize(node, hub);
                sys.Add(constraintxyz);

                auto constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
                constraintD->Initialize(node, hub);
                sys.Add(constraintD);
            }

            // Add node to mesh
            mesh->AddNode(node);

            if (debugItems) {
                std::cout << "Node# " << i + numNodesOutRingTotal + spoke * numNodesSpokeTotal << ": (" << loc_x << ", "
                          << loc_y << ", " << loc_z << ") (" << dir_x << ", " << dir_y << ", " << dir_z << ")"
                          << std::endl;

                auto sph_p = chrono_types::make_shared<ChVisualShapeSphere>(0.004);
                sph_p->SetColor(ChColor(0, 0.6f, 0));
                hub->AddVisualShape(
                    sph_p,
                    ChFrame<>(ChVector3d(loc_x + scale * dir_x, loc_y + scale * dir_y, loc_z + scale * dir_z), QUNIT));
            }
        }
        if (debugItems) {
            std::cout << std::endl << std::endl;
        }

        // The first element in the spoke is a "T" element
        for (int slice = 0; slice < numElWidth; slice++) {
            // Determine the connecting element:
            int ElIdTopSlice = TElOffset + spoke * numElOutPerSpoke;

            int nodeT0 = ElIdTopSlice + numNodesOutRingSlice * (0 + slice);
            int nodeT1 = (ElIdTopSlice + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (0 + slice);
            int nodeT2 = (ElIdTopSlice + 1) % numNodesOutRingSlice + numNodesOutRingSlice * (1 + slice);
            int nodeT3 = ElIdTopSlice + numNodesOutRingSlice * (1 + slice);

            int nodeB = numNodesOutRingTotal + spoke * numNodesSpokeTotal + (slice + 0) * numElSpokeLen;
            int nodeC = numNodesOutRingTotal + spoke * numNodesSpokeTotal + (slice + 1) * numElSpokeLen;

            if (debugItems) {
                std::cout << "El# " << numElsOutCirTot + slice * numElSpokeLen << ": N0=" << nodeT0 << ", N1=" << nodeT1
                          << ", B = " << nodeB << ", C = " << nodeC << ", N2=" << nodeT2 << ", N3=" << nodeT3 << ")"
                          << std::endl;

                auto nodeT0_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT0));
                auto nodeT1_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT1));
                auto nodeB_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeB));
                auto nodeC_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeC));
                auto nodeT2_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT2));
                auto nodeT3_xyzD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(nodeT3));

                std::cout << "T Check:" << std::endl;
                std::cout << "T edge 1 dir: " << (nodeT1_xyzD->GetPos() - nodeT0_xyzD->GetPos()) / lenElOuter
                          << std::endl;
                std::cout << "T B grad dir: " << nodeB_xyzD->GetSlope1() << std::endl;
                std::cout << "T edge 2 dir: " << (nodeT2_xyzD->GetPos() - nodeT3_xyzD->GetPos()) / lenElOuter
                          << std::endl;
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

            // Set element dimensions
            element->SetDimensions(lenElSpk, widthEl, lenElOuter, Toffset, CH_PI_2);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(alpha);  // Structural damping for this element

            // Add element to mesh
            mesh->AddElement(element);

            // The remaining elements are regular 3423 shell elements
            for (int i = 1; i < numElSpokeLen; i++) {
                // Adjacent nodes
                int node0 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i - 1 + numElSpokeLen * (0 + slice);
                int node1 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i + numElSpokeLen * (0 + slice);
                int node2 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i + numElSpokeLen * (1 + slice);
                int node3 = numNodesOutRingTotal + spoke * numNodesSpokeTotal + i - 1 + numElSpokeLen * (1 + slice);

                if (debugItems) {
                    std::cout << "El# " << i + numElsOutCirTot + slice * numElSpokeLen << ": A=" << node0
                              << ", B=" << node1 << ", C=" << node2 << ", D=" << node3 << ")" << std::endl;
                }

                // Create the element and set its nodes.
                auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

                // Set element dimensions
                element->SetDimensions(lenElOuter, widthEl);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(tSpoke, 0 * CH_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(0.0);  // Structural damping for this element

                // Add element to mesh
                mesh->AddElement(element);
            }
        }
    }
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(10e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);
    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
    mcontactsurf->AddAllNodes(*mesh);
    mesh->AddContactSurface(mcontactsurf);
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
#ifdef CHRONO_PARDISO_MKL
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>(8);
#else
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
#endif

    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Set up integrator
    auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    sys.SetTimestepper(stepper);

    stepper->SetAlpha(-0.2);
    stepper->SetMaxIters(50);
    stepper->SetAbsTolerances(1e-5);
    stepper->SetModifiedNewton(true);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
    }

    return 0;
}
