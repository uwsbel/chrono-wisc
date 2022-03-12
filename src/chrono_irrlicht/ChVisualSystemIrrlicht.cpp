// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#include <codecvt>
#include <locale>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChProfiler.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChBarrelShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#ifdef CHRONO_MODAL
    #include "chrono_modal/ChModalAssembly.h"
#endif

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
#include "chrono_irrlicht/ChIrrCamera.h"
#include "chrono_irrlicht/ChIrrSkyBoxSceneNode.h"

namespace chrono {
namespace irrlicht {

using namespace irr;
using namespace irr::scene;

static std::shared_ptr<irr::video::SMaterial> default_material;

ChVisualSystemIrrlicht::ChVisualSystemIrrlicht()
    : m_device_params(irr::SIrrlichtCreationParameters()),
      m_device(nullptr),
      m_container(nullptr),
      m_win_title(""),
      m_yup(true),
      m_use_effects(false),
      m_symbol_scale(1) {
    // Set default device parameter values
    m_device_params.AntiAlias = true;
    m_device_params.Bits = 32;
    m_device_params.Fullscreen = false;
    m_device_params.DriverType = irr::video::EDT_DIRECT3D9;
    m_device_params.WindowSize = irr::core::dimension2d<irr::u32>(640, 480);
    m_device_params.Stencilbuffer = false;
    m_device_params.LoggingLevel = irr::ELL_INFORMATION;

    // Create shared meshes
    sphereMesh = createEllipticalMesh(1.0, 1.0, -2, +2, 0, 15, 8);
    cubeMesh = createCubeMesh(core::vector3df(2, 2, 2));  // -/+ 1 unit each xyz axis
    cylinderMesh = createCylinderMesh(1, 1, 32);
    capsuleMesh = createCapsuleMesh(1, 1, 32, 32);

    // if (sphereMesh)
    //  sphereMesh->grab();
    if (cubeMesh)
        cubeMesh->grab();
    if (cylinderMesh)
        cylinderMesh->grab();
    if (capsuleMesh)
        capsuleMesh->grab();
}

ChVisualSystemIrrlicht::~ChVisualSystemIrrlicht() {
    if (sphereMesh)
        sphereMesh->drop();
    if (cubeMesh)
        cubeMesh->drop();
    if (cylinderMesh)
        cylinderMesh->drop();
    if (capsuleMesh)
        capsuleMesh->drop();

    if (m_device)
        m_device->drop();
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::SetAntialias(bool val) {
    m_device_params.AntiAlias = val;
}
void ChVisualSystemIrrlicht::SetFullscreen(bool val) {
    m_device_params.Fullscreen = val;
}
void ChVisualSystemIrrlicht::SetShadows(bool val) {
    m_device_params.Stencilbuffer = val;
}
void ChVisualSystemIrrlicht::SetDriverType(irr::video::E_DRIVER_TYPE driver_type) {
    m_device_params.DriverType = driver_type;
}
void ChVisualSystemIrrlicht::SetWindowSize(const ChVector2<int>& win_size) {
    m_device_params.WindowSize = irr::core::dimension2d<irr::u32>((u32)win_size.x(), (u32)win_size.y());
}
void ChVisualSystemIrrlicht::SetWindowTitle(const std::string& win_title) {
    m_win_title = win_title;
}
void ChVisualSystemIrrlicht::SetLogLevel(irr::ELOG_LEVEL log_level) {
    m_device_params.LoggingLevel = log_level;
}
void ChVisualSystemIrrlicht::SetCameraVertical(CameraVerticalDir vert) {
    m_yup = (vert == CameraVerticalDir::Y);
}
void ChVisualSystemIrrlicht::SetSymbolScale(double scale) {
    m_symbol_scale = scale;
    if (m_gui)
        m_gui->SetSymbolscale(scale);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::Initialize() {
    if (m_device)
        return;

    // Create Irrlicht device using current parameter values.
    m_device = irr::createDeviceEx(m_device_params);
    if (!m_device) {
        std::cerr << "Cannot use default video driver - fall back to OpenGL" << std::endl;
        m_device_params.DriverType = irr::video::EDT_OPENGL;
        m_device = irr::createDeviceEx(m_device_params);
        if (!m_device) {
            std::cerr << "Failed to create the video driver - giving up" << std::endl;
            return;
        }
    }

    m_device->grab();

    std::wstring title = std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_win_title);
    m_device->setWindowCaption(title.c_str());

    // Xeffects for shadow maps!
    if (m_device_params.AntiAlias)
        m_effect_handler = std::unique_ptr<EffectHandler>(
            new EffectHandler(m_device, GetVideoDriver()->getScreenSize() * 2, true, false, true));
    else
        m_effect_handler = std::unique_ptr<EffectHandler>(
            new EffectHandler(m_device, GetVideoDriver()->getScreenSize(), true, false, true));
    m_effect_handler->setAmbientColor(irr::video::SColor(255, 122, 122, 122));
    m_use_effects = false;  // will be true as sson as a light with shadow is added

    // Create the container Irrlicht scene node
    m_container = GetSceneManager()->addEmptySceneNode();

    // Create default Irrlicht material
    if (!default_material) {
        auto irr_mat = tools::ToIrrlichtMaterial(ChVisualMaterial::Default(), GetVideoDriver());
        default_material = std::make_shared<irr::video::SMaterial>(irr_mat);
    }

    // If the visualization system is already attached to a ChSystem
    if (m_system) {
        // Create an Irrlicht GUI
        assert(!m_gui);
        CreateGUI();

        // Parse the mechanical assembly and create a ChIrrNodeModel for each physics item with a visual model.
        // This is a recursive call to accomodate any existing sub-assemblies.
        BindAll();
    }
}

// -----------------------------------------------------------------------------

bool ChVisualSystemIrrlicht::Run() {
    return m_device->run();
}

void ChVisualSystemIrrlicht::OnAttach() {
    // If the visualization system is already initialized
    if (m_device) {
        assert(!m_gui);
        CreateGUI();

        // Parse the mechanical assembly and create a ChIrrNodeModel for each physics item with a visual model.
        // This is a recursive call to accomodate any existing sub-assemblies.
        BindAll();
    }
}

void ChVisualSystemIrrlicht::OnSetup() {
    // Remove Irrlicht nodes associated with a deleted physics item
    std::vector<ChPhysicsItem*> items_to_remove;
    for (auto& node : m_nodes) {
        if (node.second->GetPhysicsItem().expired()) {
            node.second->removeAll();
            node.second->remove();
            items_to_remove.emplace_back(node.first);
        }
    }
    for (auto&& item : items_to_remove)
        m_nodes.erase(item);
}

void ChVisualSystemIrrlicht::OnUpdate() {
    for (auto& node : m_nodes) {
        node.second->UpdateChildren();
    }
}

// -----------------------------------------------------------------------------

// Called from Initialize() or from OnAttach(), whichever is invoked last.
void ChVisualSystemIrrlicht::CreateGUI() {
    assert(m_device);
    assert(m_system);

    m_gui = std::unique_ptr<ChIrrGUI>(new ChIrrGUI(m_device, m_system));

    m_gui->SetSymbolscale(m_symbol_scale);
    for (auto recv : m_user_receivers)
        m_gui->AddUserEventReceiver(recv);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::AddLogo(const std::string& logo_filename) {
    if (!m_device)
        return;

    GetGUIEnvironment()->addImage(GetVideoDriver()->getTexture(logo_filename.c_str()),
                                  irr::core::position2d<irr::s32>(10, 10));
}

void ChVisualSystemIrrlicht::AddCamera(const ChVector<>& pos, ChVector<> targ) {
    if (!m_device)
        return;

    // create and init camera
    RTSCamera* camera =
        new RTSCamera(m_device, GetSceneManager()->getRootSceneNode(), GetSceneManager(), -1, -160.0f, 1.0f, 0.003f);

    // camera->bindTargetAndRotation(true);
    if (!m_yup)
        camera->setZUp();
    camera->setPosition(irr::core::vector3dfCH(pos));
    camera->setTarget(irr::core::vector3dfCH(targ));

    camera->setNearValue(0.1f);
    camera->setMinZoom(0.6f);
}

void ChVisualSystemIrrlicht::AddTypicalLights() {
    if (!m_device)
        return;

    if (m_yup) {
        AddLight(ChVector<>(30, 80, +30), 280, ChColor(0.7f, 0.7f, 0.7f, 1.0f));
        AddLight(ChVector<>(30, 80, -30), 280, ChColor(0.7f, 0.7f, 0.7f, 1.0f));
    } else {
        AddLight(ChVector<>(30, +30, 80), 280, ChColor(0.7f, 0.7f, 0.7f, 1.0f));
        AddLight(ChVector<>(30, -30, 80), 280, ChColor(0.7f, 0.7f, 0.7f, 1.0f));
    }
}

void ChVisualSystemIrrlicht::AddSkyBox(const std::string& texture_dir) {
    if (!m_device)
        return;

    // create sky box
    std::string str_lf = texture_dir + "sky_lf.jpg";
    std::string str_up = texture_dir + "sky_up.jpg";
    std::string str_dn = texture_dir + "sky_dn.jpg";

    irr::video::ITexture* map_skybox_side = GetVideoDriver()->getTexture(str_lf.c_str());

    // Create a skybox scene node
    auto skybox = new irr::scene::CSkyBoxSceneNode(GetVideoDriver()->getTexture(str_up.c_str()),
                                                   GetVideoDriver()->getTexture(str_dn.c_str()), map_skybox_side,
                                                   map_skybox_side, map_skybox_side, map_skybox_side,
                                                   GetSceneManager()->getRootSceneNode(), GetSceneManager(), -1);
    skybox->drop();

    if (!m_yup)
        skybox->setRotation(irr::core::vector3df(90, 0, 0));
}

irr::scene::ILightSceneNode* ChVisualSystemIrrlicht::AddLight(const ChVector<>& pos, double radius, ChColor color) {
    if (!m_device)
        return nullptr;

    irr::scene::ILightSceneNode* mlight = GetSceneManager()->addLightSceneNode(
        0, irr::core::vector3dfCH(pos), tools::ToIrrlichtSColorf(color), (irr::f32)radius);
    return mlight;
}

irr::scene::ILightSceneNode* ChVisualSystemIrrlicht::AddLightWithShadow(const ChVector<>& pos,
                                                                        const ChVector<>& aim,
                                                                        double radius,
                                                                        double near_value,
                                                                        double far_value,
                                                                        double angle,
                                                                        unsigned int resolution,
                                                                        ChColor color,
                                                                        bool directional,
                                                                        bool clipborder) {
    if (!m_device)
        return nullptr;

    irr::scene::ILightSceneNode* mlight = GetSceneManager()->addLightSceneNode(
        0, irr::core::vector3dfCH(pos), tools::ToIrrlichtSColorf(color), (irr::f32)radius);

    m_effect_handler->addShadowLight(SShadowLight(
        (irr::u32)resolution, irr::core::vector3dfCH(pos), irr::core::vector3dfCH(aim), tools::ToIrrlichtSColorf(color),
        (irr::f32)near_value, (irr::f32)far_value, (irr::f32)angle * irr::core::DEGTORAD, directional));
    
    if (clipborder == false) {
        m_effect_handler->getShadowLight(m_effect_handler->getShadowLightCount() - 1).setClipBorder(clipborder);
    }

    m_use_effects = true;
    return mlight;
}

void ChVisualSystemIrrlicht::AddUserEventReceiver(irr::IEventReceiver* receiver) {
    if (m_gui)
        m_gui->AddUserEventReceiver(receiver);
    else
        m_user_receivers.push_back(receiver);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::EnableShadows(std::shared_ptr<ChPhysicsItem> item) {
    if (!m_device) {
        std::cerr << "EnableShadows - visualization system not initialized" << std::endl;
        return;
    }

    if (!m_system) {
        std::cerr << "EnableShadows - visualization system not attached to a ChSystem" << std::endl;
        return;
    }

    if (item) {
        auto node = m_nodes.find(item.get());
        if (node != m_nodes.end())
            AddShadowToIrrNode(node->second.get());
    } else {
        for (auto& body : m_system->Get_bodylist()) {
            EnableShadows(body);
        }
        for (auto& link : m_system->Get_linklist()) {
            EnableShadows(link);
        }
        for (auto& mesh : m_system->Get_meshlist()) {
            EnableShadows(mesh);
        }
        for (auto& ph : m_system->Get_otherphysicslist()) {
            EnableShadows(ph);
        }
    }
}

void ChVisualSystemIrrlicht::AddShadowToIrrNode(scene::ISceneNode* node) {
    scene::ISceneNodeList::ConstIterator it = node->getChildren().begin();
    for (; it != node->getChildren().end(); ++it) {
        AddShadowToIrrNode(*it);
    }

    // Add shadow only to leaves
    if (node->getChildren().getSize() == 0)
        m_effect_handler->addShadowToNode(node);
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::EnableContactDrawing(IrrContactsDrawMode mode) {
    if (!m_device)
        return;
    m_gui->SetContactsDrawMode(mode);
}

void ChVisualSystemIrrlicht::EnableLinkDrawing(IrrLinkDrawMode mode) {
    if (!m_device)
        return;
    m_gui->SetLinksDrawMode(mode);
}

void ChVisualSystemIrrlicht::EnableBodyFrameDrawing(bool val) {
    if (!m_device)
        return;
    m_gui->SetPlotCOGFrames(val);
}

void ChVisualSystemIrrlicht::EnableCollisionShapeDrawing(bool val) {
    if (!m_device)
        return;
    m_gui->SetPlotCollisionShapes(val);
}

// -----------------------------------------------------------------------------

// Clean canvas at beginning of scene.
void ChVisualSystemIrrlicht::BeginScene(bool backBuffer, bool zBuffer, ChColor color) {
    utils::ChProfileManager::Reset();
    utils::ChProfileManager::Start_Profile("Irrlicht loop");
    utils::ChProfileManager::Increment_Frame_Counter();

    GetVideoDriver()->beginScene(backBuffer, zBuffer, tools::ToIrrlichtSColor(color));

#ifdef CHRONO_MODAL
    if (this->modal_phi || this->modal_show) {
        if (this->modal_show)
            this->modal_phi += this->modal_speed * 0.01;
        else
            this->modal_phi = 0;  // return to normal dynamics
        // scan for modal assemblies, if any
        for (auto item : this->system->Get_otherphysicslist()) {
            if (auto mmodalassembly = std::dynamic_pointer_cast<modal::ChModalAssembly>(item)) {
                try {
                    // superposition of modal shape
                    mmodalassembly->SetFullStateWithModeOverlay(this->modal_mode_n, this->modal_phi,
                                                                this->modal_amplitude);
                    // fetch Hz of this mode
                    this->modal_current_freq = mmodalassembly->Get_modes_frequencies()(this->modal_mode_n);
                } catch (...) {
                    // something failed in SetFullStateWithModeOverlay(), ex. node n was higher than available ones
                    mmodalassembly->SetFullStateReset();
                }
            }
        }
        this->modal_current_mode_n = this->modal_mode_n;
    }
#endif

    m_gui->BeginScene();
}

// Call this to end the scene draw at the end of each animation frame.
void ChVisualSystemIrrlicht::EndScene() {
    utils::ChProfileManager::Stop_Profile();

    m_gui->EndScene();

    GetVideoDriver()->endScene();
}

void ChVisualSystemIrrlicht::DrawAll() {
    if (m_use_effects)
        m_effect_handler->update();  // draw 3D scene using Xeffects for shadow maps
    else
        GetSceneManager()->drawAll();  // draw 3D scene the usual way, if no shadow maps

    m_gui->DrawAll();
}

void ChVisualSystemIrrlicht::WriteImageToFile(const std::string& filename) {
    video::IImage* image = GetVideoDriver()->createScreenShot();
    if (image) {
        GetVideoDriver()->writeImageToFile(image, filename.c_str());
        image->drop();
    }
}

// -----------------------------------------------------------------------------

void ChVisualSystemIrrlicht::BindItem(std::shared_ptr<ChPhysicsItem> item) {
    if (!m_system || !m_device)
        return;

    CreateIrrNode(item);
}

void ChVisualSystemIrrlicht::BindAll() {
    if (!m_system || !m_device)
        return;

    std::unordered_set<const ChAssembly*> trace;
    CreateIrrNodes(&m_system->GetAssembly(), trace);
}

void ChVisualSystemIrrlicht::CreateIrrNodes(const ChAssembly* assembly, std::unordered_set<const ChAssembly*>& trace) {
    // Do nothing if the assembly was already processed
    if (!trace.insert(assembly).second)
        return;

    for (auto& body : assembly->Get_bodylist()) {
        CreateIrrNode(body);
    }

    for (auto& link : assembly->Get_linklist()) {
        CreateIrrNode(link);
    }

    for (auto& mesh : assembly->Get_meshlist()) {
        CreateIrrNode(mesh);
    }

    for (auto& ph : assembly->Get_otherphysicslist()) {
        CreateIrrNode(ph);

        // Recursively process sub-assemblies
        if (auto a = std::dynamic_pointer_cast<ChAssembly>(ph)) {
            CreateIrrNodes(a.get(), trace);
        }
    }

#ifdef CHRONO_MODAL
    // Modal assemblies contain custom internal items that might be useful to visualize
    if (auto myassy_modal = dynamic_cast<const chrono::modal::ChModalAssembly*>(massy)) {
        for (auto body : myassy_modal->Get_internal_bodylist()) {
            Bind(body);
        }
        for (auto& mesh : myassy_modal->Get_internal_meshlist()) {
            Bind(mesh);
        }
        for (auto ph : myassy_modal->Get_internal_otherphysicslist()) {
            Bind(ph);
            // If the assembly holds another assemblies, also bind their contents.
            if (auto myassy = std::dynamic_pointer_cast<ChAssembly>(ph)) {
                BindAllContentsOfAssembly(myassy.get(), mtrace);
            }
        }
        for (auto link : myassy_modal->Get_internal_linklist()) {
            Bind(link);
        }
    }
#endif
}

void ChVisualSystemIrrlicht::CreateIrrNode(std::shared_ptr<ChPhysicsItem> item) {
    if (!item->GetVisualModel())
        return;

    // Create a new ChIrrNodeModel and, if this is first insertion, populate it.
    if (m_nodes.insert({item.get(), chrono_types::make_shared<ChIrrNodeModel>(item, m_container, GetSceneManager(), 0)})
            .second) {
        auto& node = m_nodes[item.get()];
        assert(node);

        // Remove all Irrlicht scene nodes from the ChIrrNodeModel
        node->removeAll();

        // If the physics item uses clones of its visual model, create an intermediate Irrlicht scene node
        irr::scene::ISceneNode* fillnode = node.get();
        if (item->GetNumVisualModelClones() > 0) {
            fillnode = GetSceneManager()->addEmptySceneNode(node.get());
        }

        // Recursively populate the ChIrrNodeModel with Irrlicht scene nodes for each visual shape.
        // Begin with identity transform relative to the physics item.
        ChFrame<> frame;
        PopulateIrrNode(fillnode, item->GetVisualModel(), frame);
    }
}

static void mflipSurfacesOnX(IMesh* mesh) {
    if (!mesh)
        return;

    const u32 bcount = mesh->getMeshBufferCount();
    for (u32 b = 0; b < bcount; ++b) {
        IMeshBuffer* buffer = mesh->getMeshBuffer(b);
        const u32 idxcnt = buffer->getIndexCount();

        const u32 vertcnt = buffer->getVertexCount();
        for (u32 i = 0; i < vertcnt; i++) {
            buffer->getPosition(i).X = -buffer->getPosition(i).X;  // mirror vertex
            buffer->getNormal(i).X = -buffer->getNormal(i).X;      // mirrors normal on X
        }
    }
}

static void SetVisualMaterial(ISceneNode* node, std::shared_ptr<ChVisualShape> shape) {
    if (shape->GetMaterials().empty()) {
        // Use default material
        node->getMaterial(0) = *default_material;
    } else {
        // Use the first material in the list
        node->getMaterial(0) =
            tools::ToIrrlichtMaterial(shape->GetMaterial(0), node->getSceneManager()->getVideoDriver());
    }

    // Do not use vertex coloring
    node->getMaterial(0).ColorMaterial = irr::video::ECM_NONE;
}

void ChVisualSystemIrrlicht::PopulateIrrNode(irr::scene::ISceneNode* node,
                                             std::shared_ptr<ChVisualModel> model,
                                             const ChFrame<>& parent_frame) {
    //// RADU TODO - We do not use Pos and Rot from box geometry, or center from sphere geometry.
    ////             THIS WILL BE OBSOLETED!!!

    for (const auto& shape_instance : model->GetShapes()) {
        auto& shape = shape_instance.first;
        auto& shape_frame = shape_instance.second;
        irr::core::matrix4CH shape_m4(shape_frame);

        if (!shape->IsVisible())
            continue;

        if (auto obj = std::dynamic_pointer_cast<ChObjShapeFile>(shape)) {
            bool irrmesh_already_loaded = false;
            if (GetSceneManager()->getMeshCache()->getMeshByName(obj->GetFilename().c_str()))
                irrmesh_already_loaded = true;
            IAnimatedMesh* genericMesh = GetSceneManager()->getMesh(obj->GetFilename().c_str());
            if (genericMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(obj, node);
                ISceneNode* mchildnode = GetSceneManager()->addAnimatedMeshSceneNode(genericMesh, mproxynode);
                mproxynode->drop();

                // mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, false);

                // Note: the Irrlicht loader of .OBJ files flips the X to correct its left-handed nature, but
                // this goes wrong with our assemblies and links. Restore the X flipping of the mesh.
                if (!irrmesh_already_loaded)
                    mflipSurfacesOnX(((IAnimatedMeshSceneNode*)mchildnode)->getMesh());

                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, true);
            }
        } else if (auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
            // Create a number of Irrlicht mesh buffers equal to the number of materials.
            // If no materials defined, create a single mesh buffer.
            SMesh* smesh = new SMesh;
            int nbuffers = (int)trimesh->GetNumMaterials();
            nbuffers = std::max(nbuffers, 1);
            for (int ibuffer = 0; ibuffer < nbuffers; ibuffer++) {
                CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
                smesh->addMeshBuffer(buffer);
                buffer->drop();
            }

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(trimesh, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(smesh, mproxynode);
            smesh->drop();

            mchildnode->setPosition(shape_m4.getTranslation());
            mchildnode->setRotation(shape_m4.getRotationDegrees());

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            mchildnode->setMaterialFlag(video::EMF_WIREFRAME, trimesh->IsWireframe());
            mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trimesh->IsBackfaceCull());
        } else if (auto surf = std::dynamic_pointer_cast<ChSurfaceShape>(shape)) {
            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
            SMesh* newmesh = new SMesh;
            newmesh->addMeshBuffer(buffer);
            buffer->drop();

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(surf, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
            newmesh->drop();

            mchildnode->setPosition(shape_m4.getTranslation());
            mchildnode->setRotation(shape_m4.getRotationDegrees());

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            mchildnode->setMaterialFlag(video::EMF_WIREFRAME, surf->IsWireframe());
        } else if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
            if (sphereMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(sphere, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(sphereMesh, mproxynode);
                mproxynode->drop();

                double mradius = sphere->GetSphereGeometry().rad;
                mchildnode->setScale(core::vector3dfCH(ChVector<>(mradius, mradius, mradius)));
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, sphere);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
            if (sphereMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(ellipsoid, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(sphereMesh, mproxynode);
                mproxynode->drop();

                mchildnode->setScale(core::vector3dfCH(ellipsoid->GetEllipsoidGeometry().rad));
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, ellipsoid);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
            if (cylinderMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(cylinder, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(cylinderMesh, mproxynode);
                mproxynode->drop();

                double rad = cylinder->GetCylinderGeometry().rad;
                ChVector<> dir = cylinder->GetCylinderGeometry().p2 - cylinder->GetCylinderGeometry().p1;
                double height = dir.Length();

                // Calculate transform from asset to geometry
                dir.Normalize();
                ChVector<> mx, my, mz;
                dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
                ChMatrix33<> mrot;
                mrot.Set_A_axis(mx, my, mz);
                ChVector<> mpos = 0.5 * (cylinder->GetCylinderGeometry().p2 + cylinder->GetCylinderGeometry().p1);

                // Calculate transform from node to geometry (concatenate node - asset and asset - geometry)
                ChFrame<> frame = shape_frame * ChFrame<>(mpos, mrot);
                irr::core::matrix4CH m4(frame);

                core::vector3df irrsize((f32)rad, (f32)(0.5 * height), (f32)rad);
                mchildnode->setScale(irrsize);
                mchildnode->setPosition(m4.getTranslation());
                mchildnode->setRotation(m4.getRotationDegrees());


                SetVisualMaterial(mchildnode, cylinder);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto capsule = std::dynamic_pointer_cast<ChCapsuleShape>(shape)) {
            if (capsuleMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(capsule, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(capsuleMesh, mproxynode);
                mproxynode->drop();

                double rad = capsule->GetCapsuleGeometry().rad;
                double hlen = capsule->GetCapsuleGeometry().hlen;

                core::vector3df irrsize((f32)rad, (f32)hlen, (f32)rad);
                mchildnode->setScale(irrsize);
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());


                SetVisualMaterial(mchildnode, capsule);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
            if (cubeMesh) {
                ISceneNode* mproxynode = new ChIrrNodeShape(box, node);
                ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(cubeMesh, mproxynode);
                mproxynode->drop();

                mchildnode->setScale(core::vector3dfCH(box->GetBoxGeometry().Size));
                mchildnode->setPosition(shape_m4.getTranslation());
                mchildnode->setRotation(shape_m4.getRotationDegrees());

                SetVisualMaterial(mchildnode, box);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
        } else if (auto barrel = std::dynamic_pointer_cast<ChBarrelShape>(shape)) {
            auto mbarrelmesh = createEllipticalMesh((irr::f32)(barrel->GetRhor()), (irr::f32)(barrel->GetRvert()),
                                                    (irr::f32)(barrel->GetHlow()), (irr::f32)(barrel->GetHsup()),
                                                    (irr::f32)(barrel->GetRoffset()), 15, 8);
            ISceneNode* mproxynode = new ChIrrNodeShape(barrel, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(mbarrelmesh, mproxynode);
            mproxynode->drop();

            mchildnode->setPosition(shape_m4.getTranslation());
            mchildnode->setRotation(shape_m4.getRotationDegrees());

            SetVisualMaterial(mchildnode, barrel);
            mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
        } else if (auto glyphs = std::dynamic_pointer_cast<ChGlyphs>(shape)) {
            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
            SMesh* newmesh = new SMesh;
            newmesh->addMeshBuffer(buffer);
            buffer->drop();

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(glyphs, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
            newmesh->drop();

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            SetVisualMaterial(mchildnode, glyphs);

            ////mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
            ////mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
        } else if (std::dynamic_pointer_cast<ChPathShape>(shape) || std::dynamic_pointer_cast<ChLineShape>(shape)) {
            CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
            SMesh* newmesh = new SMesh;
            newmesh->addMeshBuffer(buffer);
            buffer->drop();

            ChIrrNodeShape* mproxynode = new ChIrrNodeShape(shape, node);
            ISceneNode* mchildnode = GetSceneManager()->addMeshSceneNode(newmesh, mproxynode);
            newmesh->drop();

            mproxynode->Update();  // force syncing of triangle positions & face indexes
            mproxynode->drop();

            SetVisualMaterial(mchildnode, shape);

            ////mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
            ////mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
        }
    }
}

// -----------------------------------------------------------------------------

}  // namespace irrlicht
}  // namespace chrono