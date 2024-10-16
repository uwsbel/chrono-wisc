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
// Authors: Hammad Mazhar
// =============================================================================
// Renders contact points as a point cloud
// =============================================================================

#include <iostream>

#include "chrono_opengl/UI/ChOpenGLContacts.h"
#include "chrono_opengl/ChOpenGLMaterials.h"

#ifdef CHRONO_MULTICORE
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/ChDataManager.h"
#endif

namespace chrono {
namespace opengl {
using namespace glm;

ChOpenGLContacts::ChOpenGLContacts() {}

bool ChOpenGLContacts::Initialize(ChOpenGLMaterial mat, ChOpenGLShader* shader) {
    if (GLReturnedError("Contacts::Initialize - on entry"))
        return false;
    contact_data.push_back(glm::vec3(0, 0, 0));
    contacts.Initialize(contact_data, mat, shader);
    contacts.SetPointSize(0.01f);
    return true;
}

void ChOpenGLContacts::UpdateChrono(ChSystem* system) {
    //  ChContactContainerNSC* container = (ChContactContainerNSC*)system->GetContactContainer();
    //  std::list<ChContactContainerNSC::ChContact_6_6*> list = container-GetContactList();
    //  int num_contacts = container->GetNumContacts();
    //  int counter = 0;
    //  contact_data.resize(num_contacts * 2);
    //
    //  for (std::list<ChContactContainerNSC::ChContact_6_6*>::const_iterator iterator = list.begin(), end = list.end();
    //  iterator != end; ++iterator) {
    //    ChVector3d p1 = (*iterator)->GetContactP1();
    //    ChVector3d p2 = (*iterator)->GetContactP2();
    //
    //    contact_data[counter] = glm::vec3(p1.x, p1.y, p1.z);
    //    contact_data[counter + num_contacts] = glm::vec3(p2.x, p2.y, p2.z);
    //    counter++;
    //  }
}

void ChOpenGLContacts::UpdateChronoMulticore(ChSystemMulticore* system) {
#ifdef CHRONO_MULTICORE
    ChMulticoreDataManager* data_manager = system->data_manager;
    const auto num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;
    const auto num_rigid_fluid_contacts = data_manager->cd_data->num_rigid_fluid_contacts;
    int num_contacts = num_rigid_contacts + num_rigid_fluid_contacts;

    // std::cout << "CONTACT RENDER: " << num_contacts << std::endl;

    if (num_contacts == 0) {
        return;
    }

    contact_data.resize(num_rigid_contacts * 2 + num_rigid_fluid_contacts);

    //#pragma omp parallel for
    for (int i = 0; i < (signed)num_rigid_contacts; i++) {
        real3 cpta = data_manager->cd_data->cpta_rigid_rigid[i];
        real3 cptb = data_manager->cd_data->cptb_rigid_rigid[i];

        contact_data[i] = glm::vec3(cpta.x, cpta.y, cpta.z);
        contact_data[i + num_rigid_contacts] = glm::vec3(cptb.x, cptb.y, cptb.z);
    }

    int offset = num_rigid_contacts * 2;
    for (int p = 0; p < (signed)data_manager->num_fluid_bodies; p++) {
        int start = data_manager->cd_data->c_counts_rigid_fluid[p];
        int end = data_manager->cd_data->c_counts_rigid_fluid[p + 1];
        for (int index = start; index < end; index++) {
            int i = index - start;
            real3 cpta = data_manager->cd_data->cpta_rigid_fluid[p * ChNarrowphase::max_rigid_neighbors + i];
            contact_data[index + offset] = glm::vec3(cpta.x, cpta.y, cpta.z);
        }
    }
#endif
}

void ChOpenGLContacts::Update(ChSystem* physics_system) {
    contact_data.clear();
#ifdef CHRONO_MULTICORE
    if (ChSystemMulticore* system_mc = dynamic_cast<ChSystemMulticore*>(physics_system)) {
        UpdateChronoMulticore(system_mc);
    } else
#endif
    {
        UpdateChrono(physics_system);
    }

    contacts.Update(contact_data);
}

void ChOpenGLContacts::TakeDown() {
    contacts.TakeDown();
    contact_data.clear();
}

void ChOpenGLContacts::Draw(const mat4& projection, const mat4& view) {
    glm::mat4 model(1);
    contacts.Draw(projection, view * model);
}
}
}
