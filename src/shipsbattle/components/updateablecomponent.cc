#include <shipsbattle/components/updateablecomponent.h>

#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/system/task.h>

#include <list>

namespace {
    bool created_update_task;
    std::list<shipsbattle::components::UpdateableComponent*> component_list;
}

namespace shipsbattle {
namespace components {

UpdateableComponent::~UpdateableComponent() {
    component_list.remove(this);
}

void UpdateableComponent::OnTaken() {
    component_list.push_back(this);

    if (!created_update_task) {
        owner()->scene().AddTask(ugdk::system::Task(
            [](double dt) {
            for (auto tl : component_list) {
                tl->Update(dt);
            }
        }));
        created_update_task = true;
    }
}

} // namespace components
} // namespace shipsbattle
