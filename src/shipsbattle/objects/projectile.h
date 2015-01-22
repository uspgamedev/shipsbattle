#ifndef SHIPSBATTLE_OBJECTS_PROJECTILE_H_
#define SHIPSBATTLE_OBJECTS_PROJECTILE_H_

#include <ugdk/action/3D/element.h>

namespace Ogre {
class Vector3;
}
namespace ugdk {
namespace action {
namespace mode3d {
class Element;
namespace component {
class Body;
class View;
}}}}

namespace shipsbattle {
namespace components {
class ProjectileController;
}

namespace objects {
class Ship;
class ProjectileModel;
class Target;

class Projectile {
public:
    Projectile(const Ship& parent_ship, const ProjectileModel& model, const Target& target,
        const Ogre::Vector3& position, const Ogre::Vector3& direction);
    Projectile(const std::shared_ptr<ugdk::action::mode3d::Element>& projectile);
    ~Projectile();

    ugdk::action::mode3d::component::Body* body();
    ugdk::action::mode3d::component::View* view();
    shipsbattle::components::ProjectileController* controller();

    std::shared_ptr<ugdk::action::mode3d::Element> operator->() const { return projectile_.lock(); }
    ugdk::action::mode3d::Element& operator*() const { return *projectile_.lock(); }

    bool valid() const { return !projectile_.expired(); }


protected:
    std::weak_ptr<ugdk::action::mode3d::Element> projectile_;
};

} // namespace objects
} // namespace shipsbattle
#endif //SHIPSBATTLE_OBJECTS_PROJECTILE_H_
