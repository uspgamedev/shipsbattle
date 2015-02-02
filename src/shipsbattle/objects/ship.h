#ifndef SHIPSBATTLE_OBJECTS_SHIP_H_
#define SHIPSBATTLE_OBJECTS_SHIP_H_

#include <ugdk/action/3D/element.h>

namespace ugdk {
namespace action {
namespace mode3d {
class Scene3D;
namespace component {
class Body;
class View;
}}}}

namespace shipsbattle {
namespace components {
class Hull;
class PowerSystem;
class Tactical;
class Navigation;
class Motion;
}

namespace objects {

class Ship {
public:
    Ship(ugdk::action::mode3d::Scene3D& scene, const std::string& name, const std::string& meshName);
    Ship(const std::shared_ptr<ugdk::action::mode3d::Element>& ship);
    ~Ship();

    ugdk::action::mode3d::component::Body* body();
    ugdk::action::mode3d::component::View* view();
    shipsbattle::components::Hull* hull();
    shipsbattle::components::PowerSystem* power();
    shipsbattle::components::Tactical* tactical();
    shipsbattle::components::Navigation* navigation();
    shipsbattle::components::Motion* motion();
    
    std::shared_ptr<ugdk::action::mode3d::Element> operator->() const { return ship_.lock(); }
    ugdk::action::mode3d::Element& operator*() const { return *ship_.lock(); }

    bool valid() const { 
        return !ship_.expired();
    }

protected:
    std::weak_ptr<ugdk::action::mode3d::Element> ship_;
};

} // namespace objects
} // namespace shipsbattle
#endif //SHIPSBATTLE_OBJECTS_SHIP_H_
