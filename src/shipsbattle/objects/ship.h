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
namespace objects {

class Ship {
public:
    Ship(ugdk::action::mode3d::Scene3D& scene, const std::string& name, const std::string& meshName);
    Ship(ugdk::action::mode3d::Element* ship);
    ~Ship();

    ugdk::action::mode3d::component::Body* body();
    ugdk::action::mode3d::component::View* view();
    
    ugdk::action::mode3d::Element* operator->() const { return ship_; }
    ugdk::action::mode3d::Element operator*() const { return *ship_; }

protected:
    ugdk::action::mode3d::Element* ship_;
};

} // namespace objects
} // namespace shipsbattle
#endif //SHIPSBATTLE_OBJECTS_SHIP_H_
