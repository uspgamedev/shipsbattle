#include <shipsbattle/objects/ship.h>
#include <shipsbattle/objects/objecttypes.h>
#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/subhull.h>
#include <ugdk/action/3D/component/physicsbody.h>
#include <ugdk/action/3D/component/view.h>
#include <ugdk/action/3D/scene3d.h>

#include <BtOgreGP.h>
#include <OgreEntity.h>

#include <iostream>

using std::string;
using std::cout;
using std::endl;
using ugdk::action::mode3d::Scene3D;
using ugdk::action::mode3d::component::Body;
using ugdk::action::mode3d::component::PhysicsBody;
using ugdk::action::mode3d::component::View;
using ugdk::action::mode3d::component::CollisionAction;
using ugdk::action::mode3d::component::ElementPtr;
using ugdk::action::mode3d::component::ManifoldPointVector;
using shipsbattle::components::Hull;
using shipsbattle::components::subsystems::SubHull;

namespace shipsbattle {
namespace objects {

Ship::Ship(Scene3D& scene, const string& name, const string& meshName) {
    ship_ = scene.AddElement(name).get();

    //View
    ship_->AddComponent(std::make_shared<View>());
    auto entity = view()->AddEntity(name, meshName);

    // Body
    PhysicsBody::PhysicsData data;
    auto meshShapeConv = BtOgre::StaticMeshToShapeConverter(entity);
    data.shape = meshShapeConv.createConvex();
    data.mass = 80;
    data.collision_group = ObjectTypes::SHIP;
    data.collides_with = ObjectTypes::SHIP;
    PhysicsBody* pbody = new PhysicsBody(*scene.physics(), data);
    ship_->AddComponent(std::shared_ptr<PhysicsBody>(pbody));
    pbody->set_damping(.5, .5);

    pbody->AddCollisionAction(ObjectTypes::SHIP,
        [](const ElementPtr& self, const ElementPtr& target, const ManifoldPointVector& pts) {
        cout << self->name() << " colidindo com " << target->name() << " (" << pts.size() << ")" << endl;
    });

    // Hull
    Hull* hullsys = new Hull();
    SubHull* mainhull = new SubHull("MainHull");
    mainhull->SetVolume(entity->getBoundingRadius(), btVector3(0.0,0.0,0.0));
    mainhull->set_max_armor_rating(50);
    mainhull->set_armor_rating(50);
    hullsys->AddSubHull(std::shared_ptr<SubHull>(mainhull));
    ship_->AddComponent(std::shared_ptr<Hull>(hullsys));
}
Ship::Ship(ugdk::action::mode3d::Element* ship) : ship_(ship) {
    //FIXME: make sure the element is a ship element.
}
Ship::~Ship() {

}

Body* Ship::body() {
    return ship_->component<Body>();
}
View* Ship::view() {
    return ship_->component<View>();
}
Hull* Ship::hull() {
    return ship_->component<Hull>();
}


} // namespace objects
} // namespace shipsbattle
