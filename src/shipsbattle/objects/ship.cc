#include <shipsbattle/objects/ship.h>
#include <shipsbattle/objects/objecttypes.h>
#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/subhull.h>
#include <shipsbattle/components/powersystem.h>
#include <shipsbattle/components/subsystems/powergenerator.h>
#include <shipsbattle/components/subsystems/battery.h>
#include <shipsbattle/components/tactical.h>
#include <shipsbattle/components/subsystems/projectileweapon.h>
#include <shipsbattle/objects/projectilemodel.h>

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
using ugdk::action::mode3d::component::ContactPointVector;
using shipsbattle::components::Hull;
using shipsbattle::components::subsystems::SubHull;
using shipsbattle::components::PowerSystem;
using shipsbattle::components::subsystems::PowerGenerator;
using shipsbattle::components::subsystems::Battery;
using shipsbattle::components::Tactical;
using shipsbattle::components::subsystems::ProjectileWeapon;
using shipsbattle::objects::ProjectileModel;

namespace shipsbattle {
namespace objects {

Ship::Ship(Scene3D& scene, const string& name, const string& meshName) {
    ship_ = scene.AddElement(name);
    auto& ship = ship_.lock();

    //View
    ship->AddComponent(std::make_shared<View>());
    auto entity = view()->AddEntity(name, meshName);

    // Body
    PhysicsBody::PhysicsData data;
    auto meshShapeConv = BtOgre::StaticMeshToShapeConverter(entity);
    data.shape = meshShapeConv.createConvex();
    data.mass = 80;
    data.collision_group = ObjectTypes::SHIP;
    data.collides_with = ObjectTypes::SHIP | ObjectTypes::PROJECTILE;
    PhysicsBody* pbody = new PhysicsBody(*scene.physics(), data);
    ship->AddComponent(std::shared_ptr<PhysicsBody>(pbody));
    pbody->set_damping(.5, .5);
    pbody->set_restitution(0.3);
    //TODO: maybe its good to set ContinuousCollisionDetection for ships

    pbody->AddCollisionAction(ObjectTypes::SHIP,
        [](const ElementPtr& self, const ElementPtr& target, const ContactPointVector& pts) {
        //cout << self->name() << " colidindo com " << target->name() << " (" << pts.size() << ")" << endl;
        Ship me(self);
        Ship them(target);
        
        for (auto pt : pts) {
            auto localPtA = pt.world_positionA - BtOgre::Convert::toBullet(me.body()->position());
            auto localPtB = pt.world_positionB - BtOgre::Convert::toBullet(them.body()->position());
            auto velA = me.body()->GetVelocityInPoint(BtOgre::Convert::toOgre(localPtA));
            auto velB = them.body()->GetVelocityInPoint(BtOgre::Convert::toOgre(localPtB));
            auto angle = velA.angleBetween(velB);
            auto speed_diff = velA.length() - velB.length();
            cout << me->name() << " colliding with " << them->name() << "; angle=" << angle << " (A:" << velA.length() << "/B:" << velB.length() << ")" << endl;
            double dmg = 2;
            double piercing = 0.5;
            double splash = 1.0;
            me.hull()->TakeDamage(dmg, piercing, splash, localPtA, components::subsystems::DecaymentFunctions::CONSTANT);

            //body_->setCollisionFlags(body_->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        }
    });

    // Hull
    Hull* hullsys = new Hull();
    //due to the way we're doing damageablesystem registration, we need to add hull system before adding any
    //subsystem to any other component system.
    ship->AddComponent(std::shared_ptr<Hull>(hullsys));
    SubHull* mainhull = new SubHull("MainHull");
    mainhull->SetVolume(entity->getBoundingRadius(), btVector3(0.0,0.0,0.0));
    mainhull->set_max_armor_rating(100);
    mainhull->set_armor_rating(100);
    mainhull->set_required(true);
    hullsys->AddSubHull(std::shared_ptr<SubHull>(mainhull));

    // PowerSystem
    ship->AddComponent(std::make_shared<PowerSystem>());
    auto power = this->power();
    PowerGenerator* pgen = new PowerGenerator("Power Plant");
    pgen->SetVolume(0.8, btVector3(0.0, 0.0, -1.0));
    power->AddPowerGenerator(std::shared_ptr<PowerGenerator>(pgen));
    Battery* bat = new Battery("Energy Cells");
    bat->SetVolume(0.5, btVector3(0.0, 0.1, 0.7));
    power->AddBattery(std::shared_ptr<Battery>(bat));

    // Tactical
    ship->AddComponent(std::make_shared<Tactical>());
    auto tact = this->tactical();
    ProjectileWeapon* gun = new ProjectileWeapon("Cannon");
    ProjectileModel ammunition ("HEAmmo");
    ammunition.set_mesh_name("Ammo");
    gun->set_projectile(ammunition);
    gun->set_direction(Ogre::Vector3::UNIT_Z);
    gun->SetVolume(0.1, btVector3(0.0, 0.0, 3.6));
    tact->AddWeapon(std::shared_ptr<ProjectileWeapon>(gun));
}
Ship::Ship(const std::shared_ptr<ugdk::action::mode3d::Element>& ship) : ship_(ship) {
    //FIXME: make sure the element is a ship element.
}
Ship::~Ship() {

}

Body* Ship::body() {
    return ship_.lock()->component<Body>();
}
View* Ship::view() {
    return ship_.lock()->component<View>();
}
Hull* Ship::hull() {
    return ship_.lock()->component<Hull>();
}
PowerSystem* Ship::power() {
    return ship_.lock()->component<PowerSystem>();
}
Tactical* Ship::tactical() {
    return ship_.lock()->component<Tactical>();
}


} // namespace objects
} // namespace shipsbattle
