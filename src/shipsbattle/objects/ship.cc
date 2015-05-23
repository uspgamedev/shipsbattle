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
#include <shipsbattle/components/navigation.h>
#include <shipsbattle/components/subsystems/sensorarray.h>
#include <shipsbattle/components/motion.h>
#include <shipsbattle/components/subsystems/impulseengine.h>
#include <shipsbattle/components/subsystems/thruster.h>

#include <ugdk/action/3D/component/physicsbody.h>
#include <ugdk/action/3D/component/view.h>
#include <ugdk/action/3D/scene3d.h>

#include <BtOgreGP.h>
#include <OgreEntity.h>

#include <iostream>

using std::string;
using std::cout;
using std::endl;
using Ogre::Vector3;
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
using shipsbattle::components::Navigation;
using shipsbattle::components::subsystems::SensorArray;
using shipsbattle::components::Motion;
using shipsbattle::components::subsystems::ImpulseEngine;
using shipsbattle::components::subsystems::Thruster;

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
    //pbody->set_damping(.5, .5);
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
    pgen->SetVolume(0.4, btVector3(0.0, -0.05, -0.75));
    power->AddPowerGenerator(std::shared_ptr<PowerGenerator>(pgen));
    Battery* bat = new Battery("Energy Cells");
    bat->SetVolume(0.2, btVector3(0.0, -0.05, -0.2));
    power->AddBattery(std::shared_ptr<Battery>(bat));

    // Tactical
    ship->AddComponent(std::make_shared<Tactical>());
    auto tact = this->tactical();
    ProjectileWeapon* gun = new ProjectileWeapon("Forward Cannon");
    ProjectileModel ammunition ("HEAmmo");
    ammunition.set_mesh_name("Ammo");
    ammunition.set_linear_speed(50.0);
    ammunition.set_angular_speed( 20 * 3.14/180);
    gun->set_projectile(ammunition);
    gun->set_direction(Ogre::Vector3::UNIT_Z);
    gun->set_launching_speed(10.0);
    gun->SetVolume(0.075, btVector3(0.0, -0.2, 3.6)); // z = 2.8
    tact->AddWeapon(std::shared_ptr<ProjectileWeapon>(gun));

    // Navigation
    ship->AddComponent(std::make_shared<Navigation>());
    auto nav = this->navigation();
    SensorArray* sensor = new SensorArray("Sensor Array");
    sensor->SetVolume(0.2, btVector3(0.0, 0.4, 0.25));
    nav->AddSensorArray(std::shared_ptr<SensorArray>(sensor));

    // Motion
    ship->AddComponent(std::make_shared<Motion>());
    auto motion = this->motion();
    // port = esquerda / starboard = direita
    const int num_engs = 3;
    const btVector3 eng_poses[num_engs] = { btVector3(0.95, -0.3, -0.7), btVector3(-0.95, -0.3, -0.7), btVector3(0.0, -0.175, -3.15) };
    const string eng_names[num_engs] = { "Port Engine", "Starboard Engine", "Aft Engine" };
    for (int i = 0; i < num_engs; i++) {
        ImpulseEngine* engine = new ImpulseEngine(eng_names[i]);
        engine->SetVolume(0.4, eng_poses[i]);
        engine->set_exhaust_angle(Ogre::Degree(120.0).valueRadians());
        engine->set_exhaust_power(100.0);
        motion->AddImpulseEngine(std::shared_ptr<ImpulseEngine>(engine));
    }
    const int num_ts = 12;
    const btVector3 t_poses[num_ts] = { btVector3(0.45, 0.0, 1.75), btVector3(-0.45, 0.0, 1.75), btVector3(0.0, 0.4, 1.75), btVector3(0.0, -0.3, 1.75), btVector3(0.75, -0.05, 0.0),
        btVector3(0.75, -0.05, 0.0), btVector3(-0.75, -0.05, 0.0), btVector3(-0.75, -0.05, 0.0), btVector3(0.2, -0.15, -2.45), btVector3(-0.2, -0.15, -2.45),
        btVector3(0.0, -0.05, -2.45), btVector3(0.0, -0.3, -2.45) };
    const Vector3 t_dirs[num_ts] = { Vector3::UNIT_X, Vector3::NEGATIVE_UNIT_X, Vector3::UNIT_Y, Vector3::NEGATIVE_UNIT_Y, Vector3::UNIT_Y, Vector3::NEGATIVE_UNIT_Y,
        Vector3::UNIT_Y, Vector3::NEGATIVE_UNIT_Y, Vector3::UNIT_X, Vector3::NEGATIVE_UNIT_X, Vector3::UNIT_Y, Vector3::NEGATIVE_UNIT_Y };
    const string t_names[num_ts] = { "FrontPort Thruster", "FrontStarboard Thruster", "FrontDorsal Thruster", "FrontVentral Thruster", "MidPortDorsal Thruster", "MidPortVentral Thruster", 
        "MidStarboardDorsal Thruster", "MidStarboardVentral Thruster", "AftPort Thruster", "AftStarboard Thruster", "AftDorsal Thruster", "AftVentral Thruster" };
    for (int i = 0; i < num_ts; i++) {
        Thruster* tr = new Thruster(t_names[i]);
        tr->SetVolume(0.05, t_poses[i]);
        tr->set_thrust_power(Ogre::Degree(720.0).valueRadians());
        tr->set_thrust_direction(t_dirs[i]);
        motion->AddThruster(std::shared_ptr<Thruster>(tr));
    }
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
Navigation* Ship::navigation() {
    return ship_.lock()->component<Navigation>();
}
Motion* Ship::motion() {
    return ship_.lock()->component<Motion>();
}

} // namespace objects
} // namespace shipsbattle
