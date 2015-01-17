#include <shipsbattle/objects/projectile.h>
#include <shipsbattle/objects/objecttypes.h>
#include <shipsbattle/objects/ship.h>
#include <shipsbattle/components/projectilecontroller.h>
#include <shipsbattle/components/timedlife.h>
#include <shipsbattle/components/subsystems/damageablesystem.h>
#include <shipsbattle/components/hull.h>

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
using shipsbattle::components::ProjectileController;
using shipsbattle::components::TimedLife;
using shipsbattle::components::subsystems::DamageableSystem;
using Ogre::Vector3;

namespace shipsbattle {
namespace objects {

Projectile::Projectile(const Ship& parent_ship, const ProjectileModel& model, DamageableSystem* target, const Vector3& position, const Vector3& direction) {
    auto& scene = parent_ship->scene();
    projectile_ = scene.AddElement();
    auto& projectile = projectile_.lock();

    //View
    projectile->AddComponent(std::make_shared<View>());
    auto entity = view()->AddEntity(model.mesh_name());

    // Body
    PhysicsBody::PhysicsData data;
    auto meshShapeConv = BtOgre::StaticMeshToShapeConverter(entity);
    data.shape = meshShapeConv.createCapsule();
    data.mass = model.mass();
    data.initial = btTransform(BtOgre::Convert::toBullet(Vector3::UNIT_Z.getRotationTo(direction)), BtOgre::Convert::toBullet(position));
    data.collision_group = ObjectTypes::PROJECTILE;
    data.collides_with = ObjectTypes::SHIP | ObjectTypes::PROJECTILE;
    PhysicsBody* pbody = new PhysicsBody(*scene.physics(), data);
    projectile->AddComponent(std::shared_ptr<PhysicsBody>(pbody));
    pbody->set_damping(.25, .25);
    pbody->SetRespondsOnContact(false);
    pbody->SetContinuousCollisionDetection(100.0, entity->getBoundingRadius() / 2.0);

    pbody->AddCollisionAction(ObjectTypes::SHIP,
        [&model](const ElementPtr& self, const ElementPtr& target, const ContactPointVector& pts) {
        cout << "Projetil tipo '" << model.name() << "' acertando nave " << target->name() << " (" << pts.size() << ")" << endl;
        if (self->marked_for_removal()) return;
        Projectile shot(self);
        Ship ship(target);
        // apply velocity push
        auto shot_vel = shot.body()->linear_velocity();
        // calculate actual total damage (dmg + velocityDamage)
        /****/
        // do damage
        for (auto pt : pts) {
            auto localPtB = pt.world_positionB - BtOgre::Convert::toBullet(ship.body()->position());
            ship.hull()->TakeDamage(model.damage()/pts.size(), model.armor_piercing(), model.splash_radius(), localPtB, model.decayment());
            ship.body()->ApplyImpulse(shot_vel / pts.size(), BtOgre::Convert::toOgre(localPtB));
        }
        // call onhit
        self->component<ProjectileController>()->projectile().OnHit(shot, ship, pts);
        // remove projectile
        self->scene().DestroyAndRemoveElement(self);
    });
    pbody->AddCollisionAction(ObjectTypes::PROJECTILE,
        [](const ElementPtr& self, const ElementPtr& target, const ContactPointVector& pts) {
        cout << self->name() << " acertando projetil " << target->name() << " (" << pts.size() << ")" << endl;

        //TODO: create explosion from both projectiles

        // remove projectiles
        self->scene().DestroyAndRemoveElement(self);
        target->scene().DestroyAndRemoveElement(target);
    });

    // ProjectileController
    projectile->AddComponent(std::make_shared<ProjectileController>(parent_ship, model, target));

    // TimedLife
    projectile->AddComponent(std::make_shared<TimedLife>(model.lifetime()));
}
Projectile::Projectile(const std::shared_ptr<ugdk::action::mode3d::Element>& projectile) : projectile_(projectile) {
    //FIXME: make sure the element is a projectile element.
}
Projectile::~Projectile() {

}

Body* Projectile::body() {
    return projectile_.lock()->component<Body>();
}
View* Projectile::view() {
    return projectile_.lock()->component<View>();
}
ProjectileController* Projectile::controller() {
    return projectile_.lock()->component<ProjectileController>();
}

} // namespace objects
} // namespace shipsbattle
