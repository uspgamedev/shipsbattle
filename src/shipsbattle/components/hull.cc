#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/subhull.h>
#include <shipsbattle/components/subsystems/typedefs.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/debug/log.h>
#include <btBulletCollisionCommon.h>
#include <OgreVector3.h>

#include <iostream>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::DecaymentFunction;
using shipsbattle::components::subsystems::DamageableSystem;
using shipsbattle::components::subsystems::SubHull;
using shipsbattle::components::subsystems::DecaymentFunction;


namespace shipsbattle {
namespace components {

Hull::~Hull() {
    delete world_;
    delete broadphase_;
    delete dispatcher_;
    delete config_;
}

void Hull::AddSubHull(const shared_ptr<SubHull>& subhull) {
    if (subhull_indexes_.count(subhull->name()) > 0) {
        Log(LogLevel::WARNING, "Hull System", "SubHull with name '" + subhull->name() + "' already exists in ship '" + owner()->name() + "'. Discarding this SubHull.");
        return;
    }
    subhulls_.push_back(subhull);
    subhull_indexes_[subhull->name()] = subhulls_.size() - 1;
    subhull->RegisteredTo(this);
}
const shared_ptr<SubHull>& Hull::GetSubHull(size_t index) {
    return subhulls_.at(index);
}
const shared_ptr<SubHull>& Hull::GetSubHull(const std::string& name) {
    return GetSubHull(subhull_indexes_[name]);
}

void Hull::OnTaken() {
    config_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(config_);
    broadphase_ = new btDbvtBroadphase();
    world_ = new btCollisionWorld(dispatcher_, broadphase_, config_);
    
    for (auto sys : damageables_) {
        world_->addCollisionObject(sys->volume());
    }
}

void Hull::RegisterDamageableSystem(subsystems::DamageableSystem* dmgable_sys) {
    damageables_.push_back(dmgable_sys);

    if (owner()) {
        world_->addCollisionObject(dmgable_sys->volume());
    }
}

struct TakeDamageCallback : public btCollisionWorld::ContactResultCallback
{
    double dmg;
    double piercing;
    double radius;
    DecaymentFunction decayment;

    TakeDamageCallback(double _dmg, double _piercing, double _radius, const DecaymentFunction& _decayment)
        : dmg(_dmg), piercing(_piercing), radius(_radius), decayment(_decayment) {}

    virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,
        int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
    {
        DamageableSystem* sys;
        if (colObj0Wrap->getCollisionObject()->getUserPointer() == nullptr) {
            sys = static_cast<DamageableSystem*>(colObj1Wrap->getCollisionObject()->getUserPointer());
        }
        else if (colObj1Wrap->getCollisionObject()->getUserPointer() == nullptr) {
            sys = static_cast<DamageableSystem*>(colObj0Wrap->getCollisionObject()->getUserPointer());
        }
        else {
            return 0; //Both objects are not the damage collision object
        }
        if (!sys) {
            std::cout << "TakeDamageCallback WARNING: got a pair with something which is not a DamageableSystem" << std::endl;
            return 0;
        }

        if (sys->destroyed()) return 0;

        auto dist = (cp.getPositionWorldOnA() - cp.getPositionWorldOnB()).length();
        double dmg_factor = decayment(dist, radius);
        //std::cout << "TakeDamageCallback: sys=" << sys->name() << std::endl;
        sys->TakeDamage(dmg * dmg_factor, piercing);

        return 0;
    }
};

void Hull::TakeDamage(double dmg, double piercing, double splash_radius, const btVector3& pos, const DecaymentFunction& decayment) {
    btCollisionObject* hit = new btCollisionObject();
    hit->setCollisionShape(new btSphereShape(static_cast<btScalar>(splash_radius)));
    hit->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(pos.x(), pos.y(), pos.z())));
    hit->setUserPointer(nullptr);

    //std::cout << "HULL TAKE DAMAGE d/p/s/xyz: " << dmg << "/" << piercing << "/" << splash_radius;
    //std::cout << "/(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;

    //TODO: para implementar ordem e peso nos sistemas pra divisao do dano, o TDC tem que simplesmente 
    // armazenar os sistemas afetados numa lista, e depois da chamada de contactTest nos tratamos dessa lista
    TakeDamageCallback tdc(dmg, piercing, splash_radius, decayment);
    world_->contactTest(hit, tdc);
    //std::cout << "FINISHED HULL TAKE DAMAGE" << std::endl;

    delete hit->getCollisionShape();
    delete hit;
}

} // namespace components
} // namespace shipsbattle
