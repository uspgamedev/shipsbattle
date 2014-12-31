#include <shipsbattle/components/hull.h>
#include <shipsbattle/components/subsystems/subhull.h>

#include <ugdk/action/3D/element.h>
#include <ugdk/debug/log.h>
#include <btBulletCollisionCommon.h>
#include <OgreVector3.h>

#include <iostream>

using std::shared_ptr;
using ugdk::debug::Log;
using ugdk::debug::LogLevel;
using shipsbattle::components::subsystems::DamageableSystem;
using shipsbattle::components::subsystems::SubHull;


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

struct HitData {
    double dmg;
    double piercing;

    HitData(double _dmg, double _piercing) : dmg(_dmg), piercing(_piercing) {}
};

struct TakeDamageCallback : public btCollisionWorld::ContactResultCallback
{
    virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,
        int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
    {
        HitData* data = static_cast<HitData*>(colObj0Wrap->getCollisionObject()->getUserPointer());
        DamageableSystem* sys;
        if (!data) {
            data = static_cast<HitData*>(colObj1Wrap->getCollisionObject()->getUserPointer());
            if (!data) {
                return 0; //Both objects are not the damage collision object
            }
            sys = static_cast<DamageableSystem*>(colObj0Wrap->getCollisionObject()->getUserPointer());
        }
        else {
            sys = static_cast<DamageableSystem*>(colObj1Wrap->getCollisionObject()->getUserPointer());
        }
        if (!sys) {
            std::cout << "TakeDamageCallback WARNING: got a pair with a HitData and something else which is not a DamageableSystem" << std::endl;
            return 0;
        }

        if (sys->destroyed()) return 0;

        //std::cout << "TakeDamageCallback: sys=" << sys->name() << std::endl;
        sys->TakeDamage(data->dmg, data->piercing);

        return 0;
    }
};

void Hull::TakeDamage(double dmg, double piercing, double splash_radius, const btVector3& pos) {
    btCollisionObject* hit = new btCollisionObject();
    hit->setCollisionShape(new btSphereShape(static_cast<btScalar>(splash_radius)));
    hit->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(pos.x(), pos.y(), pos.z())));
    HitData* hitdata = new HitData(dmg, piercing);
    hit->setUserPointer(hitdata);

    //std::cout << "HULL TAKE DAMAGE d/p/s/xyz: " << dmg << "/" << piercing << "/" << splash_radius;
    //std::cout << "/(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;

    //TODO: para implementar ordem e peso nos sistemas pra divisao do dano, o TDC tem que simplesmente 
    // armazenar os sistemas afetados numa lista, e depois da chamada de contactTest nos tratamos dessa lista
    TakeDamageCallback tdc;
    world_->contactTest(hit, tdc);

    //std::cout << "FINISHED HULL TAKE DAMAGE" << std::endl;

    delete hit->getCollisionShape();
    delete hitdata;
    delete hit;
}

} // namespace components
} // namespace shipsbattle
