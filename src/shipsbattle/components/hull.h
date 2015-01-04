#ifndef SHIPSBATTLE_COMPONENTS_HULL_H
#define SHIPSBATTLE_COMPONENTS_HULL_H

#include <ugdk/action/3D/component.h>
#include <shipsbattle/components/subsystems/typedefs.h>
#include <vector>
#include <unordered_map>

class btVector3;
class btBroadphaseInterface;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btCollisionWorld;

namespace shipsbattle {
namespace components {

namespace subsystems {
class DamageableSystem;
class SubHull;
}

class Hull : public ugdk::action::mode3d::Component {
public:
    Hull() {}
    ~Hull();

    virtual std::type_index type() const override;

    void AddSubHull(const std::shared_ptr<subsystems::SubHull>& subhull);
    const std::shared_ptr<subsystems::SubHull>& GetSubHull(size_t index);
    const std::shared_ptr<subsystems::SubHull>& GetSubHull(const std::string& name);
    size_t GetNumSubHulls() const { return subhulls_.size(); }

    void TakeDamage(double dmg, double piercing, double splash_radius, const btVector3& pos, const subsystems::DecaymentFunction& decayment);

protected:
    void OnTaken() override;

    friend class subsystems::DamageableSystem;
    void RegisterDamageableSystem(subsystems::DamageableSystem* dmgable_sys);

    std::vector<std::shared_ptr<subsystems::SubHull>>    subhulls_;
    std::unordered_map<std::string, size_t>    subhull_indexes_;
    std::vector<subsystems::DamageableSystem*>  damageables_;

    btBroadphaseInterface* broadphase_;
    btDefaultCollisionConfiguration* config_;
    btCollisionDispatcher* dispatcher_;
    btCollisionWorld* world_;

};

inline std::type_index Hull::type() const {
    return typeid(Hull);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_HULL_H
