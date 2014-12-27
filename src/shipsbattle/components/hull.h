#ifndef SHIPSBATTLE_COMPONENTS_HULL_H
#define SHIPSBATTLE_COMPONENTS_HULL_H

#include <ugdk/action/3D/component.h>
#include <vector>
#include <unordered_map>

namespace shipsbattle {
namespace components {

namespace subsystems {
class DamageableSystem;
class SubHull;
}

class Hull : public ugdk::action::mode3d::Component {
public:
    Hull() {}

    virtual std::type_index type() const override;

    void AddSubHull(const std::shared_ptr<subsystems::SubHull>& subhull);
    const std::shared_ptr<subsystems::SubHull>& GetSubHull(size_t index);
    const std::shared_ptr<subsystems::SubHull>& GetSubHull(const std::string& name);
    size_t GetNumSubHulls() const { return subhulls_.size(); }

protected:
    void OnTaken() override;

    friend class subsystems::DamageableSystem;
    void RegisterDamageableSystem(subsystems::DamageableSystem* dmgable_sys);

    std::vector<std::shared_ptr<subsystems::SubHull>>    subhulls_;
    std::unordered_map<std::string, size_t>    subhull_indexes_;
    std::vector<subsystems::DamageableSystem*>  damageables_;
};

inline std::type_index Hull::type() const {
    return typeid(Hull);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_HULL_H
