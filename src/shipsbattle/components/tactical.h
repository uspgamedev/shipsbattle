#ifndef SHIPSBATTLE_COMPONENTS_TACTICAL_H
#define SHIPSBATTLE_COMPONENTS_TACTICAL_H

#include <shipsbattle/components/updateablecomponent.h>
#include <vector>
#include <unordered_map>

namespace shipsbattle {
namespace components {
namespace subsystems {
class DamageableSystem;
class Weapon;
}

class Tactical : public UpdateableComponent {
public:
    Tactical() {}

    virtual std::type_index type() const override;

    void AddWeapon(const std::shared_ptr<subsystems::Weapon>& weapon);
    const std::shared_ptr<subsystems::Weapon>& GetWeapon(size_t index);
    const std::shared_ptr<subsystems::Weapon>& GetWeapon(const std::string& name);
    size_t GetNumWeapons() const { return weapons_.size(); }

    void Update(double dt) override;
    void FireAll(subsystems::DamageableSystem* target);

protected:
    std::vector<std::shared_ptr<subsystems::Weapon>>    weapons_;
    std::unordered_map<std::string, size_t>    weapon_indexes_;
};

inline std::type_index Tactical::type() const {
    return typeid(Tactical);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_TACTICAL_H
