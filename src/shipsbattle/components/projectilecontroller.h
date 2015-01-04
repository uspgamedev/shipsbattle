#ifndef SHIPSBATTLE_COMPONENTS_PROJECTILECONTROLLER_H
#define SHIPSBATTLE_COMPONENTS_PROJECTILECONTROLLER_H

#include <shipsbattle/components/updateablecomponent.h>
#include <shipsbattle/objects/projectilemodel.h>
#include <shipsbattle/objects/ship.h>

namespace shipsbattle {
namespace components {
namespace subsystems {
class DamageableSystem;
}

class ProjectileController : public UpdateableComponent {
public:
    ProjectileController(const objects::Ship& parent_ship, const objects::ProjectileModel& projectile, subsystems::DamageableSystem* target);

    virtual std::type_index type() const override;

    void Update(double dt) override;

    objects::Ship parent_ship() const { return parent_ship_; }
    objects::ProjectileModel projectile() const { return projectile_; }
    subsystems::DamageableSystem* target() const { return target_; }

  protected:
    void OnTaken() override;

private:
    objects::Ship parent_ship_;
    objects::ProjectileModel projectile_;
    subsystems::DamageableSystem* target_;
};

inline std::type_index ProjectileController::type() const {
    return typeid(ProjectileController);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_PROJECTILECONTROLLER_H
