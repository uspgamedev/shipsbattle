#ifndef SHIPSBATTLE_SUBSYSTEMS_SUBHULL_H
#define SHIPSBATTLE_SUBSYSTEMS_SUBHULL_H

#include <shipsbattle/components/subsystems/damageablesystem.h>

namespace shipsbattle {
namespace components {
class Hull;

namespace subsystems {

class SubHull : public DamageableSystem {
public:
    SubHull(const std::string& name) : DamageableSystem(name) {}

    Hull* parent() const { return parent_; }

protected:
    friend class components::Hull;

    Hull* parent_;

    void OnRegister(Hull* parent) { parent_ = parent; RegisterToHull(parent); }
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_SUBHULL_H
