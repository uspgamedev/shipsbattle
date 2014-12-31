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

protected:
    friend class components::Hull;
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_SUBHULL_H
