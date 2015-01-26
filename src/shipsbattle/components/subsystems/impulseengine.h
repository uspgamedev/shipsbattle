#ifndef SHIPSBATTLE_SUBSYSTEMS_IMPULSEENGINE_H
#define SHIPSBATTLE_SUBSYSTEMS_IMPULSEENGINE_H

#include <shipsbattle/components/subsystems/poweredsystem.h>

namespace shipsbattle {
namespace components {
class Motion;

namespace subsystems {

class ImpulseEngine : public PoweredSystem {
public:
    ImpulseEngine(const std::string& name);

    virtual double NeedsRecharge() override;
    virtual void OnRecharge(double energy) override;


protected:

    friend class Motion;
    virtual void Update(double dt);
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_IMPULSEENGINE_H
