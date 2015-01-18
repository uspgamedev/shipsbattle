#ifndef SHIPSBATTLE_SUBSYSTEMS_SENSORARRAY_H
#define SHIPSBATTLE_SUBSYSTEMS_SENSORARRAY_H

#include <shipsbattle/components/subsystems/poweredsystem.h>

namespace shipsbattle {
namespace components {
class Navigation;

namespace subsystems {

class SensorArray : public PoweredSystem {
public:
    SensorArray(const std::string& name);

    virtual double NeedsRecharge() override;
    virtual void OnRecharge(double energy) override;



protected:
    

    friend class Navigation;
    virtual void Update(double dt);
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_SENSORARRAY_H
