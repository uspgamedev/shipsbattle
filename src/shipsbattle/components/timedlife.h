#ifndef SHIPSBATTLE_COMPONENTS_TIMEDLIFE_H
#define SHIPSBATTLE_COMPONENTS_TIMEDLIFE_H

#include <shipsbattle/components/updateablecomponent.h>

namespace shipsbattle {
namespace components {

class TimedLife : public UpdateableComponent {

public:
    TimedLife(double lifetime);

    virtual std::type_index type() const override;
    
    void Update(double dt) override;

  protected:
    double lifetime_;
    double elapsed_;
};

inline std::type_index TimedLife::type() const {
    return typeid(TimedLife);
}

} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_COMPONENTS_TIMEDLIFE_H
