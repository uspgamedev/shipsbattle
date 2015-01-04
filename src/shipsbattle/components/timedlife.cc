#include <shipsbattle/components/timedlife.h>

#include <ugdk/action/3D/scene3d.h>
#include <ugdk/action/3D/element.h>
#include <ugdk/system/task.h>

#include <list>
#include <iostream>


namespace shipsbattle {
namespace components {

TimedLife::TimedLife(double lifetime) : lifetime_(lifetime), elapsed_(0.0) {

}

void TimedLife::Update(double dt) {
    elapsed_ += dt;
    if (elapsed_ >= lifetime_) {
        std::cout << "TIMED LIFE EXPIRED - should destroy " << owner()->name() << std::endl;
        owner()->scene().DestroyAndRemoveElement(owner());
    }
}

} // namespace components
} // namespace shipsbattle
