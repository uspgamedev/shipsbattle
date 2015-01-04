#ifndef SHIPSBATTLE_OBJECTS_TYPES_H_
#define SHIPSBATTLE_OBJECTS_TYPES_H_

namespace shipsbattle {
namespace objects {

#define BIT(x) (1<<(x))
enum ObjectTypes {
    SHIP = BIT(6),
    PROJECTILE = BIT(7),
    //WAT2 = BIT(8),
    //WAT3 = BIT(9),
    //WAT4 = BIT(10)
};


} // namespace objects
} // namespace shipsbattle
#endif //SHIPSBATTLE_OBJECTS_TYPES_H_
