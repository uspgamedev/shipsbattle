#ifndef SHIPSBATTLE_SUBSYSTEMS_TYPEDEFS_H
#define SHIPSBATTLE_SUBSYSTEMS_TYPEDEFS_H

#include <functional>

namespace shipsbattle {
namespace components {
namespace subsystems {

/** Decayment Function for damage calculations. Receives distance to the point of impact, 
and splash radius (max distance). Should return a value to multiply damage. (0.5 -> half damage, and so on)*/
using DecaymentFunction = std::function<double(double,double)>;

namespace DecaymentFunctions {
const DecaymentFunction CONSTANT = [](double, double) { return 1.0; };
const DecaymentFunction LINEAR = [](double dist, double splash) { return dist / splash; };
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_TYPEDEFS_H

