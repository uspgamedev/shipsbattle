#ifndef SHIPSBATTLE_UTILS_MATRIX_H_
#define SHIPSBATTLE_UTILS_MATRIX_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <limits>

namespace shipsbattle {
namespace utils {

class Matrix : public Eigen::MatrixXd {
public:
    Matrix(int rows, int cols) : Eigen::MatrixXd(rows, cols) {}

    Eigen::MatrixXd pseudoInverse(double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(*this, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(cols(), rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
};


} // namespace objects
} // namespace shipsbattle
#endif //SHIPSBATTLE_UTILS_MATRIX_H_
