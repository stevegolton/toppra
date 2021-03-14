#ifndef TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_VARYING_HPP
#define TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_VARYING_HPP

#include <toppra/constraint/cartesian_velocity_norm.hpp>

namespace toppra {
namespace constraint {

/**
\brief A Joint Velocity Constraint class where velocity limits vary across the path, and these velocities are known upfront.
*/
class CartesianVelocityNormVarying : public CartesianVelocityNorm
{
public:
  /**
  \brief Constructor.
  \param nDof The number of joints on the arm, or the number of elements
  \param lower nxm size matrix where n==nDof and m==number of points
   * 
   */
  CartesianVelocityNormVarying(const Matrix& S, toppra::Vector limit, std::vector<toppra::value_type> times)
    : CartesianVelocityNorm(S)
    , m_limit_vec (limit)
    , m_times(times)
  {
    //
  }

  void computeVelocityLimit(toppra::value_type time) override
  {
    auto iter = std::lower_bound(m_times.begin(), m_times.end(), time);
    if (iter == m_times.end())
    {
      // throw something here!
    }

    auto in = iter - m_times.begin();
    m_limit = m_limit_vec(in);
  }

private:
  toppra::Vector m_limit_vec;
  std::vector<toppra::value_type> m_times;
}; // class LinearJointVelocityVarying
} // namespace constraint
} // namespace toppra

#endif
