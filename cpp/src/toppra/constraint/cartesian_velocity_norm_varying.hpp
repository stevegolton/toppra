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
    auto second = std::lower_bound(m_times.begin(), m_times.end(), time);
    if (second == m_times.end())
    {
      second--;
      auto index = second - m_times.begin();
      m_limit = m_limit_vec(index);
    }
    else if (second == m_times.begin())
    {
      m_limit = m_limit_vec(0);
    }
    else
    {
      auto first = second - 1;
      auto a_index = first - m_times.begin();
      auto a = *first;
      auto b_index = second - m_times.begin();
      auto b = *second;
      auto alpha = (time - a) / (b - a);
      m_limit = lerp(m_limit_vec(a_index), m_limit_vec(b_index), alpha);
    }
  }

private:
  toppra::value_type lerp(toppra::value_type a, toppra::value_type b, double alpha)
  {
    return (a*(1.0-alpha) + b*alpha);
  }
  toppra::Vector m_limit_vec;
  std::vector<toppra::value_type> m_times;
}; // class LinearJointVelocityVarying
} // namespace constraint
} // namespace toppra

#endif
