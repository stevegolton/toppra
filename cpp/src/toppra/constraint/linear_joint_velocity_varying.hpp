#ifndef TOPPRA_CONSTRAINT_LINEAR_JOINT_VELOCITY_VARYING_HPP
#define TOPPRA_CONSTRAINT_LINEAR_JOINT_VELOCITY_VARYING_HPP

#include <toppra/constraint/linear_joint_velocity.hpp>

namespace toppra {
namespace constraint {

/**
\brief A Joint Velocity Constraint class where velocity limits vary across the path, and these velocities are known upfront.
*/
class LinearJointVelocityVarying : public toppra::constraint::LinearJointVelocity
{
public:
  /**
  \brief Constructor.
  \param nDof The number of joints on the arm, or the number of elements
  \param lower nxm size matrix where n==nDof and m==number of points
   * 
   */
  LinearJointVelocityVarying(const int nDof, toppra::Matrix lower, toppra::Matrix upper, std::vector<toppra::value_type> times)
    : LinearJointVelocity(nDof)
    , m_lower_vec (lower)
    , m_upper_vec (upper)
    , m_times(times)
  {
    //
  }

  void computeVelocityLimits(toppra::value_type time) override
  {
    auto iter = std::lower_bound(m_times.begin(), m_times.end(), time);
    if (iter == m_times.end())
    {
      // throw something here!
    }

    auto in = iter - m_times.begin();

    m_lower = m_lower_vec.row(in);
    m_upper = m_upper_vec.row(in);
  }

private:
  toppra::Matrix m_lower_vec, m_upper_vec;
  std::vector<toppra::value_type> m_times;
}; // class LinearJointVelocityVarying
} // namespace constraint
} // namespace toppra

#endif
