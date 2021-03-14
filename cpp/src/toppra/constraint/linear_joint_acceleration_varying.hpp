#ifndef TOPPRA_CONSTRAINT_LINEAR_JOINT_ACCELERATION_VARYING_HPP
#define TOPPRA_CONSTRAINT_LINEAR_JOINT_ACCELERATION_VARYING_HPP

#include <toppra/constraint/linear_joint_acceleration.hpp>

namespace toppra {
namespace constraint {

/**
\brief A Joint Acceleration Constraint class where velocity limits vary across the path, and these velocities are known upfront.
*/
class LinearJointAccelerationVarying : public toppra::constraint::LinearJointAcceleration
{
public:
  /**
  \brief Constructor.
  \param nDof The number of joints on the arm, or the number of elements
  \param lower nxm size matrix where n==nDof and m==number of points
   * 
   */
  LinearJointAccelerationVarying(const int nDof, toppra::Matrix lower, toppra::Matrix upper, std::vector<toppra::value_type> times)
    : LinearJointAcceleration(nDof)
    , m_lower_vec (lower)
    , m_upper_vec (upper)
    , m_times(times)
  {
    //
  }

  void computeAccelerationLimits(value_type time) override
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
  /**
   \brief Linear interpolation between two vectors.
   
   \param a The first vector.
   \param b The second vector.
   \param alpha The blend amount, 0.0 == all a, 1.0 == all b.
   \return toppra::Vector The resultnat interpolated vector.
  */
  toppra::Vector lerp(const toppra::Vector &a, const toppra::Vector &b, double alpha)
  {
    //
  }

  toppra::Matrix m_upper_vec;
  toppra::Matrix m_lower_vec;
  std::vector<toppra::value_type> m_times;
}; // class LinearJointAccelerationVarying
} // namespace constraint
} // namespace toppra

#endif
