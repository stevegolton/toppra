#ifndef TOPPRA_CONSTRAINT_LINEAR_JOINT_ACCELERATION_HPP
#define TOPPRA_CONSTRAINT_LINEAR_JOINT_ACCELERATION_HPP

#include <toppra/constraint.hpp>

namespace toppra {
namespace constraint {

/// A Joint Acceleration Constraint class.
class LinearJointAcceleration : public LinearConstraint {
  public:
    LinearJointAcceleration (const Vector& lowerAlimit, const Vector& upperAlimit)
      : LinearConstraint (lowerAlimit.size() * 2, lowerAlimit.size(), false, false, false)
      , m_lower (lowerAlimit)
      , m_upper (upperAlimit)
    {
      check();
    }

    virtual std::ostream& print(std::ostream& os) const;

  protected:
    LinearJointAcceleration (const int nDof)
      : LinearConstraint (2*nDof, nDof, false, false, false)
      , m_lower (nDof)
      , m_upper (nDof)
    {

    }
    /**
      \brief Computes the acceleration limit at time \c time or waypoint index \c index.

      The result must be stored into attributes
      LinearJointAcceleration::m_lower and LinearJointAcceleration::m_upper.
      */
    virtual void computeAccelerationLimits(value_type time) { (void)time; }

    Vector m_lower, m_upper;

  private:
    void check();

    void computeParams_impl(const GeometricPath& path,
        const Vector& gridpoints,
        Vectors& a, Vectors& b, Vectors& c,
        Matrices& F, Vectors& g,
        Bounds& ubound, Bounds& xbound);

}; // class LinearJointAcceleration
} // namespace constraint
} // namespace toppra

#endif
