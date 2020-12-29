#ifndef TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_KDL_HPP
#define TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_KDL_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <toppra/constraint/cartesian_velocity_norm.hpp>

namespace toppra {
namespace constraint {
namespace cartesianVelocityNorm {

class Kdl : public CartesianVelocityNorm {
  public:

    std::ostream& print(std::ostream& os) const
    {
      return CartesianVelocityNorm::print(os << "KDL - ");
    }

    void computeVelocity (const Vector& q, const Vector& qdot, Vector& v)
    {
        KDL::JntArrayVel joints(q.size());
        for (auto i = 0; i < q.size(); ++i)
        {
            joints.q(i) = q(i);
            joints.qdot(i) = qdot(i);
        }

        KDL::FrameVel frame;
        if (m_solver.JntToCart(joints, frame))
        {
            throw std::runtime_error("KDL JntToCart Error: " + std::string(m_solver.strError(m_solver.getError())));
        }

        v(0) = frame.p.v.x();
        v(1) = frame.p.v.y();
        v(2) = frame.p.v.z();

        v(3) = frame.M.w.x();
        v(4) = frame.M.w.y();
        v(5) = frame.M.w.z();
    }

    /// Constructor for constant velocity limit.
    Kdl (const KDL::Chain& chain, const Matrix& S, const double& limit)
      : CartesianVelocityNorm (S, limit)
      , m_chain (chain)
      , m_solver (m_chain)
    {
        // TODO
    }

  private:
    const KDL::Chain& m_chain;
    KDL::ChainFkSolverVel_recursive m_solver;
}; // class Kdl

} // namespace cartesianVelocityNorm
} // namespace constraint
} // namespace toppra

#endif
