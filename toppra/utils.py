"""
Some utility functions need to generate PathConstraints. Most are
specific to different scenarios.
"""
import logging
import numpy as np

LOGGER = logging.getLogger(__name__)


def compute_jacobian_wrench(robot, link, point):
    """ Compute the wrench Jacobian for link at point point.

    We look for J_wrench such that
          J_wrench.T * wrench = J_trans.T * F + J_rot.T * tau
    return the induced generalized joint torques.

    J_wrench is computed by stacking J_translation and J_rotation

    """
    J_trans = robot.ComputeJacobianTranslation(link.GetIndex(), point)
    J_rot = robot.ComputeJacobianAxisAngle(link.GetIndex())
    J_wrench = np.vstack((J_trans, J_rot))
    return J_wrench


def inv_dyn(rave_robot, q, qd, qdd, forceslist=None, returncomponents=True):
    """Inverse dynamics equation.

    Simple wrapper around OpenRAVE's ComputeInverseDynamics
    function. Return the numerical values of the components of the
    inverse dynamics equation.

          M(q) qdd + C(q, qd) qd + g(q)
        = t1 + t2 + t3

    Parameters
    ----------
    rave_robot : OpenRAVE.robot
    q : (N, ) ndarray
        Joint position.
    qd : (N, ) ndarray
        Joint velocity.
    qdd : (N, ) ndarray
        Joint acceleration.
    returncomponents : Bool
        If True, return the list [t1, t2, t3]
        If False, return t1 + t2 + t3

    Returns
    -------
    res : (3, ) List, or ndarray
        See returncomponents parameter.
    """
    if np.isscalar(q):  # Scalar case
        q_ = [q]
        qd_ = [qd]
        qdd_ = [qdd]
    else:
        q_ = q
        qd_ = qd
        qdd_ = qdd

    # Temporary remove kinematic Limits
    vlim = rave_robot.GetDOFVelocityLimits()
    alim = rave_robot.GetDOFAccelerationLimits()
    rave_robot.SetDOFVelocityLimits(100 * vlim)
    rave_robot.SetDOFAccelerationLimits(100 * alim)
    # Do computation
    with rave_robot:
        rave_robot.SetDOFValues(q_)
        rave_robot.SetDOFVelocities(qd_)
        res = rave_robot.ComputeInverseDynamics(
            qdd_, forceslist, returncomponents=returncomponents)
    # Restore kinematic limits
    rave_robot.SetDOFVelocityLimits(vlim)
    rave_robot.SetDOFAccelerationLimits(alim)
    return res


def smooth_singularities(pp, us, xs, vs=None):
    """Smooth jitters due to singularities.

    Solving TOPP for discrete problem generated from collocation
    scheme tends to create jitters. This function finds and smooths
    them.

    Parameters
    ----------
    pp: PathParameterization
    us: ndarray
    xs: ndarray
    vs: ndarray, optional

    Returns
    -------
    us_smth: ndarray, shaped (N, )
    xs_smth: ndarray, shaped (N+1, )
    vs_smth: ndarray, shaped (N+1, m)
    """
    # Find the indices
    singular_indices = []
    uds = np.diff(us, n=1)
    for i in range(pp.N - 3):
        if uds[i] < 0 and uds[i+1] > 0 and uds[i+2] < 0:
            LOGGER.debug("Found potential singularity at {:d}".format(i))
            singular_indices.append(i)
    LOGGER.debug("All singularities found: %s", singular_indices)

    # Smooth the singularities
    xs_smth = np.copy(xs)
    us_smth = np.copy(us)
    if vs is not None:
        vs_smth = np.copy(vs)
    for index in singular_indices:
        idstart = max(0, index)
        idend = min(pp.N, index + 4)
        xs_smth[range(idstart, idend + 1)] = (
            xs_smth[idstart] + (xs_smth[idend] - xs_smth[idstart]) *
            np.linspace(0, 1, idend + 1 - idstart))
        if vs is not None:
            data = [vs_smth[idstart] +
                    (xs_smth[idend] - xs_smth[idstart]) * frac
                    for frac in np.linspace(0, 1, idend + 1 - idstart)]
            vs_smth[range(idstart, idend + 1)] = np.array(data)

    for i in range(pp.N):
        us_smth[i] = (xs_smth[i+1] - xs_smth[i]) / 2 / (pp.ss[i+1] - pp.ss[i])

    if vs is not None:
        return us_smth, xs_smth, vs_smth
    return us_smth, xs_smth
