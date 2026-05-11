#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}

bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const
{
    Eigen::Map<const Eigen::Vector3d> p_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

    Eigen::Map<const Eigen::Vector3d> p_x(x);
    Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

    Eigen::Map<Eigen::Vector3d> dp(y_minus_x);
    Eigen::Map<Eigen::Vector3d> dtheta(y_minus_x + 3);

    dp = p_y - p_x;
    Eigen::Quaterniond dq = q_y * q_x.inverse();
    dtheta = 2.0 * dq.vec();

    return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    
    // Jacobian of Minus operation
    // d(y - x) / d(x) 
    // Position part: d(p_y - p_x) / d(p_x) = -I
    j.block<3, 3>(0, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
    
    // Rotation part: d(2*vec(q_y * q_x^-1)) / d(q_x)
    // Using quaternion calculus, for small perturbations
    Eigen::Map<const Eigen::Quaterniond> q(x + 3);
    
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();
    
    // Jacobian of quaternion inverse conjugate operation
    j(3, 3) = -qw;  j(3, 4) = qz;  j(3, 5) = -qy;  j(3, 6) = qx;
    j(4, 3) = -qz;  j(4, 4) = -qw; j(4, 5) = qx;   j(4, 6) = qy;
    j(5, 3) = qy;   j(5, 4) = -qx; j(5, 5) = -qw;  j(5, 6) = qz;
    
    j.block<3, 4>(3, 3) *= 2.0;

    return true;
}
