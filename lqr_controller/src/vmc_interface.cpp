// vmc_interface.cpp
#include "lqr_controller/vmc_interface.h"
#include <cmath>

VMCInterface::VMCInterface(ros::NodeHandle& nh) {
    // 从参数服务器加载参数
    loadParameters(nh);
}

void VMCInterface::loadParameters(ros::NodeHandle& nh) {
    // 从参数服务器加载杆长参数
    nh.param("vmc/l1", l1, 0.05);   // 默认 5cm
    nh.param("vmc/l2", l2, 0.105);  // 默认 10.5cm
    nh.param("vmc/l3", l3, 0.105);  // 默认 10.5cm
    nh.param("vmc/l4", l4, 0.05);   // 默认 5cm
    nh.param("vmc/L", L, 0.06);     // 默认 6cm
    
    ROS_INFO("VMC参数已加载: l1=%.3f, l2=%.3f, l3=%.3f, l4=%.3f, L=%.3f", 
             l1, l2, l3, l4, L);
}

Eigen::Vector2d VMCInterface::calculateLegPosition(double phi1, double phi4) {
    // C++ implementation of leg_pos.m
    Eigen::Vector2d pos;
    
    // Calculate intermediate terms
    double cos_phi1 = cos(phi1);
    double cos_phi4 = cos(phi4);
    double sin_phi1 = sin(phi1);
    double sin_phi4 = sin(phi4);
    
    // Calculate coordinates of points B and D
    double xb = l1 * cos_phi1;
    double yb = l1 * sin_phi1;
    double xd = L + l4 * cos_phi4;
    double yd = l4 * sin_phi4;
    
    // Calculate phi2 using the geometry of the four-bar linkage
    double A0 = 2 * l2 * (xd - xb);
    double B0 = 2 * l2 * (yd - yb);
    double lBD_squared = pow(xd - xb, 2) + pow(yd - yb, 2);
    double C0 = pow(l2, 2) + lBD_squared - pow(l3, 2);
    double phi2 = 2 * atan2(B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2)), A0 + C0);
    
    // Calculate coordinates of point C
    double xc = xb + l2 * cos(phi2);
    double yc = yb + l2 * sin(phi2);
    
    // Calculate l0 and phi0
    double l0 = sqrt(pow(yc, 2) + pow(xc - L/2, 2));
    double phi0 = atan2(yc, xc - L/2);
    
    pos << l0, phi0;
    return pos;
}

Eigen::Vector2d VMCInterface::calculateLegSpeed(double dphi1, double dphi4, double phi1, double phi4) {
    // C++ implementation of leg_spd.m from the MATLAB generated function
    Eigen::Vector2d spd;
    
    // We'll implement a simplified version based on computing the Jacobian matrix analytically
    // and then multiplying by the angular velocities
    
    // Calculate intermediate terms
    double cos_phi1 = cos(phi1);
    double cos_phi4 = cos(phi4);
    double sin_phi1 = sin(phi1);
    double sin_phi4 = sin(phi4);
    
    // Calculate coordinates of points B and D
    double xb = l1 * cos_phi1;
    double yb = l1 * sin_phi1;
    double xd = L + l4 * cos_phi4;
    double yd = l4 * sin_phi4;
    
    // Calculate phi2 using the geometry of the four-bar linkage
    double A0 = 2 * l2 * (xd - xb);
    double B0 = 2 * l2 * (yd - yb);
    double lBD_squared = pow(xd - xb, 2) + pow(yd - yb, 2);
    double C0 = pow(l2, 2) + lBD_squared - pow(l3, 2);
    double discriminant = pow(A0, 2) + pow(B0, 2) - pow(C0, 2);
    
    if (discriminant < 0) {
        // This can happen due to numerical errors or if the mechanism is near a singularity
        ROS_WARN("Mechanism near singularity: discriminant = %f", discriminant);
        discriminant = 0;
    }
    
    double phi2 = 2 * atan2(B0 + sqrt(discriminant), A0 + C0);
    
    // Calculate dphi2 (derivative of phi2 with respect to time)
    // This is a complex expression derived from the four-bar linkage kinematics
    double cos_phi2 = cos(phi2);
    double sin_phi2 = sin(phi2);
    
    // Calculate partial derivatives of phi2 with respect to phi1 and phi4
    // Simplified approach using numerical differentiation
    const double delta = 1e-6;
    
    // Get phi2 at neighboring points
    double A0_dphi1 = 2 * l2 * (xd - (l1 * cos(phi1 + delta)));
    double B0_dphi1 = 2 * l2 * (yd - (l1 * sin(phi1 + delta)));
    double lBD_squared_dphi1 = pow(xd - (l1 * cos(phi1 + delta)), 2) + 
                               pow(yd - (l1 * sin(phi1 + delta)), 2);
    double C0_dphi1 = pow(l2, 2) + lBD_squared_dphi1 - pow(l3, 2);
    double disc_dphi1 = pow(A0_dphi1, 2) + pow(B0_dphi1, 2) - pow(C0_dphi1, 2);
    if (disc_dphi1 < 0) disc_dphi1 = 0;
    double phi2_dphi1 = 2 * atan2(B0_dphi1 + sqrt(disc_dphi1), A0_dphi1 + C0_dphi1);
    
    double A0_dphi4 = 2 * l2 * ((L + l4 * cos(phi4 + delta)) - xb);
    double B0_dphi4 = 2 * l2 * ((l4 * sin(phi4 + delta)) - yb);
    double lBD_squared_dphi4 = pow((L + l4 * cos(phi4 + delta)) - xb, 2) + 
                              pow((l4 * sin(phi4 + delta)) - yb, 2);
    double C0_dphi4 = pow(l2, 2) + lBD_squared_dphi4 - pow(l3, 2);
    double disc_dphi4 = pow(A0_dphi4, 2) + pow(B0_dphi4, 2) - pow(C0_dphi4, 2);
    if (disc_dphi4 < 0) disc_dphi4 = 0;
    double phi2_dphi4 = 2 * atan2(B0_dphi4 + sqrt(disc_dphi4), A0_dphi4 + C0_dphi4);
    
    // Calculate partial derivatives
    double dphi2_dphi1 = (phi2_dphi1 - phi2) / delta;
    double dphi2_dphi4 = (phi2_dphi4 - phi2) / delta;
    
    // Calculate dphi2
    double dphi2 = dphi2_dphi1 * dphi1 + dphi2_dphi4 * dphi4;
    
    // Calculate the coordinates of point C
    double xc = xb + l2 * cos_phi2;
    double yc = yb + l2 * sin_phi2;
    
    // Calculate derivatives of xc and yc
    double dxc = -l1 * sin_phi1 * dphi1 - l2 * sin_phi2 * dphi2;
    double dyc = l1 * cos_phi1 * dphi1 + l2 * cos_phi2 * dphi2;
    
    // Calculate l0 and phi0
    double l0 = sqrt(pow(yc, 2) + pow(xc - L/2, 2));
    // double phi0 = atan2(yc, xc - L/2);
    
    // Calculate dl0 and dphi0
    double dl0 = ((yc * dyc) + ((xc - L/2) * dxc)) / l0;
    double dphi0 = (dyc * (xc - L/2) - yc * dxc) / (pow(yc, 2) + pow(xc - L/2, 2));
    
    spd << dl0, dphi0;
    return spd;
}

Eigen::Vector2d VMCInterface::convertToMotorTorques(double F, double Tp, double phi1, double phi4) {
    // C++ implementation of leg_conv.m
    Eigen::Vector2d T;
    
    // Calculate intermediate terms
    double cos_phi1 = cos(phi1);
    double cos_phi4 = cos(phi4);
    double sin_phi1 = sin(phi1);
    double sin_phi4 = sin(phi4);
    
    // Calculate coordinates of points B and D
    double xb = l1 * cos_phi1;
    double yb = l1 * sin_phi1;
    double xd = L + l4 * cos_phi4;
    double yd = l4 * sin_phi4;
    
    // Calculate phi2 using the geometry of the four-bar linkage
    double A0 = 2 * l2 * (xd - xb);
    double B0 = 2 * l2 * (yd - yb);
    double lBD_squared = pow(xd - xb, 2) + pow(yd - yb, 2);
    double C0 = pow(l2, 2) + lBD_squared - pow(l3, 2);
    double discriminant = pow(A0, 2) + pow(B0, 2) - pow(C0, 2);
    
    if (discriminant < 0) {
        ROS_WARN("Mechanism near singularity: discriminant = %f", discriminant);
        discriminant = 0;
    }
    
    double phi2 = 2 * atan2(B0 + sqrt(discriminant), A0 + C0);
    
    // Calculate the coordinates of point C
    double xc = xb + l2 * cos(phi2);
    double yc = yb + l2 * sin(phi2);
    
    // Calculate the Jacobian matrix to convert from [l0, phi0] to [phi1, phi4]
    // We'll use the same numerical approach as in calculateLegSpeed
    
    // Get position at current state
    Eigen::Vector2d pos_current = calculateLegPosition(phi1, phi4);
    double l0 = pos_current(0);
    double phi0 = pos_current(1);
    
    // Get positions at perturbed states
    const double delta = 1e-6;
    Eigen::Vector2d pos_dphi1 = calculateLegPosition(phi1 + delta, phi4);
    Eigen::Vector2d pos_dphi4 = calculateLegPosition(phi1, phi4 + delta);
    
    // Compute Jacobian
    Eigen::Matrix2d J;
    J << (pos_dphi1(0) - l0) / delta, (pos_dphi4(0) - l0) / delta,
         (pos_dphi1(1) - phi0) / delta, (pos_dphi4(1) - phi0) / delta;
    
    // Convert forces to torques using J transpose
    Eigen::Vector2d forces;
    forces << F, Tp;
    T = J.transpose() * forces;
    
    return T;
}