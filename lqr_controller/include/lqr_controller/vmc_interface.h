// vmc_interface.h
#ifndef VMC_INTERFACE_H
#define VMC_INTERFACE_H

#include <Eigen/Dense>
#include <ros/ros.h>

/**
 * @brief Interface for VMC (Virtual Model Control) functions
 * 
 * This class implements C++ versions of the MATLAB functions:
 * - leg_pos: Calculate leg length (l0) and angle (phi0) from motor positions
 * - leg_spd: Calculate leg length and angle derivatives from motor positions and velocities
 * - leg_conv: Convert from force F and torque Tp to motor torques T1 and T2
 */
class VMCInterface {
public:
    VMCInterface(ros::NodeHandle& nh);
    ~VMCInterface() {}

    /**
     * @brief Calculate leg position (length and angle) from motor angles
     * 
     * C++ implementation of the MATLAB leg_pos function
     * 
     * @param phi1 Left motor angle (rad)
     * @param phi4 Right motor angle (rad)
     * @return Eigen::Vector2d [l0, phi0] - leg length and angle
     */
    Eigen::Vector2d calculateLegPosition(double phi1, double phi4);

    /**
     * @brief Calculate leg velocity (length and angle rates) from motor states
     * 
     * C++ implementation of the MATLAB leg_spd function
     * 
     * @param dphi1 Left motor angular velocity (rad/s)
     * @param dphi4 Right motor angular velocity (rad/s)
     * @param phi1 Left motor angle (rad)
     * @param phi4 Right motor angle (rad)
     * @return Eigen::Vector2d [dl0, dphi0] - leg length and angle rates
     */
    Eigen::Vector2d calculateLegSpeed(double dphi1, double dphi4, double phi1, double phi4);

    /**
     * @brief Convert from VMC force and torque to motor torques
     * 
     * C++ implementation of the MATLAB leg_conv function
     * 
     * @param F Desired force
     * @param Tp Desired torque
     * @param phi1 Left motor angle (rad)
     * @param phi4 Right motor angle (rad)
     * @return Eigen::Vector2d [T1, T2] - motor torques
     */
    Eigen::Vector2d convertToMotorTorques(double F, double Tp, double phi1, double phi4);

private:
    // 从参数服务器加载参数
    void loadParameters(ros::NodeHandle& nh);

    // 杆长参数
    double l1;  // 连杆1长度(m)
    double l2;  // 连杆2长度(m)
    double l3;  // 连杆3长度(m)
    double l4;  // 连杆4长度(m)
    double L;   // 底盘长度(m)
};

#endif // VMC_INTERFACE_H