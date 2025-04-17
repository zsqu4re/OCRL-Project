#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <math.h>

// 导入功能函数
#include "leg_pos.h"
#include "leg_spd.h"
#include "leg_conv.h"
#include "lqr_k.h"
#include "pid.h"  // 导入单独的PID头文件

// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(balance_robot_controller::BalanceRobotController, controller_interface::ControllerBase)


// 腿部姿态结构体
struct LegPos
{
  float angle, length;   // rad, m
  float dAngle, dLength; // rad/s, m/s
  float ddLength;        // m/s^2
};

// 状态变量结构体
struct StateVar
{
  float theta, dTheta;
  float x, dx;
  float phi, dPhi;
};

// 目标量结构体
struct Target
{
  float position;   // m
  float speedCmd;   // m/s
  float speed;      // m/s
  float yawSpeedCmd; // rad/s
  float yawAngle;   // rad
  float rollAngle;  // rad
  float legLength;  // m
};

// 触地检测数据结构体
struct GroundDetector
{
  float leftSupportForce, rightSupportForce;
  bool isTouchingGround, isCushioning;
};

namespace balance_robot_controller {

// 平衡机器人控制器类
class BalanceRobotController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
private:
  // 关节句柄
  std::vector<hardware_interface::JointHandle> joints_;
  
  // ROS相关
  ros::Subscriber imu_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher state_pub_;
  ros::Publisher joint_cmd_pub_;

  
  // IMU数据
  struct {
    float yaw, pitch, roll;     // rad
    float yawSpd, pitchSpd, rollSpd; // rad/s
    float zAccel;               // m/s^2
  } imuData;
  
  // 电机参数
  struct Motor {
    float speed;               // rad/s
    float angle, offsetAngle;  // rad
    float torque, torqueRatio; // Nm
    float dir;                 // 1 or -1
  } leftJoint[2], rightJoint[2], leftWheel, rightWheel;
  
  // 腿部姿态
  LegPos leftLegPos, rightLegPos;
  
  // 状态变量
  StateVar stateVar;
  
  // 目标量
  Target target;
  
  // 触地检测
  GroundDetector groundDetector;
  
  // 控制器
  CascadePID legAnglePID, legLengthPID;
  CascadePID yawPID, rollPID;
  
  // 控制参数
  const float wheelRadius = 0.026f; // m, 车轮半径
  const float legMass = 0.05f;     // kg, 腿部质量
  
  ros::Time last_time_;
  double dt_;
  
public:
  BalanceRobotController() {}
  
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
  {
    // 获取关节句柄
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names))
    {
      ROS_ERROR("No 'joints' parameter in controller");
      return false;
    }
    
    if (joint_names.size() != 6)
    {
      ROS_ERROR("Expected 6 joints in controller");
      return false;
    }
    
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception getting joint handle: " << e.what());
        return false;
      }
    }
    
    joint_cmd_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/balance_robot/joint_cmd", 1);

    // 初始化电机参数
    Motor_InitAll();
    
    // 初始化控制器
    Ctrl_Init();


    
    // 初始化目标值
    target.rollAngle = 0.0f;
    target.legLength = 0.07f;
    target.speed = 0.0f;
    target.position = 0.0f;
    target.yawAngle = 0.0f;
    target.yawSpeedCmd = 0.0f;
    target.speedCmd = 0.0f;
    
    // 初始化触地检测
    groundDetector.leftSupportForce = 10.0f;
    groundDetector.rightSupportForce = 10.0f;
    groundDetector.isTouchingGround = true;
    groundDetector.isCushioning = false;
    
    // 订阅IMU数据
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &BalanceRobotController::imuCallback, this);
    
    // 订阅速度指令
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &BalanceRobotController::cmdVelCallback, this);
    
    // 发布状态数据
    state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/robot_state", 1);
    
    ROS_INFO("Balance Robot Controller initialized");
    
    return true;
  }

  void starting(const ros::Time& time)
  {
    // 初始时间
    last_time_ = time;
    
    // 清空PID
    ClearAllPID();
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // 计算时间间隔
    dt_ = period.toSec();
    
    // 更新腿部姿态
    UpdateLegPos();
    
    // 更新状态变量
    UpdateStateVar();
    
    // 更新目标量
    UpdateTarget();
    
    // 执行控制
    Control();
    
    // 发布状态
    PublishState();
  }

  void stopping(const ros::Time& time) {}

private:
  // 初始化所有电机
  void Motor_InitAll()
  {
    // 左腿前关节
    leftJoint[0].offsetAngle = 1.431f;
    leftJoint[0].torqueRatio = 0.0316f;
    leftJoint[0].dir = -1.0f;
    
    // 左腿后关节
    leftJoint[1].offsetAngle = -7.76f;
    leftJoint[1].torqueRatio = 0.0317f;
    leftJoint[1].dir = 1.0f;
    
    // 左轮
    leftWheel.offsetAngle = 0.0f;
    leftWheel.torqueRatio = 0.0096f;
    leftWheel.dir = 1.0f;
    
    // 右腿前关节
    rightJoint[0].offsetAngle = 0.343f;
    rightJoint[0].torqueRatio = 0.0299f;
    rightJoint[0].dir = -1.0f;
    
    // 右腿后关节
    rightJoint[1].offsetAngle = -2.446f;
    rightJoint[1].torqueRatio = 0.0321f;
    rightJoint[1].dir = -1.0f;
    
    // 右轮
    rightWheel.offsetAngle = 0.0f;
    rightWheel.torqueRatio = 0.0101f;
    rightWheel.dir = 1.0f;
  }

  // 初始化控制器
  void Ctrl_Init()
  {
    // 初始化各个PID参数
    // Yaw控制PID
    PID_Init(&yawPID.inner, 0.01f, 0.0f, 0.0f, 0.0f, 0.1f);
    PID_Init(&yawPID.outer, 10.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    
    // Roll控制PID
    PID_Init(&rollPID.inner, 1.0f, 0.0f, 5.0f, 0.0f, 5.0f);
    PID_Init(&rollPID.outer, 20.0f, 0.0f, 0.0f, 0.0f, 3.0f);
    PID_SetErrLpfRatio(&rollPID.inner, 0.1f);
    
    // 腿长控制PID
    PID_Init(&legLengthPID.inner, 10.0f, 1.0f, 30.0f, 2.0f, 10.0f);
    PID_Init(&legLengthPID.outer, 5.0f, 0.0f, 0.0f, 0.0f, 0.5f);
    PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);
    
    // 腿角控制PID
    PID_Init(&legAnglePID.inner, 0.04f, 0.0f, 0.0f, 0.0f, 1.0f);
    PID_Init(&legAnglePID.outer, 12.0f, 0.0f, 0.0f, 0.0f, 20.0f);
    PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);
  }

  // 清空所有PID
  void ClearAllPID()
  {
    PID_Clear(&yawPID.inner);
    PID_Clear(&yawPID.outer);
    PID_Clear(&rollPID.inner);
    PID_Clear(&rollPID.outer);
    PID_Clear(&legLengthPID.inner);
    PID_Clear(&legLengthPID.outer);
    PID_Clear(&legAnglePID.inner);
    PID_Clear(&legAnglePID.outer);
  }

  // IMU回调函数
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    // 从IMU获取欧拉角
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // 更新IMU数据
    imuData.roll = roll;
    imuData.pitch = pitch;
    imuData.yaw = yaw;
    
    // 更新角速度
    imuData.rollSpd = msg->angular_velocity.x;
    imuData.pitchSpd = msg->angular_velocity.y;
    imuData.yawSpd = msg->angular_velocity.z;
    
    // 更新Z轴加速度
    imuData.zAccel = msg->linear_acceleration.z;
  }

  // 速度指令回调函数
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // 更新速度和转向指令
    target.speedCmd = msg->linear.x;
    target.yawSpeedCmd = msg->angular.z;
  }

  // 更新腿部姿态
  void UpdateLegPos()
  {
    // 根据实际关节映射调整索引
    leftJoint[0].angle = joints_[0].getPosition();  // joint01_left (左前关节)
    leftJoint[0].speed = joints_[0].getVelocity();
    
    rightJoint[1].angle = joints_[1].getPosition(); // joint01_right (右后关节)
    rightJoint[1].speed = joints_[1].getVelocity();
    
    leftJoint[1].angle = joints_[2].getPosition();  // joint04_left (左后关节)
    leftJoint[1].speed = joints_[2].getVelocity();
    
    rightJoint[0].angle = joints_[3].getPosition(); // joint04_right (右前关节)
    rightJoint[0].speed = joints_[3].getVelocity();
    
    leftWheel.angle = joints_[4].getPosition();     // joint_tire_left
    leftWheel.speed = joints_[4].getVelocity();
    
    rightWheel.angle = joints_[5].getPosition();    // joint_tire_right
    rightWheel.speed = joints_[5].getVelocity();
    
  
    float legPos[2], legSpd[2];
    float lastLeftDLength = leftLegPos.dLength;
    float lastRightDLength = rightLegPos.dLength;
    
    // 计算左腿位置
    leg_pos(leftJoint[1].angle, leftJoint[0].angle, legPos);
    leftLegPos.length = legPos[0];
    leftLegPos.angle = legPos[1];
    
    // 计算左腿速度
    leg_spd(leftJoint[1].speed, leftJoint[0].speed, leftJoint[1].angle, leftJoint[0].angle, legSpd);
    leftLegPos.dLength = legSpd[0];
    leftLegPos.dAngle = legSpd[1];
    
    // 计算左腿加速度 (简单差分)
    leftLegPos.ddLength = (leftLegPos.dLength - lastLeftDLength) / dt_;
    
    // 计算右腿位置
    leg_pos(rightJoint[1].angle, rightJoint[0].angle, legPos);
    rightLegPos.length = legPos[0];
    rightLegPos.angle = legPos[1];
    
    // 计算右腿速度
    leg_spd(rightJoint[1].speed, rightJoint[0].speed, rightJoint[1].angle, rightJoint[0].angle, legSpd);
    rightLegPos.dLength = legSpd[0];
    rightLegPos.dAngle = legSpd[1];
    
    // 计算右腿加速度 (简单差分)
    rightLegPos.ddLength = (rightLegPos.dLength - lastRightDLength) / dt_;
  }

  // 更新状态变量
  void UpdateStateVar()
  {
    stateVar.phi = imuData.pitch;
    stateVar.dPhi = imuData.pitchSpd;
    stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
    stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
    stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - imuData.pitch;
    stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imuData.pitchSpd;
  }

  // 更新目标量
  void UpdateTarget()
  {
    // 计算速度斜坡
    float legLength = (leftLegPos.length + rightLegPos.length) / 2;
    float speedSlopeStep = -(legLength - 0.07f) * 0.02f + 0.002f;
    
    if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
      target.speed = target.speedCmd;
    else
    {
      if(target.speedCmd - target.speed > 0)
        target.speed += speedSlopeStep;
      else
        target.speed -= speedSlopeStep;
    }
    
    // 计算位置目标
    target.position += target.speed * dt_;
    
    // 限制位置目标
    if(target.position - stateVar.x > 0.1f)
      target.position = stateVar.x + 0.1f;
    else if(target.position - stateVar.x < -0.1f)
      target.position = stateVar.x - 0.1f;
    
    // 限制速度目标
    if(target.speed - stateVar.dx > 0.3f)
      target.speed = stateVar.dx + 0.3f;
    else if(target.speed - stateVar.dx < -0.3f)
      target.speed = stateVar.dx - 0.3f;
    
    // 计算yaw方位角目标
    target.yawAngle += target.yawSpeedCmd * dt_;
  }

  // 主控制函数
  void Control()
  {
    // 手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
    float kRatio[2][6] = {
      {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
      {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}
    };
    float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;
    
    // 计算LQR反馈矩阵
    float kRes[12] = {0}, k[2][6] = {0};
    float legLength = (leftLegPos.length + rightLegPos.length) / 2;
    
    lqr_k(legLength, kRes);
    
    if(groundDetector.isTouchingGround) // 正常触地状态
    {
      for (int i = 0; i < 6; i++)
      {
        for (int j = 0; j < 2; j++)
          k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
      }
    }
    else // 腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
    {
      memset(k, 0, sizeof(k));
      k[1][0] = kRes[1] * -2;
      k[1][1] = kRes[3] * -10;
    }
    
    // 准备状态变量
    float x[6] = {
      stateVar.theta, 
      stateVar.dTheta, 
      stateVar.x - target.position, 
      stateVar.dx - target.speed, 
      stateVar.phi, 
      stateVar.dPhi
    };
    
    // 矩阵相乘，计算LQR输出
    float lqrOutT = 0, lqrOutTp = 0;
    for (int i = 0; i < 6; i++) {
      lqrOutT += k[0][i] * x[i];
      lqrOutTp += k[1][i] * x[i];
    }
    
    // 计算yaw轴PID输出
    PID_CascadeCalc(&yawPID, target.yawAngle, imuData.yaw, imuData.yawSpd);
    
    // 设定车轮电机输出扭矩
    float leftWheelTorque = 0, rightWheelTorque = 0;
    
    if(groundDetector.isTouchingGround) // 正常接地状态
    {
      leftWheelTorque = -lqrOutT * lqrTRatio - yawPID.output;
      rightWheelTorque = -lqrOutT * lqrTRatio + yawPID.output;
    }
    
    // 计算腿长PID输出
    float targetLegLength = groundDetector.isTouchingGround && !groundDetector.isCushioning ? target.legLength : 0.12f;
    PID_CascadeCalc(&legLengthPID, targetLegLength, legLength, (leftLegPos.dLength + rightLegPos.dLength) / 2);
    
    // 计算roll轴PID输出
    PID_CascadeCalc(&rollPID, target.rollAngle, imuData.roll, imuData.rollSpd);
    
    // 计算左右腿推力
    float leftForce = legLengthPID.output;
    float rightForce = legLengthPID.output;
    
    if(groundDetector.isTouchingGround && !groundDetector.isCushioning)
    {
      leftForce += 6 - rollPID.output;
      rightForce += 6 + rollPID.output;
    }
    
    // 保护腿部不能伸太长
    if(leftLegPos.length > 0.12f)
      leftForce -= (leftLegPos.length - 0.12f) * 100;
    if(rightLegPos.length > 0.12f)
      rightForce -= (rightLegPos.length - 0.12f) * 100;
    
    // 计算左右腿的地面支持力
    groundDetector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (leftLegPos.ddLength - imuData.zAccel);
    groundDetector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (rightLegPos.ddLength - imuData.zAccel);
    
    // 更新离地检测器数据
    static ros::Time lastTouchTime;
    bool isTouchingGround = groundDetector.leftSupportForce > 3 && groundDetector.rightSupportForce > 3;
    
    if(!isTouchingGround && (ros::Time::now() - lastTouchTime).toSec() < 1.0)
      isTouchingGround = true;
    
    if(!groundDetector.isTouchingGround && isTouchingGround)
    {
      target.position = stateVar.x;
      groundDetector.isCushioning = true;
      lastTouchTime = ros::Time::now();
    }
    
    if(groundDetector.isCushioning && legLength < target.legLength)
      groundDetector.isCushioning = false;
    
    groundDetector.isTouchingGround = isTouchingGround;
    
    // 计算左右腿角度差PID输出
    PID_CascadeCalc(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, leftLegPos.dAngle - rightLegPos.dAngle);
    
    // 计算髋关节扭矩输出
    float leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.07f);
    float rightTp = lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.07f);
    
    //使用VMC计算各关节电机输出扭矩
    float leftJointTorque[2] = {0};
    leg_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
    
    float rightJointTorque[2] = {0};
    leg_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);

    // float leftJointTorque[2] = {0};
    // leg_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
    
    // float rightJointTorque[2] = {0};
    // leg_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);
    
    // 保护腿部角度不超限
    float leftTheta = leftLegPos.angle - imuData.pitch - M_PI_2;
    float rightTheta = rightLegPos.angle - imuData.pitch - M_PI_2;
    
    bool angleExceeded = (leftTheta < -M_PI_4 || leftTheta > M_PI_4 ||
                          rightTheta < -M_PI_4 || rightTheta > M_PI_4 ||
                          imuData.pitch > M_PI_4 || imuData.pitch < -M_PI_4);
    
    // if(angleExceeded)
    // {
    //   // 角度超限，关闭所有电机
    //   leftWheelTorque = 0;
    //   rightWheelTorque = 0;
    //   leftJointTorque[0] = 0;
    //   leftJointTorque[1] = 0;
    //   rightJointTorque[0] = 0;
    //   rightJointTorque[1] = 0;
    // }
    
    // 设置关节电机输出
    // joints_[0].setCommand(-leftJointTorque[0]);    // joint01_left，轴向为-1，负号正确
    // joints_[1].setCommand(rightJointTorque[1]);    // joint01_right，轴向为1，应去掉负号
    // joints_[2].setCommand(-leftJointTorque[1]);    // joint04_left，轴向为-1，负号正确
    // joints_[3].setCommand(rightJointTorque[0]);    // joint04_right，轴向为1，应去掉负号
    // joints_[4].setCommand(leftWheelTorque);        // joint_tire_left，轴向为-1，可能需要调整
    // joints_[5].setCommand(rightWheelTorque);      // joint_tire_right，轴向为1，可能需要调整

    joints_[0].setCommand(-leftJointTorque[0]);    // joint01_left，轴向为-1，负号正确
    joints_[1].setCommand(rightJointTorque[0]);    // joint01_right，轴向为1，应去掉负号
    joints_[2].setCommand(-leftJointTorque[1]);    // joint04_left，轴向为-1，负号正确
    joints_[3].setCommand(rightJointTorque[1]);    // joint04_right，轴向为1，应去掉负号
    joints_[4].setCommand(-leftWheelTorque);        // joint_tire_left，轴向为-1，可能需要调整
    joints_[5].setCommand(rightWheelTorque);      // joint_tire_right，轴向为1，可能需要调整


    std_msgs::Float32MultiArray cmd_msg;
    cmd_msg.data.resize(6);

    // cmd_msg.data[0] = -leftJointTorque[0];    // joint01_left
    // cmd_msg.data[1] = rightJointTorque[1];    // joint01_right
    // cmd_msg.data[2] = -leftJointTorque[1];    // joint04_left
    // cmd_msg.data[3] = rightJointTorque[0];    // joint04_right
    // cmd_msg.data[4] = leftWheelTorque;        // joint_tire_left
    // cmd_msg.data[5] = -rightWheelTorque;      // joint_tire_right

    cmd_msg.data[0] = leftJointTorque[0];    // joint01_left
    cmd_msg.data[1] = rightJointTorque[0];    // joint01_right
    cmd_msg.data[2] = leftJointTorque[1];    // joint04_left
    cmd_msg.data[3] = rightJointTorque[1];    // joint04_right
    cmd_msg.data[4] = leftWheelTorque;        // joint_tire_left
    cmd_msg.data[5] = rightWheelTorque;      // joint_tire_right
  
    joint_cmd_pub_.publish(cmd_msg);
  }

  // 发布状态数据
  void PublishState()
  {
    std_msgs::Float32MultiArray state_msg;
    state_msg.data.resize(15);
    
    state_msg.data[0] = stateVar.theta;
    state_msg.data[1] = stateVar.dTheta;
    state_msg.data[2] = stateVar.x;
    state_msg.data[3] = stateVar.dx;
    state_msg.data[4] = stateVar.phi;
    state_msg.data[5] = stateVar.dPhi;
    state_msg.data[6] = leftLegPos.length;
    state_msg.data[7] = leftLegPos.angle;
    state_msg.data[8] = rightLegPos.length;
    state_msg.data[9] = rightLegPos.angle;
    state_msg.data[10] = target.position;
    state_msg.data[11] = target.speed;
    state_msg.data[12] = target.yawAngle;
    state_msg.data[13] = groundDetector.leftSupportForce;
    state_msg.data[14] = groundDetector.rightSupportForce;

    state_pub_.publish(state_msg);

  }
};
}

// 注册控制器到pluginlib
// PLUGINLIB_EXPORT_CLASS(BalanceRobotController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(balance_robot_controller::BalanceRobotController, controller_interface::ControllerBase)

