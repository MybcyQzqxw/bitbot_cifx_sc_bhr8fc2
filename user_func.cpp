#include "user_func.h"
#include "bitbot_cifx/device/joint_elmo.h"
#include "bitbot_cifx/device/joint_elmo_pushrod.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <fstream>
#include <regex>

#define _ControlT 0.001

constexpr double deg2rad = M_PI / 180.0;
constexpr double rad2deg = 180.0 / M_PI;

bitbot::JointElmo *joint1 = nullptr, *joint2 = nullptr, *joint3 = nullptr;
bitbot::JointElmoPushrod *joint4 = nullptr;

// 全部关节容器：id -> 指针
std::unordered_map<int, bitbot::JointElmo *> joints_elmo_map;
std::unordered_map<int, bitbot::JointElmoPushrod *> joints_pushrod_map;

// 复位位置存储，由用户调用 SetJointResetPosition 指定
std::unordered_map<int, double> joint_reset_positions_elmo;
std::unordered_map<int, double> joint_reset_positions_pushrod;

void SetJointResetPosition(int id, double pos)
{
  // 优先尝试放到 elmo map，否则放到 pushrod map
  if (joints_elmo_map.find(id) != joints_elmo_map.end())
    joint_reset_positions_elmo[id] = pos;
  else
    joint_reset_positions_pushrod[id] = pos;
}

void ClearJointResetPositions()
{
  joint_reset_positions_elmo.clear();
  joint_reset_positions_pushrod.clear();
}

void LoadResetPositionsFromXML(const std::string &filename)
{
  std::ifstream ifs(filename);
  if (!ifs)
  {
    std::cout << "LoadResetPositionsFromXML: file not found: " << filename << std::endl;
    return;
  }

  // 简单正则匹配: <joint id="(\d+)">([\-0-9.eE]+)</joint>
  std::regex re(R"(<joint\s+id=\"(\d+)\">\s*([\-0-9.eE]+)\s*</joint>)");
  std::string line;
  while (std::getline(ifs, line))
  {
    std::smatch m;
    if (std::regex_search(line, m, re))
    {
      try
      {
        int id = std::stoi(m[1].str());
        double pos = std::stod(m[2].str());
        if (joints_elmo_map.find(id) != joints_elmo_map.end())
          joint_reset_positions_elmo[id] = pos;
        else
          joint_reset_positions_pushrod[id] = pos;
        std::cout << "LoadResetPositionsFromXML: joint " << id << " -> " << pos << std::endl;
      }
      catch (...) {}
    }
  }
}

std::vector<std::vector<double>> traj_data;

void ConfigFunc(const bitbot::CifxBus &bus, UserData &)
{
  // 读取并初始化 XML 中定义的所有关节设备
  // JointElmo IDs: 12,13,18,16,11,10,19,21,6,2,1,7,3,4
  // JointElmoPushrod IDs: 14,15,20,22,5,0

  const std::vector<int> elmo_ids = {12, 13, 18, 16, 11, 10, 19, 21, 6, 2, 1, 7, 3, 4};
  const std::vector<int> pushrod_ids = {14, 15, 20, 22, 5, 0};

  // 初始化 JointElmo 设备并保存到 map
  for (int id : elmo_ids)
  {
    auto dev = bus.GetDevice<bitbot::JointElmo>(id);
    if (dev)
    {
      auto j = dev.value();
      joints_elmo_map[id] = j;
      std::cout << "ConfigFunc: JointElmo id=" << id << " found. pos=" << j->GetActualPosition()
                << " vel=" << j->GetActualVelocity() << std::endl;
      j->SetTargetCurrent(0);
      j->SetMode(bitbot::CANopenMotorMode::CST);
      j->SetTargetPosition(j->GetActualPosition());

      // 不对单独的 joint1/2/3 进行特殊赋值，所有关节通过 joints_elmo_map 暴露
    }
    else
    {
      std::cout << "ConfigFunc: JointElmo id=" << id << " not found" << std::endl;
    }
  }

  // 初始化 JointElmoPushrod 设备
  for (int id : pushrod_ids)
  {
    auto dev = bus.GetDevice<bitbot::JointElmoPushrod>(id);
    if (dev)
    {
      auto j = dev.value();
      joints_pushrod_map[id] = j;
      std::cout << "ConfigFunc: JointElmoPushrod id=" << id << " found. pos=" << j->GetActualPosition()
                << " vel=" << j->GetActualVelocity() << std::endl;
      j->SetTargetCurrent(0);
      j->SetMode(bitbot::CANopenMotorMode::CST);
      j->SetTargetPosition(j->GetActualPosition());

      // 不对单独的 joint4 进行特殊赋值，所有 pushrod 通过 joints_pushrod_map 暴露
    }
    else
    {
      std::cout << "ConfigFunc: JointElmoPushrod id=" << id << " not found" << std::endl;
    }
  }

  // 额外：检查 IMU / Force 传感器是否存在（仅报告，不调用特定方法）
  // IDs from XML: ImuMti300 id=8, ForceSri6d ids=17,23
  // 前向声明使用时不需要完整类型定义（只用于检测存在性）
  {
    auto imu = bus.GetDevice<bitbot::ImuMti300>(8);
    if (imu)
      std::cout << "ConfigFunc: ImuMti300 id=8 found" << std::endl;
    else
      std::cout << "ConfigFunc: ImuMti300 id=8 not found" << std::endl;
  }
  {
    auto f1 = bus.GetDevice<bitbot::ForceSri6d>(17);
    if (f1)
      std::cout << "ConfigFunc: ForceSri6d id=17 found" << std::endl;
    else
      std::cout << "ConfigFunc: ForceSri6d id=17 not found" << std::endl;
  }
  {
    auto f2 = bus.GetDevice<bitbot::ForceSri6d>(23);
    if (f2)
      std::cout << "ConfigFunc: ForceSri6d id=23 found" << std::endl;
    else
      std::cout << "ConfigFunc: ForceSri6d id=23 not found" << std::endl;
  }

  // 从 reset_positions.xml 加载复位位置（如果存在）
  LoadResetPositionsFromXML("reset_positions.xml");
}

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue, UserData &)
{
  return static_cast<bitbot::StateId>(States::InitPos);
}

std::optional<bitbot::StateId> EventToFallPos1(bitbot::EventValue value, UserData &user_data)
{
  return static_cast<bitbot::StateId>(States::ToFallPos1);
}

std::optional<bitbot::StateId> EventToFallPos2(bitbot::EventValue value, UserData &user_data)
{
  return static_cast<bitbot::StateId>(States::ToFallPos2);
}

void StateWaiting(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
}

void StateInitPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  // 全关节复位：使用用户指定的目标位置（若不存在则保持当前位置）
  static bool init = false;
  static double start_time = 0;
  static std::unordered_map<int, double> initial_positions_elmo;
  static std::unordered_map<int, double> initial_positions_pushrod;
  static constexpr double duration = 20.0; // 秒

  double time = kernel.GetPeriodsCount() * _ControlT;

  if (!init)
  {
    start_time = time;
    // 记录每个关节的初始位置
    for (const auto &p : joints_elmo_map)
    {
      initial_positions_elmo[p.first] = p.second->GetActualPosition();
    }
    for (const auto &p : joints_pushrod_map)
    {
      initial_positions_pushrod[p.first] = p.second->GetActualPosition();
    }
    init = true;
  }

  double t = time - start_time;
  double s = 0.0;
  if (t < duration)
    s = (1.0 - std::cos(M_PI * t / duration)) / 2.0; // 余弦插值系数

  // 更新 elmo 关节目标位姿
  for (auto &p : joints_elmo_map)
  {
    int id = p.first;
    auto j = p.second;
    double init_pos = initial_positions_elmo.count(id) ? initial_positions_elmo[id] : j->GetActualPosition();
    double target = init_pos; // 默认保持当前位置
    if (joint_reset_positions_elmo.count(id))
      target = joint_reset_positions_elmo[id];

    double pos = (t >= duration) ? target : (init_pos + (target - init_pos) * s);
    j->SetTargetPosition(pos);
  }

  // 更新 pushrod 关节目标位姿
  for (auto &p : joints_pushrod_map)
  {
    int id = p.first;
    auto j = p.second;
    double init_pos = initial_positions_pushrod.count(id) ? initial_positions_pushrod[id] : j->GetActualPosition();
    double target = init_pos;
    if (joint_reset_positions_pushrod.count(id))
      target = joint_reset_positions_pushrod[id];

    double pos = (t >= duration) ? target : (init_pos + (target - init_pos) * s);
    j->SetTargetPosition(pos);
  }
}

void StateToFallPos1(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  static bool init = false;
  static double target_pos1 = 0;
  static double target_pos2 = 0;
  static double target_pos3 = 0;
  static double target_pos4 = 0;

  // PD 参数（根据实际电机调整）
  static constexpr double Kp1 = 50, Kd1 = 0.5;
  static constexpr double Kp2 = 50, Kd2 = 0.5;
  static constexpr double Kp3 = 50, Kd3 = 0.5;
  static constexpr double Kp4 = 50, Kd4 = 0.5;

  // static constexpr double Kp1 = 100, Kd1 = 1.0;
  // static constexpr double Kp2 = 100, Kd2 = 1.0;
  // static constexpr double Kp3 = 100, Kd3 = 1.0;
  // static constexpr double Kp4 = 100, Kd4 = 1.0;
  
  if (!init)
  {
    target_pos1 = joint1->GetActualPosition();
    target_pos2 = joint2->GetActualPosition();
    target_pos3 = joint3->GetActualPosition();
    target_pos4 = joint4->GetActualPosition();

    joint1->SetTargetCurrent(0);
    joint1->SetMode(bitbot::CANopenMotorMode::CST);
    joint2->SetTargetCurrent(0);
    joint2->SetMode(bitbot::CANopenMotorMode::CST);
    joint3->SetTargetCurrent(0);
    joint3->SetMode(bitbot::CANopenMotorMode::CST);
    joint4->SetTargetCurrent(0);
    joint4->SetMode(bitbot::CANopenMotorMode::CST);
    init = true;
  }

  // PD 控制: tau = Kp * (target - actual) - Kd * velocity
  double err1 = target_pos1 - joint1->GetActualPosition();
  double err2 = target_pos2 - joint2->GetActualPosition();
  double err3 = target_pos3 - joint3->GetActualPosition();
  double err4 = target_pos4 - joint4->GetActualPosition();

  double vel1 = joint1->GetActualVelocity();
  double vel2 = joint2->GetActualVelocity();
  double vel3 = joint3->GetActualVelocity();
  double vel4 = joint4->GetActualVelocity();

  double tau1 = Kp1 * err1 - Kd1 * vel1;
  double tau2 = Kp2 * err2 - Kd2 * vel2;
  double tau3 = Kp3 * err3 - Kd3 * vel3;
  double tau4 = Kp4 * err4 - Kd4 * vel4;

  joint1->SetTargetCurrent(tau1);
  joint2->SetTargetCurrent(tau2);
  joint3->SetTargetCurrent(tau3);
  joint4->SetTargetCurrent(tau4);
}

void StateToFallPos2(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  static bool init = false;
  static double target_pos1 = 0;
  static double target_pos2 = 0;
  static double target_pos3 = 0;
  static double target_pos4 = 0;

  // PD 参数（根据实际电机调整）
  static constexpr double Kp1 = 50, Kd1 = 0.5;
  static constexpr double Kp2 = 50, Kd2 = 0.5;
  static constexpr double Kp3 = 50, Kd3 = 0.5;
  static constexpr double Kp4 = 50, Kd4 = 0.5;

  // static constexpr double Kp1 = 100, Kd1 = 1.0;
  // static constexpr double Kp2 = 100, Kd2 = 1.0;
  // static constexpr double Kp3 = 100, Kd3 = 1.0;
  // static constexpr double Kp4 = 100, Kd4 = 1.0;
  
  if (!init)
  {
    target_pos1 = joint1->GetActualPosition();
    target_pos2 = joint2->GetActualPosition();
    target_pos3 = joint3->GetActualPosition();
    target_pos4 = joint4->GetActualPosition();

    joint1->SetTargetCurrent(0);
    joint1->SetMode(bitbot::CANopenMotorMode::CST);
    joint2->SetTargetCurrent(0);
    joint2->SetMode(bitbot::CANopenMotorMode::CST);
    joint3->SetTargetCurrent(0);
    joint3->SetMode(bitbot::CANopenMotorMode::CST);
    joint4->SetTargetCurrent(0);
    joint4->SetMode(bitbot::CANopenMotorMode::CST);
    init = true;
  }

  // PD 控制: tau = Kp * (target - actual) - Kd * velocity
  double err1 = target_pos1 - joint1->GetActualPosition();
  double err2 = target_pos2 - joint2->GetActualPosition();
  double err3 = target_pos3 - joint3->GetActualPosition();
  double err4 = target_pos4 - joint4->GetActualPosition();

  double vel1 = joint1->GetActualVelocity();
  double vel2 = joint2->GetActualVelocity();
  double vel3 = joint3->GetActualVelocity();
  double vel4 = joint4->GetActualVelocity();

  double tau1 = Kp1 * err1 - Kd1 * vel1;
  double tau2 = Kp2 * err2 - Kd2 * vel2;
  double tau3 = Kp3 * err3 - Kd3 * vel3;
  double tau4 = Kp4 * err4 - Kd4 * vel4;

  joint1->SetTargetCurrent(tau1);
  joint2->SetTargetCurrent(tau2);
  joint3->SetTargetCurrent(tau3);
  joint4->SetTargetCurrent(tau4);
}
