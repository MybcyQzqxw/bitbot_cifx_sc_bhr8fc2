#include "user_func.h"
#include "bitbot_cifx/device/joint_elmo.h"
#include "bitbot_cifx/device/joint_elmo_pushrod.h"
// 需要完整类型以支持在模板中使用 dynamic_cast
#include "bitbot_cifx/device/imu_mti300.h"
#include "bitbot_cifx/device/force_sri6d.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <regex>
#include <cmath>
#include <filesystem>
#include <unistd.h>

#define _ControlT 0.001

constexpr double deg2rad = M_PI / 180.0;
constexpr double rad2deg = 180.0 / M_PI;

// 所有关节通过 `joints_elmo_map` 与 `joints_pushrod_map` 管理

// 全部关节容器：id -> 指针
std::unordered_map<int, bitbot::JointElmo *> joints_elmo_map;
std::unordered_map<int, bitbot::JointElmoPushrod *> joints_pushrod_map;

// 复位位置存储，由用户调用 SetJointResetPosition 指定
std::unordered_map<int, double> joint_reset_positions_elmo;
std::unordered_map<int, double> joint_reset_positions_pushrod;

// 每个关节由 XML 配置的电流上限（取正值，表示绝对值上限）
std::unordered_map<int, double> joint_current_max_map;

// 从 reset_positions.xml 读取的全局复位运动时长（秒），默认 20s
double reset_duration = 20.0;
// StateToFallPos1 的时长与目标仅由 fall1_positions.xml 指定
// 从 fall1_positions.xml 读取的专用配置（用于 StateToFallPos1）
bool fall1_loaded = false;
double fall1_t1 = 0.0;
double fall1_t2 = 0.0;
double fall1_deltat = 0.0;
std::unordered_map<int, double> fall1_targets;

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
  // 仅尝试固定位置：当前工作目录向上两级（不尝试当前目录或一级上层）
  std::ifstream ifs;
  try
  {
    auto cwd = std::filesystem::current_path();
    auto p = cwd.parent_path().parent_path() / filename;
    std::cout << "LoadResetPositionsFromXML: try path: " << p << std::endl;
    if (std::filesystem::exists(p))
    {
      ifs.open(p.string());
      if (ifs)
        std::cout << "LoadResetPositionsFromXML: found file at: " << p << std::endl;
    }
  }
  catch (...) {}

  if (!ifs)
  {
    std::cout << "LoadResetPositionsFromXML: file not found: " << filename << std::endl;
    return;
  }
  // 读取整个文件到字符串，先查找 duration 信息（支持根节点属性或单独元素），再用正则匹配 joint
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // 支持 <reset_positions duration="20.0"> 或 <duration>20.0</duration>
  std::smatch m_attr;
  std::regex re_attr(R"(<reset_positions[^>]*\bduration\s*=\s*\"([\-0-9.eE]+)\"[^>]*>)");
  if (std::regex_search(content, m_attr, re_attr))
  {
    try { reset_duration = std::stod(m_attr[1].str()); }
    catch (...) {}
  }
  else
  {
    std::smatch m_elem;
    std::regex re_elem(R"(<duration>\s*([\-0-9.eE]+)\s*</duration>)");
    if (std::regex_search(content, m_elem, re_elem))
    {
      try { reset_duration = std::stod(m_elem[1].str()); }
      catch (...) {}
    }
  }

  // 简单正则匹配: <joint id="(\d+)" [reset="0|1"]>VALUE</joint>
  std::regex re(R"(<joint\s+id=\"(\d+)\"(?:\s+reset=\"([01])\")?\s*>\s*([\-0-9.eE]+)\s*</joint>)");
  std::sregex_iterator it(content.begin(), content.end(), re);
  std::sregex_iterator end;
  for (; it != end; ++it)
  {
    try
    {
      int id = std::stoi((*it)[1].str());
      std::string reset_flag = (*it)[2].str(); // may be empty
      double pos = std::stod((*it)[3].str());
      bool do_reset = true;
      if (!reset_flag.empty())
        do_reset = (reset_flag == "1");

      if (do_reset)
      {
        if (joints_elmo_map.find(id) != joints_elmo_map.end())
          joint_reset_positions_elmo[id] = pos;
        else
          joint_reset_positions_pushrod[id] = pos;
        std::cout << "LoadResetPositionsFromXML: joint " << id << " -> " << pos << " (reset=1)" << std::endl;
      }
      else
      {
        std::cout << "LoadResetPositionsFromXML: joint " << id << " skip reset (reset=0)" << std::endl;
      }
    }
    catch (...) {}
  }
}

void LoadCurrentLimitsFromXML(const std::string &filename)
{
  std::ifstream ifs;
  try
  {
    auto cwd = std::filesystem::current_path();
    auto p = cwd.parent_path().parent_path() / filename;
    std::cout << "LoadCurrentLimitsFromXML: try path: " << p << std::endl;
    if (std::filesystem::exists(p))
    {
      ifs.open(p.string());
      if (ifs)
        std::cout << "LoadCurrentLimitsFromXML: found file at: " << p << std::endl;
    }
  }
  catch (...) {}

  if (!ifs)
  {
    std::cout << "LoadCurrentLimitsFromXML: file not found: " << filename << std::endl;
    return;
  }

  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  // 匹配 <device ... id="(\d+)" ... current_max="([\-0-9.eE]+)" .../>
  std::regex re(R"(<device[^>]*\bid\s*=\s*\"(\d+)\"[^>]*\bcurrent_max\s*=\s*\"([\-0-9.eE]+)\"[^>]*>)");
  std::sregex_iterator it(content.begin(), content.end(), re);
  std::sregex_iterator end;
  for (; it != end; ++it)
  {
    try
    {
      int id = std::stoi((*it)[1].str());
      double vmax = std::stod((*it)[2].str());
      joint_current_max_map[id] = std::abs(vmax);
      std::cout << "LoadCurrentLimitsFromXML: id=" << id << " current_max=" << joint_current_max_map[id] << std::endl;
    }
    catch (...) {}
  }
}

void LoadFall1PositionsFromXML(const std::string &filename)
{
  std::ifstream ifs;
  try
  {
    auto cwd = std::filesystem::current_path();
    // 尝试多级上溯以覆盖不同运行目录（优先当前、上一级、上两级）
    std::vector<std::filesystem::path> tries = {cwd / filename, cwd.parent_path() / filename, cwd.parent_path().parent_path() / filename};
    for (auto &p : tries)
    {
      std::cout << "LoadFall1PositionsFromXML: try path: " << p << std::endl;
      if (std::filesystem::exists(p))
      {
        ifs.open(p.string());
        if (ifs)
        {
          std::cout << "LoadFall1PositionsFromXML: found file at: " << p << std::endl;
          break;
        }
      }
    }
  }
  catch (...) {}

  if (!ifs)
  {
    std::cout << "LoadFall1PositionsFromXML: file not found: " << filename << std::endl;
    fall1_loaded = false;
    // clear previous parsed data to avoid accidental reuse
    fall1_t1 = 0.0;
    fall1_t2 = 0.0;
    fall1_targets.clear();
    return;
  }

  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  // 解析到临时容器，最后验证是否包含所有需要的字段
  double parsed_t1 = 0.0;
  double parsed_t2 = 0.0;
  double parsed_deltat = 0.0;
  std::unordered_map<int, double> parsed_targets;

  // 根节点属性或元素形式的 t1 / t2 / deltat
  std::smatch m_t1_attr;
  std::smatch m_t2_attr;
  std::smatch m_dt_attr;
  std::regex re_t1_attr(R"(<fall1_positions[^>]*\bt1\s*=\s*\"([\-0-9.eE]+)\"[^>]*>)");
  std::regex re_t2_attr(R"(<fall1_positions[^>]*\bt2\s*=\s*\"([\-0-9.eE]+)\"[^>]*>)");
  std::regex re_dt_attr(R"(<fall1_positions[^>]*\bdeltat\s*=\s*\"([\-0-9.eE]+)\"[^>]*>)");
  if (std::regex_search(content, m_t1_attr, re_t1_attr))
  {
    try { parsed_t1 = std::stod(m_t1_attr[1].str()); } catch(...) { parsed_t1 = 0.0; }
  }
  else
  {
    std::smatch m_t1_elem;
    std::regex re_t1_elem(R"(<t1>\s*([\-0-9.eE]+)\s*</t1>)");
    if (std::regex_search(content, m_t1_elem, re_t1_elem))
    {
      try { parsed_t1 = std::stod(m_t1_elem[1].str()); } catch(...) { parsed_t1 = 0.0; }
    }
  }

  if (std::regex_search(content, m_t2_attr, re_t2_attr))
  {
    try { parsed_t2 = std::stod(m_t2_attr[1].str()); } catch(...) { parsed_t2 = 0.0; }
  }
  else
  {
    std::smatch m_t2_elem;
    std::regex re_t2_elem(R"(<t2>\s*([\-0-9.eE]+)\s*</t2>)");
    if (std::regex_search(content, m_t2_elem, re_t2_elem))
    {
      try { parsed_t2 = std::stod(m_t2_elem[1].str()); } catch(...) { parsed_t2 = 0.0; }
    }
  }

  // 解析 deltat（可作为属性或元素）
  if (std::regex_search(content, m_dt_attr, re_dt_attr))
  {
    try { /* parsed below */ } catch(...) {}
  }
  else
  {
    std::smatch m_dt_elem;
    std::regex re_dt_elem(R"(<deltat>\s*([\-0-9.eE]+)\s*</deltat>)");
    if (std::regex_search(content, m_dt_elem, re_dt_elem))
    {
      try { /* parsed below */ } catch(...) {}
    }
  }

  // 提取 deltat 的值（再做一次匹配以统一处理异常）
  std::smatch m_dt_val;
  std::regex re_dt_any(R"(<fall1_positions[^>]*\bdeltat\s*=\s*\"([\-0-9.eE]+)\"[^>]*>|<deltat>\s*([\-0-9.eE]+)\s*</deltat>)");
  if (std::regex_search(content, m_dt_val, re_dt_any))
  {
    try
    {
      std::string v = m_dt_val[1].matched ? m_dt_val[1].str() : m_dt_val[2].str();
      parsed_deltat = std::stod(v);
    }
    catch(...) { parsed_deltat = 0.0; }
  }

  // 解析 joint 条目，格式: <joint id="6">VALUE</joint>
  std::regex re(R"(<joint\s+id=\"(\d+)\"(?:\s+reset=\"([01])\")?\s*>\s*([\-0-9.eE]+)\s*</joint>)");
  std::sregex_iterator it(content.begin(), content.end(), re);
  std::sregex_iterator end;
  for (; it != end; ++it)
  {
    try
    {
      int id = std::stoi((*it)[1].str());
      double pos = std::stod((*it)[3].str());
      parsed_targets[id] = pos;
      std::cout << "LoadFall1PositionsFromXML: parsed joint " << id << " -> " << pos << std::endl;
    }
    catch (...) {}
  }

  // 验证：要求 t1 和 t2 均为正，且包含下面这 8 个关节
  const std::vector<int> required_ids = {6, 5, 7, 0, 18, 15, 19, 22};
  bool ok = true;
  if (!(parsed_t1 > 0.0))
  {
    std::cout << "LoadFall1PositionsFromXML: missing or invalid t1" << std::endl;
    ok = false;
  }
  if (!(parsed_t2 > 0.0))
  {
    std::cout << "LoadFall1PositionsFromXML: missing or invalid t2" << std::endl;
    ok = false;
  }
  if (!(parsed_deltat >= 0.0))
  {
    std::cout << "LoadFall1PositionsFromXML: missing or invalid deltat" << std::endl;
    ok = false;
  }
  for (int id : required_ids)
  {
    if (parsed_targets.find(id) == parsed_targets.end())
    {
      std::cout << "LoadFall1PositionsFromXML: missing target for joint " << id << std::endl;
      ok = false;
    }
  }

  if (!ok)
  {
    std::cout << "LoadFall1PositionsFromXML: incomplete configuration, aborting load (fall1 not enabled)" << std::endl;
    fall1_loaded = false;
    fall1_t1 = 0.0;
    fall1_t2 = 0.0;
    fall1_targets.clear();
    return;
  }

  // 验证通过：提交到全局变量
  fall1_t1 = parsed_t1;
  fall1_t2 = parsed_t2;
  fall1_deltat = parsed_deltat;
  fall1_targets = std::move(parsed_targets);
  fall1_loaded = true;
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

      // 所有 elmo 关节通过 `joints_elmo_map` 暴露
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

      // 所有 pushrod 关节通过 `joints_pushrod_map` 暴露
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
  // 从 sc_bhr8fc2.xml 加载每个 device 的 current_max（如果存在）
  LoadCurrentLimitsFromXML("sc_bhr8fc2.xml");
}

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue, UserData &)
{
  return static_cast<bitbot::StateId>(States::InitPos);
}

std::optional<bitbot::StateId> EventMaintainPos(bitbot::EventValue, UserData &)
{
  return static_cast<bitbot::StateId>(States::MaintainPos);
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
  static constexpr double reset_duration_const = 20.0; // 秒

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
    // 对需要复位的关节切换到位置控制（CSP）并初始化目标位置；未被标记复位的关节将把控制电流置为 0
    for (const auto &p : joints_elmo_map)
    {
      int id = p.first;
      auto j = p.second;
      if (joint_reset_positions_elmo.count(id))
      {
        j->SetMode(bitbot::CANopenMotorMode::CSP);
        j->SetTargetPosition(initial_positions_elmo[id]);
      }
      else
      {
        j->SetTargetCurrent(0);
      }
    }
    for (const auto &p : joints_pushrod_map)
    {
      int id = p.first;
      auto j = p.second;
      if (joint_reset_positions_pushrod.count(id))
      {
        j->SetMode(bitbot::CANopenMotorMode::CSP);
        j->SetTargetPosition(initial_positions_pushrod[id]);
      }
      else
      {
        j->SetTargetCurrent(0);
      }
    }
    init = true;
  }

    double t = time - start_time;
    double duration_val = reset_duration;
    double s = 0.0;
    if (t < duration_val)
      s = (1.0 - std::cos(M_PI * t / duration_val)) / 2.0; // 余弦插值系数

  // 更新 elmo 关节目标位姿
  for (auto &p : joints_elmo_map)
  {
    int id = p.first;
    auto j = p.second;
    double init_pos = initial_positions_elmo.count(id) ? initial_positions_elmo[id] : j->GetActualPosition();
    double target = init_pos; // 默认保持当前位置
    if (joint_reset_positions_elmo.count(id))
      target = joint_reset_positions_elmo[id];

    if (joint_reset_positions_elmo.count(id))
    {
      // 平滑插值（余弦平滑）
      double pos = (t >= duration_val) ? target : (init_pos + (target - init_pos) * s);
      j->SetTargetPosition(pos);
      // 调试日志：周期性打印复位关节信息，帮助定位为何不动
      if ((static_cast<long long>(kernel.GetPeriodsCount()) % 1000) == 0 || t == 0.0 || t >= duration_val)
      {
        std::cout << "StateInitPos[elmo]: id=" << id
                  << " init=" << init_pos
                  << " target=" << target
                  << " pos=" << pos
                  << " t=" << t
                  << " s=" << s
                  << " dur=" << duration_val
                  << " mode=" << j->GetMode()
                  << " status=0x" << std::hex << j->GetStatus() << std::dec
                  << " actual_cur=" << j->GetActualCurrent()
                  << " target_cur=" << j->GetTargetCurrent()
                  << std::endl;
      }
    }
    else
    {
      // 未标记复位：不施加位置控制，保持不驱动（电流置零）
      j->SetTargetCurrent(0);
    }
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

    if (joint_reset_positions_pushrod.count(id))
    {
      double pos = (t >= duration_val) ? target : (init_pos + (target - init_pos) * s);
      j->SetTargetPosition(pos);
      if ((static_cast<long long>(kernel.GetPeriodsCount()) % 1000) == 0 || t == 0.0 || t >= duration_val)
      {
        std::cout << "StateInitPos[pushrod]: id=" << id
                  << " init=" << init_pos
                  << " target=" << target
                  << " pos=" << pos
                  << " t=" << t
                  << " s=" << s
                  << " dur=" << duration_val
                  << " mode=" << j->GetMode()
                  << " status=0x" << std::hex << j->GetStatus() << std::dec
                  << " actual_cur=" << j->GetActualCurrent()
                  << " target_cur=" << j->GetTargetCurrent()
                  << std::endl;
      }
    }
    else
    {
      j->SetTargetCurrent(0);
    }
  }
}

void StateMaintainPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  // 切换到力矩（电流）控制，并用简单的 PD 回路把关节维持到 StateInitPos 中的目标位置。
  // 目标位置优先使用 joint_reset_positions_* 中指定的复位位置，否则在第一次进入时记当前位姿为目标。
  static bool init = false;
  static std::unordered_map<int, double> desired_pos_elmo;
  static std::unordered_map<int, double> desired_pos_pushrod;

  // PD 参数（保守默认值，可根据机械臂特性调整）
  const double Kp = 100.0;   // 位置比例增益
  const double Kd = 1.0;    // 速度比例（阻尼）增益

  // 首次进入：确定每个关节的期望位置并切换到电流/力矩控制模式
  if (!init)
  {
    for (const auto &p : joints_elmo_map)
    {
      int id = p.first;
      auto j = p.second;
      double des = j->GetActualPosition();
      if (joint_reset_positions_elmo.count(id))
        des = joint_reset_positions_elmo[id];
      desired_pos_elmo[id] = des;
      j->SetMode(bitbot::CANopenMotorMode::CST); // 电流/力矩控制
      // 初始化输出为 0，避免突变
      j->SetTargetCurrent(0);
    }

    for (const auto &p : joints_pushrod_map)
    {
      int id = p.first;
      auto j = p.second;
      double des = j->GetActualPosition();
      if (joint_reset_positions_pushrod.count(id))
        des = joint_reset_positions_pushrod[id];
      desired_pos_pushrod[id] = des;
      j->SetMode(bitbot::CANopenMotorMode::CST);
      j->SetTargetCurrent(0);
    }

    init = true;
  }

  // 每周期计算 PD 输出（电流）并下发
  for (auto &p : joints_elmo_map)
  {
    int id = p.first;
    auto j = p.second;
    double des = desired_pos_elmo.count(id) ? desired_pos_elmo[id]
                                             : (joint_reset_positions_elmo.count(id) ? joint_reset_positions_elmo[id] : j->GetActualPosition());
    double act = j->GetActualPosition();
    double vel = j->GetActualVelocity();
    double err = des - act;
    double derr = -vel; // 期望速度为 0
    double cur = Kp * err + Kd * derr;
    double max_cur = joint_current_max_map.count(id) ? joint_current_max_map[id] : 10.0;
    if (cur > max_cur)
      cur = max_cur;
    if (cur < -max_cur)
      cur = -max_cur;
    j->SetTargetCurrent(cur);

    if ((static_cast<long long>(kernel.GetPeriodsCount()) % 1000) == 0)
    {
      std::cout << "StateMaintainPos[elmo]: id=" << id
                << " des=" << des
                << " act=" << act
                << " vel=" << vel
                << " cur=" << cur
                << " mode=" << j->GetMode()
                << " status=0x" << std::hex << j->GetStatus() << std::dec
                << std::endl;
    }
  }

  for (auto &p : joints_pushrod_map)
  {
    int id = p.first;
    auto j = p.second;
    double des = desired_pos_pushrod.count(id) ? desired_pos_pushrod[id]
                                                : (joint_reset_positions_pushrod.count(id) ? joint_reset_positions_pushrod[id] : j->GetActualPosition());
    double act = j->GetActualPosition();
    double vel = j->GetActualVelocity();
    double err = des - act;
    double derr = -vel;
    double cur = Kp * err + Kd * derr;
    double max_cur = joint_current_max_map.count(id) ? joint_current_max_map[id] : 10.0;
    if (cur > max_cur)
      cur = max_cur;
    if (cur < -max_cur)
      cur = -max_cur;
    j->SetTargetCurrent(cur);

    if ((static_cast<long long>(kernel.GetPeriodsCount()) % 1000) == 0)
    {
      std::cout << "StateMaintainPos[pushrod]: id=" << id
                << " des=" << des
                << " act=" << act
                << " vel=" << vel
                << " cur=" << cur
                << " mode=" << j->GetMode()
                << " status=0x" << std::hex << j->GetStatus() << std::dec
                << std::endl;
    }
  }
}

void StateToFallPos1(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  // 在进入此状态时，假定所有关节已处于 CST（电流/力矩）模式。
  static bool init = false;
  static double start_time = 0.0;
  static std::unordered_map<int, double> init_positions;

  // 使用与 StateInitPos / reset_positions 相同的全局 duration 进行切分（均分为 t1 和 t2）
  double time = kernel.GetPeriodsCount() * _ControlT;
  if (!init)
  {
    start_time = time;
    // 尝试从根目录读取 fall1_positions.xml，覆盖 t1/t2 和指定的关节目标
    LoadFall1PositionsFromXML("fall1_positions.xml");
    // 记录所有已存在关节的初始位置
    for (const auto &p : joints_elmo_map)
      init_positions[p.first] = p.second->GetActualPosition();
    for (const auto &p : joints_pushrod_map)
      init_positions[p.first] = p.second->GetActualPosition();

    // 确保参与运动的关节处于 CST 模式，并清零初始输出以避免突变
    const int stage1_ids[] = {6, 5, 7, 0};
    const int stage2_ids[] = {18, 15, 19, 22};
    for (int id : stage1_ids)
    {
      if (joints_elmo_map.count(id))
      {
        joints_elmo_map[id]->SetMode(bitbot::CANopenMotorMode::CST);
        joints_elmo_map[id]->SetTargetCurrent(0);
      }
      if (joints_pushrod_map.count(id))
      {
        joints_pushrod_map[id]->SetMode(bitbot::CANopenMotorMode::CST);
        joints_pushrod_map[id]->SetTargetCurrent(0);
      }
    }
    for (int id : stage2_ids)
    {
      if (joints_elmo_map.count(id))
      {
        joints_elmo_map[id]->SetMode(bitbot::CANopenMotorMode::CST);
        joints_elmo_map[id]->SetTargetCurrent(0);
      }
      if (joints_pushrod_map.count(id))
      {
        joints_pushrod_map[id]->SetMode(bitbot::CANopenMotorMode::CST);
        joints_pushrod_map[id]->SetTargetCurrent(0);
      }
    }

    // 额外：确保其它关节也切到 CST 并清零初始输出，以便后续 PD 控制可以生效（保持进入本函数时的位置）
    for (const auto &q : joints_elmo_map)
    {
      int id = q.first;
      if (std::find(std::begin(stage1_ids), std::end(stage1_ids), id) == std::end(stage1_ids) &&
          std::find(std::begin(stage2_ids), std::end(stage2_ids), id) == std::end(stage2_ids))
      {
        q.second->SetMode(bitbot::CANopenMotorMode::CST);
        q.second->SetTargetCurrent(0);
      }
    }
    for (const auto &q : joints_pushrod_map)
    {
      int id = q.first;
      if (std::find(std::begin(stage1_ids), std::end(stage1_ids), id) == std::end(stage1_ids) &&
          std::find(std::begin(stage2_ids), std::end(stage2_ids), id) == std::end(stage2_ids))
      {
        q.second->SetMode(bitbot::CANopenMotorMode::CST);
        q.second->SetTargetCurrent(0);
      }
    }

    init = true;
  }

  double t = time - start_time;
  double total = reset_duration > 0.0 ? reset_duration : 10.0; // 兜底
  // 使用 `fall1_positions.xml` 中的 t1/t2 和关节目标：
  // 如果文件成功加载（fall1_loaded==true），将严格使用其中的 t1/t2 和对应目标。
  // 若未加载或加载失败，则回退到基于 `reset_duration` 的合理默认值。
  double t1 = 0.0, t2 = 0.0;
  if (fall1_loaded && fall1_t1 > 0.0 && fall1_t2 > 0.0)
  {
    t1 = fall1_t1;
    t2 = fall1_t2;
  }
  else
  {
    const double fallback = (reset_duration > 0.0) ? (reset_duration * 0.5) : 5.0;
    t1 = fallback;
    t2 = fallback;
    if (!fall1_loaded)
      std::cout << "StateToFallPos1: fall1_positions.xml not loaded; using fallback t1/t2=" << fallback << std::endl;
  }
  double deltat = 0.0;
  if (fall1_loaded)
    deltat = fall1_deltat;

  // 帮助 lambda: 获取目标复位位置（优先使用 joint_reset_positions_*，否则使用硬编码备选值）
  auto get_reset_target = [&](int id) -> double {
    // 优先使用 fall1_positions.xml 中的值（仅限加载时提供的关节）
    if (fall1_loaded && fall1_targets.count(id))
      return fall1_targets[id];
    if (joint_reset_positions_elmo.count(id))
      return joint_reset_positions_elmo[id];
    if (joint_reset_positions_pushrod.count(id))
      return joint_reset_positions_pushrod[id];
    // 备用硬编码（与 reset_positions.xml 中期望值一致）
    switch (id)
    {
      case 6: return 0.0;
      case 5: return 1.5;
      case 7: return 0.0;
      case 0: return 1.5;
      case 18: return -0.270;
      case 15: return -0.48;
      case 19: return -0.255;
      case 22: return -0.49;
      default: return 0.0;
    }
  };

  // PD 参数（与 StateMaintainPos 保持一致）
  const double Kp = 100.0;
  const double Kd = 1.0;

  // 运动阶段集合
  const std::unordered_set<int> stage1_set = {6, 5, 7, 0};
  const std::unordered_set<int> stage2_set = {18, 15, 19, 22};

  // 为每个关节计算期望位置并下发电流（CST）
  auto process_joint = [&](int id, auto jptr, bool isPushrod)
  {
    double init_pos = init_positions.count(id) ? init_positions[id] : jptr->GetActualPosition();
    double final_target = get_reset_target(id);
    double desired = init_pos; // 默认保持当前位置

    // 两个阶段同时开始，但有各自持续时间 t1 / t2
    if (stage1_set.count(id))
    {
      double tt = t - deltat; // stage1 starts after deltat
      double s = (tt <= 0.0) ? 0.0 : (1.0 - std::cos(M_PI * std::min(tt, t1) / t1)) / 2.0;
      desired = (tt >= t1) ? final_target : (init_pos + (final_target - init_pos) * s);
    }
    else if (stage2_set.count(id))
    {
      double tt = t;
      double s = (tt <= 0.0) ? 0.0 : (1.0 - std::cos(M_PI * std::min(tt, t2) / t2)) / 2.0;
      desired = (tt >= t2) ? final_target : (init_pos + (final_target - init_pos) * s);
    }
    else
    {
      // 非参与运动的关节：将期望保持为进入本函数时记录的初始位姿，使用 PD 回路维持此位置
      desired = init_pos;
    }

    double act = jptr->GetActualPosition();
    double vel = jptr->GetActualVelocity();
    double err = desired - act;
    double derr = -vel;
    double cur = Kp * err + Kd * derr;
    double max_cur = joint_current_max_map.count(id) ? joint_current_max_map[id] : 10.0;
    if (cur > max_cur) cur = max_cur;
    if (cur < -max_cur) cur = -max_cur;
    jptr->SetTargetCurrent(cur);

    if ((static_cast<long long>(kernel.GetPeriodsCount()) % 1000) == 0)
    {
      std::cout << "StateToFallPos1[" << (isPushrod?"pushrod":"elmo") << "]: id=" << id
                << " des=" << desired
                << " act=" << act
                << " vel=" << vel
                << " cur=" << cur
                << " mode=" << jptr->GetMode()
                << " status=0x" << std::hex << jptr->GetStatus() << std::dec
                << std::endl;
    }
  };

  // 处理 elmo
  for (auto &p : joints_elmo_map)
  {
    process_joint(p.first, p.second, false);
  }

  // 处理 pushrod
  for (auto &p : joints_pushrod_map)
  {
    process_joint(p.first, p.second, true);
  }
}

void StateToFallPos2(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
}
