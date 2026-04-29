#pragma once

#include "bitbot_cifx/kernel/cifx_kernel.hpp"
#include "bitbot_cifx/device/joint_elmo_pushrod.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <fstream>
#include <sstream>
#include <unordered_map>
// 保证头文件自包含性
#include <optional>
#include <vector>

// 前向声明，避免在头文件中包含过多实现依赖
namespace bitbot {
class JointElmo;
class ImuMti300;
class ForceSri6d;
}

enum Events
{
  InitPos = 1001,
  MaintainPos = 1002,
  ToFallPos1 = 1003,
  ToFallPos2 = 1004,
};

enum class States : bitbot::StateId
{
  Waiting = 2001,
  InitPos = 2002,
  MaintainPos = 2003,
  ToFallPos1 = 2004,
  ToFallPos2 = 2005,
};

extern bitbot::JointElmoPushrod *joint_x;
extern std::unordered_map<int, bitbot::JointElmo *> joints_elmo_map;
extern std::unordered_map<int, bitbot::JointElmoPushrod *> joints_pushrod_map;

// 复位位置接口：用户可为每个关节设置复位目标位置（单位：弧度）
extern std::unordered_map<int, double> joint_reset_positions_elmo;
extern std::unordered_map<int, double> joint_reset_positions_pushrod;

// 设置/清除复位位置
void SetJointResetPosition(int id, double pos);
void ClearJointResetPositions();
void LoadResetPositionsFromXML(const std::string &filename);

struct UserData
{
};

using CifxKernel = bitbot::CifxKernel<UserData, "setup_time", "solve_time", "pin_time">;

void ConfigFunc(const bitbot::CifxBus &bus, UserData &);

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventMaintainPos(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventToFallPos1(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventToFallPos2(bitbot::EventValue value, UserData &user_data);

void StateWaiting(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateInitPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateMaintainPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateToFallPos1(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateToFallPos2(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);

static std::vector<std::vector<double>> ReadCSV(std::string filename)
{
  using namespace std;
  vector<vector<double>> data;
  ifstream file(filename);

  string line;
  while (getline(file, line))
  {
    stringstream liness(line);
    string cell;
    vector<double> row;

    while (getline(liness, cell, ','))
    {
      row.push_back(stod(cell));
    }

    if (!row.empty())
      data.push_back(row);
  }

  return data;
}
