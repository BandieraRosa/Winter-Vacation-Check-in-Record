# 2.5

今天在调试机器人时，准备开始开发RoboMaster比赛中的英雄机器人部署模式吊射功能。

![image-20260205013824428](/home/bandiera/.config/Typora/typora-user-images/image-20260205013824428.png)

![image-20260205013903230](/home/bandiera/.config/Typora/typora-user-images/image-20260205013903230.png)



> 当英雄机器人选择了“远程优先”的整机类型且位于己方可部署区域（详见“图 5-19 己方可部署区域示
> 意图”）时，可以选择进入“部署模式”。确认进入“部署模式”2 秒后，英雄机器人进入“部署模式”，
> 在该模式下，英雄机器人底盘断电，失去图传画面，获得 25%防御增益，射击初速度上限提升为 16.5m/s，
> 且发射 42 mm 弹丸攻击基地时具有 150%攻击增益。

即：操作手无视野的情况下超远距离抛射弹丸。

超远距离意味着正常的相机完全无法获取目标，也意味着不确定性大大增加。

基于这个背景，我决定探究一下双相机的可能性。

首先需要知道相机在什么地方会比较合适。基于此，我设计了一个测试用的小工具，输入目标距离与相对高度，即可输出所需pitch、弹丸飞行时间与末端速度；也可输入pitch获取其飞行最高/最远距离。也就是进行单次解算与逆解算。

关键代码片段：

```c++
  // 进行单次解算，考虑多级误差以保证精确和有解
  std::vector<double> SolvePitchLevel(int error_level)
  {
    auto ge = SolvePitch(MAX_ERROR);
    if (std::isnan(ge[0]))
    {
      return {NAN, NAN, NAN};
    }
    for (int i = 1; i <= error_level; i++)
    {
      ge = SolvePitch(MAX_ERROR / error_level * i);
      if (!std::isnan(ge[0]))
      {
        return ge;
      }
    }
    return {NAN, NAN, NAN};
  }
```

```c++
  // RK4 单步积分
  State RK4Step(const State& state, double h)
  {
    auto k1 = AirODE(state);
    State state1(state.x + 0.5 * h * k1[0], state.y + 0.5 * h * k1[1],
                 state.vx + 0.5 * h * k1[2], state.vy + 0.5 * h * k1[3]);

    auto k2 = AirODE(state1);
    State state2(state.x + 0.5 * h * k2[0], state.y + 0.5 * h * k2[1],
                 state.vx + 0.5 * h * k2[2], state.vy + 0.5 * h * k2[3]);

    auto k3 = AirODE(state2);
    State state3(state.x + h * k3[0], state.y + h * k3[1], state.vx + h * k3[2],
                 state.vy + h * k3[3]);

    auto k4 = AirODE(state3);

    State new_state(state.x, state.y, state.vx, state.vy);
    new_state.x += h * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6.0;
    new_state.y += h * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6.0;
    new_state.vx += h * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6.0;
    new_state.vy += h * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6.0;

    return new_state;
  }
```

```c++
int main(int argc, char** argv)
{
  std::vector<double> ans;
  if (argc == 2)
  {
    double pitch = atof(argv[1]);
    SolveTrajectory solve = SolveTrajectory(BULLET_V, 0, {}, {});
    ans = solve.SolveHeightAndLength(pitch);
  }
  if (argc == 3)
  {
    double distance = atof(argv[1]);
    double height = atof(argv[2]);
    SolveTrajectory solve = SolveTrajectory(BULLET_V, 0, distance, height);
    ans = solve.SolvePitchLevel(ERROR_LEVEL);
  }
  else
  {
    std::cout << "Usage: trajectory_test <pitch>\n"
              << "       trajectory_test <distance> <height>\n";
    return 0;
  }
  if (ans.size() == 2)
  {
    std::cout << "Max Height: " << ans[0] << " Max Length: " << ans[1] << '\n';
  }
  if (ans.size() == 3)
  {
    std::cout << "Pitch: " << ans[0] << " Time: " << ans[1] << " Velocity: " << ans[2]
              << '\n';
  }
  return 0;
}
```

编译：

```bash
bandiera@BR:~/Desktop/v/test_outpost/vision_dev$ colcon build --symlink-install
Starting >>> auto_aim_interfaces
Starting >>> hik_camera
Starting >>> rm_gimbal_description
Starting >>> rm_hand_eye_calibrate
Finished <<< rm_gimbal_description [0.06s]
Finished <<< hik_camera [0.07s]
Finished <<< rm_hand_eye_calibrate [0.09s]                                                                     
Finished <<< auto_aim_interfaces [0.29s]                     
Starting >>> rm_serial_driver
Starting >>> armor_detector
Starting >>> armor_tracker
Starting >>> rm_simulator_driver
Finished <<< rm_simulator_driver [0.07s]                                                                                               
Finished <<< armor_detector [0.08s]
Finished <<< rm_serial_driver [0.13s]                                                               
Starting >>> rm_vision_bringup
Finished <<< rm_vision_bringup [0.04s]
Finished <<< armor_tracker [0.59s]                     

Summary: 9 packages finished [1.05s]
```

无报错。试运行：

```bash
bandiera@BR:~/Desktop/v/test_outpost/vision_dev$ ./build/armor_tracker/trajectory_test 16.0 0.75
Pitch: 0.395801 Time: 1.13869 Velocity: 13.8538
```

提交：

![image-20260205020516314](/home/bandiera/.config/Typora/typora-user-images/image-20260205020516314.png)

由于仓库私有，故仅截图。

![image-20260205020546149](/home/bandiera/.config/Typora/typora-user-images/image-20260205020546149.png)
