# 交给 Codex 的实现提示词：GAP-Step 最小可行实验系统

请直接复制以下提示词给 Codex。

```text
你是一名资深 Python / PyTorch / 强化学习工程师。请你根据我提供的实验实施文档，实现一个名为 GAP-Step 的最小可行研究项目。

项目目标：
实现一个二维时变多窗口穿越环境，用于验证“特权教师演示 + 视觉学生行为克隆 + 窗口状态辅助预测 + PPO 微调”是否能让视觉学生策略学会窗口选择与穿越时机判断。

重要要求：
1. 不要实现完整 3D 四旋翼。
2. 不要实现完整 SITT 代理学生机制。
3. 不要实现 world model 或未来视频预测。
4. 不要实现主动相机控制。
5. 先实现 2D / 2.5D 最小可行版本。
6. 项目必须可以本地运行、训练、评估、可视化。

在开始写代码前，请先做以下事情：

第一，创建一个专门用于该项目的 conda 环境。
环境名建议：

    gap-step

第二，如果你准备安装 GPU 版 PyTorch 或任何依赖 CUDA 的包，请先向我提问，确认我的 CUDA 版本和显卡驱动信息。不要猜测 CUDA 版本。请让我提供：

    nvidia-smi 输出
    CUDA version
    操作系统
    是否需要 GPU 训练

如果我没有提供 CUDA 信息，默认先创建 CPU 版环境，或者先停止并询问我。

第三，请提供 environment.yml 和 README.md，README 中必须包含完整命令：

    conda env create -f environment.yml
    conda activate gap-step
    python scripts/render_random_policy.py
    python trainers/generate_demos.py
    python trainers/train_bc.py
    python trainers/train_ppo.py
    python trainers/evaluate.py

建议依赖：
- python=3.10
- pytorch
- numpy
- gymnasium
- matplotlib
- pygame 或 opencv-python
- pyyaml
- tqdm
- pandas
- tensorboard
- pytest

如果使用 GPU 版 PyTorch，请在确认 CUDA 版本后再写对应安装命令。

请实现以下项目结构：

    gap_step/
      envs/
        gap_step_env.py
        gate_dynamics.py
        renderer.py
      teachers/
        heuristic_teacher.py
      models/
        student_policy.py
        cnn_encoder.py
      trainers/
        generate_demos.py
        train_bc.py
        train_ppo.py
        evaluate.py
      configs/
        env_e1.yaml
        env_e2.yaml
        env_e3.yaml
        train_bc.yaml
        train_ppo.yaml
      scripts/
        render_random_policy.py
        render_trained_policy.py
      tests/
        test_env.py
        test_gate_dynamics.py
        test_teacher.py
      README.md
      environment.yml

环境设计：
- 二维平面 workspace = [0, W] × [0, H]
- 默认 W=12.0, H=8.0
- 中间墙 x = W/2
- 墙上有 1 或 2 个时变窗口
- 智能体从左侧出发，目标在右侧
- 智能体必须穿过某个窗口才能到达目标
- 智能体是半径为 robot_radius 的圆盘
- 动力学使用二维双积分模型
- 动作是二维加速度 [a_x, a_y]

窗口动态：
- 每个窗口有中心位置 y_i
- 当前窗口宽度 d_i(t)
- 当前安全标志 s_i(t) = 1[d_i(t) > 2 * robot_radius + safe_margin]
- 初始实现周期变化：

    d_i(t)=d_min+(d_max-d_min)/2*(1+sin(omega_i*t+phi_i))

教师设计：
- 实现 heuristic teacher
- 教师可以看到当前真实状态和当前窗口宽度/安全标志
- 教师不能看到未来窗口状态
- 如果存在当前安全窗口，选择路径代价最小的安全窗口
- 如果没有安全窗口，选择当前宽度最大的窗口，在窗口前等待或靠近 staging point
- 教师输出 teacher_acc 和 teacher_gate

学生输入：
- image_stack：最近 K_obs 帧灰度图像，默认 K_obs=4
- proprio：速度、目标相对位置、上一动作

学生网络：
- CNN 编码 image_stack
- MLP 编码 proprio
- 拼接后输出 latent
- 输出 head 包括：
  1. acc head：二维连续加速度
  2. gate head：窗口选择分类
  3. width head：预测每个窗口当前宽度 d_i(t)
  4. safe head：预测每个窗口当前是否安全 s_i(t)

训练流程：
1. generate_demos.py
   - 用 heuristic teacher 生成演示数据
   - 保存 image_stack, proprio, teacher_acc, teacher_gate, true_widths, true_safe_flags

2. train_bc.py
   - 训练 BC-only
   - 训练 BC+Aux
   - loss 包括：
     L_BC = MSE(acc_pred, teacher_acc) + lambda_g * CE(gate_pred, teacher_gate)
     L_width = L1(width_pred, true_widths)
     L_safe = BCE(safe_pred, true_safe_flags)
     L_total = L_BC + lambda_w * L_width + lambda_s * L_safe

3. train_ppo.py
   - 从 BC+Aux checkpoint 初始化
   - 使用 PPO 或简化 actor-critic 微调连续加速度策略
   - 可以保留 auxiliary loss
   - 如果实现完整 PPO 太复杂，请先实现一个清晰可运行的版本，并在 README 中说明限制

4. evaluate.py
   - 评估多个模型：Teacher-Heuristic, Visual-PPO, BC-only, BC+Aux, BC+Aux+PPO
   - 输出 CSV 或 JSON
   - 指标包括：success_rate, collision_rate, crossing_success_rate, closed_gate_attempt_rate, time_to_goal, gate_choice_accuracy, width_mae, safe_f1, return

可视化：
- render_random_policy.py：随机策略 rollout 可视化
- render_trained_policy.py：加载 checkpoint 并保存 rollout 视频或 GIF
- 渲染图中应显示：智能体、墙、窗口开口大小、目标、轨迹

配置：
- env_e1.yaml：单窗口固定开放
- env_e2.yaml：单窗口周期变化
- env_e3.yaml：双窗口异步变化
- train_bc.yaml：BC 训练超参数
- train_ppo.yaml：PPO 训练超参数

测试：
- test_gate_dynamics.py：检查窗口宽度和安全标志计算正确
- test_env.py：检查 reset/step/render 不报错
- test_teacher.py：检查教师动作维度正确，并能在简单环境中达到较高成功率

验收标准：
1. 能创建 conda 环境。
2. 能运行随机策略可视化。
3. 能生成教师数据。
4. 能训练 BC-only 和 BC+Aux。
5. 能进行 PPO 微调。
6. 能评估并输出表格。
7. 能保存 checkpoint 和 rollout 可视化。
8. README 命令清晰，别人可以复现。

代码风格要求：
- 每个模块职责清晰。
- 配置参数不要硬编码，优先放到 yaml。
- 所有随机种子可设置。
- 训练日志保存到 runs/ 或 logs/。
- checkpoint 保存到 checkpoints/。
- 不要写过度复杂的抽象，先保证可运行。
- 如果某个设计有歧义，请先给出你的实现假设，并尽量选择最小可行方案。

现在请先检查需求，并询问我是否需要 GPU/CUDA 版环境。如果需要，请让我提供 CUDA 版本；如果我回答使用 CPU，请直接创建 CPU 版 conda 环境配置和项目代码。
```
