# 模型搭建&算法仿真

**简体中文 | [English](README_en.md)**

本项目使用 MATLAB(R2022b) 进行平衡算法的设计和仿真，主要进行了：

- 腿部连杆姿态计算
- 系统状态空间方程建立
- LQR 反馈矩阵计算和拟合
- Simscape Multibody 物理模型搭建
- Simulink 控制算法仿真

---

## 控制算法

平衡算法主要参考了这个[专栏文章](https://zhuanlan.zhihu.com/p/563048952)，仅有少许不同，因此不在此处详述，仅描述大致过程，包含以下几个步骤：

> 注：代码中变量名称与该文章所述基本保持一致，读者可以对照查看

1. 对腿部结构进行化简，求得化简前后各符号的对照关系，从而可以使用电机数据计算出腿部的姿态
2. 求得化简后的虚拟腿受力到电机扭矩的映射关系，从而可以使用VMC思想对化简后的腿部进行控制
3. 进行经典力学分析和符号化简，求得系统状态空间方程
4. 设定LQR中的权重系数，求取反馈矩阵K，并对不同腿长求出的K进行拟合
5. 进行状态反馈控制

---

## 文件介绍

- `leg_func_calc.m`：进行腿部解算和 VMC 映射计算，包含上述步骤1-2，导出以下三个M函数：
	- `leg_pos.m`：可由关节电机角度求得腿部姿态
	- `leg_spd.m`：可由关节电机角度速度求得腿部运动速度
	- `leg_conv.m`：可由虚拟腿目标扭矩和推力求得电机所需输出的力矩
- `sys_calc.m`：求得系统状态方程，求得反馈矩阵，包含上述步骤3-4，导出下述M函数：
	- `lqr_k.m`：代入腿长 L0 后返回该腿长对应的反馈矩阵 K
- `leg_sim.slx`：腿部仿真 Simulink 模型，包含腿部 Multibody 物理模型，用于腿部VMC算法验证
	- `leg_sim_2020a.slx`：导出的旧版本模型，适用 R2020a 及以上版本
- `sys_sim.slx`：整机平衡仿真 Simulink 模型，包含整机 Multibody 物理模型，用于 VMC+LQR 整体算法验证
	- `sys_sim_2020a.slx`：导出的旧版本模型，适用 R2020a 及以上版本

> 注：导出的M函数都在仿真模型中直接使用，使用 MATLAB Coder 工具箱直接转换为C语言后即可直接在实机代码中使用，在本项目主控代码中可以找到同名文件
