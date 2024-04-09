# MILPSafetyGuard

This repository documents the codebase for simulations presented in the 6th Annual Learning for Dynamics & Control Conference (L4DC) paper entitled "System-level Safety Guard: Safe Tracking Control through Uncertain Neural Network Dynamics Models". The development is on MATLAB based on several 3rd party toolboxs. Simulation videos are available on the webpage https://xiaolisean.github.io/publication/2023-11-01-L4DC2024.
 
Please cite as:
```
@article{li2023system,
  title={System-level Safety Guard: Safe Tracking Control through Uncertain Neural Network Dynamics Models},
  author={Li, Xiao and Li, Yutong and Girard, Anouck and Kolmanovsky, Ilya},
  journal={arXiv preprint arXiv:2312.06810},
  year={2023}
}
```

## Installation
1. Install **MATLAB Toolbox Manager** [https://tbxmanager.com/]. It's recommended to save the path where the tbxmanager is installed to MATLAB.

```
urlwrite('http://www.tbxmanager.com/tbxmanager.m', 'tbxmanager.m');
tbxmanager
savepath
tbxmanager restorepath
```

2. Install **YALMIP** using tbxmanager [https://yalmip.github.io/tutorial/installation/]

```
tbxmanager install yalmip
```

3. Install **CORA** using MATLAB Add-On Explorer (A Toolbox for Reachability Analysis) [https://tumcps.github.io/CORA/]

4. Run ```TestOminidirectionalRobot.m```
