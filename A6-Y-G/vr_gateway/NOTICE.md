# VR input sources

The gateway is a new, minimal implementation of the input protocol described in
`docs/VR遥操作/虚拟机械臂/VR_RVIZ_PLAN.md`. Event field names and column-major matrix interpretation
were checked against the local `Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py`
(the reference repository's root license is Apache 2.0). No Piper model, motor
sender, private key, camera pipeline, or training code is included here.

Vuer is a separately installed third-party dependency, licensed under MIT.
The original reference and per-file evidence are listed in
`docs/VR遥操作/参考方案/VR_REFERENCE_ANALYSIS.md`.
