# 界面截图来源

这些图片来自程序实际窗口。空闲截图用于展示操作界面，不表示完成了真机性能测量。

## 当前动力示教界面

[teaching-current.png](teaching-current.png)于 2026-11-27 从本次发布的 `tools/arm_gravity/teaching_view.py` 创建窗口后截取，使用 Qt offscreen、系统 Python 与 Noto Sans CJK SC 字体，分辨率为 1380 × 1063。

窗口保留默认两组配对和“离线模拟”，没有启动会话。串口与启动入口被拦截并确认未调用；报告目录为空，未加载历史日志，也未填入合成反馈。图中展示当前夹爪百分比跟随、轻阻尼、接触反馈与停止控件。

## 既有截图

以下图片于 2026-11-26 截取，保留供历史引用：

| 图片 | 内容 | 截取时状态 |
| --- | --- | --- |
| [diagnostics-20261126.png](diagnostics-20261126.png) | 七电机扫描／标零窗口 | 默认只读选项，未选臂、未扫描 |
| [moveit-20261126.png](moveit-20261126.png) | MoveIt 选择／控制／曲线窗口 | 预选运动左臂，模拟模式，未启动 |
| [teaching-20261126.png](teaching-20261126.png) | 旧版动力示教配对／日志窗口 | 默认两组配对，模拟模式，尚未接入当前夹爪反馈 |

旧图的文字与参数属于当时版本。当前首页使用 `teaching-current.png`，功能与参数以[运行配置参考](../reference/runtime-and-configuration.md)为准。
