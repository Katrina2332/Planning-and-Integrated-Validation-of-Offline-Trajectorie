# AR2010 ROS-Industrial 模型

本目录取自 [ros-industrial/motoman 的 `motoman_ar2010_support`](https://github.com/ros-industrial/motoman/tree/846fbbc57c60223f12bf4795ea6404982d57709e/motoman_ar2010_support)，固定提交 `846fbbc57c60223f12bf4795ea6404982d57709e`。上游 `package.xml` 标注许可证为 BSD-3-Clause；条款及来源见根目录的 `THIRD_PARTY_NOTICES.md`。使用前须核对具体机器人配置。

`upstream/` 保留原始 Xacro 和包元数据；`meshes/` 是上游原始 visual 与 collision STL。`ar2010.urdf` 由原始 Xacro 展开，只将 ROS `package://` 网格路径改为本地相对路径，并内联 Yaskawa 蓝色材质，便于当前 Windows/Python 环境直接加载。机器人关节结构、轴、位姿和限位来自上游 Xacro。

原轻量模型仍保留在项目根目录的 `ar2010_planning.urdf`，可通过 `catalogs/robots.yaml` 切回。当前资源字典选用本目录的网格模型。网格与控制器、实物的一致性尚未验证。
