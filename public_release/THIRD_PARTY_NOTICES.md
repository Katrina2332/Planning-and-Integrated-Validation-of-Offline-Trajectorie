# 第三方素材与依赖

## AR2010 模型

`models/ar2010_ros_industrial/` 的 Xacro、网格和由 Xacro 展开的 URDF 来自 ROS-Industrial 的 [`motoman_ar2010_support`](https://github.com/ros-industrial/motoman/tree/846fbbc57c60223f12bf4795ea6404982d57709e/motoman_ar2010_support)，固定提交 `846fbbc57c60223f12bf4795ea6404982d57709e`。本仓库仅将 URDF 的 ROS 网格路径改为本地相对路径，并内联显示材质。根目录的 `ar2010_planning.urdf` 也沿用上游关节链与限位，改用简化几何。上游 `package.xml` 标注 `BSD-3-Clause`，其作者及维护者记录保留在 `models/ar2010_ros_industrial/upstream/package.xml`。这些模型的上游版权归原作者及贡献者所有，不受本仓库 MIT 许可声明覆盖。

BSD 3-Clause License:

Copyright (c) the authors and contributors of ROS-Industrial `motoman_ar2010_support`.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Python 依赖

本仓库不包含第三方 Python 库的源码。`requirements.txt` 安装 Robotics Toolbox、Ruckig、Viser 等依赖，它们各自适用上游许可证。Ruckig 的许可证见 [上游 MIT 文本](https://github.com/pantor/ruckig/blob/main/LICENSE)。
