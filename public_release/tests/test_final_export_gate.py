"""最终轨迹未通过复核时，不发布可供后续使用的 CSV。"""

import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

try:
    import main as app
    from config import load_config
    from robot_scene import get_start_goal
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(f"机器人环境未安装离线规划依赖：{exc}") from exc


class FinalExportGate(unittest.TestCase):
    def run_pipeline(self, safe):
        config = load_config()
        config["output"]["directory"] = Path("output")
        config["output"]["save_csv"] = True
        config["output"]["save_figures"] = False
        config["output"]["show_visualization"] = False
        q_start, q_goal = get_start_goal()
        path = np.vstack([q_start, q_goal])
        with patch.object(app, "load_config", return_value=config), \
                patch.object(app, "birrt_connect", return_value=(path, 0)), \
                patch.object(app, "sampled_trajectory_valid", return_value=safe), \
                patch.object(app, "evaluate_kinematics", return_value={}), \
                patch.object(app, "print_kinematic_metrics"), \
                patch.object(app, "keypoint_metrics", return_value={}), \
                patch.object(app, "print_metrics_table"), \
                patch.object(app, "print_dynamic_metrics_table"), \
                patch.object(Path, "mkdir"), \
                patch.object(Path, "unlink", autospec=True) as unlink, \
                patch.object(app, "export_controller_program") as controller_export, \
                patch.object(app, "export_ruckig_trajectory") as trajectory_export, \
                patch.object(app, "export_keypoints"), \
                patch.object(app, "export_metrics_comparison"):
            error = None
            try:
                app.main()
            except RuntimeError as exc:
                error = exc
            return error, unlink, controller_export, trajectory_export

    def test_invalid_result_removes_stale_final_csv(self):
        error, unlink, controller_export, trajectory_export = self.run_pipeline(False)
        self.assertIsNotNone(error)
        self.assertIn("复核", str(error))
        files = load_config()["output"]["files"]
        removed = [call.args[0].name for call in unlink.call_args_list]
        self.assertIn(files["controller_program"], removed)
        self.assertIn(files["ruckig_trajectory"], removed)
        controller_export.assert_not_called()
        trajectory_export.assert_not_called()

    def test_valid_result_exports_both_existing_csv(self):
        error, _, controller_export, trajectory_export = self.run_pipeline(True)
        self.assertIsNone(error)
        controller_export.assert_called_once()
        trajectory_export.assert_called_once()


if __name__ == "__main__":
    unittest.main()
