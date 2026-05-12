from __future__ import annotations

import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from zed_positional_measurement.config import PipelineConfig
from zed_positional_measurement.geometry import scale_bbox_xyxy, scale_point


class ConfigTests(unittest.TestCase):
    def test_pipeline_config_round_trip_preserves_nested_classes(self) -> None:
        config = PipelineConfig.from_dict(
            {
                "recording": {
                    "enable_svo_recording": False,
                },
                "paper": {
                    "mode": "external_boxes",
                    "classes": {
                        "0": {"label": "paper", "color": "red"},
                        "1": {"label": "paper", "color": "green"},
                    },
                },
                "runtime": {
                    "detector_image_size": [640, 360],
                    "session_id": "demo-session",
                },
                "plane_detection": {
                    "include_floor_plane": True,
                },
            }
        )

        payload = config.to_dict()

        self.assertEqual(payload["paper"]["classes"]["0"]["color"], "red")
        self.assertEqual(payload["runtime"]["session_id"], "demo-session")
        self.assertEqual(config.paper.classes["1"].label, "paper")
        self.assertFalse(config.recording.enable_svo_recording)
        self.assertTrue(config.plane_detection.include_floor_plane)
        self.assertEqual(len(config.plane_detection.scene_probe_points_normalized), 9)

    def test_scale_helpers_convert_detector_coordinates_to_native_image_size(self) -> None:
        self.assertEqual(scale_bbox_xyxy((10, 5, 20, 15), (50, 25), (100, 50)), (20, 10, 40, 30))
        self.assertEqual(scale_point((5, 5), (50, 25), (100, 50)), (10, 10))


if __name__ == "__main__":
    unittest.main()
