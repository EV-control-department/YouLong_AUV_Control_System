from pathlib import Path
import xml.etree.ElementTree as ET


WORLD = Path(__file__).parents[1] / "worlds" / "lesson04_diff_drive.sdf"


def test_diff_drive_world_contains_correct_wheel_axes_and_plugin():
    root = ET.parse(WORLD).getroot()
    axes = [axis.text.strip() for axis in root.findall(".//joint/axis/xyz")]

    assert axes[:2] == ["0 0 1", "0 0 1"]
    plugins = root.findall(".//plugin")
    assert any(plugin.get("name", "").endswith("::DiffDrive") for plugin in plugins)
