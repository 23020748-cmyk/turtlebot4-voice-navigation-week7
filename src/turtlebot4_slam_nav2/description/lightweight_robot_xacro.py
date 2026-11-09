import subprocess
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

xacro_file = sys.argv[1]
namespace = sys.argv[2] if len(sys.argv) > 2 else ''
low_resource = len(sys.argv) < 4 or sys.argv[3] == 'true'
camera_enabled = len(sys.argv) > 4 and sys.argv[4] == 'true'
result = subprocess.run(
    ['xacro', xacro_file, 'gazebo:=ignition', f'namespace:={namespace}',
     f'camera_enabled:={str(camera_enabled).lower()}'],
    check=True, capture_output=True, text=True,
)
root = ET.fromstring(result.stdout)
sensors = {e.get('name'): e for e in root.iter('sensor')}
lidar = sensors.get('rplidar')
if lidar is None:
    raise RuntimeError('Could not find TurtleBot 4 RPLIDAR sensor in expanded xacro')
for engine in root.iter('render_engine'):
    engine.text = 'ogre2'
lidar.find('update_rate').text = '3.0'
lidar.find('visualize').text = '0'
lidar.find('lidar/range/min').text = '0.164'
horizontal = lidar.find('lidar/scan/horizontal')
horizontal.find('samples').text = '180'
horizontal.find('min_angle').text = str(-3.141592653589793)
horizontal.find('max_angle').text = str(3.141592653589793)
for name, sensor in sensors.items():
    if name == 'rgbd_camera':
        sensor.find('update_rate').text = '1.0'
        sensor.find('visualize').text = '0'
for plugin in root.iter('plugin'):
    if 'ros2_control' in plugin.get('filename', ''):
        ros = plugin.find('ros')
        if ros is None:
            ros = ET.SubElement(plugin, 'ros')
        if low_resource:
            ET.SubElement(ros, 'remapping').text = 'diffdrive_controller/cmd_vel_unstamped:=cmd_vel'
            ET.SubElement(ros, 'remapping').text = 'diffdrive_controller/odom:=/odom'
            params = plugin.find('parameters')
            if params is not None:
                params.text = str(Path(__file__).resolve().parent.parent / 'config' / 'low_resource_control.yaml')
    for rate in plugin.findall('update_frequency'):
        rate.text = '15'
keep = {'rplidar'} | ({'rgbd_camera'} if camera_enabled else set())
for gazebo in root.iter('gazebo'):
    for child in list(gazebo):
        if child.tag == 'sensor' and child.get('name') not in keep:
            gazebo.remove(child)
sys.stdout.write(ET.tostring(root, encoding='unicode'))
