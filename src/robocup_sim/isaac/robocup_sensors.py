import math
import yaml
import os
import xml.etree.ElementTree as ET

import omni.graph.core as og
import usdrt.Sdf
from isaacsim.ros2.nodes import (Ros2CameraGraphConfig, Ros2RtxLidarGraphConfig,
                                 create_ros2_camera_graph, create_ros2_rtx_lidar_graph)
from isaacsim.sensors.experimental.physics import IMU
from isaacsim.sensors.experimental.rtx import Lidar
from pxr import Gf, Sdf, UsdGeom, Vt

# config v zdrojakoch: isaac/../config/slam/sensors/sensors.yaml
SENSORS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'config', 'sensors', 'sensors.yaml')
CORE = 'omni:sensor:Core:'
EMITTER = CORE + 'emitterState:s001:'
EMITTER_STATE = 'OmniSensorGenericLidarCoreEmitterStateAPI:s001'
def frame_pose(urdf, frame):
    """Pose of a URDF link in base_link as Gf.Matrix4d (row-vector convention)."""
    joints = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().iter('joint')}
    pose = Gf.Matrix4d(1.0)
    link = frame
    while link != 'base_link':
        joint = joints[link]
        origin = joint.find('origin')
        xyz = [float(v) for v in (origin.get('xyz') or '0 0 0').split()]
        r, p, y = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
        rot = (Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(r))
               * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(p))
               * Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(y)))
        pose = pose * Gf.Matrix4d().SetTransform(rot, Gf.Vec3d(*xyz))
        link = joint.find('parent').get('link')
    return pose

def set_local_pose(stage, path, pose):
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(pose.ExtractTranslation())
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(pose.ExtractRotationQuat())


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)

def load_sensor_configs(path):
    config = load_yaml(path)
    folder = os.path.dirname(path)
    for group in ('lidars', 'imus'):
        for sensor in config.get(group, []):
            sensor['params'] = load_yaml(os.path.join(folder, sensor['model']))
            sensor['name'] = os.path.splitext(sensor['model'])[0]
    return config


def rotary_lidar_attributes(p):
    ch = p['channels']
    n = ch['count']
    hz = p['scan_rate_hz']
    firing_hz = int(round(hz * 360.0 / p['horizontal_resolution_deg']))
    step_ns = int(1e9/firing_hz/n)
    min_elevation = ch['elevation_min_deg']
    max_elevation = ch['elevation_max_deg']
    elevations = [min_elevation + (max_elevation - min_elevation) * i / (n - 1) for i in range(n)]
    return {
        CORE + 'scanType': p['scan_type'],
        CORE + 'rotationDirection': p['rotation_direction'],
        CORE + 'scanRateBaseHz': hz,
        CORE + 'patternFiringRateHz': firing_hz,
        CORE + 'reportRateBaseHz': firing_hz,
        CORE + 'numberOfEmitters': n,
        CORE + 'numberOfChannels': n,
        CORE + 'nearRangeM': p['near_range_m'],
        CORE + 'rangeOffsetM': p['range_offset_m'],
        CORE + 'farRangeM': p['far_range_m'],
        CORE + 'minReflectance': p['min_reflectance'],
        CORE + 'minReflectionRangeM': p['min_reflectance_range_m'],
        CORE + 'rangeAccuracyM': p['range_accuracy_m'],
        CORE + 'rangeResolutionM': p['range_resolution_m'],
        CORE + 'maxReturns': p['max_returns'],
        EMITTER + 'azimuthDeg': Vt.FloatArray([0.0] * n),
        EMITTER + 'elevationDeg': Vt.FloatArray(elevations),
        EMITTER + 'fireTimeNs': Vt.UIntArray([i * step_ns for i in range(n)]),
        EMITTER + 'channelId': Vt.UIntArray(list(range(1, n + 1))),
    }

def create_lidar(stage, path, pose, hz,attributes):
    Lidar.create(path=path, tick_rate=float(hz), schemas=[EMITTER_STATE], accumulate_outputs=True)
    prim = stage.GetPrimAtPath(path)

    for name, value in attributes.items():
        attr = prim.GetAttribute(name)
        #  isaac sim 6.0+ does not have this attribute
        if not attr.IsValid() and name == CORE + 'reportRateBaseHz':
            attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.UInt)
        if not attr.IsValid():
            raise RuntimeError(f'{path}: no attribute {name}')
        attr.Set(value)

    set_local_pose(stage, path, pose)

def create_sensors(stage, base_link, urdf, config):
    for lidar in config.get('lidars', []):
        p = lidar['params']
        lidar['prim'] = f"{base_link}/sensor_{lidar['name']}"       
        pose = frame_pose(urdf, lidar['frame'])
        create_lidar(stage, lidar['prim'], pose, p['scan_rate_hz'], rotary_lidar_attributes(p))

    for imu in config.get('imus', []):
        imu['prim'] = f"{base_link}/sensor_{imu['name']}"
        IMU.create(imu['prim'])
        set_local_pose(stage, imu['prim'], frame_pose(urdf, imu['frame']))

def create_imu_graph(imu_name, imu_path, frame_id, topic, read_gravity):
    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': f"/Graph/ROS_{imu_name}", 'evaluator_name': 'execution',
         # OnPhysicsStep tika len v on-demand grafe (IMU s krokom fyziky, 200 Hz)
         'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            keys.CREATE_NODES: [
                ('Tick', 'isaacsim.core.nodes.OnPhysicsStep'),
                ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('ReadImu', 'isaacsim.sensors.physics.IsaacReadIMU'),
                ('PubImu', 'isaacsim.ros2.bridge.ROS2PublishImu'),
            ],
            keys.SET_VALUES: [
                ('SimTime.inputs:resetOnStop', True),
                ('ReadImu.inputs:imuPrim', [usdrt.Sdf.Path(imu_path)]),
                ('ReadImu.inputs:readGravity', read_gravity),
                ('PubImu.inputs:frameId', frame_id),
                ('PubImu.inputs:topicName', topic),
            ],
            keys.CONNECT: [
                ('Tick.outputs:step', 'ReadImu.inputs:execIn'),
                ('ReadImu.outputs:execOut', 'PubImu.inputs:execIn'),
                ('ReadImu.outputs:angVel', 'PubImu.inputs:angularVelocity'),
                ('ReadImu.outputs:linAcc', 'PubImu.inputs:linearAcceleration'),
                ('ReadImu.outputs:orientation', 'PubImu.inputs:orientation'),
                ('Context.outputs:context', 'PubImu.inputs:context'),
                ('SimTime.outputs:simulationTime', 'PubImu.inputs:timeStamp'),
            ],
        },
    )

def create_sensor_graphs(config):
    for lidar in config.get('lidars', []):
        create_ros2_rtx_lidar_graph(Ros2RtxLidarGraphConfig(
            graph_path=f"/Graph/ROS_{lidar['name']}", lidar_prim=lidar['prim'], 
            frame_id=lidar['frame'], publish_laser_scan=False, publish_point_cloud=True, 
            point_cloud_topic=lidar['params']['topic']
        ))

    for imu in config.get('imus', []):
        create_imu_graph(imu_name=imu['name'], imu_path=imu['prim'], frame_id=imu['frame'],
                         topic=imu['params']['topic'], read_gravity=imu['params']['read_gravity'])