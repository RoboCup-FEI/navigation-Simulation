# Isaac Sim strana simulacie: nacita arenu z USD, importuje robota shoddy
# z xacro v robocup_description a napoji ho na ROS 2:
#   von: /clock, /odom + TF odom->base_link a senzory podla modelu:
#        v1 (robot.urdf.xacro): 2D lidar Hokuyo -> /scan_raw (frame lidar_frame,
#           robocup.launch.py ho preposiela na /scan)
#        v2 (robot_v2.urdf.xacro): Hesai XT16, Livox Horizon s IMU a kamera
#           Basler, vid robocup_sensors.py
#   dnu: /cmd_vel
#
# Podvozok je kinematicky: cmd_vel sa nastavi priamo ako rychlost tela a kolesa
# su bez trenia. Fyzikalny 4-kolesovy skid steer v PhysX s tymto URDF
# vibroval, nevedel sa otocit na mieste a prevracal sa.
#
# Spusta ho ../../../run.sh. Rucne cez Python Isaacu zo shellu BEZ
# /opt/ros/<distro> (bridge ma vlastne ROS kniznice pre Python 3.12, distro
# vyberie podla Ubuntu). xacro sa spusta s /opt/ros/$ROBOCUP_ROS_DISTRO (jazzy):
#   env -i HOME=$HOME DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY PATH=/usr/bin:/bin \
#     ~/isaacsim/python.sh src/robocup_sim/isaac/robocup_isaac.py
import argparse
import math
import os
import shutil
import subprocess
import traceback
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
DEFAULT_WORLD = os.path.join(
    WS, 'src/robocup_sim/worlds/robocup_arena/2026_09_01_RoboCup_HCC_Arena_V1.usd')

parser = argparse.ArgumentParser()
parser.add_argument('--world', default=DEFAULT_WORLD, help='USD prostredie')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--x', type=float, default=0.0)
parser.add_argument('--y', type=float, default=0.0)
parser.add_argument('--yaw', type=float, default=0.0)
parser.add_argument('--model', choices=['v1', 'v2'], default='v2', help='Model robota')
parser.add_argument('--frames', type=int, default=0,
                    help='Skonci po tolkych snimkach (0 = bez konca)')
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({'headless': args.headless})

import isaacsim.core.experimental.utils.app as app_utils

for ext in ('omni.scene.optimizer.core', 'isaacsim.robot.schema', 'isaacsim.ros2.bridge'):
    app_utils.enable_extension(ext)
simulation_app.update()

import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.ros2.nodes import (Ros2ClockGraphConfig, Ros2RtxLidarGraphConfig,
                                 create_ros2_clock_graph, create_ros2_rtx_lidar_graph)
from isaacsim.sensors.experimental.rtx import Lidar
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ROS_DISTRO sa Isaacu nedava, bridge by potom hladal systemove ROS kniznice
ROS_DISTRO = os.environ.get('ROBOCUP_ROS_DISTRO', 'jazzy')
# model -> xacro v robocup_description/urdf
MODELS = {'v1': 'robot.urdf.xacro', 'v2': 'robot_v2.urdf.xacro'}
XACRO = MODELS[args.model]
SCAN_FRAME = 'lidar_frame'  # 2D lidar modelu v1
# v2: fyzika 200 Hz, IMU Livoxu publikuje pri kazdom kroku (BMI088 ma 200 Hz; 5 ms deli aj
# 100 ms snimok lidaru), render 50 Hz (4 kroky fyziky na snimok), aby sa kamera a lidary
# nekreslili 200x za sekundu
PHYSICS_DT = 1.0 / 200.0 if args.model == 'v2' else 1.0 / 60.0
RENDERING_DT = 1.0 / 50.0 if args.model == 'v2' else 1.0 / 60.0
MAX_ACCELERATION = 2.0
MAX_ANGULAR_ACCELERATION = 2 * MAX_ACCELERATION / 0.48
CMD_VEL_NODE = '/Graph/ROS_Base/CmdVel'
# kolesa sa vznasaju tesne nad zemou: podvozok je kinematicky a pri kontakte so zemou
# sa kolesa zaborili, fyzika robota odtlacala a trenie brzdilo otacanie
WHEEL_CLEARANCE = 0.005


def build_world(stage, usd_file):
    UsdLux.DomeLight.Define(stage, '/World/DomeLight').CreateIntensityAttr(800.0)
    sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
    sun.CreateIntensityAttr(2000.0)
    UsdGeom.XformCommonAPI(sun).SetRotate((30.0, 20.0, 0.0))

    ground = UsdGeom.Cube.Define(stage, '/World/ground')
    UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -0.05))
    UsdGeom.XformCommonAPI(ground).SetScale((25.0, 25.0, 0.05))
    ground.CreateDisplayColorAttr([(0.6, 0.6, 0.6)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    load_usd_world(stage, usd_file)


def load_usd_world(stage, usd_file):
    src = Usd.Stage.Open(usd_file)
    scale = UsdGeom.GetStageMetersPerUnit(src)
    y_up = UsdGeom.GetStageUpAxis(src) == UsdGeom.Tokens.y
    env = UsdGeom.Xform.Define(stage, '/World/environment')
    stage.DefinePrim('/World/environment/asset').GetReferences().AddReference(usd_file)
    xf = UsdGeom.XformCommonAPI(env)
    xf.SetScale((scale, scale, scale))
    if y_up:
        xf.SetRotate((90.0, 0.0, 0.0))
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']) \
        .ComputeWorldBound(env.GetPrim()).ComputeAlignedRange()
    center = (box.GetMin() + box.GetMax()) / 2
    xf.SetTranslate((-center[0], -center[1], -box.GetMin()[2]))
    meshes, boxed = 0, []
    xform_cache = UsdGeom.XformCache()
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
    for prim in Usd.PrimRange(env.GetPrim()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        world = xform_cache.GetLocalToWorldTransform(prim)
        axes = [world.GetRow3(i).GetLength() for i in range(3)]
        if max(axes) > 10 * min(axes):
            r = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            cube = UsdGeom.Cube.Define(stage, f'/World/environment_colliders/box_{len(boxed)}')
            UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d((r.GetMin() + r.GetMax()) / 2))
            UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(*(r.GetSize() / 2)))
            cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            boxed.append(f'{prim.GetName()} {r.GetSize()[0]:.2f}x{r.GetSize()[1]:.2f}x{r.GetSize()[2]:.2f}m')
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
        meshes += 1
    for prim in Usd.PrimRange(env.GetPrim()):
        if prim.GetName().lower() == 'ceiling':
            UsdGeom.Imageable(prim).MakeInvisible()
    size = box.GetMax() - box.GetMin()
    ViewportManager.set_camera_view('/OmniverseKit_Persp', eye=[0.0, -0.9 * size[1], 0.9 * size[1]],
                                    target=[0.0, 0.0, 0.0])
    print(f'[robocup_isaac] {usd_file}: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} m, '
          f'{meshes} mesh colliders, box colliders for {boxed}')


def import_robot():
    out_dir = os.path.expanduser('~/.cache/robocup_isaac')
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir)
    urdf = os.path.join(out_dir, 'shoddy.urdf')
    xacro_file = os.path.join(WS, 'src/robocup_description/urdf', XACRO)
    subprocess.run(
        ['bash', '-c', f'source /opt/ros/{ROS_DISTRO}/setup.bash && source {WS}/install/setup.bash '
                       f'&& xacro {xacro_file} > {urdf}'],
        env={'HOME': os.path.expanduser('~'), 'PATH': '/usr/bin:/bin'},
        check=True)

    cfg = URDFImporterConfig()
    cfg.urdf_path = urdf
    cfg.usd_path = out_dir
    cfg.fix_base = False
    cfg.merge_fixed_joints = True
    cfg.ros_package_paths = [{'name': 'robocup_description',
                              'path': os.path.join(WS, 'src/robocup_description')}]
    cfg.joint_target_type = 'none'
    usd_file = URDFImporter(cfg).import_urdf()
    if not usd_file:
        raise RuntimeError(f'URDF import failed for {urdf}')
    return usd_file, urdf


def lidar_offset(urdf):
    for joint in ET.parse(urdf).getroot().iter('joint'):
        if joint.find('child').get('link') == SCAN_FRAME:
            assert joint.find('parent').get('link') == 'base_link'
            return [float(v) for v in joint.find('origin').get('xyz').split()]
    raise RuntimeError(f'no joint to {SCAN_FRAME} in {urdf}')


def base_height(urdf):
    """Height of base_link above the floor: lowest wheel point plus WHEEL_CLEARANCE.

    Wheels are the children of continuous joints with a cylinder collision; their
    joint chains to base_link must be pure translations (true for v1 and v2).
    """
    root = ET.parse(urdf).getroot()
    joints = {j.find('child').get('link'): j for j in root.iter('joint')}
    lowest = 0.0
    for joint in root.iter('joint'):
        if joint.get('type') != 'continuous':
            continue
        link = joint.find('child').get('link')
        cylinder = root.find(f"link[@name='{link}']/collision/geometry/cylinder")
        if cylinder is None:
            continue
        z = 0.0
        while link != 'base_link':
            origin = joints[link].find('origin')
            rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
            assert not any(rpy), f'rotated joint on the way to wheel {link}'
            z += float(origin.get('xyz').split()[2])
            link = joints[link].find('parent').get('link')
        lowest = max(lowest, float(cylinder.get('radius')) - z)
    if not lowest:
        raise RuntimeError(f'no wheels found in {urdf}')
    return lowest + WHEEL_CLEARANCE


def find_prims(root_prim):
    articulation = base_link = None
    for prim in Usd.PrimRange(root_prim):
        if articulation is None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation = prim.GetPath().pathString
        if prim.GetName() == 'base_link' and base_link is None:
            base_link = prim.GetPath().pathString
    if not (articulation and base_link):
        raise RuntimeError(f'robot prims not found: articulation={articulation} '
                           f'base_link={base_link}')
    return articulation, base_link


def make_frictionless(stage, robot):
    material = UsdShade.Material.Define(stage, '/World/PhysicsMaterials/frictionless')
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(0.0)
    physics_material.CreateDynamicFrictionAttr(0.0)
    physics_material.CreateRestitutionAttr(0.0)
    # PhysX kombinuje trenie dvoch materialov priemerom: s podlahou bez materialu (0.5)
    # by vyslo 0.25, 'min' dava naozaj 0
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr('min')
    physx_material.CreateRestitutionCombineModeAttr('min')
    UsdShade.MaterialBindingAPI.Apply(robot).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, 'physics')


def create_base_graph(articulation):
    keys = og.Controller.Keys
    target = [usdrt.Sdf.Path(articulation)]
    og.Controller.edit(
        {'graph_path': '/Graph/ROS_Base', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('Tick', 'omni.graph.action.OnPlaybackTick'),
                ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('CmdVel', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                ('Odom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
                ('PubOdom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ('PubTF', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'),
            ],
            keys.SET_VALUES: [
                ('SimTime.inputs:resetOnStop', True),
                ('CmdVel.inputs:topicName', '/cmd_vel'),
                ('Odom.inputs:chassisPrim', target),
                ('PubOdom.inputs:topicName', '/odom'),
                ('PubOdom.inputs:odomFrameId', 'odom'),
                ('PubOdom.inputs:chassisFrameId', 'base_link'),
                ('PubTF.inputs:parentFrameId', 'odom'),
                ('PubTF.inputs:childFrameId', 'base_link'),
            ],
            keys.CONNECT: [
                ('Tick.outputs:tick', 'CmdVel.inputs:execIn'),
                ('Tick.outputs:tick', 'Odom.inputs:execIn'),
                ('Context.outputs:context', 'CmdVel.inputs:context'),
                ('Odom.outputs:execOut', 'PubOdom.inputs:execIn'),
                ('Odom.outputs:execOut', 'PubTF.inputs:execIn'),
                ('Odom.outputs:position', 'PubOdom.inputs:position'),
                ('Odom.outputs:orientation', 'PubOdom.inputs:orientation'),
                ('Odom.outputs:linearVelocity', 'PubOdom.inputs:linearVelocity'),
                ('Odom.outputs:angularVelocity', 'PubOdom.inputs:angularVelocity'),
                ('Odom.outputs:position', 'PubTF.inputs:translation'),
                ('Odom.outputs:orientation', 'PubTF.inputs:rotation'),
                ('Context.outputs:context', 'PubOdom.inputs:context'),
                ('Context.outputs:context', 'PubTF.inputs:context'),
                ('SimTime.outputs:simulationTime', 'PubOdom.inputs:timeStamp'),
                ('SimTime.outputs:simulationTime', 'PubTF.inputs:timeStamp'),
            ],
        },
    )


class KinematicBase:

    def __init__(self, articulation, height):
        # Articulation az v prvom kroku: physics views existuju az po play(), ale
        # podvozok musi drzat robota uz od prveho kroku, inak sa pred nim robot
        # pohne a odometria si zoberie naklonenu pociatocnu pozu
        self.path = articulation
        self.robot = None
        self.height = height
        self.lin = og.Controller.attribute(CMD_VEL_NODE + '.outputs:linearVelocity')
        self.ang = og.Controller.attribute(CMD_VEL_NODE + '.outputs:angularVelocity')
        self.v = self.w = 0.0

    @staticmethod
    def _ramp(current, target, limit):
        step = limit * PHYSICS_DT
        return current + max(-step, min(step, target - current))

    def step(self):
        if self.robot is None:
            self.robot = Articulation(self.path)
        self.v = self._ramp(self.v, float(self.lin.get()[0]), MAX_ACCELERATION)
        self.w = self._ramp(self.w, float(self.ang.get()[2]), MAX_ANGULAR_ACCELERATION)
        positions, orientations = self.robot.get_world_poses()
        x, y, _ = positions.numpy()[0]
        qw, qx, qy, qz = orientations.numpy()[0]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
        self.robot.set_world_poses([[x, y, self.height]],
                                   [[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]])
        self.robot.set_velocities([[self.v * math.cos(yaw), self.v * math.sin(yaw), 0.0]],
                                  [[0.0, 0.0, self.w]])


def main():
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/World').GetPrim())

    build_world(stage, args.world)

    usd_file, urdf = import_robot()
    height = base_height(urdf)
    spawn = UsdGeom.Xform.Define(stage, '/World/shoddy')
    UsdGeom.XformCommonAPI(spawn).SetTranslate((args.x, args.y, height))
    UsdGeom.XformCommonAPI(spawn).SetRotate((0.0, 0.0, math.degrees(args.yaw)))
    robot = stage.DefinePrim('/World/shoddy/robot')
    robot.GetReferences().AddReference(usd_file)
    robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    make_frictionless(stage, robot)
    simulation_app.update()
    articulation, base_link = find_prims(robot)
    print(f'[robocup_isaac] model={args.model} articulation={articulation} base_link={base_link} '
          f'base height {height:.4f} m')

    if args.model == 'v1':
        lidar_path = base_link + '/rtx_lidar'
        Lidar.create(path=lidar_path, config='Example_Rotary_2D', tick_rate=10.0,
                     translations=[lidar_offset(urdf)],
                     attributes={'omni:sensor:Core:nearRangeM': 0.15,
                                 'omni:sensor:Core:farRangeM': 4.0})
    else:
        import robocup_sensors
        sensors = robocup_sensors.create_sensors(stage, base_link, urdf)

    SimulationManager.setup_simulation(dt=PHYSICS_DT, device='cpu')
    RenderingManager.set_dt(RENDERING_DT)
    simulation_app.update()

    create_ros2_clock_graph(Ros2ClockGraphConfig(graph_path='/Graph/ROS_Clock'))
    create_base_graph(articulation)
    if args.model == 'v1':
        create_ros2_rtx_lidar_graph(Ros2RtxLidarGraphConfig(
            graph_path='/Graph/ROS_Lidar', lidar_prim=lidar_path,
            frame_id=SCAN_FRAME, laser_scan_topic='/scan_raw'))
    else:
        robocup_sensors.create_sensor_graphs(sensors)
    simulation_app.update()

    base = KinematicBase(articulation, height)
    SimulationManager.register_callback(lambda dt, context: base.step(),
                                        event=SimulationEvent.PHYSICS_PRE_STEP)
    app_utils.play()
    simulation_app.update()
    print('[robocup_isaac] simulation running', flush=True)
    frame = 0
    while simulation_app.is_running():
        simulation_app.update()
        frame += 1
        if args.frames and frame >= args.frames:
            break
    app_utils.stop()


try:
    main()
except BaseException:
    traceback.print_exc()
finally:
    simulation_app.close()
