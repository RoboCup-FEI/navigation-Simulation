# Chodec v simulacii cez IsaacSim Replicator Agent (IRA): animovana postava z NVIDIA
# assetov (stahuje sa z S3), chodi dokola po zadanych bodoch (IRA spravanie 'patrol')
# po navmeshi. Pouziva len casti IRA (navmesh, CharacterLoader); scenu, fyziku a ROS
# riadi robocup_isaac.py. Postavy pocas behu posuva AgentsManager z
# omni.metropolis.pipeline.
#
# Navmesh sa peci len z meshov (nie z gprimov ako Cube) a volume treba pred pecenim
# zaregistrovat par aktualizaciami. Kolizia postavy z IRA je len trigger: robot cez
# chodca fyzicky prejde, lidar ho vidi.
import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.experimental.utils import app as app_utils
from pxr import Gf, Sdf, UsdGeom

EXTENSIONS = ('omni.anim.navigation.core', 'isaacsim.replicator.agent.core')
CHARACTER = 'Isaac/People/Characters/M_Medical_01/M_Medical_01.usd'


def enable_extensions():
    for ext in EXTENSIONS:
        app_utils.enable_extension(ext)


def bake_navmesh(stage, half_size, height=3.0):
    """Navmesh nad stvorcom 2*half_size okolo pociatku, podlaha je na z=0."""
    import omni.anim.navigation.core as nav
    omni.kit.commands.execute('CreateNavMeshVolumeCommand', parent_prim_path=Sdf.Path('/World'),
                              volume_type=nav.NAVMESH_VOLUME_INCLUDE)
    volume = stage.GetPrimAtPath('/World/NavMeshVolume')
    UsdGeom.Xformable(volume).ClearXformOpOrder()
    UsdGeom.Boundable(volume).GetExtentAttr().Set(
        [Gf.Vec3f(-half_size, -half_size, -0.5), Gf.Vec3f(half_size, half_size, height)])
    inav = nav.acquire_interface()
    app = omni.kit.app.get_app()
    for _ in range(5):
        app.update()
    if not inav.start_navmesh_baking():
        raise RuntimeError('navmesh baking did not start (floor must be a mesh, not a gprim)')
    for _ in range(300):
        app.update()
        if inav.get_navmesh() is not None and not inav.is_navmesh_baking():
            break
    if inav.get_navmesh() is None:
        raise RuntimeError('navmesh baking failed')
    return volume.GetPath()


async def _load_person(stage, points, speed):
    from isaacsim.replicator.agent.core.configuration.models.character import (CharacterConfig,
                                                                                IRACharacterGroup)
    from isaacsim.replicator.agent.core.configuration.models.defs.behaviors import (
        AnyCharacterBehavior, CharacterPatrolBehavior)
    from isaacsim.replicator.agent.core.randomizer import Randomizer
    from isaacsim.replicator.agent.core.scene_assembly.character_loader import CharacterLoader
    from isaacsim.storage.native import get_assets_root_path

    url = f'{get_assets_root_path()}/{CHARACTER}'

    class OneCharacterLoader(CharacterLoader):
        def _read_assets_from_directory(self, directory_url, file_extensions=None):
            return [url]

    path = [[float(x), float(y), 0.0] for x, y in points]
    group = IRACharacterGroup(
        num=1, asset_path=url.rsplit('/', 2)[0] + '/', spawn_positions=[tuple(path[0])],
        routines=[AnyCharacterBehavior(value=CharacterPatrolBehavior(speed_range=[speed, speed],
                                                                     path_points=path))])
    config = CharacterConfig(groups={'person': group})
    return await OneCharacterLoader(config, Randomizer(0), stage).load()


def add_person(simulation_app, stage, points, speed):
    """Prida chodca, ktory chodi dokola po bodoch [(x, y), ...] rychlostou speed [m/s]."""
    from omni.kit.async_engine import run_coroutine
    task = run_coroutine(_load_person(stage, points, speed))
    while not task.done():
        simulation_app.update()
    if task.result() != 1:
        raise RuntimeError(f'person loading failed ({task.result()} characters loaded)')
    print(f'[robocup_people] person walking {points} at {speed} m/s', flush=True)
