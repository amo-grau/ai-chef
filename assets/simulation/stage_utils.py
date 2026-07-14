
# All Omniverse imports must come after SimulationApp is instantiated.
import omni.usd
import isaacsim.core.experimental.utils.stage as stage_utils
from pxr import Gf, Usd, UsdGeom, UsdPhysics

def add_prop(
    prim_path: str,
    usd_path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float] = (0, 0, 0),
    dynamic: bool = False,
) -> None:
    """Reference a prop under an Xform we own, place it, and optionally make it
    a dynamic rigid body that falls under gravity and collides."""
    stage = omni.usd.get_context().get_stage()

    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*orientation))
    xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
    stage_utils.add_reference_to_stage(usd_path=usd_path, path=f"{prim_path}/Model")

    if dynamic:
        body = stage.GetPrimAtPath(prim_path)
        UsdPhysics.RigidBodyAPI.Apply(body)
        # Dynamic bodies require a convex collision approximation, not a trimesh.
        for mesh in Usd.PrimRange(body):
            if mesh.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(mesh)
                UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(
                    "convexHull"
                )
