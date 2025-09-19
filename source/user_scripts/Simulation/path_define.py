# asset_paths.py
import os
from pathlib import Path, PurePosixPath

BASE = os.environ.get("OMNI_ASSETS").rstrip("/")

def _is_omniverse(uri: str) -> bool:
    return uri.startswith("omniverse://")

def apath(*parts: str) -> str:
    """
    Join path segments under a base that can be either:
    - local filesystem (/mnt/omniverse_assets, Z:\OmniverseAssets, etc.)
    - Omniverse URI (omniverse://host/Share)
    Always returns a string suitable for Isaac/Omniverse APIs.
    """
    if _is_omniverse(BASE):
        p = PurePosixPath(BASE)
        for part in parts:
            # Normalize separators so join is predictable
            p = p / str(part).replace("\\", "/").lstrip("/")
        return str(p)
    else:
        return str(Path(BASE, *parts))

# Your project paths (all relative to BASE)
CFG_FILE           = apath("bindings.yaml")
STAGE_PATH_OGMAR         = apath("Ogmar80","odm_texturing", "odm_textured_model_geo.usd")
STAGE_PATH_GAN_SHOMRON         = apath("Gan_Shomron","odm_texturing", "odm_textured_model_geo.usd")
CAR_ORANGE_ASSET_PATH     = apath("car", "car_v7.usd")
CAR_BLACK_ASSET_PATH     = apath("car", "car_v9_rough.usd")
TEXTURE_SKY        = apath("sky_chat.png")
CESIUM_TRANSFORM   = apath("ogmar", "conf.JSON")
RCS_FILE_PATH      = apath("radar", "radar_rcs_maps", "rcs_ford_raptor_1.pkl")
RADAR_PROP_PATH    = apath("radar", "MAGOS.yaml")

def validate_local_paths():
    """
    Optional: quick sanity check when using local filesystem.
    Skips validation if BASE is an omniverse:// URI.
    """
    if _is_omniverse(BASE):
        return []
    import os
    candidates = [
        CFG_FILE, STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY,
        CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH
    ]
    missing = [p for p in candidates if not os.path.exists(p)]
    return missing
