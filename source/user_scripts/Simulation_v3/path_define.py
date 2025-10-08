# asset_paths.py
import os
from pathlib import Path, PurePosixPath

BASE = os.environ.get("OMNI_ASSET_LOCAL").rstrip("/")

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
CFG_FILE           = apath("bindings_tgl.yaml")
STAGE_PATH_OGMAR         = apath("Ogmar80","odm_texturing", "odm_textured_model_geo.usd")
STAGE_PATH_GAN_SHOMRON         = apath("Gan_Shomron","odm_texturing", "odm_textured_model_geo.usd")
STAGE_PATH_TEL_KUDNA         = apath("Tel-Kudna-North-all","odm_texturing", "odm_textured_model_geo.usd")
STAGE_PATH_LATRUN_4         = apath("latrun","latrun_1_3000-all","odm_texturing", "cropped_1_3000.usd")
STAGE_PATH_LATRUN_3       = apath("latrun","Copy-of-latrun_3001_5500-all", "odm_texturing", "odm_textured_model_geo.usd")
STAGE_PATH_LATRUN_2       = apath("latrun","latrun_5000_7999-all","odm_texturing", "cropped_5000_7999.usd")
STAGE_PATH_LATRUN_1       = apath("latrun","latrun_7500_10300","odm_texturing", "odm_textured_model_geo.usd")
STAGE_PATH_LATRUN_5    = apath("latrun"," latrun_last_scan_east", "odm_texturing","odm_textured_model_geo.usd")

CAR_ORANGE_ASSET_PATH     = apath("assets","car", "car_v7.usd")
CAR_BLACK_ASSET_PATH     = apath("assets","car", "car_v9_rough.usd")
WHITE_TOYOTA     = apath("assets","White_Pickup_Truck_0920164041_texture_obj", "white_toyota.usd")
TANK     = apath("assets","tank", "tank.usd")
MEHOLA     = apath("assets","mehola","clean_cargo","mehola2.usd")
HAMAS = apath("assets","soldiers","hamas.usd")
RADAR = apath("assets","radar","radar_cam.usd")
KELA = apath("assets","kelian","kelian.usd")

TEXTURE_SKY        = apath("assets","sky","sky_4.png")
CESIUM_TRANSFORM   = apath("ogmar", "conf.JSON")
RCS_FILE_PATH      = apath("radar", "radar_rcs_maps", "rcs_ford_raptor_1.pkl")
RADAR_PROP_PATH    = apath("radar", "MAGOS.yaml")
COORDINATES_GS    = apath("Gan_Shomron", "odm_georeferencing","coords.txt")
COORDINATES_TK    = apath("Tel-Kudna-North-all", "odm_georeferencing","coords.txt")
COORDINATES_LATRUN_4    = apath("latrun","latrun_1_3000-all", "odm_georeferencing","coords.txt")
COORDINATES_LATRUN_3    = apath("latrun","Copy-of-latrun_3001_5500-all", "odm_georeferencing","coords.txt")
COORDINATES_LATRUN_2    = apath("latrun","latrun_5000_7999-all", "odm_georeferencing","coords.txt")
COORDINATES_LATRUN_1    = apath("latrun","latrun_7500_10300", "odm_georeferencing","coords.txt")
COORDINATES_LATRUN_5    = apath("latrun"," latrun_last_scan_east", "odm_georeferencing","coords.txt")

GEOJSON_GS    = apath("gs_coordinates")
GEOJSON_TK    = apath("tk_coordinates")
GEOJSON_LATRUN    = apath("latrun_coordinates")


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
