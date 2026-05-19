# Use importlib's invalidate_caches() and reload() to aid development
import importlib
importlib.invalidate_caches()
import sys

if "export_mjcf" in locals():
    print("Reloading MJCF Export Extension")
    importlib.reload(export_mjcf)
    importlib.reload(stl_export)
else:
    from . import export_mjcf
    from . import stl_export

def register():
    stl_export.register()
    export_mjcf.register()

def unregister():
    stl_export.unregister()
    export_mjcf.unregister()

if __name__ == "__main__":
    register()
    
