bl_info = {
    "name": "Export MJCF",
    "description": "Tool to define MJCF properties and export",
    "author": "Jay Jasper",
    "version": (0, 5),
    "blender": (4, 2, 0),
    "location": "View3D > Export MJCF Panel",
    "warning": "",
    "category": "Import-Export"
}

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
    
