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
    
