from pxr import Plug

registry = Plug.Registry()

plugin_name = "stepTessellation"
plugin = registry.GetPluginWithName(plugin_name)

if plugin:
    print(f"Plugin '{plugin_name}' discovered!")
    print(f"Path: {plugin.path}")
    
    # Check if it's currently loaded into memory
    if not plugin.isLoaded:
        print("⏳ Plugin found but not loaded. Attempting to load...")
        plugin.Load()
        
    print(f" Plugin Is Loaded: {plugin.isLoaded}")
else:
    print(f" Plugin '{plugin_name}' not found. Check your PXR_PLUGINPATH.")

from pxr import CadTessellation
