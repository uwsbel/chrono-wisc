#!/usr/bin/env python3
import sys
import os
import subprocess

def main():
    # Get the path to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Path to blender executable (edit this path as needed)
    blender_executable = os.environ.get('BLENDER_PATH', '/home/huzaifa/Downloads/blender-4.0.0-linux-x64/blender')
    
    # Ensure blender_executable exists
    if not os.path.exists(blender_executable):
        print(f"ERROR: Blender executable not found at {blender_executable}")
        print("Please set the BLENDER_PATH environment variable to the correct path")
        return 1
    
    # Get arguments minus the script name
    args = sys.argv[1:]
    
    # Set environment variables for headless rendering
    my_env = os.environ.copy()
    
    # Set PYTHONPATH to include current directory and blender path
    my_env["PYTHONPATH"] = os.pathsep.join([
        script_dir,                    # Current directory with chrono_import.py
        blender_executable,            # Blender executable for headless mode
        my_env.get("PYTHONPATH", "")   # Existing PYTHONPATH
    ])
    
    print(f"Using PYTHONPATH: {my_env['PYTHONPATH']}")
    
    my_env["BLENDER_SKIP_PYTHON_PATH_CLEANUP"] = "1"
    
    # Force Mesa/NVIDIA to use offscreen rendering
    my_env["DISPLAY"] = ""
    my_env["PYOPENGL_PLATFORM"] = "egl"  # Use EGL instead of GLX
    
    # For NVIDIA GPUs on headless systems
    my_env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    my_env["CUDA_VISIBLE_DEVICES"] = os.environ.get("CUDA_VISIBLE_DEVICES", "0")
    
    # Build the command
    cmd = [
        blender_executable,
        "--background",  # Run in background (no UI)
        "--factory-startup",  # Start with default settings
        "--python", os.path.join(script_dir, "chrono_blender_render.py"),
        "--",
    ] + args
    
    print("Running command:", " ".join(cmd))
    
    # Run the process
    try:
        result = subprocess.run(cmd, env=my_env, check=True)
        return result.returncode
    except subprocess.CalledProcessError as e:
        print(f"Error running Blender: {e}")
        return e.returncode
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 