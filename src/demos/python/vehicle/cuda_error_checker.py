"""
Simple CUDA error checker for Chrono FSI SPH simulations.

This module provides a simple way to check for CUDA errors that occur during
SPH simulations without causing the Python process to crash.
"""

import pychrono.core as chrono
import pychrono.fsi as fsi

def check_cuda_error():
    """
    Check if a CUDA error has occurred in the SPH system.
    
    Returns:
        tuple: (error_occurred: bool, error_message: str)
    """
    try:
        # Use the SWIG-wrapped functions
        error_occurred = fsi.CheckCudaError()
        error_message = fsi.GetCudaErrorString() if error_occurred else ""
        return error_occurred, error_message
    except Exception as e:
        return True, f"Error checking CUDA status: {e}"

def clear_cuda_error():
    """
    Clear any CUDA error state.
    
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Use the SWIG-wrapped function
        fsi.ResetCudaError()
        return True
    except Exception as e:
        print(f"Error clearing CUDA state: {e}")
        return False

def safe_advance(terrain, step_size):
    """
    Safely advance the terrain simulation by one step, checking for CUDA errors.
    
    Args:
        terrain: The CRM terrain object
        step_size: The time step size
        
    Returns:
        tuple: (success: bool, error_message: str)
    """
    try:
        # Clear any previous errors
        clear_cuda_error()
        
        # Advance the simulation
        terrain.Advance(step_size)
        
        # Check for errors after the step
        error_occurred, error_message = check_cuda_error()
        
        if error_occurred:
            return False, error_message
        else:
            return True, ""
            
    except Exception as e:
        return False, f"Exception during terrain advance: {e}"

def safe_synchronize(vehicle, time, driver_inputs, terrain):
    """
    Safely synchronize the vehicle with the terrain.
    
    Args:
        vehicle: The vehicle object
        time: Current simulation time
        driver_inputs: Driver input object
        terrain: The CRM terrain object
        
    Returns:
        tuple: (success: bool, error_message: str)
    """
    try:
        # Clear any previous errors
        clear_cuda_error()
        
        # Synchronize the vehicle
        vehicle.Synchronize(time, driver_inputs, terrain)
        
        # Check for errors after synchronization
        error_occurred, error_message = check_cuda_error()
        
        if error_occurred:
            return False, error_message
        else:
            return True, ""
            
    except Exception as e:
        return False, f"Exception during vehicle sync: {e}"
