#!/usr/bin/env python3
"""
Test script to verify CUDA error handling functions are available in Python.
"""

import sys
import os

# Add the current directory to the path so we can import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pychrono.core as chrono
    import pychrono.fsi as fsi
    
    print("Testing CUDA error handling functions...")
    
    # Test if the functions are available
    print(f"Has CheckCudaError: {hasattr(fsi, 'CheckCudaError')}")
    print(f"Has GetCudaErrorString: {hasattr(fsi, 'GetCudaErrorString')}")
    print(f"Has ResetCudaError: {hasattr(fsi, 'ResetCudaError')}")
    
    if hasattr(fsi, 'CheckCudaError'):
        print("Testing CheckCudaError...")
        error_occurred = fsi.CheckCudaError()
        print(f"CheckCudaError() returned: {error_occurred}")
        
        if hasattr(fsi, 'GetCudaErrorString'):
            print("Testing GetCudaErrorString...")
            error_msg = fsi.GetCudaErrorString()
            print(f"GetCudaErrorString() returned: '{error_msg}'")
        
        if hasattr(fsi, 'ResetCudaError'):
            print("Testing ResetCudaError...")
            fsi.ResetCudaError()
            print("ResetCudaError() completed successfully")
            
            # Test again after reset
            error_occurred_after = fsi.CheckCudaError()
            print(f"CheckCudaError() after reset returned: {error_occurred_after}")
    
    print("All tests completed successfully!")
    
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure pychrono.fsi is properly built and installed")
except Exception as e:
    print(f"Error during testing: {e}")
    import traceback
    traceback.print_exc()
