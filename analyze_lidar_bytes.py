#!/usr/bin/env python3
"""
Analyze the byte pattern from LiDAR data to understand what values we're getting
"""

import struct

# The repeating pattern from your data
byte_pattern = [
    0, 0, 0, 0,      # First 4 bytes (X)
    0, 0, 0, 128,    # Next 4 bytes (Y)
    0, 0, 0, 128     # Next 4 bytes (Z)
]

print("Analyzing LiDAR byte pattern...")
print("="*50)

# Interpret as floats
for i in range(0, len(byte_pattern), 4):
    bytes_chunk = bytes(byte_pattern[i:i+4])
    try:
        # Little-endian float (standard for x86/x64)
        float_val = struct.unpack('<f', bytes_chunk)[0]
        
        # Check for special values
        if float_val == 0.0:
            special = " (zero)"
        elif float_val == float('inf'):
            special = " (infinity)"
        elif float_val == float('-inf'):
            special = " (negative infinity)"
        elif float_val != float_val:  # NaN check
            special = " (NaN)"
        elif abs(float_val) < 1e-30:
            special = " (essentially zero)"
        else:
            special = ""
            
        coord = ['X', 'Y', 'Z'][i//4]
        print(f"{coord}: bytes={list(bytes_chunk)} -> float={float_val:.2e}{special}")
        
        # Also show as hex for debugging
        hex_val = ''.join(f'{b:02x}' for b in bytes_chunk)
        print(f"   Hex: 0x{hex_val}")
        
    except Exception as e:
        print(f"Error interpreting bytes {i}-{i+3}: {e}")

print("\n" + "="*50)
print("Interpretation:")
print("The pattern [0,0,0,0, 0,0,0,128, 0,0,0,128] represents:")
print("X=0.0, Y≈1.18e-38, Z≈1.18e-38")
print("\nThis suggests the LiDAR is returning essentially zero values")
print("but with tiny non-zero Y and Z coordinates.")
print("\nPossible causes:")
print("1. The Range parameter is missing (defaults to very small value)")
print("2. The sensor is underground (Z=-1 in settings)")
print("3. No objects within sensor range")
print("4. Coordinate transformation issue")