#!/usr/bin/env python3
"""
PlatformIO pre-build script to enable ccache for faster compilation
"""

Import("env")
import os

# Set ccache environment variables
env['ENV']['CCACHE_DIR'] = os.path.expanduser('~/Library/Caches/ccache')
env['ENV']['CCACHE_TEMPDIR'] = '/tmp'
env['ENV']['CCACHE_COMPRESS'] = '1'
env['ENV']['CCACHE_MAXSIZE'] = '2G'

# Configure ccache for the build tools
ccache_path = "/usr/local/bin/ccache"

# Check if ccache exists
if os.path.exists(ccache_path):
    print("✅ Configuring ccache for faster builds...")

    # Get the current toolchain prefix
    cc = env.get("CC")
    cxx = env.get("CXX")

    if cc and not str(cc).startswith(ccache_path):
        env.Replace(CC=[ccache_path] + (cc if isinstance(cc, list) else [cc]))
    if cxx and not str(cxx).startswith(ccache_path):
        env.Replace(CXX=[ccache_path] + (cxx if isinstance(cxx, list) else [cxx]))

    print(f"  CC:  {env.get('CC')}")
    print(f"  CXX: {env.get('CXX')}")

    # Also try setting via environment for ESP-IDF toolchain
    original_cc = env.get("CC")
    original_cxx = env.get("CXX")
    if original_cc:
        env['ENV']['CC'] = f"{ccache_path} {' '.join(original_cc) if isinstance(original_cc, list) else original_cc}"
    if original_cxx:
        env['ENV']['CXX'] = f"{ccache_path} {' '.join(original_cxx) if isinstance(original_cxx, list) else original_cxx}"
else:
    print("⚠️  ccache not found, builds will not be cached")