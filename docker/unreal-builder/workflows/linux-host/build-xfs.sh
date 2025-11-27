#!/bin/bash
#
# Convenient wrapper for building Xfs project
# Calls the actual build script in scripts/examples/
#
# Usage:
#   ./build-xfs.sh                    # Build Xfs project
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Call the actual build script
exec "${SCRIPT_DIR}/scripts/examples/build-xfs-example.sh" "$@"
