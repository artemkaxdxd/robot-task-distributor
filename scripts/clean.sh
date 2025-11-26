#!/bin/bash

# Clean build artifacts

set -e

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

echo "Cleaning build artifacts..."

cd "$WORKSPACE_DIR"

rm -rf build/ install/ log/

echo "Clean completed!"