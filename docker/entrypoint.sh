#!/bin/bash
set -e

cd "$WORKSPACE"

if [ ! -f Vocabulary/ORBvoc.txt ]; then
  echo "Decompressing ORB vocabulary..."
  tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
fi

if [ ! -f lib/libORB_SLAM3.so ]; then
  echo "Building ORB-SLAM3 (first run)..."
  ./build.sh
fi

exec "$@"
