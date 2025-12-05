#!/usr/bin/env bash
set -e

# To build sim and docs, you can run the following to mount the scons cache to the same place as in CI:
# mkdir -p .ci_cache/scons_cache
# sudo mount --bind /tmp/scons_cache/ .ci_cache/scons_cache

SCRIPT_DIR=$(dirname "$0")
OPENPILOT_DIR=$SCRIPT_DIR/../../
if [ -n "$TARGET_ARCHITECTURE" ]; then
  PLATFORM="linux/$TARGET_ARCHITECTURE"
  TAG_SUFFIX="-$TARGET_ARCHITECTURE"
else
  PLATFORM="linux/$(uname -m)"
  TAG_SUFFIX=""
fi

source $SCRIPT_DIR/docker_common_sp.sh $1 "$TAG_SUFFIX"

# Add timeout and reduce cache operations to prevent hanging
# For base image builds, we'll only use local cache and avoid registry cache operations that can hang
if [ "$1" = "base" ]; then
  # For base image builds, use only local cache to avoid hanging
  # Add timeout to prevent indefinite hanging
  timeout 600 DOCKER_BUILDKIT=1 docker buildx build --provenance false --platform $PLATFORM --load --cache-to type=inline,mode=min --cache-from type=local,src=.ci_cache/buildx_cache --cache-to type=local,dest=.ci_cache/buildx_cache -t $DOCKER_IMAGE:latest -t $REMOTE_TAG -t $LOCAL_TAG -f $OPENPILOT_DIR/$DOCKER_FILE $OPENPILOT_DIR || {
    echo "Base build timed out or failed, retrying without cache operations..."
    DOCKER_BUILDKIT=1 docker buildx build --provenance false --platform $PLATFORM --load --no-cache -t $DOCKER_IMAGE:latest -t $REMOTE_TAG -t $LOCAL_TAG -f $OPENPILOT_DIR/$DOCKER_FILE $OPENPILOT_DIR
  }
else
  # For other builds, keep the original cache operations but with timeout
  timeout 900 DOCKER_BUILDKIT=1 docker buildx build --provenance false --pull --platform $PLATFORM --load --cache-to type=inline --cache-from type=registry,ref=$REMOTE_TAG -t $DOCKER_IMAGE:latest -t $REMOTE_TAG -t $LOCAL_TAG -f $OPENPILOT_DIR/$DOCKER_FILE $OPENPILOT_DIR || {
    echo "Build timed out or failed, retrying without cache operations..."
    DOCKER_BUILDKIT=1 docker buildx build --provenance false --pull --platform $PLATFORM --load --no-cache -t $DOCKER_IMAGE:latest -t $REMOTE_TAG -t $LOCAL_TAG -f $OPENPILOT_DIR/$DOCKER_FILE $OPENPILOT_DIR
  }
fi

if [ -n "$PUSH_IMAGE" ]; then
  docker push $REMOTE_TAG
  docker tag $REMOTE_TAG $REMOTE_SHA_TAG
  docker push $REMOTE_SHA_TAG
fi
