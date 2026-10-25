#!/bin/bash
# Build and (optionally) push the simulation image.
#   bash docker/build.sh            # build only
#   bash docker/build.sh --push     # build and push to Docker Hub (needs `docker login`)
set -euo pipefail
cd "$(dirname "$0")/.."
IMAGE=${IMAGE:-abdulmannan617/wind-aware-delivery}
TAG=${TAG:-v2}
DATE=$(date +%Y-%m-%d)
# REPO_COMMIT changes with every pushed commit, so the clone of the repository is never taken from the cache
docker build -f docker/Dockerfile --build-arg REPO_COMMIT="$(git ls-remote https://github.com/abdul-mannan-khan/wind-aware-delivery.git main | cut -c1-12)" \
  -t "$IMAGE:$TAG" -t "$IMAGE:$DATE" .
if [ "${1:-}" = "--push" ]; then
  docker push "$IMAGE:$TAG"
  docker push "$IMAGE:$DATE"
fi
echo "built $IMAGE:$TAG (also :$DATE)"
