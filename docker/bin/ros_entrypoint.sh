#!/bin/bash
set -e

# setup ros environment for the container
source "/app/devel/setup.bash"
exec "$@"
