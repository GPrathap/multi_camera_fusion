#!/bin/bash
$(pwd)/docker/scripts/dev_start_inno.sh
echo "Starting bootstrap ..."
docker exec -u race apollo_dev /apollo/scripts/bootstrap.sh
