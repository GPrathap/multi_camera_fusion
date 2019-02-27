#!/bin/bash
$(pwd)/docker/scripts/dev_start_inno.sh
echo "Starting bootstrap ..."
<<<<<<< Updated upstream
docker exec -u race apollo_dev /apollo/scripts/bootstrap.sh
=======
#docker exec apollo_dev /apollo/scripts/bootstrap.sh
>>>>>>> Stashed changes
