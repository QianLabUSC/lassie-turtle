#!/bin/bash
#
# ----- run the container, with dynamic port assignment
# ----- container options are set through environment variables
# ----- NEWUSER	   : username to create in the container
# ----- NEWUID     : uid for the new user
# ----- NEWGID     : gid for the new user
# ----- RESOLUTION : screen resolution inside the container
# ----- This also maps the users home directory into the container
# ----- as the "/host" mount point
#
# ----- to view the possible resolutions, exec into the running container
# ---- and look at the /etc/X11/xorg.conf file
#
CONTAINER=traveler:v1
script_dir=$(dirname "$(realpath "$0")")
parent_dir=$(dirname "$script_dir")
echo "load thhe folder highlevel in  $parent_dir to docker ..."
sudo docker run -it -e NEWUID=$(id -u) -e DISPLAY=$DISPLAY	\
        -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
	-v $parent_dir/src/:/home/traveler/src/ \
	-v $parent_dir/highlevel/:/home/traveler/highlevel/ \
	-v $parent_dir/install/:/home/traveler/install/ \
	-v /dev:/dev --privileged \
	--ipc host --network=host --name=traveler_container  $CONTAINER $*
