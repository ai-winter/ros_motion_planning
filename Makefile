# Build docker project
.PHONY : docker_build
docker_build:
	docker build -t rmp .

# Run docker project
.PHONY : docker_run
docker_run:
	docker run -it --network host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --entrypoint /bin/bash rmp

# Create Doxygen
.PHONY : doxygen_create
doxygen_create:
	doxygen Doxyfile


.PHONY : doxygen_viz
doxygen_viz:
	firefox docs/html/index.html

