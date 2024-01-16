# Docker Configuration

Docker is a powerful tool for managing and deploying applications in a consistent and reproducible manner, and it is also increasingly being used in robotic applications. Docker containers are designed to be reproducible, meaning that the same container image will produce the same results regardless of where it is run. This is important in robotic applications where the behavior of the robot needs to be consistent across different deployments.


First cd to the docker directory
```sh
cd docker/<distro>
```
Then build the image using docker file
```sh
docker build -t ros-motion-planning:<distro> --no-cache -f ./Dockerfile ../../
```
Finally run the container
```sh
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name=ros-motion-planning-<distro> ros-motion-planning:<distro> /bin/bash
```
Here \<distro> refers to the specific ROS distributions, i.e., `kinetic`, `melodic`, `noetic`.

