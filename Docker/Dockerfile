# Pull base image
# based on - https://larrylisky.com/2016/11/03/point-cloud-library-on-ubuntu-16-04-lts/
FROM ubuntu:16.04

# install prereqs
RUN apt-get update
RUN apt-get install -y software-properties-common
RUN apt-get update
RUN apt-get install -y git build-essential linux-libc-dev
RUN apt-get install -y cmake cmake-gui 
RUN apt-get install -y libusb-1.0-0-dev libusb-dev libudev-dev
RUN apt-get install -y mpi-default-dev openmpi-bin openmpi-common  
RUN apt-get install -y libflann1.8 libflann-dev
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y libboost-all-dev
RUN apt-get install -y libvtk5.10-qt4 libvtk5.10 libvtk5-dev
RUN apt-get install -y libqhull* libgtest-dev
RUN apt-get install -y freeglut3-dev pkg-config
RUN apt-get install -y libxmu-dev libxi-dev 
RUN apt-get install -y mono-complete
RUN apt-get install -y qt-sdk openjdk-8-jdk openjdk-8-jre
RUN apt-get install -y openssh-client

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all