FROM ubuntu:xenial

MAINTAINER Andrew Murtagh, Patrick Lynch and Conor McGinn

WORKDIR /home/

COPY . .

RUN apt-get update && apt-get -y install build-essential cmake git wget libboost-all-dev festival-dev festival

RUN ./scripts/installs/mrpt.sh

RUN ./scripts/installs/sphinxbase.sh

RUN ./scripts/installs/pocketsphinx.sh

RUN ./scripts/installs/opencv.sh

RUN ./scripts/installs/gmapping.sh

RUN ./scripts/installs/pcl.sh

RUN ./scripts/installs/caffe.sh

# git-lfs pull and re-train FacialRecognition

RUN cd scripts && ./build_project.sh

# ENTRYPOINT cd scripts && ./get_system_details.sh && ./run_benchmark.sh
