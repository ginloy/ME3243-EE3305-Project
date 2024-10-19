FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

RUN apt update -y && apt upgrade -y

RUN apt install -y \
  software-properties-common \
  ros-dev-tools \
  ros-humble-turtlebot3* \
  ros-humble-rmw-cyclonedds-cpp \
  curl

# install vscode-server
RUN curl -fsSL https://code-server.dev/install.sh | sh
# RUN echo "code code/add-microsoft-repo boolean true" | debconf-set-selections
# RUN curl -O -J -L https://go.microsoft.com/fwlink/?LinkID=760868 && \
#     apt install -y ./code_1.94.2-1728494015_amd64.deb 

RUN apt install -y glmark2 \
  fzf \
  zsh

  
# RUN apt install -y \
#   kitty

RUN groupadd -g 1000 abc
RUN useradd -u 1000 -g 1000 -m abc
RUN echo -e abc:1324 | chpasswd

RUN adduser abc sudo

USER abc
SHELL ["/usr/bin/zsh", "-c"]


EXPOSE 13337

# CMD code --no-sandbox; sleep infinity 
CMD code-server --auth none --bind-addr 0.0.0.0:13337
