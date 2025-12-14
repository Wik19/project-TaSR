FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY src project-tasr/src

RUN apt-get update && rosdep update && rosdep install -i --from-paths project-tasr/src --rosdistro humble -y
RUN cd project-tasr && source /opt/ros/humble/setup.bash && colcon build

COPY entrypoint.sh / 

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "bash" ]