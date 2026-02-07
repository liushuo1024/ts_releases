#!/bin/bash

# 容器名称
CONTAINER_NAME="amr-robot-driver-dev"
IMAGE_NAME="ros-noetic-gazebo-amr"

# 获取当前目录名
CURRENT_DIR=$(basename $(pwd))

# 设置 X11 权限，允许本地连接
xhost +local:docker > /dev/null 2>&1
xhost +local:root > /dev/null 2>&1

# 获取 XAUTHORITY 路径（如果存在）
XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"

# 确保 DISPLAY 变量已设置
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

# 进入容器后执行的命令
if [ -f devel_isolated/setup.bash ]; then
    ENTRY_CMD="cd /home/step/${CURRENT_DIR} && source devel_isolated/setup.bash && exec bash"
else
    ENTRY_CMD="cd /home/step/${CURRENT_DIR} && exec bash"
fi

# 检查容器是否存在
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "容器 ${CONTAINER_NAME} 已存在"
    
    # 检查容器是否正在运行
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "容器正在运行,直接进入..."
        docker exec -it \
          -e DISPLAY=${DISPLAY} \
          -e QT_X11_NO_MITSHM=1 \
          -e LIBGL_ALWAYS_SOFTWARE=1 \
          ${CONTAINER_NAME} /bin/bash -c "${ENTRY_CMD}"
    else
        echo "容器已停止,启动中..."
        docker start ${CONTAINER_NAME}
        docker exec -it \
          -e DISPLAY=${DISPLAY} \
          -e QT_X11_NO_MITSHM=1 \
          -e LIBGL_ALWAYS_SOFTWARE=1 \
          ${CONTAINER_NAME} /bin/bash -c "${ENTRY_CMD}"
    fi
else
    echo "容器 ${CONTAINER_NAME} 不存在,创建新容器..."
    
    # 构建 X11 相关的挂载和环境变量
    X11_ARGS=(
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw
      -e DISPLAY=${DISPLAY}
      -e QT_X11_NO_MITSHM=1
      -e LIBGL_ALWAYS_SOFTWARE=1
      -e XAUTHORITY=${XAUTH_FILE}
      -e QT_GRAPHICSSYSTEM=native
      -e _X11_NO_MITSHM=1
      -e _MESA_NO_ERROR=1
    )
    
    # 如果 XAUTHORITY 文件存在，则挂载它
    if [ -f "${XAUTH_FILE}" ]; then
      X11_ARGS+=(-v ${XAUTH_FILE}:${XAUTH_FILE}:ro)
    fi
    
    docker run -it \
      "${X11_ARGS[@]}" \
      --device=/dev/dri:/dev/dri \
      --ipc=host \
      -v $(pwd):/home/step/${CURRENT_DIR} \
      -v /opt/step_robot:/opt/step_robot \
      --network host \
      --name ${CONTAINER_NAME} \
      ${IMAGE_NAME} \
      /bin/bash -c "${ENTRY_CMD}"
fi
