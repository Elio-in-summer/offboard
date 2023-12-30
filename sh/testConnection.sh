#!/bin/bash

# 用于检测mavros是否连接到飞控，以及local_position话题是否有数据
timeout 2 rostopic echo /mavros/state | grep connected

rostopic echo /mavros/local_position/pose | grep -e '---' -e 'pose' -e 'x' -e 'y' -e 'z' -e 'w'

