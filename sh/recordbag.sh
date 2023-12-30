#!/bin/bash

rosbag record -a -O ~/sh/bag/$(date +"%Y-%m-%d_%H-%M-%S").bag
