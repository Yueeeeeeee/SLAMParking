#!/bin/bash

# Removes all code related to the rgb lights from the main file

sed -i '/rgb/Id' RaspberryPI/src-gen/main.c

echo "removeRGB done"
