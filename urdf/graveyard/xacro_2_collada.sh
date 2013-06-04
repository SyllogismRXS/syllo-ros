#!/bin/bash

# arguments: $1 = input.xacro file
#            

if [ $# -ne 1 ]
then
    echo 
    echo "Invalid input arguments"
    echo "usage: ./xacro_2_collada.sh <input-no-ext>"
    exit
fi

FILE=$1

echo $FILE

NAME=${FILE%.urdf.xacro}

echo $NAME

NAME=${NAME##*/}

echo $NAME

#rosrun xacro xacro.py "${FILE}" > "${NAME}.urdf"
#rosrun collada_urdf urdf_to_collada "${NAME}.urdf" "${NAME}.dae"
