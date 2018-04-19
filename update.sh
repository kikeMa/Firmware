#!/bin/sh


echo 'Actualizo Firmware'
git pull origin master

echo 'Actualizo gazebo'
cd Tools/sitl_gazebo
git pull origin master

echo 'Actualizo mavlink'
cd ../../mavlink/include/mavlink/v2.0/
git pull origin master


cd ../../../../
