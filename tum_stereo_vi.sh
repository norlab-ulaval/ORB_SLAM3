#!/bin/bash
pathDatasetTUM='/home/user/data/dataset-corridor4_512'

echo "Launching TUM dataset-corridor-4 with Stereo-Inertial sensor"
./Examples/Stereo-Inertial/stereo_inertial_tum_vi ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/TUM-VI.yaml $pathDatasetTUM/mav0/cam0/data $pathDatasetTUM/mav0/cam1/data ./Examples/Stereo-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt ./Examples/Stereo-Inertial/TUM_IMU/dataset-corridor4_512.txt dataset-corridor4_512_stereoi
