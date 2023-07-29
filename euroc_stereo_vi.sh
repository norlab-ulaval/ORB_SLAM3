#!/bin/bash
pathDatasetTUM='/home/user/data/MH01'

echo "Launching EuRoc MH_01_easy with Stereo-Inertial sensor"
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml $pathDatasetTUM ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
