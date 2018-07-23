#!/bin/bash
python src/offline_reconstruction.py -d images/dataset/180717 -f jpg -c calibrations/180717cameraIntrinsics.npy -D calibrations/180717cameraDistCoeff.npy -l calibrations/180717laserCalibration.npy
