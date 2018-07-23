#!/bin/bash
python src/laser_calibration.py -d images/laser_calibration/180717 -f jpg -c calibrations/180717cameraIntrinsics.npy -D calibrations/180717cameraDistCoeff.npy -o calibrations/180717
