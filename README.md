# navarch-568-proj

## Setup
1. Install MATLAB.
2. Clone this repository.
```
git clone git@github.com:audrow/navarch-568-proj.git WORKSPACE_DIR
```
3. Download the dataset from the [KITTI website](http://www.cvlibs.net/datasets/kitti/eval_odometry.php). We need the grayscale images to be put under the following folder structure:
```
WORKSPACE_DIR/dataset
    poses
        00.txt
        ...
        10.txt
    sequences
        00
            image_0/*.png
            image_1/*.png
            calib.txt
            times.txt
        ...
        21
            image_0/*.png
            image_1/*.png
            calib.txt
            times.txt
```
The validation (ground truth) poses for sequences `00` to `10` are already in `dataset/poses`.

## Demo code
- `demo_load_gt.m`: Load and visualize sequence `00` to `10` on the x-z plane.
