# navarch-568-proj

## Setup
1. Install MATLAB.
2. Clone this repository.
```
git clone git@github.com:audrow/navarch-568-proj.git WORKSPACE_DIR
```
3. Download the dataset from the [KITTI website](http://www.cvlibs.net/datasets/kitti/eval_odometry.php). We need the grayscale images an the ground truth poses to be put under the following folder structure:
```
WORKSPACE_DIR/dataset
├── poses/**.txt
└── sequences
    ├── 00
    │   ├── image_0/00****.png
    │   ├── image_1/00****.png
    │   ├── calib.txt
    │   └── times.txt
    ├── ...
    └── 21
        ├── image_0/00****.png
        ├── image_1/00****.png
        ├── calib.txt
        └── times.txt
```
4. Run `calc_SURF_features.m` to calculate SURF features of the left images for all the sequences.

## Demo code
- `demo_load_gt.m`: Load and visualize sequence `00` to `10` on the x-z plane.
