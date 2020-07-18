# Point Painting

Point Painting/Costing based on semantic-segmentation of RGB Images

The test_files folder contains standalone C++ code to try while the ros folder contains the catkin environment + separate nodes for torchscript and the pointpainting

Currently the ros package can read from topics/videos on ros while the standalone package relies on files (.pcd/.png/etc)

## TODOs
- Create colormap yaml file to match FCN output to color and vice-versa
- Add inference (onnx/trt) (Added torchscript)
- Create ways to better extract pointcloud "classes"
