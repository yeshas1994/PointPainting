# Point Painting

Point Painting/Costing based on semantic-segmentation of RGB Images

The test_files folder contains standalone C++ code to try while the ros folder contains the catkin environment + separate nodes for torchscript and the pointpainting

## TODOs
- Create colormap yaml file to match FCN output to color and vice-versa
- Add inference (onnx/trt) (Added torchscript)
- Create ways to better extract pointcloud "classes"
