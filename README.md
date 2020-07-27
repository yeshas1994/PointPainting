# Point Painting

Point Painting/Costing based on semantic-segmentation of RGB Images

The test_files folder contains standalone C++ code to try while the ros folder contains the catkin environment 

This package uses torchscript to run inference. To install torchscript go to https://pytorch.org/ and download the LibTorch package that matches your CUDA version. Unzip the file and save it in the include folder. 
The bag file is https://drive.google.com/file/d/16In6eaFCvUVthKfZriaGEfNW3HIfXbvC/view?usp=sharing and place it in the assets folder.

The ros package should look like below:
```
point_painting/
├── assets
│   ├── end2end.bag
│   ├── fcn32_torchscript.pt
│   ├── rviz_pointpaint.rviz
├── CMakeLists.txt
├── config
│   ├── color_mapping.json
│   └── config.yaml
├── include
│   ├── libtorch
│   ├── nlohmann
│   ├── point_painting
│   ├── torch_segment
├── launch
├── node
├── src
```
The ros package contains two nodes:
- segmentation (only image segmentation)
```
roslaunch point_painting torch_segment.launch
```
- point_painting (which includes segmentation)
This node also saves the colored points in a vector as [x, y, z, class].
```
roslaunch point_painting point_painting.launch
```
Do remember to edit the `config.yaml` file as required 
- change the paths 
- change `use_cuda` as required


Currently the ros package can read from topics/videos on ros while the standalone package relies on files (.pcd/.png/etc)

## TODOs
- Add inference (onnx/trt) (Added torchscript)
- Update standalone 
