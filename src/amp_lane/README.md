## Lane Detection

This is the package for AMP lane detection. At the moment, it directly publishes to `cmd_ackermann`, which controls the simulated kart. It reads from `vehicle/camera`, which is the forward facing RGBA camera mounted on the simulaated kart. 

Within this package are some lane detection files I was playing around with. 

- This fast [lane detection model](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2/tree/master). To get it to work, you have to install [Torch](https://pytorch.org/get-started/locally/) and then download the Resnet-18 CULanes model from the model repo. Then run [infer.py](amp_lane/ultra_fast_lane/infer.py)
- This MobileSAM model. To get it to work, you have to install Torch and install the model from the [repo](https://github.com/ChaoningZhang/MobileSAM). Then install it using `pip install git+https://github.com/ChaoningZhang/MobileSAM.git`. Then run [infer.py](amp_lane/nanosam/infer.py). This is the model currently used in the lane_detection module.