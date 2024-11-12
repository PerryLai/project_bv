First, have to install carla on PC
carla website: https://carla.readthedocs.io/en/latest/


Generate Data
---
1. Open Carla
/opt/carla-simulator/CarlaUE4.sh -RenderOffScreen
2. Load Map
python /opt/carla-simulator/PythonAPI/util/config.py --map Town05
3. Generate experiment scenes
cd ~/Projects/carla
python main_network_SSMCO.py 
### something it will crash, just restart it. change the for loop index to continue the folder indexing.
4. Merge view from 4 different camera
python mergeply_ego.py
This step is to merge 4 point cloud from 4 cameras into one point cloud. Also, add self point cloud model into point cloud because the camera can know scan self car body.

Test the ROI point cloud registration algorithm
---
1. Algorithm Exp
python algorithm_exp.py
The file is the main exp code of my ROI point cloud registration algorithm.
Also can try python algorithm_exp_network.py for multiple car scenario.

Test the Source Selection Algorithm
---
dpSSMCO_objectdetection.py for dynamic programming version
greddySSMCO_objectdetection.py for greedy method
