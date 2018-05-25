
# point_cloud_noise_system

this is a example about  how to use QT based on PCL 1.7, 
make a GUI and do some point cloud noise processing 
 
# compile steps 

cd (your_project_name/qt_noise_cloud)

mkdir build

cd build

cmake ..

make
   
# the outcome of cmake 
                                                                         
![the outcome of cmake](https://github.com/yaoli1992/point_cloud_noise_system/blob/master/yao_data/Screenshot1.png)

# run the executable file

./noise_cloud_cmd

![the GUI of this project](https://github.com/yaoli1992/point_cloud_noise_system/blob/master/yao_data/Screenshot2.png)


# you can select the different point cloud noise process


**passthrough filter** 

![after passthrough filter](https://github.com/yaoli1992/point_cloud_noise_system/blob/master/yao_data/Screenshot3.png)

**voxel filter**

![after voxel filter](https://github.com/yaoli1992/point_cloud_noise_system/blob/master/yao_data/Screenshot4.png)

