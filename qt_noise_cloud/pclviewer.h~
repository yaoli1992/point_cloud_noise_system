#ifndef PCLVIEWER_H
#define PCLVIEWER_H
//这是一个GUI 的头文件
// Qt
#include <QMainWindow>
#include <QFileDialog>

#include <QApplication>
#include <QFileDialog>
#include <QtCore>
#include <QKeyEvent>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

//filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/convolution_3d.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/kdtree/kdtree_flann.h>  
#include <pcl/filters/bilateral.h>  
      
      
#include <pcl/kdtree/flann.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/search/flann_search.h>  
#include <pcl/search/kdtree.h>  
// Boost
#include <boost/math/special_functions/round.hpp>
#include <iomanip>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <QApplication>
#include <QFileDialog>
#include <QtCore>
#include <QKeyEvent>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNormalT;  //关于法线的申明
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

  public:
    /** @brief Constructor */
    explicit
    PCLViewer (QWidget *parent = 0);

    /** @brief Destructor */
    ~PCLViewer ();

  public slots:
    /** @brief Triggered whenever the "Save file" button is clicked */
    void
    saveFileButtonPressed ();

    /** @brief Triggered whenever the "Load file" button is clicked */
    void
    loadFileButtonPressed ();

/*    void
    open_left_FileButtonPressed ();

    void
    open_right_FileButtonPressed ();


    /** @brief Triggered whenever a button in the "Color on axis" group is clicked 
    void
    axisChosen ();

    // @brief Triggered whenever a button in the "Color mode" group is clicked 
    void
    lookUpTableChosen ();
*/
   void 
    noise_filter_chosen();   //执行各种滤波的任务

  protected:
    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;   //载入初始点云
     
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_1;   //显示处理后的点云
    // int vp_1, vp_2;   //its left and right viewports
    std::string path_dir_;   //关于打开文件的目录地址

    /** @brief The point cloud displayed */
    PointCloudT::Ptr cloud_;

    PointCloudT::Ptr cloud_filtered;

    PointCloudT::Ptr cloud_right;
   
   int vp_1, vp_2;
   
    /** @brief Holds the color mode for @ref colorCloudDistances */
    int filters_mode_;

    /*
     点云数据中会不可避免的出现一些噪声，对点云进行预处理滤波，可将噪声点，离群点，空洞去除以便于后期
     更好的点云配准，特征提取，曲面重建，可视化等工作，常用的滤波的方法主要有双边滤波，高斯滤波，条件滤波
     直通滤波，其中对于PCL库中的几种需要滤波的情况一般有
     （1）点云数据密度不规则需要平滑处理
     （2）因为数据遮挡等问题造成离群点需要去除
     （3）点云数据量比较大需要下采样
     （4）噪声数据需要去除
    （双边滤波是根据强度信息实现双边滤波，没有强度信息的点云将无法执行双边滤波）
     */
    //void  colorCloudDistances ();
     void noise_filtering();

private:
    /** @brief ui pointer */
    Ui::PCLViewer *ui;
};


#endif // PCLVIEWER_H
