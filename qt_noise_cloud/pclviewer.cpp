#include "pclviewer.h"
#include "./build/ui_pclviewer.h"
#include <boost/make_shared.hpp>
#include <pcl/registration/transforms.h>
#include <QDebug>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer),
    filters_mode_ (5)  // = 滤波模式
{
  ui->setupUi (this);
  this->setWindowTitle ("Noise remove system");

////////////////////直通滤波  在Z 轴深度上的设置///////////////////////////////////////
  //设置份数滑动条的范围和步进
  ui->horizontalSlider_1->setRange(0,5);  ///*horizontalSlider是设置直通滤波器的深度值范围的滑动条
  ui->horizontalSlider_1->setSingleStep(1);  //设置的单步的大小
  //设置滑动条的刻度显示和刻度间隔
  ui->horizontalSlider_1->setTickPosition(QSlider::TicksBothSides);
  ui->horizontalSlider_1->setTickInterval(0.1);
  //同步份数的计数器和滑动条
  connect(ui->spinBoxCount_1, SIGNAL(valueChanged(int)),
          ui->horizontalSlider_1, SLOT(setValue(int)));
  connect(ui->horizontalSlider_1, SIGNAL(valueChanged(int)),
          ui->spinBoxCount_1, SLOT(setValue(int)));
//////////////////////////////均值滤波setRadiusSearch//////////////////////////////
  ui->horizontalSlider_2->setRange(0, 10);  ///*horizontalSlider是设置直通滤波器的深度值范围的滑动条
  ui->horizontalSlider_2->setSingleStep(1);  //设置的单步的大小

  ui->horizontalSlider_2->setTickPosition(QSlider::TicksBothSides);
  ui->horizontalSlider_2->setTickInterval(0.01);

  connect(ui->spinBoxCount_2, SIGNAL(valueChanged(int)),
          ui->horizontalSlider_2, SLOT(setValue(int)));
  connect(ui->horizontalSlider_2, SIGNAL(valueChanged(int)),
          ui->spinBoxCount_2, SLOT(setValue(int)));
/////////////////高斯滤波的参数sigma参数 1～5///////////////////////////////////////////////////
  ui->horizontalSlider_3->setRange(0,5);  ///*horizontalSlider是设置直通滤波器的深度值范围的滑动条
  ui->horizontalSlider_3->setSingleStep(1);  //设置的单步的大小

  ui->horizontalSlider_3->setTickPosition(QSlider::TicksBothSides);
  ui->horizontalSlider_3->setTickInterval(1);

  connect(ui->spinBoxCount_3, SIGNAL(valueChanged(int)),
          ui->horizontalSlider_3, SLOT(setValue(int)));
  connect(ui->horizontalSlider_3, SIGNAL(valueChanged(int)),
          ui->spinBoxCount_3, SLOT(setValue(int)));
/////////////////////体素滤波参数setLeafSize 0～0.5///////////////////////////////
  ui->horizontalSlider_4->setRange(0,5);  ///*horizontalSlider是设置直通滤波器的深度值范围的滑动条
  ui->horizontalSlider_4->setSingleStep(1);  //设置的单步的大小

  ui->horizontalSlider_4->setTickPosition(QSlider::TicksBothSides);
  ui->horizontalSlider_4->setTickInterval(1);

  connect(ui->spinBoxCount_4, SIGNAL(valueChanged(int)),
          ui->horizontalSlider_4, SLOT(setValue(int)));
  connect(ui->horizontalSlider_4, SIGNAL(valueChanged(int)),
          ui->spinBoxCount_4, SLOT(setValue(int)));
//////////////////////////离群点滤波参数setRadiusSearch/////////////////////////////////////
  ui->horizontalSlider_5->setRange(0,5);  ///*horizontalSlider是设置直通滤波器的深度值范围的滑动条
  ui->horizontalSlider_5->setSingleStep(1);  //设置的单步的大小

  ui->horizontalSlider_5->setTickPosition(QSlider::TicksBothSides);
  ui->horizontalSlider_5->setTickInterval(1);

  connect(ui->spinBoxCount_5, SIGNAL(valueChanged(int)),
          ui->horizontalSlider_5, SLOT(setValue(int)));
  connect(ui->horizontalSlider_5, SIGNAL(valueChanged(int)),
          ui->spinBoxCount_5, SLOT(setValue(int)));
////////////////////////////////////////////////////////////////////////////////

   cloud_filtered.reset(new PointCloudT);
  // Setup the cloud pointer
  cloud_.reset (new PointCloudT);
  // The number of points in the cloud
  cloud_->resize (500);

  // Fill the cloud with random points
  for (size_t i = 0; i < cloud_->points.size (); ++i)
  {
    cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }


  // Set up the QVTK window
  viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer_->setBackgroundColor (0.1, 0.1, 0.1);
  ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "Load" and "Save" buttons and their functions
  connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
  connect (ui->pushButton_save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));

  connect (ui->radioButton_PassThrough, SIGNAL(clicked ()), this, SLOT(noise_filter_chosen()));
  connect (ui->radioButton_Uniform, SIGNAL(clicked ()), this, SLOT(noise_filter_chosen()));
  connect (ui->radioButton_Gaussian, SIGNAL(clicked ()), this, SLOT(noise_filter_chosen()));
  connect (ui->radioButton_Voxel, SIGNAL(clicked ()), this, SLOT(noise_filter_chosen()));
  connect (ui->radioButton_Outliers, SIGNAL(clicked ()), this, SLOT(noise_filter_chosen()));

  // Color the randomly generated cloud
  //colorCloudDistances ();
  noise_filter_chosen();
  //viewer_->addPointCloud (cloud_, "cloud");
   viewer_->updatePointCloud (cloud_, "cloud");  
  viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");  
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
//////////////////load file//////////////////////////////
void
PCLViewer::loadFileButtonPressed ()
{
    viewer_->removePointCloud("cloud");
    viewer_->removePointCloud("cloud_filtered");
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/salm/myopencv/path_planning/data/", tr ("Point cloud data (*.pcd *.ply)")); //打开一个文件的语句

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);//申明一个定义

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }
   //可视化界面一分为二
    viewer_->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
     viewer_->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

 // noise_filter_chosen();//执行选项
  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_,0,255,0);
  viewer_->addPointCloud (cloud_, cloud_tgt_h, "cloud", vp_1);
  viewer_->updatePointCloud (cloud_, "cloud");
    
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}
////////////////////////save file///////////////////////////
void
PCLViewer::saveFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/salm/myopencv/yl_pcl/data/tutorials/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_filtered);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_filtered);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_filtered);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}

void
PCLViewer::noise_filter_chosen()
{
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons一次只能执行一种滤波
  if (ui->radioButton_PassThrough->isChecked ())
  {
    PCL_INFO("您选择了直通滤波\n");
    viewer_->removePointCloud("cloud_filtered");
    filters_mode_ = 0;
  }
  else if (ui->radioButton_Uniform->isChecked ())
  {
    PCL_INFO("您选择了均值滤波\n");
   viewer_->removePointCloud("cloud_filtered");
    filters_mode_= 1;
  }
  else if (ui->radioButton_Gaussian->isChecked ())
  {
    PCL_INFO("您选择了高斯滤波\n");
    viewer_->removePointCloud("cloud_filtered");
    filters_mode_ = 2;
  }
  else if (ui->radioButton_Voxel->isChecked ())
  {
    PCL_INFO("您选择了体素滤波\n");
    viewer_->removePointCloud("cloud_filtered");
    filters_mode_ = 3;
  }
  else
  {
    PCL_INFO("您选择了离群点滤波\n");
    viewer_->removePointCloud("cloud_filtered");
    filters_mode_ = 4;
  }

  noise_filtering ();
  viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
  ui->qvtkWidget->update ();
}

void
PCLViewer::noise_filtering ()
{
  PointCloudT::Ptr cloud_tmp (new PointCloudT);//申明一个定义
  PointCloudT::Ptr cloud_out_tmp (new PointCloudT);//处理后保存的结果
  pcl::copyPointCloud (*cloud_, *cloud_tmp);

    // Apply filter to the cloud
    switch (filters_mode_)
    {
      case 0:
        /* 执行直通滤波器
      创建直通滤波器的对象，设定参数，滤波字段设置为Z轴方向，设定的范围为（0，可设定的值）
  即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留      */      
     {   pcl::PassThrough<PointT> pass;
 	pass.setInputCloud(cloud_tmp);
 	pass.setFilterFieldName("z");
 	pass.setFilterLimits(0,ui->spinBoxCount_1->value()/1.0f);//ui->spinBoxCount->value()
 	std::cout<<"Z 的范围值:0~"<< ui->spinBoxCount_1->value()/1.0f<<std::endl;
        pass.filter(*cloud_out_tmp);

        pcl::copyPointCloud (*cloud_out_tmp,*cloud_filtered); 
     std::cout<<"after passthrough filters"<<std::endl;
   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
   PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_filtered,0,255,0);
   viewer_->addPointCloud (cloud_filtered, cloud_tgt_h, "cloud_filtered", vp_2);
   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
        break;
     }
      case 1:
        // 均值滤波
     {  std::cout<<"均值滤波"<<std::endl;
      pcl::UniformSampling<PointT> uniform;
      uniform.setInputCloud (cloud_tmp);
      uniform.setRadiusSearch (ui->spinBoxCount_2->value()*0.02f);
    std::cout<<"均值滤波搜索半径: "<<ui->spinBoxCount_2->value()*0.02f<<std::endl;
      uniform.filter (*cloud_filtered);

   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
   PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_filtered,0,255,0);
   viewer_->addPointCloud (cloud_filtered, cloud_tgt_h, "cloud_filtered", vp_2);
   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
        break;
    }
      case 2:
        // 高斯滤波方法
      {  std::cout<<"高斯滤波"<<std::endl;
        //set up the Gaussian kernel
   	pcl::filters::GaussianKernel<PointT,PointT>::Ptr kernel(new pcl::filters::GaussianKernel<PointT,PointT>);
        (*kernel).setSigma(ui->spinBoxCount_3->value());
  std::cout<<"高斯滤波sigma: "<<ui->spinBoxCount_3->value()<<std::endl;
        (*kernel).setThresholdRelativeToSigma(4);
        std::cout<<"Kernel made"<<std::endl;

         //set up the KDtree
     //  pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);
      pcl::search::KdTree<PointT>::Ptr Kdtree(new pcl::search::KdTree<PointT>);
      (*Kdtree).setInputCloud(cloud_tmp);
       std::cout<<"Kdtree made"<<std::endl;

        //set up the Convolution filter
        pcl::filters::Convolution3D<PointT,PointT,pcl::filters::GaussianKernel<PointT,PointT> > convolution;
         convolution.setKernel(*kernel);
         convolution.setInputCloud(cloud_tmp);
         convolution.setSearchMethod(Kdtree);
                     
	 convolution.setRadiusSearch(0.02);
         std::cout<<"Convolution start" <<std::endl;
         convolution.convolve(*cloud_filtered);

   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
   PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_filtered,0,255,0);
   viewer_->addPointCloud (cloud_filtered, cloud_tgt_h, "cloud_filtered", vp_2);
   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");

        break; 
      }
      case 3:
        //双边滤波方法
    { std::cout<<"体素滤波"<<std::endl;

/*      pcl::PointCloud<PointT> outcloud;  
        // 建立kdtree  
        //pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);  
        pcl::search::KdTree<PointT>::Ptr tree1(new pcl::search::KdTree<PointT>);  
        pcl::BilateralFilter<PointT> bf;  
        bf.setInputCloud(cloud_tmp);  
        bf.setSearchMethod(tree1);  
        bf.setHalfSize(5.0);  
        bf.setStdDev(0.02);  
        bf.filter(outcloud);  
*/
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud (cloud_tmp);
  voxel.setLeafSize (ui->spinBoxCount_4->value()*0.02f, ui->spinBoxCount_4->value()*0.02f, ui->spinBoxCount_4->value()*0.02f);
  std::cout<<"体素滤波大小: "<<ui->spinBoxCount_4->value()*0.02f<<std::endl;
  voxel.filter (*cloud_filtered);

   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
   PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_filtered,0,255,0);
   viewer_->addPointCloud (cloud_filtered, cloud_tgt_h, "cloud_filtered", vp_2);
   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");

        break;
     }
      default:
        // 离群点滤波方法
   {  
   std::cout<<"离群点滤波"<<std::endl;
   pcl::RadiusOutlierRemoval<PointT> outrem;  //创建滤波器
    outrem.setInputCloud(cloud_tmp);    //设置输入点云
    outrem.setRadiusSearch(ui->spinBoxCount_5->value()*0.02f);     //设置半径为0.8的范围内找临近点
  std::cout<<"离群点滤波半径: "<<ui->spinBoxCount_5->value()*0.02f<<std::endl;
    outrem.setMinNeighborsInRadius (10); //设置查询点的邻域点集数小于2的删除
    outrem.filter (*cloud_filtered);     //执行条件滤波在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
  
   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");
   PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_filtered,0,255,0);
   viewer_->addPointCloud (cloud_filtered, cloud_tgt_h, "cloud_filtered", vp_2);
   viewer_->updatePointCloud (cloud_filtered, "cloud_filtered");

  }
    
  }
}





