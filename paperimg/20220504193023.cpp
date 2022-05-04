#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3D.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
int main()
{
	//------------------------------- txt转pcd -------------------------------
	fstream modelRead;
	pcl::PointCloud<pcl::PointXYZ> cloud1;
	pcl::PCDWriter writer1;

	modelRead.open("points.txt", std::ios_base::in);
	pcl::PointXYZ pclPnt;
	while (!modelRead.eof())
	{
		modelRead >> pclPnt.x >> pclPnt.y >> pclPnt.z;
		cloud1.push_back(pclPnt);
	}
	modelRead.close();
	writer1.write("points_Cloud.pcd", cloud1);

	//------------------------------- 加载点云 -------------------------------
	cout << "->正在加载点云..." << endl;
	PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("points_Cloud.pcd", *cloud) < 0)
	{
		PCL_ERROR("\a点云文件不存在！\n");
		system("pause");
		return -1;

	}
	cout << "->加载点的个数：" << cloud->points.size() << endl;
	//========================================================================

	//------------------------------- 模型估计 -------------------------------
	cout << "->正在估计空间圆..." << endl;
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));	//选择拟合点云与几何模型
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle3D);	//创建随机采样一致性对象
	ransac.setDistanceThreshold(0.01);	//设置距离阈值，与模型距离小于0.01的点作为内点
	ransac.setMaxIterations(10000);		//设置最大迭代次数
	ransac.computeModel();				//执行模型估计
	vector<int> inliers;				//存储内点索引的向量
	ransac.getInliers(inliers);			//提取内点对应的索引

										/// 根据索引提取内点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_circle);

	/// 输出模型参数(x-x0)^2 + (y-y0)^2 = r^2;
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);

	cout << "空间圆心："
		<< "(" << coefficient[0] << ","
		<< coefficient[1] << ","
		<< coefficient[2] << ")"
		<< endl;
	cout << "半径：" << coefficient[3] << endl;
	cout << "空间圆法向量："
		<< "(" << coefficient[4] << ","
		<< coefficient[5] << ","
		<< coefficient[6] << ")"
		<< endl;
	//========================================================================

	//---------------------------- 可视化结果（选） ---------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("拟合结果"));

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");													//添加原始点云
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");	//颜色
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");	//点的大小

	viewer->addPointCloud<pcl::PointXYZ>(cloud_circle, "circle");											//添加模型点云
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "circle");	//颜色
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "circle");	//点的大小

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//========================================================================

	return 0;
}
