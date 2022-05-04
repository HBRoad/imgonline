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
	//------------------------------- txtתpcd -------------------------------
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

	//------------------------------- ���ص��� -------------------------------
	cout << "->���ڼ��ص���..." << endl;
	PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("points_Cloud.pcd", *cloud) < 0)
	{
		PCL_ERROR("\a�����ļ������ڣ�\n");
		system("pause");
		return -1;

	}
	cout << "->���ص�ĸ�����" << cloud->points.size() << endl;
	//========================================================================

	//------------------------------- ģ�͹��� -------------------------------
	cout << "->���ڹ��ƿռ�Բ..." << endl;
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));	//ѡ����ϵ����뼸��ģ��
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle3D);	//�����������һ���Զ���
	ransac.setDistanceThreshold(0.01);	//���þ�����ֵ����ģ�;���С��0.01�ĵ���Ϊ�ڵ�
	ransac.setMaxIterations(10000);		//��������������
	ransac.computeModel();				//ִ��ģ�͹���
	vector<int> inliers;				//�洢�ڵ�����������
	ransac.getInliers(inliers);			//��ȡ�ڵ��Ӧ������

										/// ����������ȡ�ڵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_circle);

	/// ���ģ�Ͳ���(x-x0)^2 + (y-y0)^2 = r^2;
	Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);

	cout << "�ռ�Բ�ģ�"
		<< "(" << coefficient[0] << ","
		<< coefficient[1] << ","
		<< coefficient[2] << ")"
		<< endl;
	cout << "�뾶��" << coefficient[3] << endl;
	cout << "�ռ�Բ��������"
		<< "(" << coefficient[4] << ","
		<< coefficient[5] << ","
		<< coefficient[6] << ")"
		<< endl;
	//========================================================================

	//---------------------------- ���ӻ������ѡ�� ---------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("��Ͻ��"));

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");													//���ԭʼ����
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");	//��ɫ
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");	//��Ĵ�С

	viewer->addPointCloud<pcl::PointXYZ>(cloud_circle, "circle");											//���ģ�͵���
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "circle");	//��ɫ
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "circle");	//��Ĵ�С

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//========================================================================

	return 0;
}
