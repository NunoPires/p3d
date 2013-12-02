#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

void pp_callback (const pcl::visualization::PointPickingEvent& event, void*)
{
	pcl::PointXYZ picked_pt;

	int idx = event.getPointIndex();

	if(idx == -1) return;

	event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);

	std::cout << idx << std::endl;

	std::cout << picked_pt.x << " " << picked_pt.y << " " << picked_pt.z << std::endl;	
}

int main(void)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_final (new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("frame67.pcd", *cloud) == -1) {

		PCL_ERROR ("Couldn't read file. \n");
		return(-1);
	} 

	std::cout << "Loaded " << cloud->width * cloud->height << " points" << std::endl;
	
	// Filtros iniciais
	pcl::PassThrough<pcl::PointXYZ> pass;
	
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.4,0.4);
	pass.filter (*cloud_filtered);

	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (1.0, 2.2);
	pass.filter (*cloud_filtered_final);
	
	// Segmentação do chão
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.7);

  seg.setInputCloud (cloud_filtered_final->makeShared ());
  seg.segment (*inliers, *coefficients);
	
	// Coeficientes do plano
	//std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
     //                                 << coefficients->values[1] << " "
     //                                 << coefficients->values[2] << " " 
     //                                 << coefficients->values[3] << std::endl;
	
	std::vector<int> indices_extraction;	

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
			// Print Inliers

		  //std::cerr << inliers->indices[i] << "    " << cloud_filtered_final->points[inliers->indices[i]].x << " "
		                                             //<< cloud_filtered_final->points[inliers->indices[i]].y << " "
		                                             //<< cloud_filtered_final->points[inliers->indices[i]].z << std::endl;
 	}

	 for (size_t j = (inliers->indices.size() - 1); j > 0; j--)
	{
			if(cloud_filtered_final->points[inliers->indices[j]].y > 0.53)
			{
   			cloud_filtered_final->points.erase(cloud_filtered_final->points.begin() + inliers->indices[j]);
			}	
	}
		
	//pcl::ExtractIndices<pcl::PointXYZ> eifilter(true);
	//eifilter.setInputCloud (cloud_filtered_final);
	//eifilter.setIndices(indices_extraction);
	//eifilter.setUserFilterValue (0);
	//eifilter.filterDirectly (cloud_filtered_final);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( 
	new pcl::visualization::PCLVisualizer ("3D Viewer"));

	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_filtered_final, "cloud");
	
	viewer->setPointCloudRenderingProperties(
	pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	
	// Color
	viewer->setPointCloudRenderingProperties (
	pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud");
	
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters();
	viewer->registerPointPickingCallback(pp_callback);

	// Cotovelo Braço Esquerdo
	// 0.436288 -0.563699 2.027
	
	// Cotovelo Braço Direito
	// -0.37501 -0.595124 2.14

	// Ponta do pé: 

	// -0.143 0.528 1.925 (frente)
	// 0.0495981 0.522688 2.003

	// Comprimento aproximado do pé (segundo z)

		 // 0.08

	//Altura do pé (segundo y)
	// 0.00372952 0.529592 1.958

	//-0.0834743 0.436343 1.992

	// Ou mais preciso?

	//	-0.0484838 0.477379 1.958

	// -0.0445029 0.526617 1.947

	// 0.05

	float coordinate_reference_z = 3;
	float coordinate_reference_y = 0;
	float coordinate_reference_x = cloud_filtered_final->points[0].x;
	int index_z = 0;
	int index_y = 0;
	float big_x = cloud_filtered_final->points[0].x;
	float small_x = cloud_filtered_final->points[0].x;	
	
	for(size_t k = 0; k < cloud_filtered_final->points.size(); k++)
	{
		if(cloud_filtered_final->points[k].x > big_x)
		{
			big_x = cloud_filtered_final->points[k].x;
		}

		if(cloud_filtered_final->points[k].x < small_x)
		{
			small_x = cloud_filtered_final->points[k].x;
		}
	}

	// Ponto médio segundo x
	coordinate_reference_x = (big_x + small_x)/2;

	std::cout << big_x << " " << small_x << " " << coordinate_reference_x << 	std::endl;

	for(size_t k = 0; k < cloud_filtered_final->points.size(); k++)
	{
		// Search for highest Y and lowest z, mas só a partir do ponto médio de x para a direita (0.01 de margem para garantir que nao apanhamos o pe esquerdo no ponto medio)

		if(cloud_filtered_final->points[k].x < (coordinate_reference_x - 0.01))
		{

			if(cloud_filtered_final->points[k].z < coordinate_reference_z)
			{
				index_z = k;
				coordinate_reference_z = cloud_filtered_final->points[k].z;
			}

			if(cloud_filtered_final->points[k].y > coordinate_reference_y)
			{
				index_y = k;
				coordinate_reference_y = cloud_filtered_final->points[k].y;
			}
		}
	}

	// Print points and index
	std::cout << index_z << " " << index_y << " " << cloud_filtered_final->points[index_z].z << " " << cloud_filtered_final->points[index_y].y << std::endl;
	
	std::cout << coordinate_reference_z << " " << coordinate_reference_y << std::endl;

	// Vamos tirar pontos só do lado direito! 

	// Print all points of the foot to a file:
	std::ofstream myfile;
  myfile.open ("foot.txt");

	// Z limit
	float z_limit = coordinate_reference_z + 0.08;
	float y_limit = coordinate_reference_y - 0.05;
  
	for(size_t l = 0; l < cloud_filtered_final->points.size(); l++)
	{
		if(cloud_filtered_final->points[l].x < coordinate_reference_x 			 
		&& cloud_filtered_final->points[l].y > y_limit 
		&& cloud_filtered_final->points[l].y < coordinate_reference_y
		&& cloud_filtered_final->points[l].z > coordinate_reference_z
		&& cloud_filtered_final->points[l].y < z_limit)
		{
			myfile << l << ": " << cloud_filtered_final->points[l].x << " " << cloud_filtered_final->points[l].y << " " << cloud_filtered_final->points[l].z << std::endl;
		}
	}

	myfile.close();


//pass.setFilterFieldName ("y");
	//pass.setFilterLimits (-0.9,0.5);
	//pass.filter (*cloud_filtered_1);

	//pass.setInputCloud (cloud_filtered_1);
	//pass.setFilterFieldName ("z");
	//pass.setFilterLimits (1.0, 2.2);
	//pass.filter (*cloud_filtered_final);

	//pcl::io::savePCDFileASCII ("ransack.pcd", *cloud_filtered_final);

	// Maintain a thread running

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		
		// custom code?

		boost::this_thread::sleep(
		boost::posix_time::microseconds (100000));
	}

	return (0);
}
