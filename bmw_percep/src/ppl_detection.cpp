#include<bmw_percep/ppl_detection.hpp>

/** 
    Utility functions for People Detection
**/

using namespace std; 

void ppl_detection::find_euclid_blobs(PointCloudT::ConstPtr cloud, 
				      PointCloudT::Ptr viz_cloud, 
				      vector<cv::Point3f> clusters, 
				      int& max_blob_id,
				      const Eigen::VectorXf ground_coeffs,
				      float leaf_size/*=0.01*/)
{

  float voxel_size=0.06;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> vg;
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.06f, 0.06f, 0.06f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

  // Ground removal and update:
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_filtered));
  ground_model->selectWithinDistance(ground_coeffs, 3*voxel_size, *inliers);
  PointCloudT::Ptr no_ground_cloud(new PointCloudT);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud);
  
  // //debug
  // pcl::copyPointCloud(*no_ground_cloud, *viz_cloud);
  // return;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (no_ground_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (2* 0.06); // 2cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (5000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (no_ground_cloud);
  ec.extract (cluster_indices);

  // std::vector<pcl::people::PersonCluster<PointT> > ppl_clusters; 

  // find_ppl_clusters(no_ground_cloud,
  // 		    cluster_indices,
  // 		    ppl_clusters,
  // 		    ground_coeffs);

  // //debug
  // cout << "No. of clusters: " << cluster_indices.size() << endl;

  // int j = 0;
  
  // //replace cluster indices with people clusters
  // int n_ppl=0;
  // cluster_indices.clear();
  // for(std::vector<pcl::people::PersonCluster<PointT> >::iterator 
  // 	it = ppl_clusters.begin(); it != ppl_clusters.end(); ++it)
  //     {
  // 	cluster_indices.push_back(it->getIndices());
  // 	n_ppl++;
  //     }
  
  // cout << "No. of people: " << n_ppl << endl; 
  // // string whatupp;
  // // if (n_ppl > 100)
  // //   cin >> whatupp;

  // visualize by painting each PC another color
  pcl::copyPointCloud(*no_ground_cloud, *cloud_filtered);
  
  viz_cloud->points.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
       it != cluster_indices.end (); ++it){
    uint8_t r(rand()%255), g(rand()%255), b(rand()%255);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
  	 pit != it->indices.end (); pit++){
      // create RGB point to push in
      PointT new_pt;
      new_pt.x = cloud_filtered->points[*pit].x;
      new_pt.y = cloud_filtered->points[*pit].y;
      new_pt.z = cloud_filtered->points[*pit].z;

      // new_pt.r = cloud_filtered->points[*pit].r;
      // new_pt.g = cloud_filtered->points[*pit].g;
      // new_pt.b = cloud_filtered->points[*pit].b;

      new_pt.r = r; new_pt.g = g; new_pt.b = b;
      new_pt.a = 1.0;
      viz_cloud->points.push_back (new_pt); //*
    }
    viz_cloud->width = viz_cloud->points.size ();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    //std::cout << "PointCloud representing the Cluster: " << viz_cloud->points.size () << " data points." << std::endl;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //j++;

  }
}
