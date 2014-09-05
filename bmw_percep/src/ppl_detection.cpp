#include<bmw_percep/ppl_detection.hpp>

/** 
    Utility functions for People Detection
**/

using namespace std; 

void ppl_detection::find_euclid_blobs(PointCloudT::ConstPtr cloud, 
				      PointCloudT::Ptr viz_cloud, 
				      // vector<cv::Point3f> clusters, 
				      vector<vector<Eigen::Vector3f> > clusters,
				      const Eigen::VectorXf ground_coeffs,
				      float leaf_size/*=0.01*/)
{

  float voxel_size=0.06;
  int max_cluster_size = 800;
  int min_cluster_size = 100;
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
  // ground_model->selectWithinDistance(ground_coeffs, 3*voxel_size, *inliers);
  ground_model->selectWithinDistance(ground_coeffs, 1.5*voxel_size, *inliers);
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
  // ec.setMaxClusterSize (5000);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (no_ground_cloud);
  ec.extract (cluster_indices);

  std::vector<pcl::people::PersonCluster<PointT> > ppl_clusters; 

  find_ppl_clusters(no_ground_cloud,
  		    cluster_indices,
  		    ppl_clusters,
  		    ground_coeffs,
		    max_cluster_size,
		    min_cluster_size);

  //debug
  cout << "No. of clusters: " << cluster_indices.size() << endl;

  int j = 0;
  
  //replace cluster indices with people clusters
  int n_ppl=0;
  cluster_indices.clear();
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator 
  	it = ppl_clusters.begin(); it != ppl_clusters.end(); ++it)
      {
  	cluster_indices.push_back(it->getIndices());
  	n_ppl++;
      }
  
  cout << "No. of people: " << n_ppl << endl; 

  //copy over the clusters to return
  clusters.clear();
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator 
  	it = ppl_clusters.begin(); it != ppl_clusters.end(); ++it)
      {
	vector<Eigen::Vector3f> temp_cl;
	temp_cl.clear();
	vector<int> c_ind = (*it).getIndices().indices;
	for (vector<int>::iterator cit = c_ind.begin(); 
	     cit!=c_ind.end(); ++cit){
	  PointT pty = no_ground_cloud->points[*cit];
	  Eigen::Vector3f pt_temp(pty.x, pty.y, pty.z);
	  temp_cl.push_back(pt_temp);
	}
	clusters.push_back(temp_cl);
      } 

  // string whatupp;
  // if (n_ppl > 100)
  //   cin >> whatupp;

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


void ppl_detection::find_ppl_clusters(const PointCloudT::Ptr cloud, 
		  vector<pcl::PointIndices>& init_indices, 
		  std::vector<pcl::people::PersonCluster<PointT> >& clusters,
				      const Eigen::VectorXf ground_coeffs,
				      const int max_c_size,
				      const int min_c_size)
{
  
  //debug
  // cout << "Started this finding.." << endl;

  float max_height_= 2.3; float min_height_= 1.2;
  int min_a_merge = 200; float max_dist_gr=0.4;
  bool camera_vertical=false, compute_head=false;

  float sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f
				(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();

  // Person clusters creation from clusters indices:
  for(std::vector<pcl::PointIndices>::const_iterator it = 
	init_indices.begin(); it != init_indices.end(); ++it)
    {
      pcl::people::PersonCluster<PointT> cluster(cloud, *it, ground_coeffs, sqrt_ground_coeffs_, compute_head, camera_vertical); // PersonCluster creation
      clusters.push_back(cluster);
    }

  //debug
  // cout << "Created ppl clusters.." << endl;

  std::vector<pcl::people::PersonCluster<PointT> > new_clusters;
  for(unsigned int i = 0; i < clusters.size(); i++) // for every cluster
    {
      // if (clusters[i].getHeight() <= max_height_)
  	new_clusters.push_back(clusters[i]);
    }

  //Merge clusters_close in floor coordinates:
  clusters.clear();
  mergeClustersCloseInFloorCoordinates(cloud, new_clusters, clusters,
				       ground_coeffs, sqrt_ground_coeffs_);

  std::vector<pcl::people::PersonCluster<PointT> > ppl_filtered;
  // Remove clusters according to rules
  rm_ppl_clusters(cloud, clusters, new_clusters,
		  ground_coeffs, sqrt_ground_coeffs_, max_height_, min_height_,
		  max_dist_gr, max_c_size, min_c_size);

  ppl_filtered.clear();
  for(unsigned int i = 0; i < new_clusters.size(); i++) // for every cluster
    {
      if (new_clusters[i].getNumberPoints() > min_a_merge){
	if (new_clusters[i].getHeight() >= min_height_){
	  if (get_min_ground_dist(cloud, new_clusters[i], ground_coeffs, 
				  sqrt_ground_coeffs_, max_dist_gr))
  	    ppl_filtered.push_back(new_clusters[i]);
	}
      }
    }

  // //debug
  // cout << "Donadone.." << endl;
  clusters.clear();
  clusters = ppl_filtered;
  return;
  // new_clusters.clear();

  // std::vector<pcl::people::PersonCluster<PointT> > subclusters;
  // int cluster_min_points_sub = int(float(min_points_) * 1.5);
}


void ppl_detection::mergeClustersCloseInFloorCoordinates 
( const PointCloudT::Ptr cloud, 
std::vector<pcl::people::PersonCluster<PointT> >& input_clusters,
 std::vector<pcl::people::PersonCluster<PointT> >& output_clusters, 
 const Eigen::VectorXf ground_coeffs_, double sqrt_ground_coeffs_)
{
  float min_distance_between_cluster_centers = 0.4; // meters
  float normalize_factor = std::pow(sqrt_ground_coeffs_, 2); // sqrt_ground_coeffs
							     // ^ 2
							     // (precomputed
							     // for
							     // speed)

  Eigen::Vector3f head_ground_coeffs = ground_coeffs_.head(3); // ground
							      // plane
							      // normal
							      // (precomputed
							      // for
							      // speed)
  std::vector <std::vector<int> > connected_clusters;
  connected_clusters.resize(input_clusters.size());
  std::vector<bool> used_clusters; // 0 in correspondence of clusters
				   // remained to process, 1 for
				   // already used clusters
  used_clusters.resize(input_clusters.size());
  // initialize clusters unused
  for(vector<bool>::iterator usit=used_clusters.begin(); usit!=used_clusters.end();
      usit++)
    *usit = false;
    
  for(unsigned int i = 0; i < input_clusters.size(); i++) // for every cluster
    {
      Eigen::Vector3f theoretical_center = input_clusters[i].getTCenter();
      float t = theoretical_center.dot(head_ground_coeffs) /
	normalize_factor; // height from the ground
      Eigen::Vector3f current_cluster_center_projection = 
	theoretical_center - head_ground_coeffs * t; // projection of
						     // the point on
						     // the
						     // groundplane
      for(unsigned int j = i+1; j < input_clusters.size(); j++) // for
								// every
								// remaining
								// cluster
	{
	  theoretical_center = input_clusters[j].getTCenter();
	  float t = theoretical_center.dot(head_ground_coeffs) 
	    / normalize_factor; // height from the ground
	  Eigen::Vector3f new_cluster_center_projection =
	    theoretical_center - head_ground_coeffs * t; // projection
							 // of the
							 // point on
							 // the
							 // groundplane
	  if (((new_cluster_center_projection - 
		current_cluster_center_projection).norm()) < 
	      min_distance_between_cluster_centers)
	    {
	      connected_clusters[i].push_back(j);
	    }
	}
    }

 for(unsigned int i = 0; i < connected_clusters.size(); i++) // for every cluster
  {
    if (!used_clusters[i]) // if this cluster has not been used yet
    {
      used_clusters[i] = true;
      if (connected_clusters[i].empty()) // no other clusters to merge
      {
        output_clusters.push_back(input_clusters[i]);
      }
      else
      {
        // Copy cluster points into new cluster:
        pcl::PointIndices point_indices;
        point_indices = input_clusters[i].getIndices();
        for(unsigned int j = 0; j < connected_clusters[i].size(); j++)
        {
          if (!used_clusters[connected_clusters[i][j]]) // if this
							// cluster has
							// not been
							// used yet
          {
            used_clusters[connected_clusters[i][j]] = true;
            for(std::vector<int>::const_iterator points_iterator = 
		  input_clusters[connected_clusters[i][j]].getIndices().
		  indices.begin();
                points_iterator != 
		  input_clusters[connected_clusters[i][j]].getIndices().
		  indices.end(); points_iterator++)
            {
              point_indices.indices.push_back(*points_iterator);
            }
          }
        }
        pcl::people::PersonCluster<PointT> cluster(cloud, point_indices, ground_coeffs_, sqrt_ground_coeffs_, false, false);
        output_clusters.push_back(cluster);
      }
    }
  }
}


bool ppl_detection::get_min_ground_dist (const PointCloudT::Ptr cloud, 
				    pcl::people::PersonCluster<PointT> person_c, 
				    const Eigen::VectorXf ground_coeffs, 
				    double sqrt_ground_coeffs, double min_dist)
{
  pcl::PointIndices p_ind = person_c.getIndices();
  double min_disto=1000.0;

  for (std::vector<int>::const_iterator it = p_ind.indices.begin (); 
       it != p_ind.indices.end(); ++it){
    Eigen::Vector4f bott_pt;
    bott_pt << cloud->points[*it].x, cloud->points[*it].y, cloud->points[*it].z, 1.0f;
    double dist = fabs(bott_pt.dot(ground_coeffs)/sqrt_ground_coeffs);
    if (min_disto > dist)
      min_disto = dist;
    if (min_dist >= dist)
      return true;
  }
  
  cout << "Minimum cluster distance is : " << min_disto << endl;
  return false;
  
}

//TODO: Fill this up
void ppl_detection::remove_robot(PointCloudT::ConstPtr cloud, 
		    PointCloudT::Ptr cloud_f){}

void ppl_detection::rm_ppl_clusters
( const PointCloudT::Ptr cloud, 
  std::vector<pcl::people::PersonCluster<PointT> >& in_cl,
  std::vector<pcl::people::PersonCluster<PointT> >& out_cl, 
  const Eigen::Vector4f ground_coeffs_, double sqrt_ground_coeffs_,
  const float max_ht, const float min_ht, 
  const float max_gr_dist,
  const int max_c_size,
  const int min_c_size)
{
  //Precomputation for point distance compute
  Eigen::Vector3f n;
  n(0) = ground_coeffs_(0);
  n(1) = ground_coeffs_(1);
  n(2) = ground_coeffs_(2);
  float n_norm = n.norm();
  n /= n_norm;
  float p = ground_coeffs_(3)/n_norm;
  // float distance = pt.dot(n) + p;

  //Ppl properties
  vector<float> heights;
  vector<float> gr_dists;
  heights.clear();
  gr_dists.clear();
  out_cl.clear();
  
  // for each person cluster
  for(vector<pcl::people::PersonCluster<PointT> >::iterator 
	persona = in_cl.begin();
      persona!= in_cl.end(); persona++){
    vector<int> temp_in;
    temp_in = ((*persona).getIndices()).indices;
    

    //debug
    cout << "\nCluster Size= "<< temp_in.size() << endl;

    //Check size of cluster
    if(temp_in.size()>max_c_size && temp_in.size()<min_c_size){
      continue;
    }
    float person_ht=0.;
    float person_dist=numeric_limits<float>::infinity();
 
    //for each point in the cluster
    for(vector<int>::iterator pin=temp_in.begin(); 
	pin != temp_in.end(); pin++){
      //Get actual point
      const PointT* pt = &cloud->points[*pin];
      Eigen::Vector3f pt_v((*pt).x, (*pt).y, (*pt).z);
      //compute ground distance
      float distance = pt_v.dot(n) + p;
      if (distance>person_ht)
	person_ht = distance;
      if(distance<person_dist)
	person_dist = distance;
    }
    heights.push_back(person_ht);
    gr_dists.push_back(person_dist);
    
    //Decide which clusters go in
    if (person_ht<=max_ht && person_ht>=min_ht && person_dist<=max_gr_dist)
      out_cl.push_back(*persona);
  }

  return;
  
}
