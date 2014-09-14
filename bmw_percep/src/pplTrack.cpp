#include <bmw_percep/pplTrack.hpp>

/**
   
   Class *implementation* for tracking people from RGBD imagery.
   Uses PCL.
**/

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PplTrack::PplTrack(Eigen::Vector4f ground_coeffs)
{
  table_link = false;
  ground_coeffs_ = ground_coeffs;
  history_per_stats_.clear();
}

void PplTrack::estimate(vector<vector<ClusterPoint> > clusters)
{
  //ASSUMPTION: Max one person in the scene
  int n_clusters = clusters.size();
  //debug
  cout << "No. of Peoples = " << n_clusters << endl;
  if (n_clusters<1) // 0 clusters
    return; // No person if no observation
  else if (n_clusters==1) // 1 cluster
    estimate(clusters[0]);
  else{ // more than one cluster
    
    estimate(clusters[getOneCluster(clusters)]); //Max sized blob is person
  }
}

void PplTrack::estimate(vector<ClusterPoint> cluster)
{
  ClusterPoint centroid(0.,0.,0.);

  for(vector<ClusterPoint>::iterator cpit=cluster.begin();
      cpit!=cluster.end(); ++cpit){
    
    centroid+= *cpit;
  }
  centroid /= cluster.size();

  cur_pos_.clear();
  cur_pos_.push_back(centroid);
}

int PplTrack::getOneCluster(const vector<vector<ClusterPoint> > clusters)
{
  //can only get cluster if size not zero
  if (clusters.size()>0){
  //Presently chooses the one with max number of points
  int max_size=0;
  int max_id=0; int cur_id=0;
    
  for (vector<vector<ClusterPoint> >::const_iterator cit=clusters.begin();
       cit!=clusters.end(); ++cit){
    if ((*cit).size()>max_size){
      max_size=(*cit).size();
      max_id = cur_id;
    }
    ++cur_id;
  }
  
  return max_id;
  }
  
  else{return -1;}
}

//Only points to the person's position yet
//Also publishes a cylinder enclosing the trunk
void PplTrack::visualize(ros::Publisher pub)
{
  //debug
  cout << "Person ID: " << person_id_ << endl;
  if(person_id_>-1){
  //markers object
  visualization_msgs::MarkerArray mark_arr;
  mark_arr.markers.clear();

  //assumption- one person only
  ClusterStats per_stats = per_stats_[person_id_];

  //Inner-cylinder marker
  visualization_msgs::Marker inn_cyl_marker;
  // Set the frame ID and timestamp.  
  inn_cyl_marker.header.frame_id = viz_frame_;
  
  // //debug
  // cout << "Frame is " << viz_frame_ << endl;
  inn_cyl_marker.header.stamp = pub_time;

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  inn_cyl_marker.ns = "human";
  //TODO: use enum and not these numbers
  inn_cyl_marker.id = 0;

  // Set the marker type to Cylinder
  uint32_t cylinder = visualization_msgs::Marker::CYLINDER;
  uint32_t sphere = visualization_msgs::Marker::SPHERE;
  inn_cyl_marker.type = cylinder;

  // Set the inn_cyl_marker action
  inn_cyl_marker.action = visualization_msgs::Marker::ADD;


  // inn_cyl_marker.scale.x = (per_stats.max(0)-per_stats.min(0));
  // inn_cyl_marker.scale.y = (per_stats.max(1)-per_stats.min(1));
  // inn_cyl_marker.scale.z = (per_stats.max(2)); // only distance from ground

  // Set the pose of the inn_cyl_marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  inn_cyl_marker.pose.position.x = per_stats.median(0);//(per_stats.max(0)+per_stats.min(0))/2;
  inn_cyl_marker.pose.position.y = per_stats.median(1);//(per_stats.max(1)+per_stats.min(1))/2;
  inn_cyl_marker.pose.position.z = (per_stats.max(2))/2; // only distance from ground

  //the cylinder is oriented along z
  inn_cyl_marker.pose.orientation.x = 0.0;
  inn_cyl_marker.pose.orientation.y = 0.0;
  inn_cyl_marker.pose.orientation.z = 0.0;
  inn_cyl_marker.pose.orientation.w = 1.0;

  // Set the scale of the inn_cyl_marker -- 1x1x1 here means 1m on a side
  inn_cyl_marker.scale.x = sqrt(per_stats.var(0))*2.0;
  inn_cyl_marker.scale.y = sqrt(per_stats.var(1))*2.0; 
  inn_cyl_marker.scale.z = (per_stats.max(2)); // only distance from ground

  
  //debug
  cout << "Positions set.. " << endl;
  
  
  // Set the color -- be sure to set alpha to something non-zero!
  inn_cyl_marker.color.r = 0.0f;
  inn_cyl_marker.color.g = 0.0f;
  inn_cyl_marker.color.b = 1.0f;
  inn_cyl_marker.color.a = 1.0;

  //debug
  cout << "Colors set.. " << endl;

  // inn_cyl_marker.pose.position.z += inn_cyl_marker.scale.z/2;
  
  inn_cyl_marker.lifetime = ros::Duration();

  mark_arr.markers.push_back(inn_cyl_marker);

  //add the outer human cylinder
  visualization_msgs::Marker out_cyl_marker;
  out_cyl_marker = inn_cyl_marker;

  out_cyl_marker.id = 1;

  // Set the scale of the out_cyl_marker -- 1x1x1 here means 1m on a side
  float scale_x = (std::max(std::fabs(per_stats.max(0)-out_cyl_marker.scale.x),
			    std::fabs(-per_stats.min(0)+out_cyl_marker.scale.x)));
  float scale_y = (std::max(std::fabs(per_stats.max(1)-out_cyl_marker.scale.y),
			    std::fabs(-per_stats.min(1)+out_cyl_marker.scale.y)));

  out_cyl_marker.scale.x = scale_x;
  out_cyl_marker.scale.y = scale_y; 
  out_cyl_marker.scale.z = (per_stats.max(2)); // only distance from ground

  // Set the color -- be sure to set alpha to something non-zero!
  out_cyl_marker.color.r = 1.0f;
  out_cyl_marker.color.g = 0.0f;
  out_cyl_marker.color.b = 0.0f;
  out_cyl_marker.color.a = 0.5;

  mark_arr.markers.push_back(out_cyl_marker);
  
  pub.publish(mark_arr);
  }
  else // in case no people detected
    {//TODO:delete visualization from earlier?
    }
}

void PplTrack::estimate(PointCloudT::Ptr& cloud, 
			vector<vector<ClusterPoint> > &clusters,
			const Eigen::VectorXf ground_coeffs,
			const Eigen::Vector3f robo_loc,
			bool got_tf_robot,
			float leaf_size/*=0.06*/)
{
  reset_vals();
  
  if (!table_link){
    PointCloudT::Ptr viz_cloud(new PointCloudT);
    //workspace_limit(cloud);
    ppl_detection::find_euclid_blobs(cloud, viz_cloud,  
				     clusters,
				     ground_coeffs, leaf_size);
    estimate(clusters);
  }
  else{
    
    PointCloudT::Ptr viz_cloud(new PointCloudT);

    workspace_limit(cloud);
    if (got_tf_robot) // in case the robots location is known
      robot_remove(cloud, robo_loc);

    int max_cluster_size = 800;
    int min_cluster_size = 100;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (2* leaf_size); // 2cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (5000);
    // ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    //merge
    merge_floor_clusters(cloud, cluster_indices);

    //remove clusters acc to rules
    rm_clusters_rules(cloud, cluster_indices);

    cout << "Problem with assigning cluster if empty?" << endl;
    //assign person clusters to the private member
    assign_ppl_clusters(cloud, cluster_indices);
    cout << "No its not, haha.." << endl;
    //this is our guy/gal
    person_id_ = getOneCluster(per_cs_);

    if (per_cs_.size()>1)
      more_than_one_=true;
    else
      more_than_one_=false;
    
    //TODO: May be publish the timestamp of PointCloud instead
    pub_time = ros::Time::now();

    //TODO:Publish people properties
    
    //TODO: Track them

    //debug
    // visualize by painting each PC another color
    // pcl::copyPointCloud(*no_ground_cloud, *cloud_filtered);
    PointCloudT::Ptr cloud_filtered;
    cloud_filtered = cloud;
    viz_cloud->points.clear();

    for (std::vector<pcl::PointIndices>::const_iterator 
	   it = cluster_indices.begin (); 
	 it != cluster_indices.end (); ++it){
      uint8_t r(rand()%255), g(rand()%255), b(rand()%255);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); 
	   pit != it->indices.end (); pit++){
	// create RGB point to push in
	PointT new_pt;
	new_pt.x = cloud_filtered->points[*pit].x;
	new_pt.y = cloud_filtered->points[*pit].y;
	new_pt.z = cloud_filtered->points[*pit].z;

	if(!RANDOM_COLORS){
	new_pt.r = cloud_filtered->points[*pit].r;
	new_pt.g = cloud_filtered->points[*pit].g;
	new_pt.b = cloud_filtered->points[*pit].b;
	}
	else{
	new_pt.r = r; new_pt.g = g; new_pt.b = b;
	}
	// new_pt.a = 1.0;
	viz_cloud->points.push_back (new_pt); //*
      }
      //
      
    //std::cout << "PointCloud representing the Cluster: " << viz_cloud->points.size () << " data points." << std::endl;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //j++;

    }

    viz_cloud->width = viz_cloud->points.size ();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    cloud = viz_cloud;

  }
}

void PplTrack::workspace_limit(PointCloudT::Ptr& cloud)
{

  // //debug
  // cout << "Initial points" << cloud->points.size() << endl;
  
  PointCloudT::Ptr cloud_f(new PointCloudT);
  
  //Groundplane removal
  cloud_f->points.clear();
  float ground_z_min = -ground_coeffs_(3);
  
  // //debug
  // cout << "Ground Z max: " << ground_z_min << endl;

  for (PointCloudT::iterator pit = cloud->begin();
       pit!= cloud->end(); ++pit){
    if(!isnan(pit->z))
      if (pit->z>ground_z_min) //Groundplane removal
	if(pit->y>0.84) //Robot table plane clipping
	  if (pit->y>1.0 || pit->z>1.0) //Robot table more precise
	  if(pit->x>0.05) //behind front table clipping
	    if(pit->x<4.3) //back range
	      if(pit->x<2.9 || pit->y>2.0)//back table
		if(pit->y<4.0) // Side range
		  //clipping the front table
		  if(!(pit->x<1.10 && pit->z<0.98 && pit->y<2.3) )
		  {cloud_f->points.push_back(*pit);}
  }

  cloud_f->width = cloud_f->points.size ();
  cloud_f->height = 1;
  cloud_f->is_dense = true;
  
  cloud = cloud_f;
  
  // //debug
  // for (PointCloudT::iterator pit = cloud->begin();
  //      pit!= cloud->end(); ++pit){
  //   if (static_cast<float>(pit->z) < static_cast<float>(ground_z_min))
  //     {cout << " What the F???" << endl;}
  // }
  //Robot table plane
  
  //Back-table block (cuboid)
  
  //Plane behind front table

  return;
}

PplTrack::PplTrack(float z){
  ground_coeffs_ =   Eigen::Vector4f(0.0,0.0,0.0,-z);
  table_link = true;

  max_height_=2.3; min_height_=1.0; max_dist_gr_=0.4;
  max_c_size_=1800; min_c_size_=100;

  history_per_stats_.clear();
}

void PplTrack::robot_remove(PointCloudT::Ptr &cloud,
			    Eigen::Vector3f robo_loc)
{
  // //debug
  // cout << "Points before robot removal = " << cloud->points.size() << endl;

  float a = 0.5;
  // shr_cv_utils::crop_axis_a_cylinder(robo_loc, cloud, a, -1.);
  shr_cv_utils::crop_axis_a_cylinder(cloud, robo_loc, a, -1.);

  // //debug
  // cout << "Points after robot removal = " << cloud->points.size() << endl;

  // cloud = cloud_f;
  // shr_cv_utils::transPoints(cloud, Eigen::Matrix4f::Zero(4,4), cloud);
}

void PplTrack::merge_floor_clusters(const PointCloudT::Ptr cloud, 
				    vector<pcl::PointIndices> &cluster_indices)
{
  vector<pcl::PointIndices> output_indices;

  vector<ClusterStats> clusters_stats;
  get_clusters_stats(cloud, cluster_indices, clusters_stats);

  std::vector <std::vector<int> > connected_clusters;
  connected_clusters.resize(cluster_indices.size());
  std::vector<bool> used_clusters; // 0 in correspondence of clusters
  				   // remained to process, 1 for
  				   // already used clusters
  used_clusters.resize(cluster_indices.size());
  
  // initialize clusters unused
  for(vector<bool>::iterator usit=used_clusters.begin(); usit!=used_clusters.end()
  	;usit++)
    *usit = false;
    
  for(unsigned int i = 0; i < cluster_indices.size(); ++i){ // for every cluster

    //Projection on ground plane is just the x,y coordinates
    Eigen::Vector2f cur_cluster_c = 
      Eigen::Vector2f(clusters_stats[i].mean(0), clusters_stats[i].mean(1));
      
    // for every remaining cluster
    for(unsigned int j = i+1; j < cluster_indices.size(); j++) {
      
      Eigen::Vector2f new_cluster_c =
      Eigen::Vector2f(clusters_stats[j].mean(0), clusters_stats[j].mean(1));

      //TODO: Pass max cluster distance as an argument
      if (((new_cluster_c-cur_cluster_c).norm()) < 0.4){
  	connected_clusters[i].push_back(j);
      }
    }
  }

  //Merge 'em
  output_indices.clear();
  for(unsigned int i = 0; i < connected_clusters.size(); i++){ // for every cluster
    if (!used_clusters[i]){ // if this cluster has not been used yet
      used_clusters[i] = true;
      if (connected_clusters[i].empty()) {// no other clusters to merge
  	output_indices.push_back(cluster_indices[i]);
      }
      else
      {
        // Copy cluster points into new cluster:
        pcl::PointIndices point_indices;
        point_indices = cluster_indices[i];
        for(unsigned int j = 0; j < connected_clusters[i].size(); j++){

  	  //if this cluster has not been used yet
          if (!used_clusters[connected_clusters[i][j]]) {
            used_clusters[connected_clusters[i][j]] = true;
            for(std::vector<int>::const_iterator points_iterator = 
  		  cluster_indices[connected_clusters[i][j]].indices.
  		  begin();
                points_iterator != 
  		  cluster_indices[connected_clusters[i][j]].
  		  indices.end(); points_iterator++)
            {
              point_indices.indices.push_back(*points_iterator);
            }
          }
        }
  	//add to new clusters
  	output_indices.push_back(point_indices);
      }
    }
  }
  
  cluster_indices = output_indices;
}

void PplTrack::get_clusters_stats(PointCloudT::ConstPtr cloud, 
				 const vector<pcl::PointIndices> cluster_indices,
				 vector<ClusterStats>& clusters_stats)
{
  //Get cluster statistics
  clusters_stats.clear();

  //Go through cluster indices and take statistics
  for(vector<pcl::PointIndices>::const_iterator cit = cluster_indices.begin();
      cit != cluster_indices.end(); ++cit){
    accumulator_set< float, stats<tag::mean, tag::variance, 
  				  tag::min, tag::max, 
				  tag::median> > x_acc, y_acc, z_acc; 
    for(vector<int>::const_iterator pint = cit->indices.begin(); 
    	pint!=cit->indices.end(); ++pint){
      PointT p = cloud->points[*pint];
      x_acc(p.x);
      y_acc(p.y);
      z_acc(p.z);
    }
    ClusterStats cluster_stats; //Stats for one cluster
    cluster_stats.mean = ClusterPoint(boost::accumulators::mean(x_acc), boost::accumulators::mean(y_acc), boost::accumulators::mean(z_acc));
    cluster_stats.var = ClusterPoint(variance(x_acc), variance(y_acc), variance(z_acc));
    cluster_stats.min = ClusterPoint(boost::accumulators::min(x_acc), boost::accumulators::min(y_acc), boost::accumulators::min(z_acc));    
    cluster_stats.max = ClusterPoint(boost::accumulators::max(x_acc), boost::accumulators::max(y_acc), boost::accumulators::max(z_acc));    
    cluster_stats.median = ClusterPoint(boost::accumulators::median(x_acc), boost::accumulators::median(y_acc), boost::accumulators::median(z_acc));    

  //   //debug
  //    cout << "Std Devs: (" << sqrt(cluster_stats.var(0)) << ',' <<
  //     sqrt(cluster_stats.var(1)) << ',' << sqrt(cluster_stats.var(2)) << ")" << endl;
  //   cout << "Diff ranges: (" << cluster_stats.max(0)-cluster_stats.min(0) << ',' <<
  //     cluster_stats.max(1)-cluster_stats.min(1) << ',' << cluster_stats.max(2)-cluster_stats.min(2)
  //      << ")" << endl;
  // // string whadupp; cin>>whadupp;

    clusters_stats.push_back(cluster_stats);
  }

}

void PplTrack::rm_clusters_rules(const PointCloudT::Ptr &cloud,
			 vector<pcl::PointIndices>& cs_indices)
{
  vector<ClusterStats> cs_stats;
  vector<pcl::PointIndices> cs_new;

  get_clusters_stats(cloud, cs_indices, cs_stats);

  for (int i=0; i<cs_indices.size(); ++i){
    ClusterStats cur_stats = cs_stats[i];
    
    //check z distance from ground
    if (cur_stats.min(2)<max_dist_gr_){
      //is height in range
      float cur_ht=cur_stats.max(2);
      if(cur_ht>min_height_ && cur_ht<max_height_){
	//cluster size in range
	int no_pts = cs_indices[i].indices.size();
	if (no_pts>min_c_size_ && no_pts<max_c_size_)
	  {	
	    cs_new.push_back(cs_indices[i]);}
      }
    }
  }
  cs_indices = cs_new;
}

void PplTrack::assign_ppl_clusters(PointCloudT::ConstPtr cloud, 
				   const vector<pcl::PointIndices> cluster_indices)
{
  per_cs_.clear();
  for(vector<pcl::PointIndices>::const_iterator cit = cluster_indices.begin();
      cit != cluster_indices.end(); ++cit){
    vector<ClusterPoint> cur_c;
    for(vector<int>::const_iterator pint = cit->indices.begin(); 
    	pint!=cit->indices.end(); ++pint){
      PointT p = cloud->points[*pint];
      ClusterPoint cur_pt(p.x, p.y,p.z);
      cur_c.push_back(cur_pt);
    }
    per_cs_.push_back(cur_c);
  }

  //get statistics
  get_clusters_stats(cloud, cluster_indices, per_stats_);
}

void PplTrack::reset_vals()
{
  person_id_ = -1;
  cur_cloud_ = PointCloudT::Ptr(new PointCloudT);
  per_cs_.clear();
  per_stats_.clear();
  cur_pos_.clear();
  more_than_one_ = false;
}
