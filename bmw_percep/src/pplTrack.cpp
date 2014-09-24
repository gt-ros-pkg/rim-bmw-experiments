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
  clear_history();
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
  inn_cyl_marker.header.stamp = pub_time_;

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
  // float scale_x = (std::max(std::fabs(per_stats.max(0)-out_cyl_marker.scale.x),
  // 			    std::fabs(-per_stats.min(0)+out_cyl_marker.scale.x)));
  // float scale_y = (std::max(std::fabs(per_stats.max(1)-out_cyl_marker.scale.y),
  // 			    std::fabs(-per_stats.min(1)+out_cyl_marker.scale.y)));
  float scale_x = (std::max(std::fabs(per_stats.max(0)-out_cyl_marker.pose.position.x),
  			    std::fabs(-per_stats.min(0)+out_cyl_marker.pose.position.x)));
  float scale_y = (std::max(std::fabs(per_stats.max(1)-out_cyl_marker.pose.position.y),
  			    std::fabs(-per_stats.min(1)+out_cyl_marker.pose.position.y)));

  out_cyl_marker.scale.x = 2*scale_x; //diameter
  out_cyl_marker.scale.y = 2*scale_y;
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


void PplTrack::visualize(ros::Publisher pub, Eigen::Vector3f color, PersProp person, string name_space)
{
  if(!(isnan(person.pos(0)) && isnan(person.pos(1)))){
    //markers object
    visualization_msgs::MarkerArray mark_arr;
    mark_arr.markers.clear();

    //Inner-cylinder marker
    visualization_msgs::Marker inn_cyl_marker;
    // Set the frame ID and timestamp.  
    inn_cyl_marker.header.frame_id = viz_frame_;
    inn_cyl_marker.header.stamp = pub_time_;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    inn_cyl_marker.ns = name_space;
    //TODO: use enum and not these numbers
    inn_cyl_marker.id = 0;

    // Set the marker type to Cylinder
    uint32_t cylinder = visualization_msgs::Marker::CYLINDER;
    inn_cyl_marker.type = cylinder;

    // Set the inn_cyl_marker action
    inn_cyl_marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the inn_cyl_marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    inn_cyl_marker.pose.position.x = person.pos(0);
    inn_cyl_marker.pose.position.y = person.pos(1);
    inn_cyl_marker.pose.position.z = person.height/2;

    //the cylinder is oriented along z
    inn_cyl_marker.pose.orientation.x = 0.0;
    inn_cyl_marker.pose.orientation.y = 0.0;
    inn_cyl_marker.pose.orientation.z = 0.0;
    inn_cyl_marker.pose.orientation.w = 1.0;

    // Set the scale of the inn_cyl_marker -- 1x1x1 here means 1m on a side
    inn_cyl_marker.scale.x = .5;//person.inn_cyl(0);
    inn_cyl_marker.scale.y = .5;//person.inn_cyl(1);
    inn_cyl_marker.scale.z = person.height; // only distance from ground

    // Set the color -- be sure to set alpha to something non-zero!
    inn_cyl_marker.color.r = color(0);
    inn_cyl_marker.color.g = color(1);
    inn_cyl_marker.color.b = color(2);
    inn_cyl_marker.color.a = 0.5;

    inn_cyl_marker.lifetime = ros::Duration();
    mark_arr.markers.push_back(inn_cyl_marker);

    //add the outer human cylinder
    visualization_msgs::Marker out_cyl_marker;
    out_cyl_marker = inn_cyl_marker;

    out_cyl_marker.id = 1;

    // Set the scale of the out_cyl_marker -- 1x1x1 here means 1m on a side
    out_cyl_marker.scale.x = person.out_cyl(0);
    out_cyl_marker.scale.y = person.out_cyl(1);
    //the z-scale remains the same

    // Set the color -- be sure to set alpha to something non-zero!
    out_cyl_marker.color.r = color(0);
    out_cyl_marker.color.g = color(1);
    out_cyl_marker.color.b = color(2);
    out_cyl_marker.color.a = 0.5;

    mark_arr.markers.push_back(out_cyl_marker);
  

    //visualize velocity if present
    if(!isnan(person.vel(0))){
      double frame_rate = 10.;
      double delta_t = 1 / frame_rate; // 1 secs
      double vel_mag = person.vel.norm();
      double mark_scale = delta_t * vel_mag;
      double vel_scale = 0.6;
      
      visualization_msgs::Marker vel_marker1, vel_marker2, vel_marker3;
      //velocity markers
      vel_marker1 = inn_cyl_marker;
      vel_marker1.id += 5;
      vel_marker1.pose.position.z = 0;

      vel_marker1.pose.position.x = person.pos(0)+ delta_t * person.vel(0);
      vel_marker1.pose.position.y = person.pos(1)+ delta_t * person.vel(1);

      vel_marker1.scale.x = .5* vel_scale + delta_t * vel_mag + mark_scale;
      vel_marker1.scale.y = .5*vel_scale + delta_t * vel_mag + mark_scale;
      vel_marker1.scale.z = 0.01;
    
      vel_marker1.color.a = 0.5f;
      vel_marker1.color.r = 0.75f;
      vel_marker1.color.b = 0.f;
      vel_marker1.color.g = 1.0f;

      vel_marker2 = vel_marker1;
      vel_marker3 = vel_marker2;

      vel_marker2.id += 2;
      vel_marker3.id += 3;
  
      vel_marker2.color.a *= (.5);
      vel_marker3.color.a *= pow((.5),2);

      vel_marker2.pose.position.x += delta_t * person.vel(0);
      vel_marker2.pose.position.y += delta_t * person.vel(1);

    vel_marker3.pose.position.x += 2 * delta_t * person.vel(0);
    vel_marker3.pose.position.y += 2 * delta_t * person.vel(1);

    vel_marker2.scale.x += mark_scale + vel_scale;
    vel_marker2.scale.y += mark_scale + vel_scale;

    vel_marker3.scale.x += 2 * mark_scale + 2*vel_scale;
    vel_marker3.scale.y += 2 * mark_scale + 2*vel_scale;

    mark_arr.markers.push_back(vel_marker1);
    mark_arr.markers.push_back(vel_marker2);
    mark_arr.markers.push_back(vel_marker3);
    }

    pub.publish(mark_arr);

  }
  else{
    //No Publish if no observation
    //TODO: delete the message
  }
}

void PplTrack::estimate(PointCloudT::Ptr& cloud, 
			vector<vector<ClusterPoint> > &clusters,
			const Eigen::Vector3f robo_loc,
			bool got_tf_robot, ros::Time pc_time,
			float leaf_size/*=0.06*/)
{
  reset_vals();
  
  pub_time_ = pc_time;
  
  if (!table_link){
    cout << "Everything expected in Table link frames. PROBLEM PROBLEM PROBLEM!!"
	 << endl;
  }
  else{
    
    // human_tracker_.set_image(cloud);
    PointCloudT::Ptr viz_cloud(new PointCloudT);

    workspace_limit(cloud);
    if (got_tf_robot) // in case the robots location is known
      robot_remove(cloud, robo_loc);

    pcl::copyPointCloud(*cloud, *viz_cloud);

    // pcl::RandomSample<PointT> ransam;
    // ransam.setInputCloud(viz_cloud);
    // ransam.setSample(1000);
    // ransam.setSeed(rand());
    // ransam.filter(*cloud);
    
    float voxel_size=0.03;

    pcl::copyPointCloud(*cloud, *viz_cloud);

    cout << "No. of points before voxel = " << cloud->size() << endl;
    boost::timer voxel_timer;
    //Voxelize the space
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    //PointCloudSM::Ptr cloud_filtered(new PointCloudSM);
    vg.setInputCloud(viz_cloud);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*cloud);
    
    // cout << "Time to Voxel = " << voxel_timer.elapsed() << endl;

    // cout << "No. of points = " << cloud->size() << endl;
    
    // Creating the KdTree object for the search method of the extraction
    boost::timer kd;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    // cout << "KD Tree takes = " << kd.elapsed() << "s" << endl;
     
    boost::timer clusterize;
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (2* leaf_size); // 2cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (5000);
    // ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // cout << "Clustering takes = " << clusterize.elapsed() << "s" << endl;

    //merge
    merge_floor_clusters(cloud, cluster_indices);

    //remove clusters acc to rules
    rm_clusters_rules(cloud, cluster_indices);

    //assign person clusters to the private member
    assign_ppl_clusters(cloud, cluster_indices);

    //set the observation now that processing is done!
    set_observation();

    if (per_cs_.size()>1)
      more_than_one_=true;
    else
      more_than_one_=false;
    
    //TODO: May be publish the timestamp of PointCloud instead

    //TODO:Publish people properties
    
    //Track them
    if (person_id_>-1){ //current observation=true
      if (history_per_stats_.size()>0){
	if (!isnan(history_per_stats_.front().pos(0))){
	  if (currently_filtering_){
	    float dt = static_cast<float>((pub_time_-prev_time_).toSec());
	    Eigen::Matrix<float, 6, 1> kf_est;
	    kf_tracker_.estimate(Eigen::Vector2f(pers_obs_.pos(0), 
						 pers_obs_.pos(1)), dt, kf_est);

	    pers_est_.pos = Eigen::Vector2f(kf_est(0), kf_est(1));
	    pers_est_.vel = Eigen::Vector2f(kf_est(2), kf_est(3));

	    //write to disk
	    // write_clusters_disk();
	  }
	  else{

	    cv::Point2f prev_pos = cv::Point2f(history_per_stats_.front().pos(0),
					       history_per_stats_.front().pos(1));
	    //kalman
	    Eigen::Vector2f jerk_std(0.8,0.8);
	    Eigen::Vector2f measur_std(0.05,0.05);
	    float delta_t = 1./15.;
	    Eigen::Matrix<float, 6, 1> x_k1;
	    x_k1.fill(0.);
	    x_k1(0,0)=pers_obs_.pos(0); x_k1(1,0)=pers_obs_.pos(1);
	    Eigen::Matrix<float, 6, 6> init_cov 
	      = 10000*Eigen::MatrixXf::Identity(6,6);
	    
	    kf_tracker_.reinitialize(jerk_std, measur_std, 
				     delta_t, x_k1, init_cov);

	    Eigen::Matrix<float, 6, 1> kf_est;
	    kf_tracker_.estimate(Eigen::Vector2f(pers_obs_.pos(0), 
						 pers_obs_.pos(1)), 
				 delta_t, kf_est);
	    
	    pers_est_.pos = Eigen::Vector2f(kf_est(0), kf_est(1));
	    pers_est_.vel = Eigen::Vector2f(kf_est(2), kf_est(3));

	    currently_filtering_ = true;
	  }
	}
	else{ // no observation -- prev frame

	  currently_filtering_ = false;
	  pers_est_.pos = pers_obs_.pos;
	  pers_est_.vel = Eigen::Vector2f(numeric_limits<float>::quiet_NaN(),
					  numeric_limits<float>::quiet_NaN());
	}
      }
    }
    else{ //no observation
      
      // cout<< "Woah, no observation. " << endl;
      // string whut; cin>>whut;
      
      currently_filtering_ = false;
      pers_est_.pos = Eigen::Vector2f(numeric_limits<float>::quiet_NaN(),
				     numeric_limits<float>::quiet_NaN());
      pers_est_.vel = Eigen::Vector2f(numeric_limits<float>::quiet_NaN(),
				     numeric_limits<float>::quiet_NaN());
    }
    
    //set estimate now that filtering is done
    set_estimate();
    
    //store history
    if (history_per_stats_.size() > history_size_-1)
      {
	history_per_stats_.pop();
      }
    history_per_stats_.push(pers_est_);

    //remember previous time
    prev_time_ = pub_time_;

    // visualize by painting each PC another color
    PointCloudT::Ptr cloud_filtered;
    cloud_filtered = cloud;
    viz_cloud->points.clear();

    for (vector<pcl::PointIndices>::const_iterator 
	   it = cluster_indices.begin (); 
	 it != cluster_indices.end (); ++it){
      uint8_t r(rand()%255), g(rand()%255), b(rand()%255);

      for (vector<int>::const_iterator pit = it->indices.begin (); 
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
  max_c_size_=18000; min_c_size_=100;
  file_no_=0;

  ws_min_ = Eigen::Vector3f(.05, .84, -ground_coeffs_(3));
  ws_max_ = Eigen::Vector3f(4.3, 4.0, 5.0);
  // human_tracker_.set_range_gran(Eigen::Vector2f(ws_min_(0), ws_min_(1)), 
  // 				Eigen::Vector2f(ws_max_(0), ws_max_(1)),
  // 				0.06);

  history_size_ = 10;
  currently_filtering_ = false;
  clear_history();
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


//TODO: Get these statistics yourself, no BOOST
void PplTrack::get_clusters_stats(PointCloudT::ConstPtr cloud, 
				 const vector<pcl::PointIndices> cluster_indices,
				 vector<ClusterStats>& clusters_stats)
{
  //Get cluster statistics
  clusters_stats.clear();


  //Go through cluster indices and take statistics
  for(vector<pcl::PointIndices>::const_iterator cit = cluster_indices.begin();
      cit != cluster_indices.end(); ++cit){
    float t_max=-100.0, t_min=100.0;

    accumulator_set< float, stats<tag::mean, tag::variance, 
  				  tag::min, tag::max, 
				  tag::median> > x_acc, y_acc, z_acc; 
    for(vector<int>::const_iterator pint = cit->indices.begin(); 
    	pint!=cit->indices.end(); ++pint){
      PointT p = cloud->points[*pint];
      x_acc(p.x);
      y_acc(p.y);
      z_acc(p.z);
      
      //debug
      if(t_min>p.x)
	t_min = p.x;
      if(t_max<p.x)
	t_max = p.x;

      
    }

    ClusterStats cluster_stats; //Stats for one cluster
    cluster_stats.mean = ClusterPoint(boost::accumulators::mean(x_acc), boost::accumulators::mean(y_acc), boost::accumulators::mean(z_acc));
    cluster_stats.var = ClusterPoint(variance(x_acc), variance(y_acc), variance(z_acc));
    cluster_stats.min = ClusterPoint(boost::accumulators::min(x_acc), boost::accumulators::min(y_acc), boost::accumulators::min(z_acc));    
    cluster_stats.max = ClusterPoint(boost::accumulators::max(x_acc), boost::accumulators::max(y_acc), boost::accumulators::max(z_acc));    
    cluster_stats.median = ClusterPoint(boost::accumulators::median(x_acc), boost::accumulators::median(y_acc), boost::accumulators::median(z_acc));    

    // //debug
    // if ((fabs(t_min-boost::accumulators::min(x_acc))>0.0001) || 
    // 	(fabs(t_max-boost::accumulators::max(x_acc))>0.0001)){
    // cout << "Accumulators : " << boost::accumulators::max(x_acc) << ", " << boost::accumulators::min(x_acc) << endl;
    // cout << "ClusterStats : " << cluster_stats.max(0) << ", " << cluster_stats.min(0) << endl;
    // cout << "Mine : " << t_max << ", " << t_min << endl;
    // string what; cin>>what;
    // }

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

void PplTrack::write_clusters_disk()
{
  //do something
  system("mkdir data/temp-clusters/");
  string path = "data/temp-clusters/";
  ofstream pts_file;
  string file_ext=".txt";
  string file_name = path + boost::lexical_cast<string>(file_no_) + file_ext;
  file_no_++;


  // pts_file.open(file_name.c_str());

  // for(vector<vector<ClusterPoint> >::iterator cit=per_cs_.begin(); 
  //     cit!=per_cs_.end(); ++cit ){
  //   for(vector<ClusterPoint>::iterator pit=cit->begin(); pit!=cit->end(); ++pit){
  //     ClusterPoint p = (*pit);
  //     pts_file << p(0) << ',' << p(1) << endl;
  //    }
  // }
  
  // pts_file.close();
  
  //Centroid File
  ofstream cen_file;
  float dt = static_cast<float>((pub_time_-prev_time_).toSec());
  string fn_centroid = path+"median"+file_ext;
  cen_file.open(fn_centroid.c_str(), ios::app);
  cen_file << pers_obs_.pos(0) << ',' << pers_obs_.pos(1) << ',' << 
    pers_est_.pos(0) << ',' << pers_est_.pos(1) << ',' << pers_est_.vel(0)
	   << ',' <<  pers_est_.vel(1) << ',' <<  dt << endl;
  cen_file.close();
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

void PplTrack::clear_history()
{
  queue<PersProp> empty_hist;
  swap(history_per_stats_, empty_hist);
}

void PplTrack::set_observation()
{
  //this is our guy/gal
  person_id_ = getOneCluster(per_cs_);

  pers_obs_.vel =   Eigen::Vector2f(numeric_limits<float>::quiet_NaN(),
				   numeric_limits<float>::quiet_NaN());

  //set the position
  if(person_id_>-1){ // in case observation made
    pers_obs_.pos = Eigen::Vector2f(per_stats_[person_id_].median(0),
    				    per_stats_[person_id_].median(1));

    get_head_center(person_id_, pers_obs_.pos);
    
    //debug 
    // cout << " Head Center" << pers_obs_.pos << endl;
    
    //assumption- one person only
    ClusterStats per_stats = per_stats_[person_id_];
    pers_obs_.inn_cyl = Eigen::Vector2f(sqrt(per_stats.var(0))*2.0, 
					sqrt(per_stats.var(1))*2.0);
    pers_obs_.height = (per_stats.max(2)); // only distance from ground
    
    float s_x = 2*std::max(std::fabs(pers_obs_.pos(0)-
				     per_stats.min(0)), 
			   std::fabs(-pers_obs_.pos(0)+
				     per_stats.max(0)));
    float s_y = 2*std::max(std::fabs(pers_obs_.pos(1)-
				     per_stats.min(1)), 
			   std::fabs(-pers_obs_.pos(1)+
				     per_stats.max(1)));
    pers_obs_.out_cyl = Eigen::Vector2f(s_x,s_y);
  }
  else{
    pers_obs_.pos = Eigen::Vector2f(numeric_limits<float>::quiet_NaN(), 
				    numeric_limits<float>::quiet_NaN());
  }
}

void PplTrack::set_estimate()
{
  // pers_est_.pos = Eigen::Vector2f(per_stats_[person_id_].median(0),
  // 				  per_stats_[person_id_].median(1));

  //assumption- one person only
  pers_est_.inn_cyl = pers_obs_.inn_cyl;
  pers_est_.out_cyl = pers_obs_.out_cyl;
  pers_est_.height = pers_obs_.height;
  
}

void PplTrack::get_head_center(int c_ind, Eigen::Vector2f &h_center)
{
  vector<vector <HMapEl> > hmap;
  float dist_from_top = 0.2;
  float head_span = 1.;
  int tbx=50, tby=50;
  // float gran = 0.05; // granularity

  Eigen::Vector2f xlims(per_stats_[c_ind].min(0), per_stats_[c_ind].max(0));
  Eigen::Vector2f ylims(per_stats_[c_ind].min(1), per_stats_[c_ind].max(1));

  float granx = (xlims(1)-xlims(0))/static_cast<float>(tbx) ;
  float grany = (ylims(1)-ylims(0))/static_cast<float>(tby) ;

  // float delta = gran/5.;

  // int total_bins_x = ceil((xlims(1) - xlims(0))/gran);
  // int total_bins_y = ceil((ylims(1) - ylims(0) )/gran);
  // int tbx=total_bins_x, tby=total_bins_y;

  hmap.resize(tby);

  //initialize map
  for(size_t i=0; i<hmap.size(); ++i){
    hmap[i].resize(tbx);
    for(size_t j=0; j<hmap[i].size(); ++j){
      hmap[i][j].loc = Eigen::Vector2f(granx * static_cast<float>(j) + xlims(0), 
				       grany * static_cast<float>(i) + ylims(0));
      hmap[i][j].height = 0.0;
      hmap[i][j].in_h_range = false;
      hmap[i][j].p_ind.clear();
    }
  }

  float max_ht = 0.0;
  Index2D max_ht_ind;

  //update map with cluster points
  for(size_t i=0; i<per_cs_[c_ind].size(); ++i){
    ClusterPoint t_cl = per_cs_[c_ind][i];
    size_t h_i_x = floor((t_cl(0)-xlims(0)) / granx);
    size_t h_i_y = floor((t_cl(1)-ylims(0)) / grany);
    if (h_i_x<0 || h_i_y<0)
      {cout << "My height map index negative " << endl;
	string whut; cin>>whut;}
    if (h_i_x>tbx-1)
      h_i_x = tbx-1;
    if (h_i_y>tby-1)
      h_i_y = tby-1;

    //push point into height map square
    hmap[h_i_y][h_i_x].p_ind.push_back(Eigen::Vector2f(t_cl(0), t_cl(1)));

    if (hmap[h_i_y][h_i_x].height<t_cl(2)){
      hmap[h_i_y][h_i_x].height = t_cl(2);
    }

    if (max_ht < t_cl(2)) {
      max_ht = t_cl(2); 
      max_ht_ind(0) = h_i_x;
      max_ht_ind(1) = h_i_y;
    }
  }

  //set flags of pixels that are in range to true
  for(size_t i=0; i<hmap.size(); ++i){
    for(size_t j=0; j<hmap[i].size(); ++j){
      if (hmap[i][j].height > (max_ht-dist_from_top)){
	hmap[i][j].in_h_range = true;
      }
    }
  }


  //find the cluster with points in range
  int h_x_span = floor(head_span/granx); int h_y_span = floor(head_span/grany);
  vector<Index2D> head_ind;

  // //debug-- check if hmap alright
  // bool h_checks=true;
  // if(hmap.size()!= 20){
  //   cout << "Hmap Ysize = " << hmap.size() << endl;
  //   h_checks=false;
  // }
  // for(int i=0; i<hmap.size(); ++i){
  //   if(hmap[i].size()!=20){
  //     cout << "Hmap Xsize = " << hmap[i].size() << endl;
  //     h_checks=false;
  //   }
  // }
  
  // if(h_checks)
  //   cout << "HMap checks Out!!!!!!" << endl;

  cluster_head(hmap, max_ht_ind(0), max_ht_ind(1), h_x_span, h_y_span, head_ind);


  //TODO: Check if any clusters are being repeated
  //get the centroid from connected clusters
  Eigen::Vector2f temp_loc(0.,0.);
  Eigen::Vector2f temp;
  accumulator_set< float, stats<tag::mean> > x_acc, y_acc;
  // cout << "**********HEAD INDICES**********" << endl;
  // cout << "MAXES " << tbx << ',' << tby << endl;
  int n_bins=0;
  for(size_t i=0; i<head_ind.size(); i++){
    // cout << "(" << head_ind[i](0) << ',' << head_ind[i](1) << ')' << endl;
  // vector<Eigen::Vector2f> cur_pts = hmap[head_ind[i](1)][head_ind[i](0)].p_ind;
    size_t tmpx = static_cast<size_t>(head_ind[i](0));
    size_t tmpy = static_cast<size_t>(head_ind[i](1));
    
    // cout << "(" << hmap[tmpy][tmpx].loc(0) << ',' << hmap[tmpy][tmpx].loc(1) << endl;
    
    temp_loc += hmap[tmpy][tmpx].loc;
    n_bins++;

    for(size_t j=0;
    	j < hmap[tmpy][tmpx].p_ind.size();
    	++j){
      temp = hmap[tmpy][tmpx].p_ind[j];
      x_acc(temp(0));
      y_acc(temp(1));
    }
  }

  // cout << "After even this..." << endl;
  
  h_center = Eigen::Vector2f(mean(x_acc), mean(y_acc));
  // cout << "Locaion= ("<<temp_loc(0) << ',' << temp_loc(1)<< '). Nbins='
  //      << n_bins << endl;
  h_center = temp_loc/static_cast<float>(n_bins);
  // cout << "Head center " << h_center(0) << ','<<h_center(1) << endl;
}

void PplTrack::cluster_head(vector<vector<HMapEl> > hmap, size_t x, size_t y, 
			    int h_x_span, int h_y_span, 
			    vector<Index2D> &i_list)
{
  int cur_x, cur_y;
  for(int i=-h_y_span; i<=h_y_span; ++i){
    cur_y = y + i;
    if (cur_y>=0 && cur_y<hmap.size()){

      for(int j=-h_y_span; j<=h_y_span; ++j){
	cur_x = x + j;
	if (cur_x>=0 && cur_x<hmap[cur_y].size()){
	  if (hmap[cur_y][cur_x].in_h_range){
	    i_list.push_back(Index2D(static_cast<int>(cur_x), 
				     static_cast<int>(cur_y)));
	    //TODO: Recurse smartly
	    // vector<Index2D> temp_list;
	    // temp_list.push_back(Index2D(cur_x, cur_y));
	    // cluster_head(hmap, static_cast<size_t>(cur_x), 
	    // 		 static_cast<size_t>(cur_y), temp_list);
	    
	  }
	}
      }
    }
  }
}
