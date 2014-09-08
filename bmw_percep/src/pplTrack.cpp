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

//Only points to the person's position yet
//Also publishes a cylinder enclosing the trunk
void PplTrack::visualize(ros::Publisher pub)
{
  visualization_msgs::MarkerArray mark_arr;

  mark_arr.markers.clear();
  for(vector<ClusterPoint>::iterator cpit=cur_pos_.begin(); 
      cpit!=cur_pos_.end(); ++cpit){
    ClusterPoint per_pos = *cpit;
    visualization_msgs::Marker pos_marker;
    // Set the frame ID and timestamp.  
    pos_marker.header.frame_id = viz_frame_;
  
    //debug
    cout << "Frame is " << viz_frame_ << endl;
    pos_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    pos_marker.ns = "human";
    pos_marker.id = 0;

    // Set the marker type to Cylinder
    uint32_t cylinder = visualization_msgs::Marker::CYLINDER;
    uint32_t sphere = visualization_msgs::Marker::SPHERE;
    pos_marker.type = cylinder;

    // Set the pos_marker action
    pos_marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the pos_marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  pos_marker.pose.position.x = per_pos(0);
  pos_marker.pose.position.y = per_pos(1);
  pos_marker.pose.position.z = per_pos(2);
  pos_marker.pose.orientation.x = 0.0;
  pos_marker.pose.orientation.y = 0.0;
  pos_marker.pose.orientation.z = 0.0;
  pos_marker.pose.orientation.w = 1.0;
  
  // Set the scale of the pos_marker -- 1x1x1 here means 1m on a side
  pos_marker.scale.x = 0.6;
  pos_marker.scale.y = 0.6;
  pos_marker.scale.z = 0.6;
  
  // Set the color -- be sure to set alpha to something non-zero!
  pos_marker.color.r = 1.0f;
  pos_marker.color.g = 0.0f;
  pos_marker.color.b = 0.0f;
  pos_marker.color.a = 1.0;

  // pos_marker.pose.position.z += pos_marker.scale.z/2;
  
  pos_marker.lifetime = ros::Duration();


  mark_arr.markers.push_back(pos_marker);
  }
  pub.publish(mark_arr);
}

void PplTrack::estimate(PointCloudT::Ptr& cloud, 
			vector<vector<ClusterPoint> > &clusters,
			const Eigen::VectorXf ground_coeffs,
			float leaf_size/*=0.06*/)
{
  if (!table_link){
    PointCloudT::Ptr viz_cloud(new PointCloudT);
    //workspace_limit(cloud);
    ppl_detection::find_euclid_blobs(cloud, viz_cloud,  
				     clusters,
				     ground_coeffs, leaf_size);
    estimate(clusters);
  }
  else{
    
    workspace_limit(cloud);

    int max_cluster_size = 800;
    int min_cluster_size = 100;
    
  }
}

void PplTrack::workspace_limit(PointCloudT::Ptr& cloud)
{
  //debug
  cout << "Initial points" << cloud->points.size() << endl;
  
  PointCloudT::Ptr cloud_f(new PointCloudT);
  
  //Groundplane removal
  cloud_f->points.clear();
  float ground_z_min = -ground_coeffs_(3);
  
  //debug
  cout << "Ground Z max: " << ground_z_min << endl;
  //
  for (PointCloudT::iterator pit = cloud->begin();
       pit!= cloud->end(); ++pit){
    if(!isnan(pit->z))
      if (pit->z>ground_z_min) //Groundplane removal
	if(pit->y>0.84) //Robot table clipping
	  if(pit->x>0.05) //front table clipping
	    if(pit->x<4.3) //back range
	      if(pit->x<2.9 || pit->y>2.0)//back table
		if(pit->y<4.0) // Side range
		  {cloud_f->points.push_back(*pit);}
  }

  cloud_f->width = cloud_f->points.size ();
  cloud_f->height = 1;
  cloud_f->is_dense = true;
  
  cloud = cloud_f;
  
  //Robot table plane
  
  //Back-table block (cuboid)
  
  //Plane behind front table

  return;
}

PplTrack::PplTrack(float z){
  ground_coeffs_ =   Eigen::Vector4f(0.0,0.0,0.0,-z);
  table_link = true;
}
