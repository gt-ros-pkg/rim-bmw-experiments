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
  ground_coeffs_ = ground_coeffs;
}

void PplTrack::estimate(vector<vector<ClusterPoint> > clusters)
{
  //ASSUMPTION: Max one person in the scene
  int n_clusters = clusters.size();
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

  cur_pos = centroid;
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
