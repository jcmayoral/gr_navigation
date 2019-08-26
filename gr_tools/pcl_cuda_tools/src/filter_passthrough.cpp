/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:$
 * @author: Koen Buys
 */

#include <pcl_cuda_tools/filters/filter_passthrough.h>
#include <pcl_cuda_tools/cuda_filter_functions.cuh>

//using namespace pcl::gpu;
//using namespace pcl;
using namespace pcl_gpu;

double FilterPassThrough::do_stuff (pcl::PointCloud<pcl::PointXYZ>  &input_cloud){
  float * x, *y, *z;
  bool *b;
  int number_points = input_cloud.points.size();
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_pointer;
  //pc_pointer = input_cloud.makeShared();//boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(input_cloud);

  x = static_cast<float*>(malloc(sizeof(float) * number_points));
  y = static_cast<float*>(malloc(sizeof(float) * number_points));
  z = static_cast<float*>(malloc(sizeof(float) * number_points));
  b = static_cast<bool*>(malloc(sizeof(bool) * number_points));

  int removed_points = 0;

  for (auto i=0; i< number_points; i++){
    x[i] = static_cast<float>(input_cloud.points[i].x);
    y[i] = static_cast<float>(input_cloud.points[i].y);
    z[i] = static_cast<float>(input_cloud.points[i].z);
  }
  memset(b, false, number_points);


  auto result = apply_cuda_filter(z,b, minimum_value_, maximum_value_, filter_value_,  number_points);

  input_cloud.points.clear();

  //TODO I SHOULD FIND A FASTER WAY
  for (int j=0; j< number_points; j++){
    if (b[j]){
      removed_points++;
      continue;
    }
    /*
    if (std::isnan(x[i]) || std::isnan(y[i]) || std::isnan(z[i])){
      removed_points++;
      continue;
    }*/
    pcl::PointXYZ point;
    point.x = static_cast<float>(x[j]);
    point.y = static_cast<float>(y[j]);
    point.z = static_cast<float>(z[j]);
    //std::cout << point.x << ","<< point.y << "," << point.z << std::endl;
    input_cloud.points.push_back(point);
    //host_cloud_->points[i].x = x[i];
    //host_cloud_->points[i].y = y[i];
    //host_cloud_->points[i].z = z[i];
  }


  /*
  for (auto it = input_cloud.fields.begin(); it!= input_cloud.fields.end(); it++){
    *(it).datatype = 8;
  }*/

  std::cout << "REMOVING "<< removed_points << std::endl;
  input_cloud.width = input_cloud.points.size();
  //input_cloud = *pc_pointer;

  return result;
}



void FilterPassThrough::applyFilter (const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >  &host_cloud_,
                  const pcl::gpu::Octree::Ptr                               &tree,
                  float                                                     tolerance,
                  std::vector<PointIndices>                                 &clusters,
                  unsigned int                                              min_pts_per_cluster,
                  unsigned int                                              max_pts_per_cluster)
{

  // Create a bool vector of processed point indices, and initialize it to false
  // cloud is a DeviceArray<PointType>
  std::vector<bool> processed (host_cloud_->points.size (), false);

  int max_answers;

  if(max_pts_per_cluster > host_cloud_->points.size())
    max_answers = host_cloud_->points.size();
  else
    max_answers = max_pts_per_cluster;
  //std::cout << "Max_answers: " << max_answers << std::endl;

  // to store the current cluster
  pcl::PointIndices r;

  DeviceArray<PointXYZ> queries_device_buffer;
  queries_device_buffer.create(max_answers);

  // Process all points in the cloud
  for (size_t i = 0; i < host_cloud_->points.size (); ++i)
  {
    // if we already processed this point continue with the next one
    if (processed[i])
      continue;
    // now we will process this point
    processed[i] = true;

    // Create the query queue on the device, point based not indices
    pcl::gpu::Octree::Queries queries_device;
    // Create the query queue on the host
	pcl::PointCloud<pcl::PointXYZ>::VectorType queries_host;
    // Push the starting point in the vector
    queries_host.push_back (host_cloud_->points[i]);
    // Clear vector
    r.indices.clear();
    // Push the starting point in
    r.indices.push_back(i);

    unsigned int found_points = queries_host.size ();
    unsigned int previous_found_points = 0;

    pcl::gpu::NeighborIndices result_device;

    // once the area stop growing, stop also iterating.
    do
    {
      // Host buffer for results
      std::vector<int> sizes, data;

      // if the number of queries is not high enough implement search on Host here
      if(queries_host.size () <= 10) ///@todo: adjust this to a variable number settable with method
      {
        //std::cout << " CPU: ";
        for(size_t p = 0; p < queries_host.size (); p++)
        {
          // Execute the radiusSearch on the host
          tree->radiusSearchHost(queries_host[p], tolerance, data, max_answers);
        }
        // Store the previously found number of points
        previous_found_points = found_points;
        // Clear queries list
        queries_host.clear();

        //std::unique(data.begin(), data.end());
        if(data.size () == 1)
          continue;

        // Process the results
        for(size_t i = 0; i < data.size (); i++)
        {
          if(processed[data[i]])
            continue;
          processed[data[i]] = true;
          queries_host.push_back (host_cloud_->points[data[i]]);
          found_points++;
          r.indices.push_back(data[i]);
        }
      }

      // If number of queries is high enough do it here
      else
      {
        //std::cout << " GPU: ";
        // Copy buffer
        queries_device = DeviceArray<PointXYZ>(queries_device_buffer.ptr(),queries_host.size());
        // Move queries to GPU
        queries_device.upload(queries_host);
        // Execute search
        tree->radiusSearch(queries_device, tolerance, max_answers, result_device);
        // Copy results from GPU to Host
        result_device.sizes.download (sizes);
        result_device.data.download (data);
        // Store the previously found number of points
        previous_found_points = found_points;
        // Clear queries list
        queries_host.clear();
        for(size_t qp = 0; qp < sizes.size (); qp++)
        {
          for(int qp_r = 0; qp_r < sizes[qp]; qp_r++)
          {
            if(processed[data[qp_r + qp * max_answers]])
              continue;
            processed[data[qp_r + qp * max_answers]] = true;
            queries_host.push_back (host_cloud_->points[data[qp_r + qp * max_answers]]);
            found_points++;
            r.indices.push_back(data[qp_r + qp * max_answers]);
          }
        }
      }
      //std::cout << " data.size: " << data.size() << " foundpoints: " << found_points << " previous: " << previous_found_points;
      //std::cout << " new points: " << found_points - previous_found_points << " next queries size: " << queries_host.size() << std::endl;
    }
    while (previous_found_points < found_points);
    // If this queue is satisfactory, add to the clusters
    if (found_points >= min_pts_per_cluster && found_points <= max_pts_per_cluster)
    {
      std::sort (r.indices.begin (), r.indices.end ());
      // @todo: check if the following is actually still needed
      //r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = host_cloud_->header;
      clusters.push_back (r);   // We could avoid a copy by working directly in the vector
    }
  }
}


//@ deprecated
void FilterPassThrough::extract (std::vector<pcl::PointIndices> &clusters)
{
/*
  // Initialize the GPU search tree
  if (!tree_)
  {
    tree_.reset (new pcl::gpu::Octree());
    ///@todo what do we do if input isn't a PointXYZ cloud?
    tree_.setCloud(input_);
  }
*/
  if (!tree_->isBuilt())
  {
    tree_->build();
  }
/*
  if(tree_->cloud_.size() != host_cloud.points.size ())
  {
    PCL_ERROR("[pcl::gpu::EuclideanClusterExtraction] size of host cloud and device cloud don't match!\n");
    return;
  }
*/
  // Extract the actual clusters
  //applyFilter (host_cloud_, tree_, cluster_tolerance_, clusters, min_pts_per_cluster_, max_pts_per_cluster_);
  //std::cout << "INFO: end of extractEuclideanClusters " << std::endl;
  // Sort the clusters based on their size (largest one first)
  //std::sort (clusters.rbegin (), clusters.rend (), comparePointClusters);
}
