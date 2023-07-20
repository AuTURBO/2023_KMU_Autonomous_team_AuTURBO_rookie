/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#include "ar_track_alvar/MultiMarkerBundle.h"
#include <opencv2/opencv.hpp>

using namespace std;

namespace alvar
{
using namespace std;

MultiMarkerBundle::MultiMarkerBundle(std::vector<int>& indices)
  : MultiMarker(indices)
{
  MeasurementsReset();
}

MultiMarkerBundle::~MultiMarkerBundle()
{
}

void MultiMarkerBundle::MeasurementsReset()
{
  optimization_error = -1;
  optimization_keyframes = 0;
  optimization_markers = 0;
  optimizing = false;
  camera_poses.clear();
  measurements.clear();
}

int n_images;    // TODO: This should not be global (use the param instead)
int n_markers;   // TODO: This should not be global (use the param instead)
Camera* camera;  // TODO: This should not be global (use the param instead)

void Est(cv::Mat& state, cv::Mat& estimation, void* param)
{
  // State: cam1, cam2, cam3, cam4, ..., X1(x,y,z), X2, X3, ...
  // Estimation: (u11,v11), (u)

  // For every image observation (camera)...
  for (int i = 0; i < n_images; ++i)
  {
    // Get camera from state
    Pose p;
    p.SetQuaternion(&state.at<double>(i * 7 + 3));

    double tra[3];
    double rodr[3];
    cv::Mat mat_translation_vector = cv::Mat(3, 1, CV_64F, tra);
    cv::Mat mat_rotation_vector = cv::Mat(3, 1, CV_64F, rodr);

    memcpy(tra, &state.at<double>(i * 7), 3 * sizeof(double));
    p.GetRodriques(mat_rotation_vector);

    // For every point in marker field
    int n_points = n_markers * 4;
    for (int j = 0; j < n_points; ++j)
    {
      int index = n_images * 7 + 3 * j;

      double object_points[3] = { state.at<double>(index + 0),
                                  state.at<double>(index + 1),
                                  state.at<double>(index + 2) };

      cv::Mat mat_object_points;
      mat_object_points = cv::Mat(1, 1, CV_64FC3, object_points);

      double proj[2] = { 0 };
      cv::Mat mat_proj = cv::Mat(1, 1, CV_64FC2, proj);

      cv::projectPoints(mat_object_points, mat_rotation_vector,
                        mat_translation_vector, camera->calib_K,
                        camera->calib_D, mat_proj);

      index = i * n_points * 2 + j * 2;
      estimation.at<double>(index + 0) = proj[0];
      estimation.at<double>(index + 1) = proj[1];
    }
  }
}

bool MultiMarkerBundle::Optimize(Camera* _cam, double stop, int max_iter,
                                 Optimization::OptimizeMethod method)
{
  // Est() needs these
  // TODO: Better way to deliver them??? Other than using global variables???
  camera = _cam;
  n_images = camera_poses.size();
  n_markers = marker_indices.size();

  if (n_images < 1)
  {
    cout << "Too few images! At least 1 images needed." << endl;
    return false;
  }

  optimizing = true;

  // Initialize
  size_t frames = camera_poses.size();
  int n_params = frames * 7 + 3 * 4 * n_markers;
  int n_meas = 2 * 4 * n_markers * frames;
  cv::Mat parameters_mat = cv::Mat::zeros(n_params, 1, CV_64F);
  cv::Mat parameters_mask_mat = cv::Mat(n_params, 1, CV_8U);
  cv::Mat measurements_mat = cv::Mat::zeros(n_meas, 1, CV_64F);
  cv::Mat weight_mat = cv::Mat(n_meas, 1, CV_64F);

  parameters_mask_mat = cv::Scalar(1);
  weight_mat = cv::Scalar(1.0);

  // Fill in the point cloud that is used as starting point for optimization
  for (size_t i = 0; i < marker_indices.size(); ++i)
  {
    int id = marker_indices[i];
    for (int j = 0; j < 4; j++)
    {
      // hop int index = frames*7 + id*(3*4) + j*3;
      int index = frames * 7 + i * (3 * 4) + j * 3;
      // Lets keep the base marker constant (1st marker given in the indices
      // list)
      if (i == 0)
      {
        parameters_mask_mat.at<uchar>(index + 0, 0) = 0;
        parameters_mask_mat.at<uchar>(index + 1, 0) = 0;
        parameters_mask_mat.at<uchar>(index + 2, 0) = 0;
      }
      if (marker_status[i] > 0)
      {
        parameters_mat.at<double>(index + 0, 0) =
            pointcloud[pointcloud_index(id, j)].x;
        parameters_mat.at<double>(index + 1, 0) =
            pointcloud[pointcloud_index(id, j)].y;
        parameters_mat.at<double>(index + 2, 0) =
            pointcloud[pointcloud_index(id, j)].z;
      }
      else
      {
        // We don't optimize known-initialized parameters?
        parameters_mask_mat.at<uchar>(index + 0, 0) = 0;
        parameters_mask_mat.at<uchar>(index + 1, 0) = 0;
        parameters_mask_mat.at<uchar>(index + 2, 0) = 0;
      }
    }
  }
  // Fill in the frame data. Camera poses into parameters and corners into
  // measurements
  int n_measurements = 0;  // number of actual measurement data points.
  for (size_t f = 0; f < frames; f++)
  {
    // cout<<"frame "<<f<<" / "<<frames<<endl;
    // Camera pose
    cv::Mat tra =
        cv::Mat(3, 1, CV_64F, &(parameters_mat.at<double>(f * 7 + 0)));
    cv::Mat qua =
        cv::Mat(4, 1, CV_64F, &(parameters_mat.at<double>(f * 7 + 3)));
    camera_poses[f].GetTranslation(tra);
    camera_poses[f].GetQuaternion(qua);
    // Measurements
    for (size_t i = 0; i < marker_indices.size(); ++i)
    {
      int id = marker_indices[i];
      if (measurements.find(measurements_index(f, id, 0)) != measurements.end())
      {
        for (int j = 0; j < 4; j++)
        {
          // cout<<measurements[measurements_index(f, id, j)].x<<endl;
          // hop int index = f*(n_markers*4*2) + id*(4*2) + j*2;
          int index = f * (n_markers * 4 * 2) + i * (4 * 2) + j * 2;
          measurements_mat.at<double>(index + 0, 0) =
              measurements[measurements_index(f, id, j)].x;
          measurements_mat.at<double>(index + 1, 0) =
              measurements[measurements_index(f, id, j)].y;
          n_measurements += 2;
        }
      }
      else
      {
        for (int j = 0; j < 4; j++)
        {
          // hop int index = f*(n_markers*4*2) + id*(4*2) + j*2;
          int index = f * (n_markers * 4 * 2) + i * (4 * 2) + j * 2;
          weight_mat.at<double>(index + 0, 0) = 0.0;
          weight_mat.at<double>(index + 1, 0) = 0.0;
        }
      }
    }
  }
  // out_matrix(measurements_mat, "measurements");
  // out_matrix(parameters_mat, "parameters");
  optimization_keyframes = n_images;
  optimization_markers = 0;
  for (size_t i = 0; i < marker_indices.size(); ++i)
    if (marker_status[i] > 0)
      optimization_markers++;
  Optimization optimization(n_params, n_meas);
  cout << "Optimizing with " << optimization_keyframes << " keyframes and "
       << optimization_markers << " markers" << endl;
  optimization_error = optimization.Optimize(
      parameters_mat, measurements_mat, stop, max_iter, Est, 0, method,
      parameters_mask_mat, cv::Mat(), weight_mat);
  optimization_error /= n_measurements;
  cout << "Optimization error per corner: " << optimization_error << endl;
  /*
  if ((frames > 3) && (optimization_error > stop)) {
    cv::Mat *err = optimization.GetErr();
    int max_k=-1;
    double max=0;
    for (int k=0; k<err->height; k++) {
      if (cvmGet(err, k, 0) > max) {
        max = cvmGet(err, k, 0);
        max_k = k;
      }
    }
    if (max_k >= 0) {
      // We remove all 8 corner measurements
      int f = (max_k - (max_k % (n_markers*4*2))) / (n_markers*4*2);
      max_k -= f*(n_markers*4*2);
      int id = (max_k - (max_k % (4*2))) / (4*2);
      cout<<"Optimization error over the threshold -- remove the biggest
  outlier: "; cout<<"frame "<<f<<" id "<<id<<endl;
      measurements.erase(measurements_index(f,id,0));
      measurements.erase(measurements_index(f,id,1));
      measurements.erase(measurements_index(f,id,2));
      measurements.erase(measurements_index(f,id,3));
      for (int j=0; j<4; j++) {
        int index = f*(n_markers*4*2) + id*(4*2) + j*2;
        cvmSet(measurements_mat, index+0, 0, measurements[measurements_index(f,
  id, j)].x); cvmSet(measurements_mat, index+1, 0,
  measurements[measurements_index(f, id, j)].y);
      }
      optimization_error = optimization.Optimize(parameters_mat,
  measurements_mat, stop, max_iter, Est, method, parameters_mask_mat);
      cout<<"Optimization error: "<<optimization_error<<endl;
    }
  }
  */
  // out_matrix(parameters_mat, "parameters");
  // out_matrix(parameters_mask_mat, "parameters_mask");

  // Fill in the point cloud with optimized values
  for (size_t i = 0; i < marker_indices.size(); ++i)
  {
    int id = marker_indices[i];
    for (int j = 0; j < 4; j++)
    {
      // hop int index = frames*7 + id*(3*4) + j*3;
      int index = frames * 7 + i * (3 * 4) + j * 3;
      pointcloud[pointcloud_index(id, j)].x =
          parameters_mat.at<double>(index + 0, 0);
      pointcloud[pointcloud_index(id, j)].y =
          parameters_mat.at<double>(index + 1, 0);
      pointcloud[pointcloud_index(id, j)].z =
          parameters_mat.at<double>(index + 2, 0);
    }
  }

  parameters_mat.release();
  parameters_mask_mat.release();
  measurements_mat.release();

  optimizing = false;
  return true;
}

void MultiMarkerBundle::_MeasurementsAdd(MarkerIterator& begin,
                                         MarkerIterator& end,
                                         const Pose& camera_pose)
{
  camera_poses.push_back(camera_pose);
  int frame_no = camera_poses.size() - 1;
  // cout<<"Adding measurements for frame "<<frame_no<<endl;
  for (MarkerIterator& i = begin.reset(); i != end; ++i)
  {
    const Marker* marker = *i;
    int id = marker->GetId();
    int index = get_id_index(id);
    if (index < 0)
      continue;
    // cout<<"Id "<<id<<" ";
    for (int j = 0; j < 4; j++)
    {
      measurements[measurements_index(frame_no, id, j)] =
          marker->marker_corners_img[j];
      // cout<<markers->at(i).marker_corners_img[j].x<<" ";
    }
    // cout<<endl;
  }
}

}  // namespace alvar
