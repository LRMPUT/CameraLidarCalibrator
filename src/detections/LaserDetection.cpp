/*
 * BSD 3-Clause License
 *
 * This file is part of the CameraLidarCalibrator distribution (https://github.com/LRMPUT/CameraLidarCalibrator).
 * Copyright (c) 2020, Michał Nowicki (michal.nowicki@put.poznan.pl)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "detections/LaserDetection.h"

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>

#include "utils/CustomVoxelGrid.h"

#include <iostream>

void LaserDetection::filterCustomVoxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::CustomVoxelGrid ourFilter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
    ourFilter.setInputCloud(cloud);
    ourFilter.setLeafSize(0.02f, 0.02f, 0.02f);
    ourFilter.filter(*cloudOut);

    *cloud = *cloudOut;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserDetection::detect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector4d &gtPlaneEq) {



//    if (cloudIdx > 0) {
        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setMin(Eigen::Vector4f(min_pt.x - 1, min_pt.y - 1, min_pt.z - 1, 1.0));
        crop.setMax(Eigen::Vector4f(max_pt.x + 1, max_pt.y + 1, max_pt.z + 1, 1.0));
//            crop.setTranslation(box_translation);
//            crop.setRotation(box_rotation);
        crop.setInputCloud(cloud);
        crop.filter(*cloud);

        std::cout << " PRE : " << cloud->size() << std::endl;
        filterCustomVoxelGrid(cloud);
        std::cout << " POST : " << cloud->size() << std::endl;
        std::cout << " -- " << cloud->points[0].intensity << std::endl;
        std::cout << " -- " << cloud->points[1].intensity << std::endl;
        std::cout << " -- " << cloud->points[2].intensity << std::endl;
        std::cout << " -- " << cloud->points[3].intensity << std::endl;
        std::cout << " -- " << cloud->points[4].intensity << std::endl;
//    }

    // Normal estimation
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // Region growing to create potentatial planes
    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize(150);
    reg.setMaxClusterSize(6000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(60);
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(0.3);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "----" << std::endl;
    std::cout << "cloudIdx = " << cloudIdx << "\t clusters: " << clusters.size() << std::endl;

    // Each candidate
    std::vector<Match> matches;
    for (int i = 0; i < clusters.size(); i++) {

        pcl::PointIndices &indices = clusters[i];

        // Selecting points for candidate
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
        *indices_ptr = indices;
        pcl::PointCloud<pcl::PointXYZI>::Ptr chessboard_candidate(
                new pcl::PointCloud<pcl::PointXYZI>);
        extract.setInputCloud(cloud);
        extract.setIndices(
                boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(indices)));
        extract.setNegative(false);
        extract.filter(*chessboard_candidate);

        // Plane matching
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.04);
        seg.setInputCloud(chessboard_candidate);
        seg.segment(*inliers, *coefficients);
        double inliers_percentage =
                double(inliers->indices.size()) / chessboard_candidate->size();

        Eigen::Vector4d planeEq = Eigen::Vector4d(coefficients->values[0], coefficients->values[1], coefficients->values[2],
                                                  coefficients->values[3]);

        // Extract plane inliers
        pcl::PointCloud<pcl::PointXYZI>::Ptr chessboard_inliers(new pcl::PointCloud<pcl::PointXYZI>);
        extract.setInputCloud(chessboard_candidate);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*chessboard_inliers);

        // Project the model inliers
//            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);
//            pcl::ProjectInliers<pcl::PointXYZI> proj;
//            proj.setModelType(pcl::SACMODEL_PLANE);
//            proj.setIndices(inliers);
//            proj.setInputCloud(chessboard_inliers);
//            proj.setModelCoefficients(coefficients);
//            proj.filter(*cloud_projected);

        // startGT
        Eigen::Vector3d planeA;
        if (cloudIdx == 0)
            planeA = gtPlaneEq.block<3,1>(0,0);
        else
            planeA = prevPlaneEq.block<3,1>(0,0);

        Eigen::Vector3d planeB = planeEq.block<3,1>(0,0);

        double angleA = acos(planeA.dot(planeB)) * 180.0 / M_PI;
        double angleB = acos(planeA.dot(-planeB)) * 180.0 / M_PI;
//            std::cout << "\tAngle: " << std::min(fabs(angleA), fabs(angleB)) << " deg" << std::endl;

        Match m;
        m.planeEq = planeEq;
        m.pointSize = indices.indices.size();
        m.angle = std::min(fabs(angleA), fabs(angleB));
        //m.cloud = cloud_projected;
        m.cloud = chessboard_inliers;
        m.indices = indices;
        matches.push_back(m);
    }

    std::sort(matches.begin(), matches.end(),
              [](const Match & a, const Match & b) -> bool
              {
                  return a.angle < b.angle;
              });

    for (int matchIdx = 0; matchIdx < matches.size(); matchIdx ++) {
        std::cout << "\tAngle: " << matches[matchIdx].angle << " Size: " << matches[matchIdx].pointSize <<
                  " PlaneEq: " << matches[matchIdx].planeEq.transpose() << std::endl;
    }


    std::cout << " +++ " << std::endl;

//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    for (auto points_it = colored_cloud->begin (); points_it != colored_cloud->end (); ++points_it) {
//        points_it->r = 255;
//        points_it->g = 255;
//        points_it->b = 255;
//    }
//
//    for (auto points_it = matches[0].indices.indices.begin(); points_it != matches[0].indices.indices.end(); ++points_it) {
//        int idx = *points_it;
//        colored_cloud->points[idx].r = 255;
//        colored_cloud->points[idx].g = 0;
//        colored_cloud->points[idx].b = 0;
//    }
//
//    if (cloudIdx % 5  == 0 ) {
//
//        pcl::visualization::CloudViewer viewer("Plane extractor");
//        viewer.showCloud(colored_cloud);
//        while (!viewer.wasStopped()) {
//        }
//    }

//        pcl::visualization::PCLVisualizer viewerPCL ("PCL visualizer");
//        viewerPCL.addPointCloud (colored_cloud, "cloud"); // Method #1
//
//        viewerPCL.addCoordinateSystem (1.0, "axis", 0);
//        viewerPCL.setBackgroundColor (0.05, 0.05, 0.05, 0);	// Setting background to a dark grey
//        viewerPCL.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
//
//        while (!viewerPCL.wasStopped ()) {
//            viewerPCL.spinOnce ();
//        }

    std::cout << "Cloud size = " << matches[0].cloud->size() << std::endl;

    Eigen::Vector4f new_centroid;
    pcl::compute3DCentroid (*matches[0].cloud, new_centroid);
    std::cout << "new_centroid = " << new_centroid.transpose() << std::endl;

    pcl::getMinMax3D(*matches[0].cloud, min_pt, max_pt);

    std::cout << "new min_pt = " << min_pt << std::endl;
    std::cout << "new max_pt = " << max_pt << std::endl;

    prevPlaneEq = matches[0].planeEq;
    prevCentroid = new_centroid;

    return matches[0].cloud;

}



pcl::PointCloud<pcl::PointXYZI>::Ptr LaserDetection::process(std::vector <std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>> &cloudBuffer) {










    pcl::PointCloud<pcl::PointXYZI>::Ptr selectedPoints (new pcl::PointCloud<pcl::PointXYZI>);

    for (int cloudIdx = 0; cloudIdx < cloudBuffer.size(); cloudIdx ++) {

        // In iteration 0 it is used to initialize, later it is used to verify
        SingleCloudPoints startGT = gtDetections[cloudIdx];

        // Cloud to process
        pcl::PointCloud<pcl::PointXYZI>::Ptr subsampled_cloud_ = cloudBuffer[cloudIdx].second;

        if (cloudIdx > 0) {
            pcl::CropBox<pcl::PointXYZI> crop;
            crop.setMin(Eigen::Vector4f(min_pt.x - 1, min_pt.y - 1, min_pt.z - 1, 1.0));
            crop.setMax(Eigen::Vector4f(max_pt.x + 1, max_pt.y + 1, max_pt.z + 1, 1.0));
//            crop.setTranslation(box_translation);
//            crop.setRotation(box_rotation);
            crop.setInputCloud(subsampled_cloud_);
            crop.filter(*subsampled_cloud_);
        }

        // Normal estimation
        pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(subsampled_cloud_);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*normals);

        // Region growing to create potentatial planes
        pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
        reg.setMinClusterSize(150);
        reg.setMaxClusterSize(6000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(60);
        reg.setInputCloud(subsampled_cloud_);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(0.3);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);


        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//        viewer.showCloud(colored_cloud);



        std::cout << "----" << std::endl;
        std::cout << "cloudIdx = " << cloudIdx << "\t clusters: " << clusters.size() << std::endl;
        std::cout << "startGT.selectedPoints.size() = " << startGT.selectedPoints.size() << std::endl;
        std::cout << "startGT.planeEq = " << startGT.planeEq.transpose() << std::endl;

        // Each candidate
        std::vector<Match> matches;
        for (int i = 0; i < clusters.size(); i++) {

            pcl::PointIndices &indices = clusters[i];

            // Selecting points for candidate
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
            *indices_ptr = indices;
            pcl::PointCloud<pcl::PointXYZI>::Ptr chessboard_candidate(
                    new pcl::PointCloud<pcl::PointXYZI>);
            extract.setInputCloud(subsampled_cloud_);
            extract.setIndices(
                    boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(indices)));
            extract.setNegative(false);
            extract.filter(*chessboard_candidate);

            // Plane matching
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.04);
            seg.setInputCloud(chessboard_candidate);
            seg.segment(*inliers, *coefficients);
            double inliers_percentage =
                    double(inliers->indices.size()) / chessboard_candidate->size();


//            std::cout << "Inlier percentages: " << inliers_percentage << " num=" << indices.indices.size() << std::endl;
//            std::cout << "Plane coefficients: " << coefficients->values[0] << " "
//                      << coefficients->values[1] << " "
//                      << coefficients->values[2] << " "
//                      << coefficients->values[3] << std::endl;
            Eigen::Vector4d planeEq = Eigen::Vector4d(coefficients->values[0], coefficients->values[1], coefficients->values[2],
                                                      coefficients->values[3]);

            // Extract plane inliers
            pcl::PointCloud<pcl::PointXYZI>::Ptr chessboard_inliers(new pcl::PointCloud<pcl::PointXYZI>);
            extract.setInputCloud(chessboard_candidate);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*chessboard_inliers);

            // Project the model inliers
//            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);
//            pcl::ProjectInliers<pcl::PointXYZI> proj;
//            proj.setModelType(pcl::SACMODEL_PLANE);
//            proj.setIndices(inliers);
//            proj.setInputCloud(chessboard_inliers);
//            proj.setModelCoefficients(coefficients);
//            proj.filter(*cloud_projected);


//            for (int testIdx = 0; testIdx < chessboard_inliers->size(); testIdx++ ) {
//                Eigen::Vector3d p = chessboard_inliers->points[testIdx].getArray3fMap().cast<double>();
//                double d = planeEq.block<3,1>(0,0).transpose() * p + planeEq(3);
////                std::cout << "test: " << d << " m" << std::endl;
//            }

//            std::cout << "\tsize = " << indices.indices.size() << "\t" ;
//            std::cout << "\tplaneEq = " << planeEq.transpose() << "\t ||||";


            // startGT
            Eigen::Vector3d planeA;
            if (cloudIdx == 0)
                planeA = startGT.planeEq.block<3,1>(0,0);
            else
                planeA = prevPlaneEq.block<3,1>(0,0);

            Eigen::Vector3d planeB = planeEq.block<3,1>(0,0);

            double angleA = acos(planeA.dot(planeB)) * 180.0 / M_PI;
            double angleB = acos(planeA.dot(-planeB)) * 180.0 / M_PI;
//            std::cout << "\tAngle: " << std::min(fabs(angleA), fabs(angleB)) << " deg" << std::endl;

            Match m;
            m.planeEq = planeEq;
            m.pointSize = indices.indices.size();
            m.angle = std::min(fabs(angleA), fabs(angleB));
            //m.cloud = cloud_projected;
            m.cloud = chessboard_inliers;
            m.indices = indices;
            matches.push_back(m);
        }

        std::sort(matches.begin(), matches.end(),
                  [](const Match & a, const Match & b) -> bool
                  {
                      return a.angle < b.angle;
                  });

        for (int matchIdx = 0; matchIdx < matches.size(); matchIdx ++) {
            std::cout << "\tAngle: " << matches[matchIdx].angle << " Size: " << matches[matchIdx].pointSize <<
            " PlaneEq: " << matches[matchIdx].planeEq.transpose() << std::endl;
        }

        std::cout << " --- TEST vs GT --- " << std::endl;
        for (int matchIdx = 0; matchIdx < matches.size(); matchIdx++) {

            Eigen::Vector4d planeEq = matches[matchIdx].planeEq;

            double avgD = 0;
            for (int pointIdx = 0; pointIdx < startGT.selectedPoints.size(); pointIdx++) {
                double d = planeEq.block<3, 1>(0, 0).transpose() *
                           startGT.selectedPoints[pointIdx].head<3>(0) + planeEq(3);
                avgD += fabs(d);
//            std::cout << d << " ";
            }
//            std::cout << std::endl;
            std::cout << "matchIdx = " << matchIdx << " | avgD = " << avgD / startGT.selectedPoints.size() << std::endl;
        }

        std::cout << " +++ " << std::endl;

        /// TODO: It is assumed that the first match is correct (might not be)
        *selectedPoints = *selectedPoints + *matches[0].cloud;


        for (auto points_it = colored_cloud->begin (); points_it != colored_cloud->end (); ++points_it) {
            points_it->r = 255;
            points_it->g = 255;
            points_it->b = 255;
        }

        for (auto points_it = matches[0].indices.indices.begin(); points_it != matches[0].indices.indices.end(); ++points_it) {
            int idx = *points_it;
            colored_cloud->points[idx].r = 255;
            colored_cloud->points[idx].g = 0;
            colored_cloud->points[idx].b = 0;
        }

        if (cloudIdx % 5  == 0 ) {

            pcl::visualization::CloudViewer viewer("Plane extractor");
            viewer.showCloud(colored_cloud);
            while (!viewer.wasStopped()) {
            }
        }

//        pcl::visualization::PCLVisualizer viewerPCL ("PCL visualizer");
//        viewerPCL.addPointCloud (colored_cloud, "cloud"); // Method #1
//
//        viewerPCL.addCoordinateSystem (1.0, "axis", 0);
//        viewerPCL.setBackgroundColor (0.05, 0.05, 0.05, 0);	// Setting background to a dark grey
//        viewerPCL.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
//
//        while (!viewerPCL.wasStopped ()) {
//            viewerPCL.spinOnce ();
//        }

        std::cout << "Cloud size = " << matches[0].cloud->size() << std::endl;

        Eigen::Vector4f new_centroid;
        pcl::compute3DCentroid (*matches[0].cloud, new_centroid);
        std::cout << "new_centroid = " << new_centroid.transpose() << std::endl;

        pcl::getMinMax3D(*matches[0].cloud, min_pt, max_pt);

        std::cout << "new min_pt = " << min_pt << std::endl;
        std::cout << "new max_pt = " << max_pt << std::endl;

        prevPlaneEq = matches[0].planeEq;
        prevCentroid = new_centroid;

//        int a;
//        std::cin >> a;
    }
    return selectedPoints;
}


void LaserDetection::verify(std::vector<SingleCloudPoints, Eigen::aligned_allocator<SingleCloudPoints>> &manualPlaneSelection) {
    for (int i=0;i<manualPlaneSelection.size();i++) {

        std::cout << "manualPlaneSelection i = " << i << " " << manualPlaneSelection[i].planeEq.transpose() << std::endl;
        std::cout << "manualPlaneSelection.size = " << manualPlaneSelection[i].selectedPoints.size() << std::endl;

        Eigen::Vector3d planeA = manualPlaneSelection[i-1].planeEq.head<3>();
        Eigen::Vector3d planeB = manualPlaneSelection[i].planeEq.head<3>();

        if (i > 0) {
            double angleA = acos(planeA.dot(planeB)) * 180.0 / M_PI;
            double angleB = acos(planeA.dot(-planeB)) * 180.0 / M_PI;
            std::cout << "\tAngle: " << std::min(fabs(angleA), fabs(angleB)) << " deg" << std::endl;
        }
    }

    gtDetections = manualPlaneSelection;
}