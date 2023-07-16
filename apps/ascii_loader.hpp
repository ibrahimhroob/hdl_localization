//
// Created by ibrahim on 14/02/2022.
//

#ifndef ASCII_READER_H
#define ASCII_READER_H

#include <stdio.h>
#include <stddef.h>
#include <locale.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <fstream>

//X Y Z Label
//number of points

struct ASCIIReader {
  static bool read(const std::string& filename, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
    std::ifstream fin;
    fin.open(filename);

    std::string name = filename.substr(filename.find_last_of('/')+1);

    if(!fin.is_open()){
      std::cerr << "ERROR: failed to open file: " << filename << std::endl;
      return false;
    }
    std::string header;
    std::getline(fin, header);

    int num_of_points;
    fin >> num_of_points;

    while(!fin.eof()){
      pcl::PointXYZI p;
      fin >> p.x >> p.y >> p.z >> p.intensity;
      cloud->push_back(p);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    //printf("%s loaded with %zu points\n", name.c_str(), cloud->size());

    return cloud->size() > 0;
  }
};

#endif//ASCII_READER_H