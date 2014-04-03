
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#define MAX_CHARS_PER_LINE 128

class AsciiPcdUtil
{
 public:
  int loadPCDwLabel(std::string filename,
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		    std::vector<int>& labels);

  int loadPCDwLabel(std::string filename,
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		    std::vector<int>& labels,
		    std::vector<int>& label_counts);
 private:
  int num_points;
  char buf[MAX_CHARS_PER_LINE];
  char token[32];
};

#endif
