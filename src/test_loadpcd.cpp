#include <pcl/point_types.h>
#include <vector>
#include <string>

#include "pcd_utils.h"

int main(int argc, char* argv[])
{
  AsciiPcdUtil pcd_util;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> labels;
  pcd_util.loadPCDwLabel(std::string(argv[1]), cloud, labels);

  return 0;
}
