#include <iostream>
#include <fstream>
#include "pcd_utils.h"

#include "label_utils.h"

union FloatInt32
{
  int i; 
  float f; 
};

int AsciiPcdUtil::
loadPCDwLabel(std::string filename,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			std::vector<int>& labels)
{
   std::vector<int> label_counts(0, 14);
   return loadPCDwLabel(filename, cloud, labels, label_counts);
}
		    
int AsciiPcdUtil::
loadPCDwLabel(std::string filename,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			std::vector<int>& labels,
			std::vector<int>& label_count)
{
  std::ifstream ifs;
  ifs.open(filename.c_str());
  if (!ifs.good())
  {
	std::cerr<<"Error, failed to load the file"<<std::endl;
    return -1;
  }
  // skip the first 12 rows:
  while(!ifs.eof())
    {
      ifs.getline(buf, MAX_CHARS_PER_LINE);
      
      sscanf(buf, "%s %*s", token);
      // Count the number of points
      if (strcmp(token, "POINTS") == 0)
	{
	  sscanf(buf, "%s %d", token, &num_points); 
	}
      else if (strcmp(token, "DATA")==0)
	{
	  sscanf(buf, "%*s %s", token);
	  if (strcmp(token, "ascii") ==0)
	    {
	      break; // proceed to data section;
	    }
	  else
	    {
	      std::cerr<< "Error, cannot load non-ascii type data"<<std::endl;
	      return -2;
	    }
	}
    }

  cloud->width = num_points;
  cloud->height = 1;
  cloud->resize(num_points);
  labels.resize(num_points);
  pcl::PointXYZ *pt = NULL;
  FloatInt32 c;
  int ph; // place holder
  int idx;
  for(int i=0; i < num_points; ++i)
    {
      ifs.getline(buf, MAX_CHARS_PER_LINE);
      pt = &cloud->points[i];
      sscanf(buf, "%f %f %f %f %d", &pt->x, &pt->y, &pt->z, &c.f, &ph);
      labels[i] = c.i; // need to convert from float to uint32;

	  // increment the label counter
	  idx = PartLabelUtil::findIdx(c.i);
	  if(idx >=0)
	  {
		label_count[idx]++;
	  }
      // Uncomment to debug output
      //printf("%-4d %8.5f %8.5f %8.5f %8d\n", i, pt->x, pt->y, pt->z, c.i);
    }
  ifs.close();
  return 1;
}
