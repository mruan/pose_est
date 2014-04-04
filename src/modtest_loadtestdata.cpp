#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZ                 			pointT_;
typedef pcl::PointCloud<pointT_>      		cloudT_;
typedef pcl::PointCloud<pcl::VFHSignature308>  featureT_;

int load_flann_vec(const std::string& filename, std::vector<float>& test_feature)
{
  std::ifstream ifs;
  ifs.open(filename.c_str());
  if (!ifs.good())
  {
	std::cerr<<"Error, failed to load the file"<<std::endl;
    return -1;
  }
  // skip the first 12 rows:
#define MAX_CHARS_PER_LINE 128
  char buf[MAX_CHARS_PER_LINE];
  char token[32];
  int num_points;

  while(!ifs.eof())
    {
      ifs.getline(buf, MAX_CHARS_PER_LINE);
      
      sscanf(buf, "%s %*s", token);
      // Count the number of points
      if (strcmp(token, "POINTS") == 0)
	{
	  sscanf(buf, "%s %d", token, &num_points);
	  if(num_points != 308)
	    {
	      pcl::console::print_error("VFH feature size is %d !=308\n", num_points);
	    }
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

  test_feature.resize(308);
  for(int i=0; i < num_points; ++i)
    {
      ifs >> test_feature[i];
      printf("%8.5f ", test_feature[i]);
    }
  printf("\n%d points loaded\n", num_points);
  return 0;
}

int load_test_files(const std::string& input_dir, 
		    std::vector<cloudT_::Ptr>& test_pcd,
		    std::vector<flann::Matrix<float> >& test_feature)
{
  // avoid reallocating space
  std::vector<float> feature_vec;

  for (boost::filesystem::directory_iterator it (input_dir); it != boost::filesystem::directory_iterator(); ++it)
  {
    if (boost::filesystem::is_regular_file (it->status ()))
    {
      if (boost::filesystem::extension (it->path ()) == ".pcd")
	{
	  cloudT_::Ptr cld_ptr(new cloudT_);
	  pcl::io::loadPCDFile (it->path().filename().string(), *cld_ptr);
	  test_pcd.push_back(cld_ptr);
	}
      else if (boost::filesystem::extension (it->path ()) == ".vfh")
	{
	  if(load_flann_vec(it->path().filename().string(), feature_vec))
	    {
	      test_feature.push_back(flann::Matrix<float>(new float[308], 1, 308));
	      flann::Matrix<float> &p = test_feature.back();
	      memcpy(&p.ptr()[0], &feature_vec[0], 308 *sizeof(float));
	    }
	}
    }
  }
}

int main(int argc, char* argv[])
{
  // Load all testing vfh features
  std::vector<cloudT_::Ptr> test_pcd;
  std::vector<flann::Matrix<float> > test_feature;
  
  if( load_test_files("./", test_pcd, test_feature) < 0)
    return -1;

  // deallocate space
  for(int i=0; i< test_feature.size(); i++)
    delete[] test_feature[i].ptr();

  return 0;
}
