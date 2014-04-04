#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZ                 		pointT_;
typedef pcl::PointCloud<pointT_>      		cloudT_;
typedef pcl::PointCloud<pcl::VFHSignature308>   featureT_;

int loadFileList(std::string& filename, std::vector<std::string>& filelist)
{
  std::ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.good ())
    {
      pcl::console::print_error ("Could not open %s for reading\n", 
				 filename.c_str ());
      return -1;
    }

  std::string line;
  while (!fs.eof ())
    {
      getline (fs, line);
      if (line.empty ())
	continue;

      filelist.push_back(line);
    }
  fs.close ();
  return 0;
}

int load_train_filelist(std::string& filename, std::vector<std::string>& filelist)
{
  // Load all training stuff
  // Check if the data has already been saved to disk
  if (boost::filesystem::exists (filename) )
  {
    return loadFileList (filename, filelist);
  }
  else
    {
      pcl::console::print_error ("Could not find training file list files %s!\n", 
				 filename.c_str ());
      return (-1);
    }
}

int load_train_features(std::string& filename, flann::Matrix<float>& data)
{
 if (boost::filesystem::exists (filename) )
  {
    flann::load_from_file(data, filename, "training_data");
    p
    return 0;
  }
 else
   {
     pcl::console::print_error ("Could not find training data models %s!\n", 
				filename.c_str ());
     return -1;
   }
}

int main(int argc, char* argv[])
{
  std::string kdtree_idx_filename      = "kdtree.idx";
  std::string train_data_h5_filename   = "training_data.h5";
  std::string train_data_list_filename = "training_data.list";

  // load the histgram
  flann::Matrix<float> data; 
  std::vector<std::string> filelist;

  // load file list and also convert it index list
  if( (load_train_features(train_data_h5_filename, data) < 0) || 
      (load_train_filelist(train_data_list_filename, filelist) < 0) )
    return -1;

  // Check if the tree index has already been saved to disk
  if (!boost::filesystem::exists (kdtree_idx_filename))
    {
      pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_filename.c_str ());
      return (-1);
    }

  flann::Index<flann::ChiSquareDistance<float> > index(data, flann::SavedIndexParams (kdtree_idx_filename));
  index.buildIndex (); // TODO: really need to rebuild it from saved file?

  return 0;
}
