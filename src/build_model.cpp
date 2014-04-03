#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>

int main(int argc, char** argv)
{
  if (argc<2)
    {
      show_help(argv[0]);
      return (-1);
    }

  std::string kdtree_idx_filename = "kdtree.idx";
  std::string train_data_h5_filename = "train_data.h5";
  std::string train_data_list_filename = "train_data.list";

  // Load the model histograms

  // Convert data into FLANN format
  flann::Matrix<float> data(new float[], rows, cols);
  
  // Save flann matrix to disk
  flann::save_to_file(data, train_data_h5_filename, "train_data");
  // save file lists ??
  
  // Build the tree index and save it to disk
  pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", 
			    kdtree_idx_filename.c_str(), (int)data.rows);

  //  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index.buildIndex();
  index.save(kdtree_idx_filename);
  delete[] data.ptr();

  return 0;
}
