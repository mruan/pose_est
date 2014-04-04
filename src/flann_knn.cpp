#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <ifstream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZ                 			pointT_;
typedef pcl::PointCloud<pointT_>      		cloudT_;
typedef pcl::PointCloud<pcl::VFHSignature308>  featureT_;
 

inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}

int loadFileList(std::string& filename, std::vector<str::string>& filelist)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_good ())
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
  if (boost::filesystem::exists ("train_data.list") )
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
 if (boost::filesystem::exists ("train_data.h5") )
  {
    flann::load_from_file(data, filename, "train_data");
    return 0;
  }
 else
   {
     pcl::console::print_error ("Could not find training data models %s!\n", 
				filename.c_str ());
     return -1;
   }
}

int load_flann_vec(std::string& filename, std::vector<float>& test_feature)
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


int load_test_files(std::string& input_dir, 
		    std::vector<cloudT_::Ptr>& test_pcd,
		    std::vector<flann::Matrix<float> >& test_feature)
{
  // avoid reallocating space
  std::vector<float> feature_vec;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_regular_file (it->status ()))
    {
      if (boost::filesystem::extension (it->path ()) == ".pcd")
	{
	  cloudT_::Ptr cld_ptr(new cloudT_);
	  pcl::io::loadPCDFile (it->path(), *cld_ptr);
	  test_pcd.push_back(cld_ptr);
	}
      else if (boost::filesystem::extension (it->path ()) == ".vfh"
	{
	  if(load_flann_vec(it->path(), feature_vec))
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
  if (argc < 2)
    {
      show_help(argv[0]);
      return (-1);
    }

  // Parameters need to be filled, currently set to default:
  int k = 6;
  double thresh = DBL_MAX;     // No threshold, disabled by default

  // Load all testing vfh features
  std::vector<cloudT_::Ptr> test_pcd;
  std::vector<flann::Matrix<float> > test_feature;
  if( load_testing_files(input_dir, test_pcd, test_feature) < 0)
    return -1;

  std::string kdtree_idx_filename      = "kdtree.idx";
  std::string train_data_h5_filename   = "train_data.h5";
  std::string train_data_list_filename = "train_data.list";

  pcl::console::parse_argument (argc, argv, "-thresh", thresh);
  // Search for the k closest matches
  pcl::console::parse_argument (argc, argv, "-k", k);

  // load the histgram
  flann::Matrix<int> k_indices;
  flann::Matrix<float> k_distances;
  flann::Matrix<float> data;
  std::vector<std::string> filelist;

  // load file list and also convert it index list
  if( (load_train_feature(train_data_h5_filename, data)<0) || 
      (load_train_filelist(train_data_list_filename, filelist)<0))
    return -1;

  


  // TODO: Body of the function
  nearestKSearch (index, histogram, k, k_indices, k_distances);

  // deallocate space
  for(int i=0; i< test_feature.size(); i++)
    delete[] test_feature[i].ptr();

  return 0;
}
