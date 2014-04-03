

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
  // load file list and also convert it index list

  // 
}
