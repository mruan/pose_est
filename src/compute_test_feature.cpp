
typedef pcl::PointXYZ                 	       pointT_;
typedef pcl::PointCloud<pointT_>        	cloudT_;
typedef pcl::PointCloud<pcl::Normal>   		normalT_;
typedef pcl::PointCloud<pcl::VFHSignature308>  featureT_;

normalT_::Ptr getNormal(cloudT_::Ptr cloudp)
{
	pcl::NormalEstimation<pointT_, pcl::Normal> ne;
	ne.setInputCloud(cloudp);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	pcl::search::KdTree<pointT_>::Ptr treep(new pcl::search::KdTree<pointT_> ());

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	normalT_::Ptr normalp(new normalT_());
	// Compute the features
	ne.compute (*normalp);
	
	return (normalp);
}

featureT_::Ptr getFeature(cloudT_::Ptr cloudp, normalT_::Ptr normp)
{
	// Compute VFH feature for each group and save them
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloudp);
	vfh.setInputNormals (normp);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	vfh.setSearchMethod (tree);
	// Output datasets
	featureT_::Ptr vfhs (new featureT_());

	// Compute the features
	vfh.compute (*vfhs);
	
	return (vfhs);
}

int main(int argc, char* argv[])
{
  // Load the pcd
  cloudT_::Ptr cloudp(new cloudT_());

  std::vector<int> tags;
  std::vector<int> tag_count;
  std::map<int, int> tag2idx;

  /* Load the pcd from file */

  // compute normal over the entire pcd
  normalT_::Ptr normalp = getNormal(cloudp);

  printf("%d normal computed. ", (int) normalp->size());

  std::vector<std::vector<int> > list_of_idx;

  //resize to number of unique tags
  const int num_unique_tags = tag_count.size()
  list_of_idx.resize(num_unique_tags); 

  // first reserve space:
  for(int i=0; i< tag_count.size(); ++i)
    {
      list_of_idx[i].reserve(tag_count[i]);
    }

  const int num_points = normalp->size();
  int count_valid = 0;
  // base on the label, seperate them into different groups
  for(int i=0; i < num_unique_tags; i++)
    {
      if (!pcl::isFinite<pcl::Normal>(normalp->points[i]))
	{
	  // normal isn't finite, vfh cannot be estimated at this point
	  continue;
	}
      list_of_idx[tag[i]].push_back(i);
      ++count_valid;
    }
  printf("%d valid points remaining.\n");

  // Compute vfh feature for each of the parts and save them to file
  char buf[128];
  int num_p;
  for(int i=0; i< num_unique_tags; ++i)
    {
      num_p = list_of_idx[i].size();
      printf("Part %8d has %d points\n", label_value, num_p);

      // TOOD: should I check number of points at this early stage?
      //      if(num_p < 50)
      //	continue;

      cloudT_::Ptr  partcl_p(new cloudT_(*cloudp, list_of_idx[i]));
      normalT_::Ptr partnm_p(new normalT_(*normalp, list_of_idx[i]));

      // compute the feature
      featureT_::Ptr vfhs = getFeature(partcl_p, partnm_p);

      // save the output point cloud to disk e.g. 
      // output_dir/mn1_c1_112345.pcd
      // output_dir/mn1_c1_112345_vfh.pcd
      sprintf(buf, "%s/%s_%08d.pcd", output_dir, aug_name, label_value);
      pcl::io::savePCDFileASCII (buf, *partcl_p);
      
      sprintf(buf, "%s/%s_%08d_vfh.pcd", output_dir, aug_name, label_value);
      pcl::io::savePCDFileASCII (buf, *vfhs);
    }
  printf("Save files to %s/%s_label_[vfh].pcd\n", output_dir, aug_name);
  return 0;
}
