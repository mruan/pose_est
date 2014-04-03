
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>

#include <vector>
#include <stdio.h>

#include "pcd_utils.h"
#include "label_utils.h"

#define NUM_BODY_PARTS 14

typedef pcl::PointXYZ                 			pointT_;
typedef pcl::PointCloud<pointT_>      		cloudT_;
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
	//std::string dir(argv[1]);
	std::string file(argv[1]);
	char* output_dir = argv[2];
	char* base_name  = argv[3];
	
	// Open the file:
	cloudT_::Ptr cloudp(new cloudT_());

	std::vector<int> labels;
	std::vector<int> label_counts(NUM_BODY_PARTS, 0);
	PartLabelUtil pl_util;
	
	AsciiPcdUtil pcd_util;
	pcd_util.loadPCDwLabel(file, cloudp, labels, label_counts);
	
	printf("%d points loaded\n", cloudp->size());
	// Compute the normal
	normalT_::Ptr normalp = getNormal(cloudp);
	
	printf("%d normals computed\n", normalp->size());
	// Separate the points and normals into different parts:
	// also filter out points with undefined normals
	std::vector< std::vector<int> > list_of_idx;
	list_of_idx.resize(NUM_BODY_PARTS);
	
	// first reserve space:
	for(int i=0; i< NUM_BODY_PARTS; ++i)
	{
		list_of_idx[i].reserve(label_counts[i]);
	}
	
	const int num_points = normalp->points.size();
	int pl_idx; // part label index
	for(int i=0; i< num_points; ++i)
	{
		if (!pcl::isFinite<pcl::Normal>(normalp->points[i]))
		{
			// normal isn't finite, vfh cannot be estimated at this point
			continue;
		}
		
		pl_idx = pl_util.findIdx(labels[i]);
		if(pl_idx >=0)
		  {
		    list_of_idx[pl_idx].push_back(i);
		  }
	}
	
	char buf[128];
	int label_value;
	int num_p;
	for(int i=0; i< NUM_BODY_PARTS; ++i)
	{
		label_value = PartLabelUtil::getLabel(i);
		num_p = list_of_idx[i].size();
		printf("Part %d has %d points\n", label_value, num_p);
		
		if(num_p < 50)
		  continue;

		cloudT_::Ptr  partcl_p(new cloudT_(*cloudp, list_of_idx[i]));
		normalT_::Ptr partnm_p(new normalT_(*normalp, list_of_idx[i]));
	
		// compute the feature
		featureT_::Ptr vfhs = getFeature(partcl_p, partnm_p);
		
		// save the output point cloud to disk e.g. 
		// output_dir/mn1_c1_112345.pcd
		// output_dir/mn1_c1_112345_vfh.pcd
		sprintf(buf, "%s%s_%d.pcd", output_dir, base_name, label_value);
		pcl::io::savePCDFileASCII (buf, *partcl_p);
		
		sprintf(buf, "%s%s_%d_vfh.pcd", output_dir, base_name, label_value);
		pcl::io::savePCDFileASCII (buf, *vfhs);
	}
	
  return 0;
}
