#include <map>
#include <string>

#ifndef LABEL_UTILS_H
#define LABEL_UTILS_H
class PartLabelUtil
{
public:
	static int getLabel(int idx);
	static int findPartName(int label, std::string& name);
	static int findIdx(int label);
private:
	static std::map<int,int>  createMap();
	// The order matters:
	static const int num_parts = 14;
	static int labels[num_parts];
	static std::string names[num_parts];
//	std::map<int, std::tuple<double, double, double> > int2rgb;
};

#endif
