#include "label_utils.h"
#include <stdio.h>
#include <stdexcept>      // std::out_of_range

int PartLabelUtil::labels[14] = {65280, 511121, 565196, 752588, 6540389, 
							7143628, 7654402, 9829375, 13369463, 13370804,
							13393162, 13408303, 13420288, 16711680};
std::string PartLabelUtil::names[14] = {"Torso", "LeftLowerArm", 
						  "RightLowerArm", 
						  "RightUpperArm", "RightFoot", "LeftUpperArm", 
						  "LeftHand", "LeftFoot","LeftLowerLeg", 
						  "RightLowerLeg", "RightUpperLeg", 
						  "LeftUpperLeg", "RightHand", "Head"};
						
std::map<int,int> PartLabelUtil::createMap()
{
	std::map<int,int> m;
	for(int i=0; i < 14; ++i)
	{
		m[labels[i]] = i;
	}
	printf("map created\n");
	return m;
}

int PartLabelUtil::getLabel(int idx)
{
	return labels[idx];
}

int PartLabelUtil::findPartName(int label, std::string& name)
{
	int idx = findIdx(label);
	if (idx < 0)
		return -1;
	else
	{
		name = names[idx];
		return 0;
	}
}

int PartLabelUtil::findIdx(int label)
{						  
	static const std::map<int, int> int2idx = createMap();
	int idx;
	try
	{
		idx = int2idx.at(label);
	}
	catch (const std::out_of_range& oor) {
		idx = -1;
	}
	return idx;
}
