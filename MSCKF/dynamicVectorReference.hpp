#ifndef DYNAMIC_VECTOR_REFERENCE
#define DYNAMIC_VECTOR_REFERENCE

#include <stdio.h>
#include <stdint.h>

#include <unordered_map>
#include <vector>
#include "dynamicVector.hpp"

//#include "dynamicVector.h"

template <typename Object> class DynamicVectorReference : public DynamicVector<Object>
{
public :
	//DynamicVectorReference(const DynamicVectorReference<Object> &toCopy);
	Object* push_back(Object* toInsert);
	void remove(std::unordered_set<Object*> &toRemove);
};

// template <typename Object>
// DynamicVectorReference<Object>::DynamicVectorReference(const DynamicVectorReference<Object> &toCopy)
// {
// 	this->objectVec =toCopy.objectVec;
// 	this->indexMap = toCopy.indexMap;
// }

// Constant
template <typename Object>
Object* DynamicVectorReference<Object>::push_back(Object* toInsert)
{
	this->indexMap.insert({toInsert, this->objectVec.size()});
	this->objectVec.push_back(toInsert);
	return toInsert;
}


// Linear in container size whatever the size of the given set of element to remove
// => more efficient to remove one big set than several small ones
template <typename Object>
void DynamicVectorReference<Object>::remove(std::unordered_set<Object*> &toRemove)
{
	// Remove element and its references in data structures
	int end = -1;
	for(int i=this->objectVec.size()-1; i>=-1; --i)
	{
		if(i>=0 && toRemove.find(this->objectVec[i]) != toRemove.end())
		{
			if(end<0)
				end = i+1;
			this->objectVec.erase(this->objectVec.begin()+i);
		}
// 		else if(i<0 || (end>=0 && toRemove.find(this->objectVec[i]) == toRemove.end()))
// 		{
// 			this->objectVec.erase(this->objectVec.begin()+i+1, this->objectVec.begin()+end);
// 			end = -1;
// 		}
	}
	
	// Rebuild indexMap
	this->indexMap.clear();
	for(int32_t i=0; i<this->objectVec.size(); ++i)
		this->indexMap.insert({this->objectVec[i], i});
}

#endif