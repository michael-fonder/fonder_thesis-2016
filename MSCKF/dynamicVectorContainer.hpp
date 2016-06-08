/**
 * Author	: Michael Fonder
 * Year		: 2016
 **/

#ifndef DYNAMIC_VECTOR_CONTAINER
#define DYNAMIC_VECTOR_CONTAINER

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <stdint.h>

#include <unordered_map>
#include <unordered_set>
#include <set>
#include <stack>
#include <vector>

#include "dynamicVector.hpp"

template <typename Object> class DynamicVectorContainer :
public DynamicVector<Object>
{
public :
 	DynamicVectorContainer();
	DynamicVectorContainer(const DynamicVectorContainer &DVC);
 	~DynamicVectorContainer();
	Object* addNew();
	Object* push_back(const Object toInsert);	
	void remove(std::unordered_set<Object*> &toRemove);
};

template <typename Object>
DynamicVectorContainer<Object>::DynamicVectorContainer()
{}

template <typename Object>
DynamicVectorContainer<Object>::DynamicVectorContainer(const DynamicVectorContainer &DVC)
{
	for(size_t i=0; i<DVC.size(); ++i)
		this->push_back(DVC[i]);
}
	
template <typename Object>
DynamicVectorContainer<Object>::~DynamicVectorContainer()
{
	for(size_t i=0; i<this->objectVec.size(); ++i)
		delete this->objectVec[i];
}

// Constant
template <typename Object>
Object* DynamicVectorContainer<Object>::addNew()
{
	Object *ptr = new Object();
	this->indexMap.insert({ptr, this->objectVec.size()});
	this->objectVec.push_back(ptr);
	return ptr;
}

// Constant
template <typename Object>
Object* DynamicVectorContainer<Object>::push_back(Object toInsert)
{
	Object *ptr = new Object(toInsert);
	this->indexMap.insert({ptr, this->objectVec.size()});
	this->objectVec.push_back(ptr);
	return ptr;
}

// Linear in container size whatever the size of the given set of element to remove
// => more efficient to remove one big set than several small ones
template <typename Object>
void DynamicVectorContainer<Object>::remove(std::unordered_set<Object*> &toRemove)
{
	// Remove element and its references in data structures
	int end = -1;
	for(int i=this->objectVec.size()-1; i>=-1; --i)
	{
		if(i>=0 && toRemove.find(this->objectVec[i]) != toRemove.end())
		{
			delete (this->objectVec[i]);
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