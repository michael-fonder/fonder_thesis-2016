/**
 * Author	: Michael Fonder
 * Year		: 2016
 **/

#ifndef DYNAMIC_VECTOR
#define DYNAMIC_VECTOR

#include <stdio.h>
#include <stdint.h>

#include <unordered_map>
#include <vector>

/**
* Abstract class used to implement a template bidirectionnal container
**/

template <typename Object> class DynamicVector
{
protected :
	std::vector<Object*> objectVec;
	std::unordered_map<Object*, int32_t> indexMap;
public :
// 	DynamicVector();
// 	~DynamicVector();
	//iterator... back();
	Object* last();
	size_t size() const;
	int getIndex(Object* ref);
	
	Object &operator[](const int &i);
	const Object &operator[](const int &i) const;
	
	Object* at(const int &i);
	Object* at(const int &i) const;
	
	Object &back();
	const Object &back() const;
	Object &front();
	const Object &front() const;
};

// template <typename Object>
// DynamicVector<Object>::DynamicVector()
// {}
// 	
// template <typename Object>
// DynamicVector<Object>::~DynamicVector()
// {
// 	for(size_t i=0; i<objectVec.size(); ++i)
// 		delete objectVec[i];
// }



// Constant
template <typename Object>
Object& DynamicVector<Object>::operator[](const int &i)
{
	return *objectVec[i];
}
// Constant
template <typename Object>
const Object& DynamicVector<Object>::operator[](const int &i) const
{
	return *objectVec[i];
}

// Constant
template <typename Object>
Object* DynamicVector<Object>::at(const int &i)
{
	return objectVec[i];
}
// Constant
template <typename Object>
Object* DynamicVector<Object>::at(const int &i) const
{
	return objectVec[i];
}

// Constant
template <typename Object>
Object& DynamicVector<Object>::front()
{
	return *objectVec[0];
}
// Constant
template <typename Object>
const Object& DynamicVector<Object>::front() const
{
	return *objectVec[0];
}

// Constant
template <typename Object>
Object& DynamicVector<Object>::back()
{
	return *objectVec[objectVec.size()-1];
}
// Constant
template <typename Object>
const Object& DynamicVector<Object>::back() const
{
	return *objectVec[objectVec.size()-1];
}


// // Constant
// template <typename Object>
// iterator... DynamicVector<Object>::back()
// {
// 	return objectVec.back();
// }

// Constant
template <typename Object>
Object* DynamicVector<Object>::last()
{
	return objectVec[objectVec.size()-1];
}

// Constant
template <typename Object>
size_t DynamicVector<Object>::size() const
{
	return objectVec.size();
}

// Constant
template <typename Object>
int DynamicVector<Object>::getIndex(Object* ref)// const
{
	if(indexMap.find(ref) != indexMap.end())
		return indexMap.find(ref)->second;
	else
		return -1;
}

#endif