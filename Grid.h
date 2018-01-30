#ifndef GRID_H
#define GRID_H

#include <map>
#include <vector>
#include <string>

#include "Position.h"

struct FGrid
{
	unsigned x, y, z;
	FPosition start, finish;
	std::vector<std::vector<std::vector<int>>> lines;

	FGrid(const std::string filename)
	{

	}

	inline void SetStart(FPosition p)
	{
		start = p;
	}

	inline void SetFinish(FPosition p)
	{
		finish = p;
	}

	inline bool operator()(unsigned xx, unsigned yy, unsigned zz) const
	{
		if (xx < x && yy < y && zz < z)
		{
			return lines[xx][yy][zz];
		}
		return false;
	}

};

#endif