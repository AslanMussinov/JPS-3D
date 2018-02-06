#ifndef GRID_H
#define GRID_H

#include <map>
#include <vector>
#include <string>

#include "Position.h"

namespace JPS {

	struct FGrid
	{
		unsigned x, y, z;
		FPosition start, finish;
		std::vector<std::vector<std::vector<int>>> lines;

		FGrid() {}
		FGrid(const std::string filename)
		{

		}
		FGrid(const unsigned xx, const unsigned yy, const unsigned zz, int * cells) : x(xx), y(yy), z(zz), lines(z, std::vector<std::vector<int>>(y, std::vector<int>(x)))
		{
			for (unsigned i = 0; i < z; ++i)
			{
				for (unsigned j = 0; j < y; ++j)
				{
					for (unsigned k = 0; k < x; ++k)
					{
						lines[i][j][k] = *cells;
						++cells;
					}
				}
			}
		}

		inline void Clear()
		{
			lines.clear();
			lines.shrink_to_fit();
		}

		inline void SetStart(FPosition p)
		{
			start = p;
		}

		inline void SetFinish(FPosition p)
		{
			finish = p;
		}

#pragma region Operator()

		inline bool operator()(unsigned xx, unsigned yy, unsigned zz) const
		{
			if (xx < x && yy < y && zz < z)
			{
				return lines[zz][yy][xx] != 0;
			}
			return false;
		}

		inline bool operator()(FPosition p) const
		{
			return operator()(p.x, p.y, p.z);
		}

#pragma endregion

	};

}

#endif