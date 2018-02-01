#ifndef SEARCHER_H
#define SEARCHER_H

#include <map>
#include <vector>
#include <algorithm>

#include "EDiagonalMovement.h"
#include "Position.h"
#include "Node.h"
#include "Grid.h"
#include "Openlist.h"

inline static FPosition NewPos(unsigned x, unsigned y, unsigned z)
{
	return FPosition(x, y, z);
}

static const FPosition InvalidPos = FPosition();

#pragma region Heuristics

inline unsigned Manhattan(const Node * a, const Node * b)
{
	return abs(int(a->pos.x - b->pos.x)) + abs(int(a->pos.y - b->pos.y)) + abs(int(a->pos.z - b->pos.z));
}

inline unsigned Euclidean(const Node* a, const Node* b)
{
	float fx = float(int(a->pos.x - b->pos.x));
	float fy = float(int(a->pos.y - b->pos.y));
	float fz = float(int(a->pos.z - b->pos.z));
	return unsigned(sqrtf(fx * fx + fy * fy + fz * fz));
}

#pragma endregion

#define GridMap std::map<FPosition, Node>
#define PositionVector std::vector<FPosition>

// uncommment to debug
#define JPS_DEBUG
#ifdef JPS_DEBUG
#include <cassert>
#define JPS_ASSERT(cond) assert(cond)
#else
#define JPS_ASSERT(cond)
#endif
//

class Searcher
{

public:

	Searcher(FGrid& g, DiagonalMovement d = DiagonalMovement::Always) : grid(g), dMove(d), finishNode(NULL), skip(1U), stepsTotal(0U) {}

	void FreeMemory()
	{
		openlist.Clear();
		GridMap().swap(gridmap);
		stepsTotal = 0U;
	}

	inline void SetSkip(unsigned s)
	{
		skip = std::max(s, 1U);
	}

	/*
		Main method of the class;
		Returns: 1) empty vector - the path does not exist or some exception has been thrown
				 2) vector with the only element - the start and the finish match
				 3) vector with the full path
	*/
	PositionVector FindPath(FPosition Start, FPosition Finish);

private:

	FGrid& grid;
	DiagonalMovement dMove;
	Openlist openlist;
	GridMap gridmap;
	Node * startNode;
	Node * finishNode;
	unsigned skip;
	unsigned stepsTotal;

#pragma region Auxiliary_Private_Methods_Declarations

	Node * getNode(const FPosition & p);
	void addToBuf(const unsigned x, const unsigned y, const unsigned z, FPosition *& buf) const;
	void addToBufCheck(const int x, const int y, const int z, FPosition *& buf) const;
	bool checkD(const int x, const int y, const int z, const int dx, const int dy, const int dz) const;

	FPosition jumpXYZ(FPosition p, const int dx, const int dy, const int dz);
	FPosition jumpXY(FPosition p, const int dx, const int dy);
	FPosition jumpXZ(FPosition p, const int dx, const int dz);
	FPosition jumpYZ(FPosition p, const int dy, const int dz);
	FPosition jumpX(FPosition p, const int dx);
	FPosition jumpY(FPosition p, const int dy);
	FPosition jumpZ(FPosition p, const int dz);
	
#pragma endregion

#pragma region Main_Private_Methods_Declarations

	inline void IdentifySuccessors(const Node * n);
	inline unsigned FindNeighbours(const Node * n, FPosition * Buf) const;
	inline FPosition Jump(const FPosition & Dest, const FPosition & Src);
	inline PositionVector BacktracePath(const Node * end) const;

#pragma endregion

};

inline PositionVector Searcher::FindPath(FPosition Start, FPosition Finish)
{
	if (!grid(Start) || !grid(Finish))
	{
		// 1) the path does not exist
		return PositionVector();
	}
	if (Start == Finish)
	{
		// 2) the start and the finish match
		PositionVector v;
		v.push_back(Start);
		return v;
	}

	for (GridMap::iterator it = gridmap.begin(); it != gridmap.end(); ++it)
	{
		it->second.ResetState();
	}
	openlist.Clear();

	Start.Normalize(skip);
	Finish.Normalize(skip);

	startNode = getNode(Start);
	finishNode = getNode(Finish);

	JPS_ASSERT(startNode && finishNode);
	if (!startNode || !finishNode)
	{
		// 1) null exception
		return PositionVector();
	}

	grid.SetStart(Start);
	grid.SetFinish(Finish);

	openlist.push(startNode);

	while (!openlist.Empty())
	{
		Node * cur = openlist.pop();
		cur->SetClosed();
		if (cur == finishNode)
		{
			// 3) full path
			return BacktracePath(cur);
		}
		IdentifySuccessors(cur);
	}
	// 1) the path does not exist
	return PositionVector();
}

#pragma region Auxiliary_Private_Methods_Definitions

inline Node * Searcher::getNode(const FPosition & p)
{
	JPS_ASSERT(grid(p.x, p.y, p.z));
	if (!grid(p.x, p.y, p.z))
	{
		return NULL;
	}
	return &gridmap.insert(std::make_pair(p, Node(p))).first->second;
}

inline void Searcher::addToBuf(const unsigned x, const unsigned y, const unsigned z, FPosition *& buf) const
{
	*buf = NewPos(x, y, z);
	++buf;
}

inline void Searcher::addToBufCheck(const int x, const int y, const int z, FPosition *& buf) const
{
	if (grid(x, y, z))
	{
		addToBuf(x, y, z, buf);
	}
}

inline bool Searcher::checkD(const int x, const int y, const int z, const int dx, const int dy, const int dz) const
{
	return grid(x + dx, y + dy, z + dz);
}

#pragma region Jumps
// partially ready
inline FPosition Searcher::jumpXYZ(FPosition p, const int dx, const int dy, const int dz)
{
	JPS_ASSERT(grid(p) && dx && dy && dz);
	if (!(grid(p) && dx && dy && dz))
	{
		return InvalidPos;
	}

	const FPosition finpos = finishNode->pos;
	unsigned steps = 0;

	switch (dMove)
	{
		case DiagonalMovement::Always:
			while (true)
			{
				if (p == finpos)
				{
					break;
				}

				++steps;

				const unsigned x = p.x;
				const unsigned y = p.y;
				const unsigned z = p.z;

				// forced
				{
					// 3D
					{
						if (grid(x - dx, y + dy, z + dz) && !grid(x - dx, y, z) ||
							grid(x + dx, y - dy, z + dz) && !grid(x, y - dy, z) ||
							grid(x + dx, y + dy, z - dz) && !grid(x, y, z - dz) ||
							grid(x - dx, y - dy, z + dz) && !grid(x - dx, y - dy, z) && !grid(x - dx, y, z) && !grid(x, y - dy, z) ||
							grid(x - dx, y + dy, z - dz) && !grid(x - dx, y, z - dz) && !grid(x - dx, y, z) && !grid(x, y, z - dz) ||
							grid(x + dx, y - dy, z - dz) && !grid(x, y - dy, z - dz) && !grid(x, y - dy, z) && !grid(x, y, z - dz))
						{
							break;
						}
					}
					// !3D
					// 2D
					{
						if (grid(x - dx, y + dy, z) && !grid(x - dx, y, z) && !grid(x - dx, y, z - dz) ||
							grid(x - dx, y, z + dz) && !grid(x - dx, y, z) && !grid(x - dx, y - dy, z) ||
							grid(x + dx, y - dy, z) && !grid(x, y - dy, z) && !grid(x, y - dy, z - dz) ||
							grid(x, y - dy, z + dz) && !grid(x, y - dy, z) && !grid(x - dx, y - dy, z) ||
							grid(x + dx, y, z - dz) && !grid(x, y, z - dz) && !grid(x, y - dy, z - dz) ||
							grid(x, y + dy, z - dz) && !grid(x, y, z - dz) && !grid(x - dx, y, z - dz))
						{
							break;
						}
					}
					// !2D
				}
				// !forced

				// recursion
				{
					if (grid(x + dx, y, z) && jumpX(NewPos(x + dx, y, z), dx).IsValid())
					{
						break;
					}
					if (grid(x, y + dy, z) && jumpY(NewPos(x, y + dy, z), dy).IsValid())
					{
						break;
					}
					if (grid(x, y, z + dz) && jumpZ(NewPos(x, y, z + dz), dz).IsValid())
					{
						break;
					}

					if (grid(x + dx, y + dy, z) && jumpXY(NewPos(x + dx, y + dy, z), dx, dy).IsValid())
					{
						break;
					}
					if (grid(x + dx, y, z + dz) && jumpXZ(NewPos(x + dx, y, z + dz), dx, dz).IsValid())
					{
						break;
					}
					if (grid(x, y + dy, z + dz) && jumpYZ(NewPos(x, y + dy, z + dz), dy, dz).IsValid())
					{
						break;
					}
				}
				// !recursion

				if (grid(x + dx, y + dy, z + dz))
				{
					p.x += dx;
					p.y += dy;
					p.z += dz;
				}
				else
				{
					p = InvalidPos;
					break;
				}
			}
			break;
	}

	stepsTotal += steps;

	return p;
}

#pragma region 2D_Jumps

inline FPosition Searcher::jumpXY(FPosition p, const int dx, const int dy)
{
	JPS_ASSERT(grid(p) && dx && dy);
	if (!(grid(p) && dx && dy))
	{
		return InvalidPos;
	}

	const FPosition finpos = finishNode->pos;
	unsigned steps = 0;
	const int skip = this->skip;

	switch (dMove)
	{
		case DiagonalMovement::Always:
			while (true)
			{
				if (p == finpos)
				{
					break;
				}

				++steps;

				const unsigned x = p.x;
				const unsigned y = p.y;
				const unsigned z = p.z;

				// forced
				{
					if (grid(x - dx, y + dy, z) && !grid(x - dx, y, z) ||
						grid(x + dx, y - dy, z) && !grid(x, y - dy, z))
					{
						break;
					}

					bool tcheck = false;
					for (int tdz = -skip; tdz < skip + 1; tdz += (skip << 1))
					{
						const int zz = z + tdz;
						if (!grid(x, y, zz))
						{
							if (grid(x + dx, y, zz) ||
								grid(x, y + dy, zz) ||
								grid(x + dx, y + dy, zz) ||
								grid(x + dx, y - dy, zz) && !grid(x, y - dy, zz) && !grid(x, y - dy, z) ||
								grid(x - dx, y + dy, zz) && !grid(x - dx, y, zz) && !grid(x - dx, y, z))
							{
								tcheck = true;
								break;
							}
						}
					}
					if (tcheck)
					{
						break;
					}
				}
				// !forced

				// recursion
				{
					if (grid(x + dx, y, z) && jumpX(NewPos(x + dx, y, z), dx).IsValid())
					{
						break;
					}
					if (grid(x, y + dy, z) && jumpY(NewPos(x, y + dy, z), dy).IsValid())
					{
						break;
					}
				}
				// !recursion

				if (grid(x + dx, y + dy, z))
				{
					p.x += dx;
					p.y += dy;
				}
				else
				{
					p = InvalidPos;
					break;
				}
			}
			break;
		case DiagonalMovement::AtLeastOnePassable:
			break;
		case DiagonalMovement::AllPassable:
			break;
		case DiagonalMovement::Never:
			/*
			while (true)
			{
			if (p == finpos)
			{
			break;
			}

			++steps;

			const unsigned x = p.x;
			const unsigned y = p.y;
			const unsigned z = p.z;


			}
			*/
			break;
		default:
			break;
	}

	stepsTotal += steps;

	return p;
}

inline FPosition Searcher::jumpXZ(FPosition p, const int dx, const int dz)
{
	JPS_ASSERT(grid(p) && dx && dz);
	if (!(grid(p) && dx && dz))
	{
		return InvalidPos;
	}

	const FPosition finpos = finishNode->pos;
	unsigned steps = 0;
	const int skip = this->skip;

	switch (dMove)
	{
		case DiagonalMovement::Always:
			while (true)
			{
				if (p == finpos)
				{
					break;
				}

				++steps;

				const unsigned x = p.x;
				const unsigned y = p.y;
				const unsigned z = p.z;

				// forced
				{
					if (grid(x - dx, y, z + dz) && !grid(x - dx, y, z) ||
						grid(x + dx, y, z - dz) && !grid(x, y, z - dz))
					{
						break;
					}

					bool tcheck = false;
					// must explain all these 'for'-s later
					//
					// DO NOT FORGET
					//
					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					for (int tdy = -skip; tdy < skip + 1; tdy += (skip << 1))
					{
						const int yy = y + tdy;
						if (!grid(x, yy, z))
						{
							if (grid(x + dx, yy, z) ||
								grid(x, yy, z + dz) ||
								grid(x + dx, yy, z + dz) ||
								grid(x + dx, yy, z - dz) && !grid(x, yy, z - dz) && !grid(x, y, z - dz) ||
								grid(x - dx, yy, z + dz) && !grid(x - dx, yy, z) && !grid(x - dx, y, z))
							{
								tcheck = true;
								break;
							}
						}
					}
					if (tcheck)
					{
						break;
					}
				}
				// !forced

				// recursion
				{
					if (grid(x + dx, y, z) && jumpX(NewPos(x + dx, y, z), dx).IsValid())
					{
						break;
					}
					if (grid(x, y, z + dz) && jumpZ(NewPos(x, y, z + dz), dz).IsValid())
					{
						break;
					}
				}
				// !recursion

				if (grid(x + dx, y, z + dz))
				{
					p.x += dx;
					p.z += dz;
				}
				else
				{
					p = InvalidPos;
					break;
				}
			}
			break;
		case DiagonalMovement::AtLeastOnePassable:
			break;
		case DiagonalMovement::AllPassable:
			break;
		case DiagonalMovement::Never:
			/*
			while (true)
			{
				if (p == finpos)
				{
					break;
				}

				++steps;

				const unsigned x = p.x;
				const unsigned y = p.y;
				const unsigned z = p.z;


			}
			*/
			break;
		default:
			break;
	}

	stepsTotal += steps;

	return p;
}

inline FPosition Searcher::jumpYZ(FPosition p, const int dy, const int dz)
{
	JPS_ASSERT(grid(p) && dy && dz);
	if (!(grid(p) && dy && dz))
	{
		return InvalidPos;
	}

	const FPosition finpos = finishNode->pos;
	unsigned steps = 0;
	const int skip = this->skip;

	switch (dMove)
	{
		case DiagonalMovement::Always:
			while (true)
			{
				if (p == finpos)
				{
					break;
				}
	
				++steps;
	
				const unsigned x = p.x;
				const unsigned y = p.y;
				const unsigned z = p.z;
	
				// forced
				{
					if (grid(x, y - dy, z + dz) && !grid(x, y - dy, z) ||
						grid(x, y + dy, z - dz) && !grid(x, y, z - dz))
					{
						break;
					}

					bool tcheck = false;
					// must explain all these 'for'-s later
					//
					// DO NOT FORGET
					//
					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					for (int tdx = -skip; tdx < skip + 1; tdx += (skip << 1))
					{
						const int xx = x + tdx;
						if (!grid(xx, y, z))
						{
							if (grid(xx, y + dy, z) ||
								grid(xx, y, z + dz) ||
								grid(xx, y + dy, z + dz) ||
								grid(xx, y + dy, z - dz) && !grid(xx, y, z - dz) && !grid(x, y, z - dz) ||
								grid(xx, y - dy, z + dz) && !grid(xx, y - dy, z) && !grid(x, y - dy, z))
							{
								tcheck = true;
								break;
							}
						}
					}
					if (tcheck)
					{
						break;
					}
				}
				// !forced

				// recursion
				{
					if (grid(x, y + dy, z) && jumpY(NewPos(x, y + dy, z), dy).IsValid())
					{
						break;
					}
					if (grid(x, y, z + dz) && jumpZ(NewPos(x, y, z + dz), dz).IsValid())
					{
						break;
					}
				}
				// !recursion

				if (grid(x, y + dy, z + dz))
				{
					p.y += dy;
					p.z += dz;
				}
				else
				{
					p = InvalidPos;
					break;
				}
			}
			break;
		case DiagonalMovement::AtLeastOnePassable:
			break;
		case DiagonalMovement::AllPassable:
			break;
		case DiagonalMovement::Never:
			/*
			while (true)
			{
				if (p == finpos)
				{
					break;
				}
	
				++steps;
	
				const unsigned x = p.x;
				const unsigned y = p.y;
				const unsigned z = p.z;
	
	
			}
			*/
			break;
		default:
			break;
	}

	stepsTotal += steps;

	return p;
}

#pragma endregion

#pragma region 1D_Jumps

inline FPosition Searcher::jumpX(FPosition p, const int dx)
{
	JPS_ASSERT(grid(p) && dx);
	if (!(grid(p) && dx))
	{
		return InvalidPos;
	}

	const FPosition finpos = finishNode->pos;
	unsigned steps = 0;
	const int skip = this->skip;

	switch (dMove)
	{
	case DiagonalMovement::Always:
		while (true)
		{
			if (p == finpos)
			{
				break;
			}

			++steps;

			const unsigned x = p.x;
			const unsigned y = p.y;
			const unsigned z = p.z;

			// forced
			{
				const int xx = x + dx;
				if (grid(xx, y + skip, z) && !grid(x, y + skip, z) ||
					grid(xx, y - skip, z) && !grid(x, y - skip, z) ||
					grid(xx, y, z + skip) && !grid(x, y, z + skip) ||
					grid(xx, y, z - skip) && !grid(x, y, z - skip) ||
					grid(xx, y + skip, z + skip) && !grid(x, y + skip, z + skip) && !grid(x, y + skip, z) && !grid(x, y, z + skip) ||
					grid(xx, y - skip, z + skip) && !grid(x, y - skip, z + skip) && !grid(x, y - skip, z) && !grid(x, y, z + skip) ||
					grid(xx, y + skip, z - skip) && !grid(x, y + skip, z - skip) && !grid(x, y + skip, z) && !grid(x, y, z - skip) ||
					grid(xx, y - skip, z - skip) && !grid(x, y - skip, z - skip) && !grid(x, y - skip, z) && !grid(x, y, z - skip))
				{
					break;
				}
			}
			// !forced

			if (grid(x + dx, y, z))
			{
				p.x += dx;
			}
			else
			{
				p = InvalidPos;
				break;
			}
		}
		break;
	case DiagonalMovement::AtLeastOnePassable:
		break;
	case DiagonalMovement::AllPassable:
		break;
	case DiagonalMovement::Never:
		/*
		while (true)
		{
			if (p == finpos)
			{
				break;
			}

			++steps;

			const unsigned x = p.x;
			const unsigned y = p.y;
			const unsigned z = p.z;


		}
		*/
		break;
	default:
		break;
	}

	stepsTotal += steps;

	return p;
}

inline FPosition Searcher::jumpY(FPosition p, const int dy)
{
	JPS_ASSERT(grid(p) && dy);
	if (!(grid(p) && dy))
	{
		return InvalidPos;
	}

	const FPosition finpos = finishNode->pos;
	unsigned steps = 0;
	const int skip = this->skip;

	switch (dMove)
	{
	case DiagonalMovement::Always:
		while (true)
		{
			if (p == finpos)
			{
				break;
			}

			++steps;

			const unsigned x = p.x;
			const unsigned y = p.y;
			const unsigned z = p.z;

			// forced
			{
				const int yy = y + dy;
				if (grid(x + skip, yy, z) && !grid(x + skip, y, z) ||
					grid(x - skip, yy, z) && !grid(x - skip, y, z) ||
					grid(x, yy, z + skip) && !grid(x, y, z + skip) ||
					grid(x, yy, z - skip) && !grid(x, y, z - skip) ||
					grid(x + skip, yy, z + skip) && !grid(x + skip, y, z + skip) && !grid(x + skip, y, z) && !grid(x, y, z + skip) ||
					grid(x - skip, yy, z + skip) && !grid(x - skip, y, z + skip) && !grid(x - skip, y, z) && !grid(x, y, z + skip) ||
					grid(x + skip, yy, z - skip) && !grid(x + skip, y, z - skip) && !grid(x + skip, y, z) && !grid(x, y, z - skip) ||
					grid(x - skip, yy, z - skip) && !grid(x - skip, y, z - skip) && !grid(x - skip, y, z) && !grid(x, y, z - skip))
				{
					break;
				}
			}
			// !forced

			if (grid(x, y + dy, z))
			{
				p.y += dy;
			}
			else
			{
				p = InvalidPos;
				break;
			}
		}
		break;
	case DiagonalMovement::AtLeastOnePassable:
		break;
	case DiagonalMovement::AllPassable:
		break;
	case DiagonalMovement::Never:
		/*
		while (true)
		{
		if (p == finpos)
		{
		break;
		}

		++steps;

		const unsigned x = p.x;
		const unsigned y = p.y;
		const unsigned z = p.z;


		}
		*/
		break;
	default:
		break;
	}

	stepsTotal += steps;

	return p;
}

inline FPosition Searcher::jumpZ(FPosition p, const int dz)
{
	JPS_ASSERT(grid(p) && dz);
	if (!(grid(p) && dz))
	{
		return InvalidPos;
	}

	const FPosition finpos = finishNode->pos;
	unsigned steps = 0;
	const int skip = this->skip;

	switch (dMove)
	{
	case DiagonalMovement::Always:
		while (true)
		{
			if (p == finpos)
			{
				break;
			}

			++steps;

			const unsigned x = p.x;
			const unsigned y = p.y;
			const unsigned z = p.z;

			// forced
			{
				const int zz = z + dz;
				if (grid(x + skip, y, zz) && !grid(x + skip, y, z) ||
					grid(x - skip, y, zz) && !grid(x - skip, y, z) ||
					grid(x, y + skip, zz) && !grid(x, y + skip, z) ||
					grid(x, y - skip, zz) && !grid(x, y - skip, z) ||
					grid(x + skip, y + skip, zz) && !grid(x + skip, y + skip, z) && !grid(x + skip, y, z) && !grid(x, y + skip, z) ||
					grid(x - skip, y + skip, zz) && !grid(x - skip, y + skip, z) && !grid(x - skip, y, z) && !grid(x, y + skip, z) ||
					grid(x + skip, y - skip, zz) && !grid(x + skip, y - skip, z) && !grid(x + skip, y, z) && !grid(x, y - skip, z) ||
					grid(x - skip, y - skip, zz) && !grid(x - skip, y - skip, z) && !grid(x - skip, y, z) && !grid(x, y - skip, z))
				{
					break;
				}
			}
			// !forced

			if (grid(x, y, z + dz))
			{
				p.z += dz;
			}
			else
			{
				p = InvalidPos;
				break;
			}
		}
		break;
	case DiagonalMovement::AtLeastOnePassable:
		break;
	case DiagonalMovement::AllPassable:
		break;
	case DiagonalMovement::Never:
		/*
		while (true)
		{
		if (p == finpos)
		{
		break;
		}

		++steps;

		const unsigned x = p.x;
		const unsigned y = p.y;
		const unsigned z = p.z;


		}
		*/
		break;
	default:
		break;
	}

	stepsTotal += steps;

	return p;
}

#pragma endregion

#pragma endregion

#pragma endregion

#pragma region Main_Private_Methods_Definitions
// ready
inline void Searcher::IdentifySuccessors(const Node * n)
{
	FPosition buf[26];
	const unsigned cnt = FindNeighbours(n, &buf[0]);

	for (unsigned i = 0; i < cnt; ++i)
	{
		FPosition jp = Jump(buf[i], n->pos);
		if (!jp.IsValid())
		{
			continue;
		}

		Node * jn = getNode(jp);
		JPS_ASSERT(jn && jn != n);
		if (!jn || jn == n || jn->IsClosed())
		{
			continue;
		}

		unsigned curG = Euclidean(jn, n);
		unsigned newG = n->G + curG;

		if (!jn->IsOpen() || newG < jn->G)
		{
			jn->G = newG;
			jn->F = jn->G + Manhattan(jn, finishNode);
			jn->parent = n;

			if (!jn->IsOpen())
			{
				jn->SetOpen();
				openlist.push(jn);
			}
			else
			{
				openlist.heapify();
			}
		}
	}
}
// partially ready
inline unsigned Searcher::FindNeighbours(const Node * n, FPosition * Buf) const
{
	FPosition * p = Buf;
	const unsigned x = n->pos.x;
	const unsigned y = n->pos.y;
	const unsigned z = n->pos.z;
	//lock skip;
	const unsigned uskip = this->skip;
	const int skip = this->skip;
	//
	bool b[3][3][3];

	/*
	* ^ y
	* |					z - skip									z										z + skip
	* ----------------------------------------	----------------------------------------	----------------------------------------
	* | b[0][2][0] | b[1][2][0] | b[2][2][0] |	| b[0][2][1] | b[1][2][1] | b[2][2][1] |	| b[0][2][2] | b[1][2][2] | b[2][2][2] |
	* ----------------------------------------	----------------------------------------	----------------------------------------
	* | b[0][1][0] | b[1][1][0] | b[2][1][0] |	| b[0][1][1] | b[1][1][1] | b[2][1][1] |	| b[0][1][2] | b[1][1][2] | b[2][1][2] |
	* ----------------------------------------	----------------------------------------	----------------------------------------
	* | b[0][0][0] | b[1][0][0] | b[2][0][0] |	| b[0][0][1] | b[1][0][1] | b[2][0][1] |	| b[0][0][2] | b[1][0][2] | b[2][0][2] |  x
	* ----------------------------------------	----------------------------------------	------------------------------------------>
	*/

	if (n->parent)
	{
		int dx = x - n->parent->pos.x;
		if (abs(dx) > 1)
		{
			dx = dx > 0 ? 1 : -1;
		}
		dx *= skip;

		int dy = y - n->parent->pos.y;
		if (abs(dy) > 1)
		{
			dy = dy > 0 ? 1 : -1;
		}
		dy *= skip;

		int dz = z - n->parent->pos.z;
		if (abs(dz) > 1)
		{
			dz = dz > 0 ? 1 : -1;
		}
		dz *= skip;

		switch (dMove)
		{
			case DiagonalMovement::Always:
				if (dx && dy && dz)
				{
					// 1D
					{
						addToBufCheck(x + dx, y, z, p);
						addToBufCheck(x, y + dy, z, p);
						addToBufCheck(x, y, z + dz, p);
					}
					// !1D

					// 2D
					{
						// Oxy
						{
							addToBufCheck(x + dx, y + dy, z, p);
							// forced
							{
								if (grid(x - dx, y + dy, z) &&
									!grid(x - dx, y, z) && !grid(x - dx, y, z - dz))
								{
									addToBuf(x - dx, y + dy, z, p);
								}
								if (grid(x + dx, y - dy, z) &&
									!grid(x, y - dy, z) && !grid(x, y - dy, z - dz))
								{
									addToBuf(x + dx, y - dy, z, p);
								}
							}
							// !forced
						}
						// !Oxy

						// Oxz
						{
							addToBufCheck(x + dx, y, z + dz, p);
							// forced
							{
								if (grid(x - dx, y, z + dz) &&
									!grid(x - dx, y, z) && !grid(x - dx, y - dy, z))
								{
									addToBuf(x - dx, y, z + dz, p);
								}
								if (grid(x + dx, y, z - dz) &&
									!grid(x, y, z - dz) && !grid(x, y - dy, z - dz))
								{
									addToBuf(x + dx, y, z - dz, p);
								}
							}
							// !forced
						}
						// !Oxz

						// Oyz
						{
							addToBufCheck(x, y + dy, z + dz, p);
							// forced
							{
								if (grid(x, y - dy, z + dz) &&
									!grid(x, y - dy, z) && !grid(x - dx, y - dy, z))
								{
									addToBuf(x, y - dy, z + dz, p);
								}
								if (grid(x, y + dy, z - dz) &&
									!grid(x, y, z - dz) && !grid(x - dx, y, z - dz))
								{
									addToBuf(x, y + dy, z - dz, p);
								}
							}
							// !forced
						}
						// !Oyz
					}
					// !2D

					// 3D
					{
						addToBufCheck(x + dx, y + dy, z + dz, p);
						// forced
						// one negative delta
						if (grid(x + dx, y + dy, z - dz) && !grid(x, y, z - dz))
						{
							addToBuf(x + dx, y + dy, z - dz, p);
						}
						if (grid(x + dx, y - dy, z + dz) && !grid(x, y - dy, z))
						{
							addToBuf(x + dx, y - dy, z + dz, p);
						}
						if (grid(x - dx, y + dy, z + dz) && !grid(x - dx, y, z))
						{
							addToBuf(x - dx, y + dy, z + dz, p);
						}
						// !one negative delta

						// two negative deltas
						if (grid(x + dx, y - dy, z - dz) &&
							!grid(x, y - dy, z - dz) && !grid(x, y - dy, z) && !grid(x, y, z - dz))
						{
							addToBuf(x + dx, y - dy, z - dz, p);
						}
						if (grid(x - dx, y + dy, z - dz) &&
							!grid(x - dx, y, z - dz) && !grid(x - dx, y, z) && !grid(x, y, z - dz))
						{
							addToBuf(x - dx, y + dy, z - dz, p);
						}
						if (grid(x - dx, y - dy, z + dz) &&
							!grid(x - dx, y - dy, z) && !grid(x - dx, y, z) && !grid(x, y - dy, z))
						{
							addToBuf(x - dx, y - dy, z + dz, p);
						}
						// !two negative deltas
						// !forced
					}
					// !3D
				}
				else if (dx && dy)
				{
					// 1D
					{
						addToBufCheck(x + dx, y, z, p);
						addToBufCheck(x, y + dy, z, p);
					}
					// !1D

					// Diagonal
					{
						addToBufCheck(x + dx, y + dy, z, p);
						// forced
						{
							if (grid(x - dx, y + dy, z) && !grid(x - dx, y, z))
							{
								addToBuf(x - dx, y + dy, z, p);
							}
							if (grid(x + dx, y - dy, z) && !grid(x, y - dy, z))
							{
								addToBuf(x + dx, y - dy, z, p);
							}
							// Oz
							for (int tdz = -skip; tdz < skip + 1; tdz += (skip << 1))
							{
								if (!grid(x, y, z + tdz))
								{
									addToBufCheck(x, y + dy, z + tdz, p);
									addToBufCheck(x + dx, y, z + tdz, p);
									addToBufCheck(x + dx, y + dy, z + tdz, p);

									if (grid(x - dx, y + dy, z + tdz) &&
										!grid(x - dx, y, z + tdz) && !grid(x - dx, y, z))
									{
										addToBuf(x - dx, y + dy, z + tdz, p);
									}
									if (grid(x + dx, y - dy, z + tdz) &&
										!grid(x, y - dy, z + tdz) && !grid(x, y - dy, z))
									{
										addToBuf(x + dx, y - dy, z + tdz, p);
									}
								}
							}
							// !Oz
						}
						// !forced
					}
					// !Diagonal

				}
				else if (dx && dz)
				{
					// 1D
					{
						addToBufCheck(x + dx, y, z, p);
						addToBufCheck(x, y, z + dz, p);
					}
					// !1D

					// Diagonal
					{
						addToBufCheck(x + dx, y, z + dz, p);
						// forced
						{
							if (grid(x - dx, y, z + dz) && !grid(x - dx, y, z))
							{
								addToBuf(x - dx, y, z + dz, p);
							}
							if (grid(x + dx, y, z - dz) && !grid(x, y, z - dz))
							{
								addToBuf(x + dx, y, z - dz, p);
							}
							// Oy
							for (int tdy = -skip; tdy < skip + 1; tdy += (skip << 1))
							{
								if (!grid(x, y + tdy, z))
								{
									addToBufCheck(x + dx, y + tdy, z, p);
									addToBufCheck(x, y + tdy, z + dz, p);
									addToBufCheck(x + dx, y + tdy, z + dz, p);

									if (grid(x - dx, y + tdy, z + dz) &&
										!grid(x - dx, y + tdy, z) && !grid(x - dx, y, z))
									{
										addToBuf(x - dx, y + tdy, z + dz, p);
									}
									if (grid(x + dx, y + tdy, z - dz) &&
										!grid(x, y + tdy, z - dz) && !grid(x, y, z - dz))
									{
										addToBuf(x + dx, y + tdy, z - dz, p);
									}
								}
							}
							// !Oy
						}
						// !forced
					}
					// !Diagonal
				}
				else if (dy && dz)
				{
					// 1D
					{
						addToBufCheck(x, y + dy, z, p);
						addToBufCheck(x, y, z + dz, p);
					}
					// !1D

					// Diagonal
					{
						addToBufCheck(x, y + dy, z + dz, p);
						// forced
						{
							if (grid(x, y - dy, z + dz) && !grid(x, y - dy, z))
							{
								addToBuf(x, y - dy, z + dz, p);
							}
							if (grid(x, y + dy, z - dz) && !grid(x, y, z - dz))
							{
								addToBuf(x, y + dy, z - dz, p);
							}
							// Ox
							for (int tdx = -skip; tdx < skip + 1; tdx += (skip << 1))
							{
								if (!grid(x + tdx, y, z))
								{
									addToBufCheck(x + tdx, y + dy, z, p);
									addToBufCheck(x + tdx, y, z + dz, p);
									addToBufCheck(x + tdx, y + dy, z + dz, p);

									if (grid(x + tdx, y - dy, z + dz) &&
										!grid(x + tdx, y - dy, z) && !grid(x, y - dy, z))
									{
										addToBuf(x + tdx, y - dy, z + dz, p);
									}
									if (grid(x + tdx, y + dy, z - dz) &&
										!grid(x + tdx, y, z - dz) && !grid(x, y, z - dz))
									{
										addToBuf(x + tdx, y + dy, z - dz, p);
									}
								}
							}
							// !Ox
						}
						// !forced
					}
					// !Diagonal
				}
				else if (dx)
				{
					addToBufCheck(x + dx, y, z, p);
					if (grid(x + dx, y + skip, z) && !grid(x, y + skip, z))
					{
						addToBuf(x + dx, y + skip, z, p);
					}
					if (grid(x + dx, y - skip, z) && !grid(x, y - skip, z))
					{
						addToBuf(x + dx, y - skip, z, p);
					}

					for (int tdz = -skip; tdz < skip + 1; tdz += (skip << 1))
					{
						if (!grid(x, y, z + tdz))
						{
							addToBufCheck(x + dx, y, z + tdz, p);

							if (grid(x + dx, y + skip, z + tdz) && !grid(x, y + skip, z + tdz))
							{
								addToBuf(x + dx, y + skip, z + tdz, p);
							}
							if (grid(x + dx, y - skip, z + tdz) && !grid(x, y - skip, z + tdz))
							{
								addToBuf(x + dx, y - skip, z + tdz, p);
							}
						}
					}
				}
				else if (dy)
				{
					addToBufCheck(x, y + dy, z, p);
					if (grid(x + skip, y + dy, z) && !grid(x + skip, y, z))
					{
						addToBuf(x + skip, y + dy, z, p);
					}
					if (grid(x - skip, y + dy, z) && !grid(x - skip, y, z))
					{
						addToBuf(x - skip, y + dy, z, p);
					}

					for (int tdz = -skip; tdz < skip + 1; tdz += (skip << 1))
					{
						if (!grid(x, y, z + tdz))
						{
							addToBufCheck(x, y + dy, z + tdz, p);

							if (grid(x + skip, y + dy, z + tdz) && !grid(x + skip, y, z + tdz))
							{
								addToBuf(x + skip, y + dy, z + tdz, p);
							}
							if (grid(x - skip, y + dy, z + tdz) && !grid(x - skip, y, z + tdz))
							{
								addToBuf(x - skip, y + dy, z + tdz, p);
							}
						}
					}
				}
				else if (dz)
				{
					addToBufCheck(x, y, z + dz, p);
					if (grid(x + skip, y, z + dz) && !grid(x + skip, y, z))
					{
						addToBuf(x + skip, y, z + dz, p);
					}
					if (grid(x - skip, y, z + dz) && !grid(x - skip, y, z))
					{
						addToBuf(x + skip, y, z + dz, p);
					}

					for (int tdy = -skip; tdy < skip + 1; tdy += (skip << 1))
					{
						if (!grid(x, y + tdy, z))
						{
							addToBufCheck(x, y + tdy, z + dz, p);

							if (grid(x + skip, y + tdy, z + dz) && !grid(x + skip, y + tdy, z))
							{
								addToBuf(x + skip, y + tdy, z + dz, p);
							}
							if (grid(x - skip, y + tdy, z + dz) && !grid(x - skip, y + tdy, z))
							{
								addToBuf(x - skip, y + tdy, z + dz, p);
							}
						}
					}
				}
				break;
			case DiagonalMovement::AtLeastOnePassable:
				if (dx && dy && dz)
				{
					// 1D
					b[2][1][1] = grid(x + dx, y, z);
					if (b[2][1][1])
					{
						addToBuf(x + dx, y, z, p);
					}
					b[1][2][1] = grid(x, y + dy, z);
					if (b[1][2][1])
					{
						addToBuf(x, y + dy, z, p);
					}
					b[1][1][2] = grid(x, y, z + dz);
					if (b[1][1][2])
					{
						addToBuf(x, y, z + dz, p);
					}
					// !1D

					// 2D
					// Oxy
					b[2][2][1] = grid(x + dx, y + dy, z) && (b[2][1][1] || b[1][2][1]);
					if (b[2][2][1])
					{
						addToBuf(x + dx, y + dy, z, p);
					}					
					// forced
					b[0][1][1] = grid(x - dx, y, z);
					b[0][2][1] = grid(x - dx, y + dy, z) && (b[1][2][1] || b[0][1][1]);
					b[0][1][0] = grid(x - dx, y, z - dz);
					if (b[0][2][1] && !b[0][1][1] && !b[0][1][0])
					{
						addToBuf(x - dx, y + dy, z, p);
					}
					b[1][0][1] = grid(x, y - dy, z);
					b[2][0][1] = grid(x + dx, y - dy, z) && (b[2][1][1] || b[1][0][1]);
					b[1][0][0] = grid(x, y - dy, z - dz);
					if (b[2][0][1] && !b[1][0][1] && !b[1][0][0])
					{
						addToBuf(x + dx, y - dy, z, p);
					}
					// !forced
					// !Oxy

					// Oxz
					b[2][1][2] = grid(x + dx, y, z + dz) && (b[2][1][1] || b[1][1][2]);
					if (b[2][1][2])
					{
						addToBufCheck(x + dx, y, z + dz, p);
					}
					// forced
					b[0][1][2] = grid(x - dx, y, z + dz);
					b[0][1][1] = grid(x - dx, y, z);
					b[0][0][1] = grid(x - dx, y - dy, z);
					if (grid(x - dx, y, z + dz) &&
						!grid(x - dx, y, z) && !grid(x - dx, y - dy, z))
					{
						addToBuf(x - dx, y, z + dz, p);
					}
					if (grid(x + dx, y, z - dz) &&
						!grid(x, y, z - dz) && !grid(x, y - dy, z - dz))
					{
						addToBuf(x + dx, y, z - dz, p);
					}
					// !forced
					// !Oxz

					// Oyz
					addToBufCheck(x, y + dy, z + dz, p);
					// forced
					if (grid(x, y - dy, z + dz) &&
						!grid(x, y - dy, z) && !grid(x - dx, y - dy, z))
					{
						addToBuf(x, y - dy, z + dz, p);
					}
					if (grid(x, y + dy, z - dz) &&
						!grid(x, y, z - dz) && !grid(x - dx, y, z - dz))
					{
						addToBuf(x, y + dy, z - dz, p);
					}
					// !forced
					// !Oyz
					// !2D

					// 3D
					addToBufCheck(x + dx, y + dy, z + dz, p);
					// forced
					// one negative delta
					if (grid(x + dx, y + dy, z - dz) && !grid(x, y, z - dz))
					{
						addToBuf(x + dx, y + dy, z - dz, p);
					}
					if (grid(x + dx, y - dy, z + dz) && !grid(x, y - dy, z))
					{
						addToBuf(x + dx, y - dy, z + dz, p);
					}
					if (grid(x - dx, y + dy, z + dz) && !grid(x - dx, y, z))
					{
						addToBuf(x - dx, y + dy, z + dz, p);
					}
					// !one negative delta

					// two negative deltas
					if (grid(x + dx, y - dy, z - dz) &&
						!grid(x, y - dy, z - dz) && !grid(x, y - dy, z) && !grid(x, y, z - dz))
					{
						addToBuf(x + dx, y - dy, z - dz, p);
					}
					if (grid(x - dx, y + dy, z - dz) &&
						!grid(x - dx, y, z - dz) && !grid(x - dx, y, z) && !grid(x, y, z - dz))
					{
						addToBuf(x - dx, y + dy, z - dz, p);
					}
					if (grid(x - dx, y - dy, z + dz) &&
						!grid(x - dx, y - dy, z) && !grid(x - dx, y, z) && !grid(x, y - dy, z))
					{
						addToBuf(x - dx, y - dy, z + dz, p);
					}
					// !two negative deltas
					// !forced
					// !3D
				}
				break;
		}


		return unsigned(p - Buf);
	}
	// there is no parent, so get all valid neighbours

#pragma region No_parent_Straight_neighbours_(1D)

	b[2][1][1] = grid(x + uskip, y, z);
	if (b[2][1][1])
	{
		addToBuf(x + uskip, y, z, p);
	}

	b[0][1][1] = grid(x - uskip, y, z);
	if (b[0][1][1])
	{
		addToBuf(x - uskip, y, z, p);
	}

	b[1][2][1] = grid(x, y + uskip, z);
	if (b[1][2][1])
	{
		addToBuf(x, y + uskip, z, p);
	}

	b[1][0][1] = grid(x, y - uskip, z);
	if (b[1][0][1])
	{
		addToBuf(x, y - uskip, z, p);
	}

	b[1][1][2] = grid(x, y, z + uskip);
	if (b[1][1][2])
	{
		addToBuf(x, y, z + uskip, p);
	}

	b[1][1][0] = grid(x, y, z - uskip);
	if (b[1][1][0])
	{
		addToBuf(x, y, z - uskip, p);
	}

	// if diagonal movement is not allowed, there is no point going further
	if (dMove == DiagonalMovement::Never)
	{
		return unsigned(p - Buf);
	}

#pragma endregion

#pragma region No_parent_2D_neighbours

#pragma region No_parent_2D_Oxy

	switch (dMove)
	{
		case DiagonalMovement::Always:
			b[0][0][1] = true;
			b[0][2][1] = true;
			b[2][2][1] = true;
			b[2][0][1] = true;
			break;

		case DiagonalMovement::AtLeastOnePassable:
			b[0][0][1] = b[0][1][1] || b[1][0][1];
			b[0][2][1] = b[0][1][1] || b[1][2][1];
			b[2][2][1] = b[2][1][1] || b[1][2][1];
			b[2][0][1] = b[2][1][1] || b[1][0][1];
			break;

		case DiagonalMovement::AllPassable:
			b[0][0][1] = b[0][1][1] && b[1][0][1];
			b[0][2][1] = b[0][1][1] && b[1][2][1];
			b[2][2][1] = b[2][1][1] && b[1][2][1];
			b[2][0][1] = b[2][1][1] && b[1][0][1];
			break;

		case DiagonalMovement::Never:
			break;

		default:
			break;
	}
	for (unsigned i = 0; i < 3; i += 2)
	{
		for (unsigned j = 0; j < 3; j += 2)
		{
			int dx = i == 0 ? -skip : skip;
			int dy = j == 0 ? -skip : skip;

			if (b[i][j][1])
			{
				if (grid(x + dx, y + dy, z))
				{
					addToBuf(x + dx, y + dy, z, p);
				}
				else
				{
					b[i][j][1] = false;
				}
			}
		}
	}

#pragma endregion

#pragma region No_parent_2D_Oxz

	switch (dMove)
	{
		case DiagonalMovement::Always:
			b[0][1][0] = true;
			b[2][1][0] = true;
			b[2][1][2] = true;
			b[0][1][2] = true;
			break;

		case DiagonalMovement::AtLeastOnePassable:
			b[0][1][0] = b[1][1][0] || b[0][1][1];
			b[2][1][0] = b[1][1][0] || b[2][1][1];
			b[2][1][2] = b[1][1][2] || b[2][1][1];
			b[0][1][2] = b[1][1][2] || b[0][1][1];
			break;

		case DiagonalMovement::AllPassable:
			b[0][1][0] = b[1][1][0] && b[0][1][1];
			b[2][1][0] = b[1][1][0] && b[2][1][1];
			b[2][1][2] = b[1][1][2] && b[2][1][1];
			b[0][1][2] = b[1][1][2] && b[0][1][1];
			break;

		case DiagonalMovement::Never:
			break;

		default:
			break;
	}
	for (unsigned i = 0; i < 3; i += 2)
	{
		for (unsigned j = 0; j < 3; j += 2)
		{
			int dx = i == 0 ? -skip : skip;
			int dz = j == 0 ? -skip : skip;

			if (b[i][1][j])
			{
				if (grid(x + dx, y, z + dz))
				{
					addToBuf(x + dx, y, z + dz, p);
				}
				else
				{
					b[i][1][j] = false;
				}
			}
		}
	}

#pragma endregion

#pragma region No_parent_2D_Oyz

	switch (dMove)
	{
		case DiagonalMovement::Always:
			b[1][0][0] = true;
			b[1][0][2] = true;
			b[1][2][2] = true;
			b[1][2][0] = true;
			break;

		case DiagonalMovement::AtLeastOnePassable:
			b[1][0][0] = b[1][0][1] || b[1][1][0];
			b[1][0][2] = b[1][0][1] || b[1][1][2];
			b[1][2][2] = b[1][2][1] || b[1][1][2];
			b[1][2][0] = b[1][2][1] || b[1][1][0];
			break;

		case DiagonalMovement::AllPassable:
			b[1][0][0] = b[1][0][1] && b[1][1][0];
			b[1][0][2] = b[1][0][1] && b[1][1][2];
			b[1][2][2] = b[1][2][1] && b[1][1][2];
			b[1][2][0] = b[1][2][1] && b[1][1][0];
			break;

		case DiagonalMovement::Never:
			break;

		default:
			break;
	}
	for (unsigned i = 0; i < 3; i += 2)
	{
		for (unsigned j = 0; j < 3; j += 2)
		{
			int dy = i == 0 ? -skip : skip;
			int dz = j == 0 ? -skip : skip;

			if (b[1][i][j])
			{
				if (grid(x, y + dy, z + dz))
				{
					addToBuf(x, y + dy, z + dz, p);
				}
				else
				{
					b[1][i][j] = false;
				}
			}
		}
	}

#pragma endregion

#pragma endregion

#pragma region No_parent_3D_neighbours

	switch (dMove)
	{
		case DiagonalMovement::Always:
			b[0][0][0] = true;
			b[0][2][0] = true;
			b[2][2][0] = true;
			b[2][0][0] = true;

			b[0][0][2] = true;
			b[0][2][2] = true;
			b[2][2][2] = true;
			b[2][0][2] = true;
			break;

		case DiagonalMovement::AtLeastOnePassable:
			b[0][0][0] = b[1][1][0] || b[1][0][1] || b[0][1][1] || b[1][0][0] || b[0][1][0] || b[0][0][1];
			b[0][2][0] = b[1][1][0] || b[1][2][1] || b[0][1][1] || b[1][2][0] || b[0][1][0] || b[0][2][1];
			b[2][2][0] = b[1][1][0] || b[1][2][1] || b[2][1][1] || b[1][2][0] || b[2][1][0] || b[2][2][1];
			b[2][0][0] = b[1][1][0] || b[1][0][1] || b[2][1][1] || b[1][0][0] || b[2][1][0] || b[2][0][1];

			b[0][0][2] = b[1][1][2] || b[1][0][1] || b[0][1][1] || b[1][0][2] || b[0][1][2] || b[0][0][1];
			b[0][2][2] = b[1][1][2] || b[1][2][1] || b[0][1][1] || b[1][2][2] || b[0][1][2] || b[0][2][1];
			b[2][2][2] = b[1][1][2] || b[1][2][1] || b[2][1][1] || b[1][2][2] || b[2][1][2] || b[2][2][1];
			b[2][0][2] = b[1][1][2] || b[1][0][1] || b[2][1][1] || b[1][0][2] || b[2][1][2] || b[2][0][1];
			break;

		case DiagonalMovement::AllPassable:
			b[0][0][0] = b[1][1][0] && b[1][0][1] && b[0][1][1] && b[1][0][0] && b[0][1][0] && b[0][0][1];
			b[0][2][0] = b[1][1][0] && b[1][2][1] && b[0][1][1] && b[1][2][0] && b[0][1][0] && b[0][2][1];
			b[2][2][0] = b[1][1][0] && b[1][2][1] && b[2][1][1] && b[1][2][0] && b[2][1][0] && b[2][2][1];
			b[2][0][0] = b[1][1][0] && b[1][0][1] && b[2][1][1] && b[1][0][0] && b[2][1][0] && b[2][0][1];

			b[0][0][2] = b[1][1][2] && b[1][0][1] && b[0][1][1] && b[1][0][2] && b[0][1][2] && b[0][0][1];
			b[0][2][2] = b[1][1][2] && b[1][2][1] && b[0][1][1] && b[1][2][2] && b[0][1][2] && b[0][2][1];
			b[2][2][2] = b[1][1][2] && b[1][2][1] && b[2][1][1] && b[1][2][2] && b[2][1][2] && b[2][2][1];
			b[2][0][2] = b[1][1][2] && b[1][0][1] && b[2][1][1] && b[1][0][2] && b[2][1][2] && b[2][0][1];
			break;

		case DiagonalMovement::Never:
			break;

		default:
			break;
	}

	for (unsigned i = 0; i < 3; i += 2)
	{
		for (unsigned j = 0; j < 3; j += 2)
		{
			int dx = i == 0 ? -skip : skip;
			int dy = j == 0 ? -skip : skip;

			if (b[i][j][0] && grid(x + dx, y + dy, z - skip))
			{
				addToBuf(x + dx, y + dy, z - skip, p);
			}
		}
	}

	for (unsigned i = 0; i < 3; i += 2)
	{
		for (unsigned j = 0; j < 3; j += 2)
		{
			int dx = i == 0 ? -skip : skip;
			int dy = j == 0 ? -skip : skip;

			if (b[i][j][2] && grid(x + dx, y + dy, z + skip))
			{
				addToBuf(x + dx, y + dy, z + skip, p);
			}
		}
	}

#pragma endregion

	return unsigned(p - Buf);
}
// ready
inline FPosition Searcher::Jump(const FPosition & Cur, const FPosition & Src)
{
	JPS_ASSERT(grid(Cur));
	if (!grid(Cur))
	{
		return InvalidPos;
	}

	if (Cur == finishNode->pos)
	{
		return Cur;
	}

	int dx = Cur.x - Src.x;
	int dy = Cur.x - Src.y;
	int dz = Cur.z - Src.z;

	JPS_ASSERT(dx || dy || dz);
	if (!(dx || dy || dz))
	{
		return InvalidPos;
	}

	if (dx && dy && dz)
	{
		return jumpXYZ(Cur, dx, dy, dz);
	}
	else if (dx && dy)
	{
		return jumpXY(Cur, dx, dy);
	}
	else if (dx && dz)
	{
		return jumpXZ(Cur, dx, dz);
	}
	else if (dy && dz)
	{
		return jumpYZ(Cur, dy, dz);
	}
	else if (dx)
	{
		return jumpX(Cur, dx);
	}
	else if (dy)
	{
		return jumpY(Cur, dy);
	}
	else if (dz)
	{
		return jumpZ(Cur, dz);
	}
	// must never reach this
	JPS_ASSERT(false);
	return FPosition();
}
// ready
inline PositionVector Searcher::BacktracePath(const Node * tail) const
{
	JPS_ASSERT(tail == finishNode);
	if (tail != finishNode)
	{
		return PositionVector();
	}

	PositionVector path;
	const Node * cur = tail;
	while (tail)
	{
		JPS_ASSERT(tail != tail->parent);
		path.push_back(tail->pos);
		tail = tail->parent;
	}
	std::reverse(path.begin(), path.end());
	return path;
}

#pragma endregion

#undef PositionVector
#undef GridMap

#undef JPS_ASSERT

#ifdef JPS_DEBUG
#undef JPS_DEBUG
#endif

#endif