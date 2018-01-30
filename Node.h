#ifndef NODE_H
#define NODE_H

#include "Position.h"

class Node
{

public:

	Node(const FPosition & p) : F(0ull), G(0ull), pos(p), parent(NULL), flag(0) {}

	uint64_t F, G;
	const FPosition pos;
	const Node * parent;

#pragma region Open/Closed_methods

	inline void SetOpen()
	{
		flag |= uint8_t(1);
	}

	inline void SetClosed()
	{
		flag |= uint8_t(2);
	}

	inline bool IsOpen() const
	{
		return !!(flag & uint8_t(1));
	}

	inline bool IsClosed() const
	{
		return !!(flag & uint8_t(2));
	}

#pragma endregion

	inline void ResetState()
	{
		F = 0ull;
		G = 0ull;
		parent = NULL;
		flag = 0;
	}

private:

	uint8_t flag;

};

#endif