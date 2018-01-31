#ifndef NODE_H
#define NODE_H

#include "Position.h"

class Node
{

public:

	Node(const FPosition & p) : F(0U), G(0U), pos(p), parent(NULL), flag(0) {}

	unsigned F, G;
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
		F = 0U;
		G = 0U;
		parent = NULL;
		flag = uint8_t(0);
	}

private:

	uint8_t flag;

};

#endif