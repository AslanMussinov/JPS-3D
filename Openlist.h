#ifndef OPENLIST_H
#define OPENLIST_H

#include <vector>
#include <algorithm>

#include "Node.h"

namespace JPS {

	class Openlist
	{

	public:

#pragma region Heap_methods

		inline void push(Node * n)
		{
			if (n)
			{
				nodes.push_back(n);
				std::push_heap(nodes.begin(), nodes.end(), cmp);
			}
		}

		inline Node * pop()
		{
			if (nodes.empty())
			{
				return NULL;
			}
			std::pop_heap(nodes.begin(), nodes.end(), cmp);
			Node * n = nodes.back();
			nodes.pop_back();
			return n;
		}

		inline void heapify()
		{
			std::make_heap(nodes.begin(), nodes.end(), cmp);
		}

#pragma endregion

		inline bool Empty() const
		{
			return nodes.empty();
		}

		inline void Clear()
		{
			nodes.clear();
			nodes.shrink_to_fit();
		}

	private:

		static inline bool cmp(const Node * a, const Node * b)
		{
			return a->F > b->F;
		}

		std::vector<Node *> nodes;

	};

}

#endif