#include <iostream>

#include "Node.h"
#include "Openlist.h"

int main()
{
	Node n = Node(FPosition(1, 2, 3));
	n.SetOpen();
	std::cout << n.IsClosed() << '\n';

	return 0;
}