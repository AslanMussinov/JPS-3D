#include <iostream>

#include "Searcher.h"

using namespace JPS;

int a[20];

int main()
{
	for (int i = 0; i < 20; ++i)
	{
		a[i] = 1;
	}
	FGrid g = FGrid(2, 2, 2, a);
	Searcher * s = new Searcher(g);
	std::vector<FPosition> res = s->FindPath(FPosition(0, 0, 0), FPosition(1, 1, 1));
	for (unsigned i = 0; i < res.size(); ++i)
	{
		std::cout << res[i] << '\n';
	}

	return 0;
}