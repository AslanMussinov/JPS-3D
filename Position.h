#ifndef POSITION_H
#define POSITION_H

#include <iostream> // for ostream operator

struct FPosition
{
	unsigned x, y, z;

	FPosition() : x(-1), y(-1), z(-1) {}

	FPosition(unsigned xx, unsigned yy, unsigned zz) : x(xx), y(yy), z(zz) {}

	inline bool IsValid() const
	{
		return x != unsigned(-1);
	}

	inline void Normalize(unsigned n)
	{
		if (n < 2U)
		{
			return;
		}
		x = (x / n) * n;
		y = (y / n) * n;
		z = (z / n) * n;
	}
};

#pragma region Operators

inline std::ostream & operator<<(std::ostream & os, FPosition const & p)
{
	os << "[" << p.x << ", " << p.y << ", " << p.z << "]";
	return os;
}

inline bool operator==(const FPosition & a, const FPosition & b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool operator!=(const FPosition & a, const FPosition & b)
{
	return !operator==(a, b);
}

inline bool operator<(const FPosition & a, const FPosition & b)
{
	return a.z < b.z || (a.z == b.z && a.y < b.y) || (a.z == b.z && a.y == b.y && a.x < b.x);
}

inline bool operator>(const FPosition & a, const FPosition & b)
{
	return operator<(b, a);
}

inline bool operator<=(const FPosition & a, const FPosition & b)
{
	return !operator>(a, b);
}

inline bool operator>=(const FPosition & a, const FPosition & b)
{
	return !operator<(a, b);
}

#pragma endregion

#endif