#ifndef DIAGNONAL_MOVEMENT_H
#define DIAGNONAL_MOVEMENT_H

namespace JPS {

	enum class DiagonalMovement : uint8_t
	{
		Always,
		AtLeastOnePassable,
		AllPassable,
		Never
	};

}

#endif // !DIAGNONAL_MOVEMENT_H