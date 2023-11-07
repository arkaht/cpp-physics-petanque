#pragma once

#include <vector>
#include "Body.h"

struct CollisionPair
{
	int a;
	int b;

	bool operator==( const CollisionPair& rhs ) const
	{
		return ( a == rhs.a && b == rhs.b ) || ( a == rhs.b && b == rhs.a );
	}
	bool operator!=( const CollisionPair& rhs ) const
	{
		return !( *this == rhs );
	}
};

struct PseudoBody
{
	int id;
	float value;
	bool is_min;

	static bool Compare( const PseudoBody& a, const PseudoBody& b )
	{
		return a.value < b.value;
	}
};

void Broadphase(
	const std::vector<Body>& bodies,
	std::vector<CollisionPair>& pairs,
	const float dt
);