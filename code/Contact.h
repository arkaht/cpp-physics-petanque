#pragma once
#include "Body.h"

class Contact
{
public:
	Vec3 worldContactA;
	Vec3 localContactA;
	Vec3 worldContactB;
	Vec3 localContactB;

	Vec3 normal;
	float separationDistance;
	float impactTime;

	Body* bodyA { nullptr };
	Body* bodyB { nullptr };

	void Resolve();

	static bool Compare( const Contact& a, const Contact& b )
	{
		return a.impactTime < b.impactTime;
	}
};