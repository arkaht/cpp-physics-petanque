#pragma once
#include "Math/Vector.h"

class Camera
{
public:
	Vec3 GetPosition() const
	{
		Vec3 pos
		{
			cosf( PositionPhi ) * sinf( PositionTheta ),
			sinf( PositionPhi ) * sinf( PositionTheta ),
			cosf( PositionTheta ),
		};
		pos *= Radius;
		pos += FocusPoint;

		return pos;
	}

	Vec3 FocusPoint;
	float PositionTheta;
	float PositionPhi;
	float Radius;
};