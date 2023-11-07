#pragma once

#include "Math/Vector.h"
#include "Math/Bounds.h"

class Shape 
{
public:
	enum class ShapeType
	{
		SHAPE_SPHERE,
	};

	Vec3 GetMassCenter() const { return massCenter; }
	
	virtual ShapeType GetType() const = 0;
	virtual Mat3 GetInertiaTensor() const = 0;

	virtual Bounds GetBounds( const Vec3& pos, const Quat& orient ) const = 0;
	virtual Bounds GetBounds() const = 0;

protected:
	Vec3 massCenter;
};

class ShapeSphere : public Shape 
{
public:
	ShapeSphere(float radiusP) : radius(radiusP)
	{
		massCenter = Vec3 { 0.0f, 0.0f, 0.0f };
	}

	ShapeType GetType() const override { return ShapeType::SHAPE_SPHERE; }
	Mat3 GetInertiaTensor() const override
	{
		Mat3 tensor;
		tensor.Zero();

		float x = 2.0f * radius * radius / 5.0f;
		tensor.rows[0][0] = x;
		tensor.rows[1][1] = x;
		tensor.rows[2][2] = x;

		return tensor;
	}

	Bounds GetBounds( const Vec3& pos, const Quat& orient ) const 
	{
		Bounds tmp;
		tmp.mins = Vec3( -radius ) + pos;
		tmp.maxs = Vec3( radius ) + pos;
		return tmp;
	}
	Bounds GetBounds() const
	{
		Bounds tmp;
		tmp.mins = Vec3( -radius );
		tmp.maxs = Vec3( radius );
		return tmp;
	}

	float radius;
};

