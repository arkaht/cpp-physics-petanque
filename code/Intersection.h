#pragma once
#include "Body.h"
#include "Shape.h"
#include "Contact.h"

class Intersection
{
public:
	static bool Intersect( Body& a, Body& b, const float dt, Contact& contact );
	static bool RaySphere( const Vec3& origin, const Vec3& dir, const Vec3& sphere_center, float sphere_radius, float& t0, float& t1 );
	static bool DynamicSphereToSphere( 
		const ShapeSphere& shape_a, const ShapeSphere& shape_b, 
		const Vec3& pos_a, const Vec3& pos_b, 
		const Vec3& vel_a, const Vec3& vel_b, 
		const float dt, 
		Vec3& point_a, Vec3& point_b, float& impact_time 
	);
};

