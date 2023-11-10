#include "Intersection.h"

bool Intersection::Intersect( Body& a, Body& b, const float dt, Contact& contact )
{
	contact.bodyA = &a;
	contact.bodyB = &b;

	const Vec3 ab = b.position - a.position;
	contact.normal = ab;
	contact.normal.Normalize();

	if ( a.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE
	  && b.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE )
	{
		ShapeSphere* sphere_a = reinterpret_cast<ShapeSphere*>( a.shape );
		ShapeSphere* sphere_b = reinterpret_cast<ShapeSphere*>( b.shape );

		if ( Intersection::DynamicSphereToSphere( 
				*sphere_a, *sphere_b, 
				a.position, b.position, 
				a.linearVelocity, b.linearVelocity, 
				dt, 
				contact.worldContactA, contact.worldContactB, 
				contact.impactTime 
			) )
		{
			a.Update( contact.impactTime );
			b.Update( contact.impactTime );
		
			contact.localContactA = a.WorldToLocal( contact.worldContactA );
			contact.localContactB = b.WorldToLocal( contact.worldContactB );

			Vec3 ab = a.position - b.position;
			contact.normal = ab;
			contact.normal.Normalize();

			a.Update( -contact.impactTime );
			b.Update( -contact.impactTime );

			float r = ab.GetMagnitude() - ( sphere_a->radius + sphere_b->radius );
			contact.separationDistance = r;
			return true;
		}
	}

	return false;
}

bool Intersection::RaySphere( 
	const Vec3& origin, 
	const Vec3& dir, 
	const Vec3& sphere_center, 
	float sphere_radius, 
	float& t0, float& t1
)
{
	const Vec3& s = sphere_center - origin;
	const float a = dir.Dot( dir );
	const float b = s.Dot( dir );
	const float c = s.Dot( s ) - sphere_radius * sphere_radius;

	const float delta = b * b - a * c;
	if ( delta < 0.0f )	return false;
	
	const float inverse_a = 1.0f / a;
	const float delta_root = sqrtf( delta );
	t0 = ( b - delta_root ) * inverse_a;
	t1 = ( b + delta_root ) * inverse_a;

	return true;
}

bool Intersection::DynamicSphereToSphere( 
	const ShapeSphere& shape_a, const ShapeSphere& shape_b, 
	const Vec3& pos_a, const Vec3& pos_b, 
	const Vec3& vel_a, const Vec3& vel_b, 
	const float dt, 
	Vec3& point_a, Vec3& point_b, 
	float& impact_time 
)
{
	const Vec3& relative_vel = vel_a - vel_b;

	const Vec3 start_point_a = pos_a;
	const Vec3 end_point_a = start_point_a + relative_vel * dt;
	const Vec3 ray_dir = end_point_a - start_point_a;

	float t0 = 0, t1 = 0;
	if ( ray_dir.GetLengthSqr() <= 0.001f * 0.001f )
	{
		//  ray is too short, check for intersection
		Vec3 ab = pos_b - pos_a;
		float radius = shape_a.radius + shape_b.radius + 0.001f;
		if ( ab.GetLengthSqr() > radius * radius )
		{
			return false;
		}
	}
	else if ( !RaySphere( start_point_a, ray_dir, pos_b, shape_a.radius + shape_b.radius, t0, t1 ) )
	{
		return false;
	}

	//  range from [0; 1] to [0; dt]
	t0 *= dt, t1 *= dt;

	//  avoid collision in the past
	if ( t1 < 0.0f ) return false;

	//  get earliest positive time of impact
	impact_time = t0 < 0.0f ? 0.0f : t0;

	//  avoid too far collision in time
	if ( impact_time > dt ) return false;

	Vec3 new_pos_a = pos_a + vel_a * impact_time;
	Vec3 new_pos_b = pos_b + vel_b * impact_time;
	Vec3 ab = new_pos_b - new_pos_a;
	ab.Normalize();

	point_a = new_pos_a + ab * shape_a.radius;
	point_b = new_pos_b - ab * shape_b.radius;
	return true;
}
