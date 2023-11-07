#include "Broadphase.h"

#include <algorithm>

#include "Math/Bounds.h"

#include "Shape.h"

void SortBodiesBounds( 
	const std::vector<Body>& bodies,
	std::vector<PseudoBody>& sorted_bodies, 
	const float dt 
)
{
	Vec3 axis = Vec3( 1, 1, 1 );
	axis.Normalize();

	for ( int i = 0; i < bodies.size(); i++ )
	{
		const Body& body = bodies[i];
		Bounds bounds = body.shape->GetBounds( body.position, body.orientation );
		
		// Expand the bounds by the linear velocity
		bounds.Expand( bounds.mins + body.linearVelocity * dt );
		bounds.Expand( bounds.maxs + body.linearVelocity * dt );

		const float epsilon = 0.01f;
		bounds.Expand( bounds.mins + Vec3( -1, -1, -1 ) * epsilon );
		bounds.Expand( bounds.maxs + Vec3( 1, 1, 1 ) * epsilon );

		sorted_bodies[i * 2 + 0].id = i;
		sorted_bodies[i * 2 + 0].value = axis.Dot( bounds.mins );
		sorted_bodies[i * 2 + 0].is_min = true;
		sorted_bodies[i * 2 + 1].id = i;
		sorted_bodies[i * 2 + 1].value = axis.Dot( bounds.maxs );
		sorted_bodies[i * 2 + 1].is_min = false;
	}

	std::sort( sorted_bodies.begin(), sorted_bodies.end(), PseudoBody::Compare );
}

void BuildPairs( 
	std::vector<PseudoBody>& sorted_bodies, 
	std::vector<CollisionPair>& pairs 
)
{
	pairs.clear();

	int count = sorted_bodies.size();
	for ( int i = 0; i < count; i++ )
	{
		const PseudoBody& a = sorted_bodies[i];
		if ( !a.is_min ) continue;

		CollisionPair pair {};
		pair.a = a.id;

		for ( int j = i + 1; j < count; j++ )
		{
			const PseudoBody& b = sorted_bodies[j];
			if ( b.id == a.id ) break;

			if ( !b.is_min ) continue;

			pair.b = b.id;
			pairs.push_back( pair );
		}
	}
}

void Broadphase( const std::vector<Body>& bodies, std::vector<CollisionPair>& pairs, const float dt )
{
	std::vector<PseudoBody> sorted_bodies( bodies.size() * 2 );

	SortBodiesBounds( bodies, sorted_bodies, dt );
	BuildPairs( sorted_bodies, pairs );
}
