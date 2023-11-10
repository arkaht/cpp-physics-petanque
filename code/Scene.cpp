//
//  Scene.cpp
//
#include "Scene.h"
#include "Shape.h"
#include "Intersection.h"
#include "Broadphase.h"

#include <algorithm>


/*
========================================================================================================

Scene

========================================================================================================
*/

Scene::Scene()
{
	bodies.reserve( 128 );
}

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() 
{
	Clean();
}

void Scene::Clean()
{
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[i].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() 
{
	Clean();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() 
{
	const int row_count = 10;
	const float gap = 2.0f;

	Vec3 center( -( row_count - 1 ) * gap * 0.5f );
	center.z = 0.0f;

	for ( int x = 0; x < row_count; x++ )
	{
		for ( int y = 0; y < row_count; y++ )
		{
			Body body;
			body.position = center + Vec3( x * gap, y * gap, 5 );
			body.orientation = Quat( 0, 0, 0, 1 );
			body.shape = new ShapeSphere( 1.0f );
			body.SetMass( 5.0f );
			body.elasticity = 0.5f;
			body.friction = 0.9f;
			body.linearVelocity = { 0.0f, 0.0f, 0.0f };
			bodies.push_back( body );
		}
	}

	/*Body bodyB;
	bodyB.position = Vec3( 0, 1, 7 );
	bodyB.orientation = Quat( 0, 0, 0, 1 );
	bodyB.shape = new ShapeSphere( 1.0f );
	bodyB.SetMass( 5.0f );
	bodyB.elasticity = 0.5f;
	bodies.push_back( bodyB );*/

	float earth_radius = 4.7f;
	earth.position = Vec3( 0, 0, -earth_radius );
	earth.orientation = Quat( 0, 0, 0, 1 );
	earth.shape = new ShapeSphere( earth_radius );
	earth.SetMass( 0.0f );
	bodies.push_back( earth );
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt ) 
{
	//  gravity
	for ( int i = 0; i < bodies.size(); i++ )
	{
		auto& body = bodies[i];
		if ( body.IsStatic() ) continue;

		//  gravity
		Vec3 gravity = earth.position - body.position;
		gravity.Normalize();

		/*Vec3 gravity { 0.0f, 0.0f, -1.0f };*/

		gravity *= 25.0f;

		body.ApplyLinearImpulse( gravity * body.GetMass() * dt );
	}


	//  collisions
	std::vector<Contact> contacts;
	contacts.reserve( bodies.size() * bodies.size() );

	//  broadphase
	std::vector<CollisionPair> collisions_pairs;
	Broadphase( bodies, collisions_pairs, dt );
	for ( int i = 0; i < collisions_pairs.size(); i++ )
	{
		const CollisionPair& pair = collisions_pairs[i];
		Body& a = bodies[pair.a];
		Body& b = bodies[pair.b];

		if ( a.IsStatic() && b.IsStatic() ) continue;

		Contact contact;
		if ( Intersection::Intersect( a, b, dt, contact ) )
		{
			contacts.push_back( contact );
		}
	}
	/*for ( int i = 0; i < bodies.size(); i++ )
	{
		Body& a = bodies[i];

		for ( int j = i + 1; j < bodies.size(); j++ )
		{
			Body& b = bodies[j];
			if ( a.IsStatic() && b.IsStatic() ) continue;

			Contact contact;
			if ( Intersection::Intersect( a, b, dt, contact ) )
			{
				contacts.push_back( contact );
			}
		}
	}*/

	std::sort( contacts.begin(), contacts.end(), Contact::Compare );

	float accumulated_time = 0.0f;
	for ( Contact& contact : contacts )
	{
		const float local_dt = contact.impactTime - accumulated_time;

		//  position
		for ( Body& body : bodies )
		{
			if ( body.IsStatic() ) continue;
			body.Update( local_dt );
		}

		contact.Resolve();
		accumulated_time += local_dt;
	}

	//  update position depending on remaining time	
	const float time_remaining = dt - accumulated_time;
	if ( time_remaining > 0.0f )
	{
		for ( Body& body : bodies )
		{
			if ( body.IsStatic() ) continue;
			body.Update( time_remaining );
		}
	}
	
}