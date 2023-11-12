//
//  Scene.cpp
//
#include "Scene.h"
#include "Shape.h"
#include "Intersection.h"
#include "Broadphase.h"
#include "application.h"

#include <algorithm>
#include <random>


/*
========================================================================================================

Scene

========================================================================================================
*/

Scene::Scene( Application* application )
	:	application( application ),
		camera( application->GetCamera() )
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
	for ( int i = 0; i < bodies.size(); i++ )
	{
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
	/*const int row_count = 5;
	const float gap = 2.0f;

	Vec3 center( -( row_count - 1 ) * gap * 0.5f );
	center.z = 0.0f;

	for ( int x = 0; x < row_count; x++ )
	{
		for ( int y = 0; y < row_count; y++ )
		{
			Body body;
			body.position = center + Vec3( x * gap, y * gap, 5 );
			body.orientation = Quat( 0.0f, 0.0f, 0.0f, 1.0f );
			body.shape = new ShapeSphere( 1.0f );
			body.SetMass( 5.0f );
			body.elasticity = 0.01f;
			body.friction = 0.95f;
			body.linearVelocity = { 0.0f, 0.0f, 0.0f };
			bodies.push_back( body );
		}
	}*/

	/*target = &bodies[0];
	canUserPlay = true;
	gameState = GameState::WaitToShoot;*/

	/*Body bodyB;
	bodyB.position = Vec3( 0, 1, 7 );
	bodyB.orientation = Quat( 0, 0, 0, 1 );
	bodyB.shape = new ShapeSphere( 1.0f );
	bodyB.SetMass( 5.0f );
	bodyB.elasticity = 0.5f;
	bodies.push_back( bodyB );*/

	//  setup players
	firstPlayerState.name = "Player 1";
	firstPlayerState.score = 0;
	secondPlayerState.name = "Player 2";
	secondPlayerState.score = 0;

	//  reset game state
	shootTime = 0.0f;
	timeToEnd = 0.0f;
	piggyBall = nullptr;
	playersBalls.clear();

	SphereSettings earth_settings {};
	earth_settings.mass = 0.0f;
	earth_settings.radius = EARTH_RADIUS;
	earth_settings.elasticity = 1.0f;
	earth_settings.friction = 0.5f;

	earth = SpawnSphere( 
		Vec3 { 0.0f, 0.0f, -earth_settings.radius }, 
		earth_settings 
	);
	
	//  spawn walls
	const float PI = 3.14159265359f;
	const float DEG_TO_RAD = PI / 180.0f;
	const float angle_iter = 360.0f / WALLS_COUNT * DEG_TO_RAD;

	/*printf( "z-ratio=%f\n", ( earth.position.z - wall_z ) / EARTH_RADIUS );
	printf( "angle_iter=%f\n", angle_iter );*/

	SphereSettings wall_settings {};
	wall_settings.mass = 0.0f;
	wall_settings.radius = WALLS_RADIUS;
	wall_settings.elasticity = 1.0f;
	wall_settings.friction = 0.5f;

	for ( int i = 0; i < WALLS_COUNT; i++ )
	{
		const float angle = angle_iter * i;

		SpawnSphere( 
			Vec3 {
				cosf( angle ) * WALLS_POSITION_RADIUS,
				sinf( angle ) * WALLS_POSITION_RADIUS,
				WALLS_Z,
			}, 
			wall_settings 
		);
	}

	SetupSettings();
	SetupBalls();

	BeginSet();
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt )
{
	//printf( "%f\n", dt );

	switch ( gameState )
	{
		//  shoot update
		case GameState::Shooting:
			//  increase shoot time
			shootTime += dt;
			printf( "%f\n", shootTime );

			//  auto-shoot when time is maxed
			if ( shootTime >= MAX_SHOOT_TIME )
			{
				Shoot();
			}
			break;
		//  stop update
		case GameState::WaitToEnd:
			//  increaase stop time
			timeToEnd += dt;
			
			//  auto-stop
			if ( timeToEnd >= MAX_TIME_TO_END )
			{
				EndTurn();
			}
			break;
	}

	//  camera focus target
	if ( target != nullptr )
	{
		camera.FocusPoint = target->GetWorldMassCenter();
	}
}

void Scene::UpdatePhysics( const float dt )
{
	//  gravity
	for ( int i = 0; i < bodies.size(); i++ )
	{
		auto& body = bodies[i];
		if ( body.IsStatic() ) continue;

		//  gravity
		Vec3 gravity = earth.position - body.position;
		gravity.Normalize();

		//Vec3 gravity { 0.0f, 0.0f, -1.0f };

		gravity *= GRAVITY_SCALE;

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

void Scene::OnKeyInput( int key, int action )
{
	if ( application->IsPaused() ) return;
	
	//  shoot inputs
	if ( key == SHOOT_KEY )
	{
		if ( gameState == GameState::WaitToShoot && action == GLFW_PRESS )
		{
			gameState = GameState::Shooting;
			shootTime = 0.0f;
		}
		else if ( gameState == GameState::Shooting && action == GLFW_RELEASE )
		{
			Shoot();
		}
	}
}

Body& Scene::SpawnSphere( const Vec3& pos, SphereSettings settings )
{
	Body body;
	body.position = pos;
	body.orientation = Quat( 0.0f, 0.0f, 0.0f, 1.0f );
	body.shape = new ShapeSphere( settings.radius );
	body.SetMass( settings.mass );
	body.elasticity = settings.elasticity;
	body.friction = settings.friction;
	bodies.push_back( body );

	//application->CreateModelForBody( body );
	return bodies.back();
}

PlayerState* Scene::GetNextTurnPlayerState()
{
	//  piggy turn? keep the same player
	if ( turnId == 0 )
	{
		return currentPlayerState;
	}

	if ( firstPlayerState.turnRemainingBalls == 0 && secondPlayerState.turnRemainingBalls == 0 )
	{
		printf( "No one has remaining balls, ending set!\n" );
		return nullptr;
	}

	//  get the disavantaged player
	if ( playersBalls.size() > 0 )
	{
		SortBallsPerProximity();

		PlayerState& aheadPlayer = *playersBalls[0].playerState;
		if ( &aheadPlayer == &firstPlayerState 
		  && secondPlayerState.turnRemainingBalls > 0 )
			return &secondPlayerState;
	}
	
	//  get either one of them, priorizing first player
	if ( firstPlayerState.turnRemainingBalls > 0 )
		return &firstPlayerState;
	if ( secondPlayerState.turnRemainingBalls > 0 )
		return &secondPlayerState;

	return nullptr;
}

PlayerState& Scene::GetRandomPlayerState()
{
	std::random_device dev;
    std::mt19937 rng( dev() );
    std::uniform_int_distribution<std::mt19937::result_type> dist( 1, 2 );

	return dist( rng ) == 1 
		? firstPlayerState
		: secondPlayerState;
}

void Scene::SetupBalls()
{
	//  create piggy ball
	piggyBall = &SpawnSphere( Vec3 { 0.0f, 0.0f, 0.0f }, piggyBallSettings );

	//  create players balls
	for ( int i = 0; i < BALLS_PER_TURN * 2; i++ )
	{
		Body& ball = SpawnSphere( 
			Vec3 { 0.0f, 0.0f, 0.0f },
			metalBallSettings
		);

		playersBalls.emplace_back( 
			nullptr, 
			&ball 
		);
	}
}

void Scene::ResetBalls()
{
	piggyBall->position = Vec3 { 0.0f, 0.0f, 0.0f };
	piggyBall->SetMass( 0.0f );  //  set as static

	for ( int i = 0; i < playersBalls.size(); i++ )
	{
		PlayerBall& player_ball = playersBalls[i];
		player_ball.ball->position = Vec3 { 
			i * 5.0f, 
			WALLS_POSITION_RADIUS, 
			WALLS_Z + 10.0f 
		};
		player_ball.ball->SetMass( 0.0f );  //  set as static
	}
}

void Scene::SortBallsPerProximity()
{
	auto comparer = PlayerBallsProximityComparer {};
	comparer.origin = piggyBall->GetWorldMassCenter();

	std::sort( playersBalls.begin(), playersBalls.end(), comparer );
}

void Scene::BeginSet()
{
	printf( "GameState: Begin Set\n" );

	ResetBalls();
	turnId = -1;

	//  prepare player states
	firstPlayerState.playersBalls.clear();
	firstPlayerState.turnRemainingBalls = BALLS_PER_TURN;

	secondPlayerState.playersBalls.clear();
	secondPlayerState.turnRemainingBalls = BALLS_PER_TURN;

	//  get first player to start
	PlayerState& player_state = GetRandomPlayerState();
	BeginTurn( player_state );
}

void Scene::EndSet()
{
	printf( "GameState: End Set\n" );

	SortBallsPerProximity();  //  probably already done but just to make sure

	//  get winning & loosing players
	PlayerState* winning_player = playersBalls[0].playerState;
	PlayerState* loosing_player = winning_player == &firstPlayerState 
		? &secondPlayerState 
		: &firstPlayerState;

	//  count score to gain
	int score_gain = 0;
	for ( const PlayerBall& ball : playersBalls )
	{
		if ( winning_player != ball.playerState ) break;

		//  add one point for each closest balls
		score_gain++;
	}

	//  gain score
	winning_player->score += score_gain;
	printf( "%s gained %d score!\n", winning_player->name.c_str(), score_gain );

	//  check score
	if ( winning_player->score >= MAX_SCORE )
	{
		printf( "Game won by %s, score: %d vs %d\n", winning_player->name.c_str(), winning_player->score, loosing_player->score );

		//  obliteration check..
		if ( loosing_player->score == 0 )
		{
			printf( "%s has to kiss Fanny :)\n", loosing_player->name.c_str() );
		}

		gameState = GameState::WaitToLaunchGame;
		printf( "Press R to restart!\n" );
	}
	else
	{
		printf( "Set won by %s, score: %d vs %d\n", winning_player->name.c_str(), winning_player->score, loosing_player->score );
	
		BeginSet();
	}
}

void Scene::BeginTurn( PlayerState& player_state )
{
	turnId++;
	currentPlayerState = &player_state;

	//  get next ball settings
	const SphereSettings& settings = turnId == 0 
		? piggyBallSettings 
		: metalBallSettings;

	printf( "%f\n", settings.radius );

	//  spawn ball
	if ( turnId > 0 )
	{
		PlayerBall& player_ball = playersBalls[turnId - 1];
		player_ball.ball->position = Vec3 { 0.0f, 0.0f, metalBallSettings.radius };
		player_ball.ball->SetMass( metalBallSettings.mass );
		player_ball.playerState = &player_state;

		//  set as camera target
		target = player_ball.ball;

		//  add balls to list
		player_state.turnRemainingBalls--;
	}
	else
	{
		piggyBall->position = Vec3 { 0.0f, 0.0f, piggyBallSettings.radius };
		piggyBall->SetMass( piggyBallSettings.mass );

		//  set as camera target
		target = piggyBall;
	}

	gameState = GameState::WaitToShoot;

	printf( "\nTurn %d for %s\n", turnId, player_state.name.c_str() );
}

void Scene::EndTurn()
{
	//  force stop physics
	for ( Body& body : bodies )
	{
		if ( body.IsStatic() ) continue;

		//body.friction = 1.0f;
		body.linearVelocity.Zero();
		body.angularVelocity.Zero();
	}

	//  reset state
	timeToEnd = 0.0f;

	printf( "GameState: End Turn\n" );

	//  move to next turn or set
	PlayerState* player_state = GetNextTurnPlayerState();
	if ( player_state == nullptr )
	{
		EndSet();
	}
	else
	{
		BeginTurn( *player_state );
	}
}

void Scene::Shoot()
{
	//  get user force
	float user_force = std::min( shootTime / MAX_SHOOT_TIME, 1.0f );
	if ( user_force <= 0.01f )
	{
		shootTime = 0.0f;
		return;
	}

	printf( "Shoot Time: %f/%fs | Force: %.0f%%\n", shootTime, MAX_SHOOT_TIME, user_force * 100.0f );

	//  get body & shape
	Body& body = *target;
	ShapeSphere* shape = reinterpret_cast<ShapeSphere*>( body.shape );

	//  camera position & direction
	Vec3 pos = camera.GetPosition();
	Vec3 dir = camera.FocusPoint - pos;
	dir.Normalize();

	//  shoot a bit upper
	dir += Vec3( 0.0f, 0.0f, 0.1f );
	dir.Normalize();

	//  get impulse
	float t0, t1;
	Intersection::RaySphere(
		pos, dir,
		body.GetWorldMassCenter(),
		shape->radius,
		t0, t1
	);
	Vec3 point = pos + dir * t1;
	Vec3 force = dir * user_force * MAX_SHOOT_FORCE;

	//  apply impulse
	body.ApplyImpulse( point, force );

	//  reset state
	gameState = GameState::WaitToEnd;
	shootTime = 0.0f;
	//canUserPlay = false;

	printf( "GameState: Shoot\n" );
}
