//
//  Scene.h
//
#pragma once
#include <vector>
#include <string>

#include "Body.h"
#include "Camera.h"

#include <GLFW/glfw3.h>

class Application;

enum class GameState
{
	WaitToLaunchGame,
	WaitToShoot,
	Shooting,
	WaitToEnd,
};

struct SphereSettings
{
	float mass;
	float radius;
	float elasticity;
	float friction;
};

struct PlayerState
{
	std::string name { "N/A" };
	int score = 0;
	int turnRemainingBalls = 0;
	std::vector<Body*> playersBalls;  //  TODO: remove
};

struct PlayerBall
{
	PlayerBall( PlayerState* state, Body* ball )
		: playerState( state ), ball( ball )
	{}

	PlayerState* playerState;
	Body* ball;
};

struct PlayerBallsProximityComparer
{
	Vec3 origin;

	bool operator()( const PlayerBall& a, const PlayerBall& b )
	{
		float dist_a = ( a.ball->GetWorldMassCenter() - origin ).GetLengthSqr();
		float dist_b = ( b.ball->GetWorldMassCenter() - origin ).GetLengthSqr();
		return dist_a < dist_b;
	}
};

/*
====================================================
Scene
====================================================
*/
class Scene {
public:
	Scene( Application* application );
	~Scene();

	void Clean();
	void Reset();
	void Initialize();

	void Update( const float dt );	
	void UpdatePhysics( const float dt );

	void OnKeyInput( int key, int action );

	std::vector<Body> bodies;

private:
	Application* application;

	Body earth;
	Body* target { nullptr };
	Camera& camera;

	//  game settings
	const float GRAVITY_SCALE = 50.0f;
	const int SHOOT_KEY = GLFW_KEY_SPACE;	//  user input to shoot
	const float MAX_SHOOT_TIME = 1.0f;		//  maximum time of user holding the shoot key
	const float MAX_SHOOT_FORCE = 75.0f;	//  maximum user shoot force, scaled w/ shoot time
	const float MAX_TIME_TO_END = 1.0f;		//  maximum time after shooting before turn is ended
	const float EARTH_RADIUS = 500.0f;
	const int BALLS_PER_TURN = 1;
	const int MAX_SCORE = 2;
	SphereSettings piggySettings;
	SphereSettings metalBallSettings;

	void SetupSettings()
	{
		piggySettings.mass = 1.0f;
		piggySettings.radius = 0.5f;
		piggySettings.elasticity = 0.5f;
		piggySettings.friction = 0.95f;

		metalBallSettings.mass = piggySettings.mass * 2.0f;
		metalBallSettings.radius = piggySettings.radius * 4.0f;
		metalBallSettings.elasticity = 0.01f;
		metalBallSettings.friction = 0.95f;
	}

	GameState gameState;
	//bool canUserPlay = false;				//  can user press gameplay inputs
	float shootTime = 0.0f;
	float timeToEnd = 0.0f;
	PlayerState firstPlayerState;
	PlayerState secondPlayerState;
	Body* piggyBall { nullptr };
	std::vector<PlayerBall> playersBalls;
	int turnId = 0;

	Body& SpawnSphere( const Vec3& pos, SphereSettings settings );
	PlayerState* GetNextTurnPlayerState();
	void RemoveBody( const Body& body );

	void SortBallsPerProximity();

	//  game states
	void BeginSet();
	void EndSet();

	void BeginTurn( PlayerState& player_state );
	void EndTurn();
	
	void Shoot();
};

