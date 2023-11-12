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

	//  world settings
	const float EARTH_RADIUS = 500.0f;		//  radius of the earth, don't mess with it unless you want to mess with the walls generation
	const float WALLS_EARTH_RADIUS_RATIO = 0.1f;
	const float WALLS_EARTH_Z_RATIO = 0.001f;
	const float WALLS_Z = -EARTH_RADIUS * WALLS_EARTH_Z_RATIO;
	const float WALLS_POSITION_RADIUS = EARTH_RADIUS * WALLS_EARTH_RADIUS_RATIO;
	const float WALLS_RADIUS = EARTH_RADIUS / 100.0f;
	const int WALLS_COUNT = 360.0f / WALLS_RADIUS;
	
	//  game settings
	const float GRAVITY_SCALE = 50.0f;		//  gravity force
	const int SHOOT_KEY = GLFW_KEY_SPACE;	//  user input for shooting
	const float MAX_SHOOT_TIME = 1.0f;		//  maximum time of user holding the shoot key
	const float MAX_SHOOT_FORCE = 75.0f;	//  maximum user shoot force, scaled w/ shoot time
	const float MAX_TIME_TO_END = 2.0f;		//  maximum time after shooting before turn is ended
	const int BALLS_PER_TURN = 3;			//  how much balls do each player have to throw each turn
	const int MAX_SCORE = 13;				//  how much do a player have to score to win the game
	SphereSettings piggyBallSettings;		//  physics settings for the piggy, see SetupSettings function below
	SphereSettings metalBallSettings;		//  physics settings for a player ball, see SetupSettings function below

	void SetupSettings()
	{
		piggyBallSettings.mass = 1.0f;
		piggyBallSettings.radius = 0.5f;
		piggyBallSettings.elasticity = 0.5f;
		piggyBallSettings.friction = 0.95f;

		metalBallSettings.mass = piggyBallSettings.mass * 2.0f;
		metalBallSettings.radius = piggyBallSettings.radius * 4.0f;
		metalBallSettings.elasticity = 0.01f;
		metalBallSettings.friction = 0.95f;
	}

	GameState gameState;
	//bool canUserPlay = false;				//  can user press gameplay inputs
	float shootTime = 0.0f;
	float timeToEnd = 0.0f;
	PlayerState firstPlayerState;
	PlayerState secondPlayerState;
	PlayerState* currentPlayerState { nullptr };
	Body* piggyBall { nullptr };
	std::vector<PlayerBall> playersBalls;
	int turnId = 0;

	Body& SpawnSphere( const Vec3& pos, SphereSettings settings );
	PlayerState* GetNextTurnPlayerState();
	PlayerState& GetRandomPlayerState();
	
	void SetupBalls();
	void ResetBalls();

	void SortBallsPerProximity();

	//  game states
	void BeginSet();
	void EndSet();

	void BeginTurn( PlayerState& player_state );
	void EndTurn();
	
	void Shoot();
};

