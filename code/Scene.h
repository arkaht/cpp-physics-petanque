//
//  Scene.h
//
#pragma once
#include <vector>

#include "Body.h"

/*
====================================================
Scene
====================================================
*/
class Scene {
public:
	Scene();
	~Scene();

	void Clean();
	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector<Body> bodies;
	Body earth;
};

