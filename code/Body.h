#pragma once
#include "Math/Vector.h"
#include "Renderer/model.h"
#include "Math/Quat.h"

class Body
{
public:
	Vec3 position;
	Quat orientation;
	Shape* shape;

	float elasticity = 1.0f;
	float friction = 0.5f;
	Vec3 linearVelocity;
	Vec3 angularVelocity;
	
	void Update( float dt );

	Vec3 GetWorldMassCenter() const;
	Vec3 GetLocalMassCenter() const;

	Mat3 GetWorldInverseInertiaTensor() const;
	Mat3 GetLocalInverseInertiaTensor() const;

	Vec3 WorldToLocal( const Vec3& world_pos ) const;
	Vec3 LocalToWorld( const Vec3& local_pos ) const;

	void ApplyImpulse( const Vec3& origin, const Vec3& impulse );
	void ApplyLinearImpulse( const Vec3& impulse );
	void ApplyAngularImpulse( const Vec3& impulse );

	void SetMass( float mass );
	float GetMass() const { return mass; }
	float GetInverseMass() const { return inverseMass; }

	bool IsStatic() const { return mass == 0.0f; }

private:
	float mass;
	float inverseMass;
};

