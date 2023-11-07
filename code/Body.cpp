#include "Body.h"
#include "Shape.h"

void Body::Update( float dt )
{
	//  apply linear velocity
	position += linearVelocity * dt;

	Vec3 mass_center = GetWorldMassCenter();
	Vec3 center_to_position = position - mass_center;

	Mat3 orient = orientation.ToMat3();
	Mat3 inertia_tensor = orient * shape->GetInertiaTensor() * orient.Transpose();
	Vec3 alpha = inertia_tensor.Inverse() * angularVelocity.Cross( inertia_tensor * angularVelocity );
	angularVelocity += alpha * dt;

	//  update orientation
	Vec3 delta_angle = angularVelocity * dt;
	Quat delta_orientation = Quat( delta_angle, delta_angle.GetMagnitude() );
	orientation = delta_orientation * orientation;
	orientation.Normalize();

	//  new position
	position = mass_center + delta_orientation.RotatePoint( center_to_position );
}

Vec3 Body::GetWorldMassCenter() const
{
	return position + orientation.RotatePoint( shape->GetMassCenter() );
}

Vec3 Body::GetLocalMassCenter() const
{
	return shape->GetMassCenter();
}

Mat3 Body::GetWorldInverseInertiaTensor() const
{
	Mat3 inertia_tensor = shape->GetInertiaTensor();
	Mat3 inverse_inertia_tensor = inertia_tensor.Inverse() * inverseMass;
	Mat3 orient = orientation.ToMat3();
	return orient * inverse_inertia_tensor * orient.Transpose();
}

Mat3 Body::GetLocalInverseInertiaTensor() const
{
	Mat3 inertia_tensor = shape->GetInertiaTensor();
	return inertia_tensor.Inverse() * inverseMass;
}

Vec3 Body::WorldToLocal( const Vec3& world_pos ) const
{
	const Quat invert_orientation = orientation.Inverse();
	return invert_orientation.RotatePoint( world_pos - GetWorldMassCenter() );
}

Vec3 Body::LocalToWorld( const Vec3& local_pos ) const
{
	return GetWorldMassCenter() + orientation.RotatePoint(local_pos);
}

void Body::ApplyImpulse( const Vec3& origin, const Vec3& impulse )
{
	if ( IsStatic() ) return;

	//  linear
	ApplyLinearImpulse( impulse );

	//  angular
	Vec3 mass_center = GetWorldMassCenter();
	Vec3 r = origin - mass_center;
	Vec3 dL = r.Cross( impulse );
	ApplyAngularImpulse( dL );
}

void Body::ApplyLinearImpulse( const Vec3& impulse )
{
	if ( IsStatic() ) return;

	linearVelocity += impulse * inverseMass;
}

void Body::ApplyAngularImpulse( const Vec3& impulse )
{
	if ( IsStatic() ) return;

	angularVelocity += GetWorldInverseInertiaTensor() * impulse;

	//  clamp angular velocity speed
	const float max_angular_speed = 30.0f;
	if ( angularVelocity.GetLengthSqr() > max_angular_speed * max_angular_speed )
	{
		angularVelocity.Normalize();
		angularVelocity *= max_angular_speed;
	}
}

void Body::SetMass( float _mass )
{
	mass = _mass;
	inverseMass = IsStatic() ? 0.0f : 1.0f / _mass;
}
