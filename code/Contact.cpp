#include "Contact.h"

void Contact::Resolve()
{
	const float inverse_mass_a = bodyA->GetInverseMass();
	const float inverse_mass_b = bodyB->GetInverseMass();

	const float elasticity = bodyA->elasticity * bodyB->elasticity;

	const Vec3 r_a = worldContactA - bodyA->GetWorldMassCenter();
	const Vec3 r_b = worldContactB - bodyB->GetWorldMassCenter();

	const Mat3 inverse_inertia_a = bodyA->GetWorldInverseInertiaTensor();
	const Mat3 inverse_inertia_b = bodyB->GetWorldInverseInertiaTensor();

	const Vec3 angular_a = ( inverse_inertia_a * r_a.Cross( normal ) ).Cross( r_a );
	const Vec3 angular_b = ( inverse_inertia_b * r_b.Cross( normal ) ).Cross( r_b );
	const float angular_factor = ( angular_a + angular_b ).Dot( normal );

	const Vec3 velocity_a = bodyA->linearVelocity + bodyA->angularVelocity.Cross( r_a );
	const Vec3 velocity_b = bodyB->linearVelocity + bodyB->angularVelocity.Cross( r_b );

	//  collision impulse
	const Vec3 velocity_ab = velocity_a - velocity_b;
	const float impulse_force = ( 1.0f + elasticity ) * velocity_ab.Dot( normal )
		                      / ( inverse_mass_a + inverse_mass_b + angular_factor );
	const Vec3 impulse = normal * impulse_force;

	bodyA->ApplyImpulse( worldContactA, impulse * -1.0f );
	bodyB->ApplyImpulse( worldContactB, impulse );

	//  friction
	const float friction = bodyA->friction * bodyB->friction;
	const Vec3 velocity_normal = normal * normal.Dot( velocity_ab );
	const Vec3 velocity_tangent = velocity_ab - velocity_normal;
	Vec3 relative_velocity_tangent = velocity_tangent;
	relative_velocity_tangent.Normalize();
	const Vec3 inertia_a = ( inverse_inertia_a * r_a.Cross( relative_velocity_tangent ) ).Cross( r_a );
	const Vec3 inertia_b = ( inverse_inertia_b * r_b.Cross( relative_velocity_tangent ) ).Cross( r_b );
	const float inverse_inertia = ( inertia_a + inertia_b ).Dot( relative_velocity_tangent );

	const float reduced_mass = 1.0f / ( inverse_mass_a + inverse_mass_b + inverse_inertia );
	const Vec3 impulse_friction = velocity_tangent * reduced_mass * friction;
	bodyA->ApplyImpulse( worldContactA, impulse_friction * -1.0f );
	bodyB->ApplyImpulse( worldContactB, impulse_friction );

	if ( impactTime == 0.0f )
	{
		//  fix contact positions
		const float time_a = inverse_mass_a / ( inverse_mass_a + inverse_mass_b );
		const float time_b = inverse_mass_b / ( inverse_mass_a + inverse_mass_b );
		const Vec3 d = worldContactB - worldContactA;

		bodyA->position += d * time_a;
		bodyB->position -= d * time_b;
	}
}
