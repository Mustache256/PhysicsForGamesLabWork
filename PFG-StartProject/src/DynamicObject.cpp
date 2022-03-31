#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "DynamicObject.h"
#include "Utility.h"

DynamicObject::DynamicObject()
{
	_force = glm::vec3(0.0f, 0.0f, 0.0f);
	_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	_bRadius = 0.0f;
	_mass = 1.0f;

	_scale = glm::vec3(1.0f, 1.0f, 1.0f);
	_start = false;
}

DynamicObject::~DynamicObject()
{

}

void DynamicObject::Update(float deltaTs)
{
	//float moon_Grav = 0.1f;
	float collision_impulse;

	glm::vec3 impulse_force;
	glm::vec3 contact_force(0.0f, 9.8 * _mass * 0.1f, 0.0f);
	
	if (_start == true)
	{
		ClearForces();

		glm::vec3 gravityForce(0.0f, -9.8 * _mass * 0.1f, 0.0f);
		AddForce(gravityForce);

		float elasticity = 0.7f;
		float r = GetBoundingRadius();
		
		glm::vec3 point_on_floor(0.0f, 0.0f, 0.0f);
		glm::vec3 floor_normal(0.0f, 1.0f, 0.0f);

		float d = PFG::DistanceToPlane(floor_normal, _position, point_on_floor);

		if (d <= r)
		{
			_position.y = r;
			//AddForce(contact_force);
			collision_impulse = (-(1 + elasticity) * glm::dot(_velocity, floor_normal)) * _mass;
			impulse_force = collision_impulse * floor_normal; // /  deltaTs;
			//glm::vec3 bounceForce = glm::vec3(0.0f, 300.0f, 0.0f);
			_velocity += impulse_force / _mass;
			//AddForce(impulse_force);
			AddForce(contact_force);
		}

		Verlet(deltaTs);
	}

	UpdateModelMatrix();
}

void DynamicObject::Euler(float deltaTs)
{
	float oneOverMass = 1 / _mass;
	_velocity += (_force * oneOverMass) * deltaTs;
	_position += _velocity * deltaTs;
}

void DynamicObject::UpdateModelMatrix()
{
	_modelMatrix = glm::translate(glm::mat4(1), _position);
	_modelMatrix = glm::scale(_modelMatrix, _scale);
	_invModelMatrix = glm::inverse(_modelMatrix);
}

void DynamicObject::RungeKutta2(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	//Evaluate once at t0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;

	//Evaluate once at t0 + deltaT/2.0 using half of k0
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	//Evaluate once at t0 + deltaT using k1
	_velocity += k1;
	_position += _velocity * deltaTs;
}

void DynamicObject::RungeKutta4(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	//Evaluate once at t0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;

	//Evaluate twice at t0 + deltaT/2.0 using half of k0 and half of k1
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	force = _force + k1 / 2.0f;
	acceleration = force / _mass;
	k2 = deltaTs * acceleration;

	//Evaluate once at t0 + deltaT using k2
	force = _force + k2;
	acceleration = force / _mass;
	k3 = deltaTs * acceleration;

	//Evaluate at t0 + deltaT using weighted sum of k0, k1, k2 and k3
	_velocity += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
	//Update position 
	_position += _velocity * deltaTs;
}

void DynamicObject::Verlet(float deltaTs)
{
	glm::vec3 acceleration;
	
	acceleration = _force / _mass;

	_previousPosition = _position - _velocity * deltaTs + 0.5f * acceleration * deltaTs * deltaTs;

	_position = -_previousPosition + 2.0f * _position + acceleration * deltaTs * deltaTs;

	_velocity = (_position - _previousPosition) / (2.0f * deltaTs);

	_velocity += acceleration * deltaTs;
	
}