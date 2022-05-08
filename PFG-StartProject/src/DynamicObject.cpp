#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>

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

void DynamicObject::Update(GameObject* otherObj, float deltaTs)
{
	
	float collision_impulse;
	glm::vec3 floor_normal(0.0f, 1.0f, 0.0f);
	float elasticity = 0.5;
	glm::vec3 impulse_force;
	glm::vec3 contact_force(0.0f, 9.8 * _mass * 0.1f, 0.0f);
	float r = GetBoundingRadius();
	
	if (_start == true)
	{
		ClearForces();

		glm::vec3 gravityForce(0.0f, -9.8 * _mass * 0.1f, 0.0f);
		AddForce(gravityForce);

		CollisionResponse(otherObj, deltaTs);

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

void DynamicObject::CollisionResponse(GameObject* otherObj, float deltaTs)
{
	const float r = GetBoundingRadius();
	float elasticity = 0.8;
	int type = otherObj->GetType();

	if (type == 0)
	{
		glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
		glm::vec3 thisObjCentre = _position;
		glm::vec3 otherObjCentre = _position + _velocity * deltaTs;
		glm::vec3 q = otherObj->GetPosition();
		glm::vec3 contactPoint;

		bool collision = PFG::MovingSphereToPlaneCollision(normal, thisObjCentre, otherObjCentre, q, r, contactPoint);

		if (collision)
		{
			glm::vec3 colliderVel = otherObj->GetInitialVelocity(); // This was getvelocity not getinitialvelocity
			glm::vec3 relativeVel = _velocity - colliderVel;
			glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f); // floor normal up
			float invColliderMass = 0.0f; // floor doesn't move

			glm::vec3 contactPosition = contactPoint;
			float eCof = -(1.0f + elasticity) * glm::dot(relativeVel, normal);
			float invMass = 1 / GetMass();
			float jLin = eCof / (invMass + invColliderMass);

			glm::vec3 collision_impulse_force = jLin * normal / deltaTs;

			glm::vec3 contact_force = glm::vec3(0.0f, 9.81f * _mass, 0.0f);
			glm::vec3 total_force = contact_force + collision_impulse_force;

			AddForce(total_force);
		}
	}
	else if (type == 1)
	{
		DynamicObject* otherDynamicObj = dynamic_cast<DynamicObject*>(otherObj);
		glm::vec3 thisObjCentre1 = otherDynamicObj->GetPosition() + otherDynamicObj->GetVelocity() * deltaTs;
		glm::vec3 otherObjCentre1 = _position + _velocity * deltaTs;
		float radius1 = GetBoundingRadius();
		float radius2 = otherDynamicObj->GetBoundingRadius();
		glm::vec3 cp;

		bool collision = PFG::SphereToSphereCollision(thisObjCentre1, otherObjCentre1, radius1, radius2, cp);

		if (collision)
		{

			std::cout << "Collided with sphere" << std::endl;

			glm::vec3 ColliderVel = otherDynamicObj->GetVelocity();
			glm::vec3 relativeVel = _velocity - ColliderVel;
			glm::vec3 normal = glm::normalize(thisObjCentre1 - otherObjCentre1);

			glm::vec3 contactPosition = radius1 * normal;
			float eCof = -(1.0f + elasticity) * glm::dot(relativeVel, normal);
			float invMass = 1 / GetMass();
			float invColliderMass = 1 / otherDynamicObj->GetMass();
			float jLin = eCof / (invMass + invColliderMass);

			glm::vec3 collision_impulse_force = jLin * normal / deltaTs;

			glm::vec3 acceleration = _force / _mass + otherDynamicObj->GetMass();
			//glm::vec3 contact_force = glm::vec3(0.0f, 9.81f * _mass, 0.0f);
			glm::vec3 contact_force = _force - _mass * acceleration;
			glm::vec3 total_force = contact_force + collision_impulse_force;

			AddForce(total_force);
		}
	}
}