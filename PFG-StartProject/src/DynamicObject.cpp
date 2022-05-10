#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>

#include "DynamicObject.h"
#include "Utility.h"

DynamicObject::DynamicObject()
{
	dForce = glm::vec3(0.0f, 0.0f, 0.0f);
	dVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
	dMass = 1.0f;

	dScale = glm::vec3(1.0f, 1.0f, 1.0f);
	dStart = false;
}

DynamicObject::~DynamicObject()
{

}

void DynamicObject::StartSimulation(bool start)
{
	dStart = start;

	dRotationQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	glm::mat3 bodyInertia;

	bodyInertia = glm::mat3{
		(2.0f / 5.0f) * dMass * std::pow(dBoundingRadius, 2), 0, 0,
		0, (2.0f / 5.0f) * dMass * std::pow(dBoundingRadius, 2), 0,
		0, 0, (2.0f / 5.0f) * dMass * std::pow(dBoundingRadius, 2)
	};

	dBodyInertiaTensorInverse = glm::inverse(bodyInertia);

	CalcInverseInertiaTensor();

	dAngularVelocity = dInertiaTensorInverse * dAngularMomentum;
}

void DynamicObject::Update(GameObject* otherObj, float deltaTs)
{
	if (dStart == true)
	{
		ClearForces();

		glm::vec3 gravityForce(0.0f, -9.8 * dMass * 0.5f, 0.0f);
		AddForce(gravityForce);

		CollisionResponse(otherObj, deltaTs);

		//Verlet(deltaTs);
		//Euler(deltaTs);
		RungeKutta4(deltaTs);
	}

	UpdateModelMatrix();
}

void DynamicObject::CalcInverseInertiaTensor()
{
	dInertiaTensorInverse = dRotationMatrix * dBodyInertiaTensorInverse * glm::transpose(dRotationMatrix);
}

void DynamicObject::Euler(float deltaTs)
{
	float oneOverMass = 1 / dMass;
	dVelocity += (dForce * oneOverMass) * deltaTs;
	dPosition += dVelocity * deltaTs;
}

void DynamicObject::UpdateModelMatrix()
{
	glm::mat4 modelRotation = glm::mat4(dRotationMatrix);

	glm::quat rotation = glm::normalize(glm::quat_cast(modelRotation));

	dRotationMatrix = glm::mat3_cast(rotation);

	dModelMatrix = glm::translate(glm::mat4(1.0f), dPosition);
	dModelMatrix = dModelMatrix * glm::mat4_cast(rotation);
	dModelMatrix = glm::scale(dModelMatrix, dScale);
	dInvModelMatrix = glm::inverse(dModelMatrix);
}

void DynamicObject::RungeKutta2(float deltaTs)
{
	glm::vec3 localForce;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	//Evaluate once at t0
	localForce = dForce;
	acceleration = localForce / dMass;
	k0 = deltaTs * acceleration;

	//Evaluate once at t0 + deltaT/2.0 using half of k0
	localForce = dForce + k0 / 2.0f;
	acceleration = localForce / dMass;
	k1 = deltaTs * acceleration;

	//Evaluate once at t0 + deltaT using k1
	dVelocity += k1;
	dPosition += dVelocity * deltaTs;
}

void DynamicObject::RungeKutta4(float deltaTs)
{
	glm::vec3 localForce;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	//Evaluate once at t0
	localForce = dForce;
	acceleration = localForce / dMass;
	k0 = deltaTs * acceleration;

	//Evaluate twice at t0 + deltaT/2.0 using half of k0 and half of k1
	localForce = dForce + k0 / 2.0f;
	acceleration = localForce / dMass;
	k1 = deltaTs * acceleration;

	localForce = dForce + k1 / 2.0f;
	acceleration = localForce / dMass;
	k2 = deltaTs * acceleration;

	//Evaluate once at t0 + deltaT using k2
	localForce = dForce + k2;
	acceleration = localForce / dMass;
	k3 = deltaTs * acceleration;

	//Evaluate at t0 + deltaT using weighted sum of k0, k1, k2 and k3
	dVelocity += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
	//Update position 
	dPosition += dVelocity * deltaTs;

	//-------ROTATION PHYSICS HERE-------//

	glm::vec3 tempTorque;

	tempTorque = dTorque;
	k0 = deltaTs * tempTorque;

	tempTorque = dTorque + k0 / 2.0f;
	k1 = deltaTs * tempTorque;

	tempTorque = dTorque + k1 / 2.0f;
	k2 = deltaTs * tempTorque;

	tempTorque = dTorque + k2;
	k3 = deltaTs * tempTorque;

	dAngularMomentum += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;

	if (dStopped)
	{
		dAngularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	CalcInverseInertiaTensor();

	dAngularVelocity = dInertiaTensorInverse * dAngularMomentum;

	glm::mat3 omegaStar = glm::mat3(0.0f, -dAngularVelocity.z, dAngularVelocity.y,
		dAngularVelocity.z, 0.0f, -dAngularVelocity.x,
		-dAngularVelocity.y, dAngularVelocity.x, 0.0f);

	dRotationMatrix += omegaStar * dRotationMatrix * deltaTs;
}

void DynamicObject::Verlet(float deltaTs)
{
	glm::vec3 acceleration;
	
	acceleration = dForce / dMass;

	dPreviousPosition = dPosition - dVelocity * deltaTs + 0.5f * acceleration * deltaTs * deltaTs;

	dPosition = -dPreviousPosition + 2.0f * dPosition + acceleration * deltaTs * deltaTs;

	dVelocity = (dPosition - dPreviousPosition) / (2.0f * deltaTs);

	dVelocity += acceleration * deltaTs;
	
}

void DynamicObject::CollisionResponse(GameObject* otherObj, float deltaTs)
{
	float elasticity = 0.8;
	int type = otherObj->GetType();

	if (type == 0)
	{
		GameObjectCollision(otherObj, deltaTs, elasticity);
	}
	else if (type == 1)
	{
		DynamicObjectCollision(otherObj, deltaTs, elasticity);
	}
}

void DynamicObject::GameObjectCollision(GameObject* otherObject, float deltaTs, float elasticity)
{
	dStopped = false;

	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 thisObjCentre = dPosition;
	glm::vec3 otherObjCentre = dPosition + dVelocity * deltaTs;
	glm::vec3 q = otherObject->GetPosition();
	glm::vec3 contactPoint;
	const float r = GetBoundingRadius();

	bool collision = PFG::MovingSphereToPlaneCollision(normal, thisObjCentre, otherObjCentre, q, r, contactPoint);

	if (collision)
	{
		dPosition = contactPoint;

		glm::vec3 r1 = GetBoundingRadius() * glm::vec3(0.0f, 1.0f, 0.0f);
		glm::vec3 relativeVel = dVelocity;
		float invMass = 1 / GetMass();
		
		glm::vec3 colliderVel = glm::vec3(0.0f, 0.0f, 0.0f); //Floor has no velocity, as it is stationary
		float invColliderMass = 0.0f; //Floor should have no mass for the equaton to work

		glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f); // floor normal up

		float linEcof = glm::dot(-(1.0f + elasticity) * (dVelocity), normal); //Coefficient used in the top half of the linear impulse calculation
		float jLin = linEcof / (invMass + invColliderMass); //Linear impulse calculation

		float angEcof = glm::dot(-(1.0f * elasticity) * (relativeVel), normal); //Coefficient used in the top half of the angular impulse calculation
		float jAng = angEcof / (invMass + invColliderMass + glm::dot(dInertiaTensorInverse * (r1 * normal), normal)); //Angular impulse calculation

		glm::vec3 collisionImpulseForce = (jAng + jLin) * normal / deltaTs; //Calculation for collision impulse force
		glm::vec3 contactForce = -(dForce) * normal; //Contact force calculation

		glm::vec3 totalForce = contactForce + collisionImpulseForce; //Getting total force

		AddForce(totalForce); //Applying total force
		dVelocity += (collisionImpulseForce / dMass);

	}
}

void DynamicObject::DynamicObjectCollision(GameObject* otherObject, float deltaTs, float elasticity)
{
	DynamicObject* otherDynamicObj = dynamic_cast<DynamicObject*>(otherObject);
	glm::vec3 thisObjCentre1 = dPosition + dVelocity * deltaTs;
	glm::vec3 otherObjCentre1 = otherDynamicObj->GetPosition() + otherDynamicObj->GetVelocity() * deltaTs;
	float radius1 = GetBoundingRadius();
	float radius2 = otherDynamicObj->GetBoundingRadius();
	glm::vec3 cp;

	bool collision = PFG::SphereToSphereCollision(thisObjCentre1, otherObjCentre1, radius1, radius2, cp);

	if (collision)
	{

		std::cout << "Collided with sphere" << std::endl;

		glm::vec3 ColliderVel = otherDynamicObj->GetVelocity();
		glm::vec3 relativeVel = dVelocity - ColliderVel;
		glm::vec3 normal = glm::normalize(thisObjCentre1 - otherObjCentre1);

		glm::vec3 contactPosition = radius1 * normal;
		float eCof = -(1.0f + elasticity) * glm::dot(relativeVel, normal);
		float invMass = 1 / GetMass();
		float invColliderMass = 1 / otherDynamicObj->GetMass();
		float jLin = eCof / (invMass + invColliderMass);

		glm::vec3 collision_impulse_force = jLin * normal / deltaTs;

		glm::vec3 acceleration = dForce / dMass + otherDynamicObj->GetMass();
		//glm::vec3 contact_force = glm::vec3(0.0f, 9.81f * _mass, 0.0f);
		glm::vec3 contact_force = dForce - dMass * acceleration;
		glm::vec3 total_force = contact_force + collision_impulse_force;

		AddForce(total_force);
	}
}