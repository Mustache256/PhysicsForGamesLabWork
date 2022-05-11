#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>

#include "DynamicObject.h"
#include "Utility.h"

DynamicObject::DynamicObject()
{
	//Setting up variables for new object creation
	dForce = glm::vec3(0.0f, 0.0f, 0.0f);
	dVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
	dMass = 0.0f;
	dBoundingRadius = 0.0f;

	dTorque = glm::vec3(0.0f, 0.0f, 0.0f);
	dAngularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
	dAngularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	dAngVelocityMat = glm::mat3(0.0f);

	dRotationMatrix = glm::mat3(1.0f);
	dRotationQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	dScale = glm::vec3(1.0f, 1.0f, 1.0f);
	dStart = false;
}

DynamicObject::~DynamicObject()
{

}

void DynamicObject::StartSimulation(bool start)
{
	//Setting start tracking bool to true to indicate simulation start
	dStart = start;

	//Setting up rotation quaternion for simulation upon simulation start
	dRotationQuat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	//Creating  body inertia matrix and calculating its value for simulation upon simulation start
	glm::mat3 bodyInertia;

	bodyInertia = glm::mat3{
		(2.0f / 5.0f) * dMass * std::pow(dBoundingRadius, 2), 0, 0,
		0, (2.0f / 5.0f) * dMass * std::pow(dBoundingRadius, 2), 0,
		0, 0, (2.0f / 5.0f) * dMass * std::pow(dBoundingRadius, 2)
	};

	//Find the inverse of body inertia for use in calculating inverse ineria tensor
	dBodyInertiaTensorInverse = glm::inverse(bodyInertia);

	//Call inverse inertia tensor calculation function
	CalcInverseInertiaTensor();

	//Calculate angular velocity vector
	dAngularVelocity = dInertiaTensorInverse * dAngularMomentum;
}

void DynamicObject::Update(GameObject* otherObj, float deltaTs)
{
	//Only update dynamic objects if the simulation has begun
	if (dStart)
	{
		//Clear vectors related to force and torque each tick, to avoid values calculated and assigned to these vectors in previous ticks from affecting the values given to them in the current tick
		ClearForces();
		ClearTorque();

		//Defining how much force gravity has and applying it each tick (Force of gravity is halved (*0.5f) as there are issues that occur when gravity is too strong)
		glm::vec3 gravityForce(0.0f, -9.8 * dMass * 0.5f, 0.0f);
		AddForce(gravityForce);

		//Call function that handles collision detection to determine whether collision has occured
		CollisionResponse(otherObj, deltaTs);

		//Currently the simulation used the RK4 method to update the dynamics object's position and rotation each tick, however both RK2 and Euler methods are available for use, with the Verlet method only being able to update the dynamic object's position
		//Verlet(deltaTs);
		//Euler(deltaTs);
		//RungeKutta2(deltaTs);
		RungeKutta4(deltaTs);
	}

	//Call function that updates the dynamic object's model matrices
	UpdateModelMatrix();
}

void DynamicObject::CalcInverseInertiaTensor()
{
	//Calculate inverse inertia tensor
	dInertiaTensorInverse = dRotationMatrix * dBodyInertiaTensorInverse * glm::transpose(dRotationMatrix);
}

void DynamicObject::Euler(float deltaTs)
{
	//Caclualte inverse mass and store it
	float oneOverMass = 1 / dMass;
	//Update dynamic object velocity and position based off of calculations
	dVelocity += (dForce * oneOverMass) * deltaTs;
	dPosition += dVelocity * deltaTs;

	//Zero out object velocity if the object has stopped moving
	if (dStopped)
	{
		dVelocity.x = 0.0f;
		dVelocity.z = 0.0f;
	}

	//Caclulate and update angular momentum vector for use in determining object rotation
	dAngularMomentum += dTorque * deltaTs;

	//Zero out angular momentum vector if the object has stopped moving
	if (dStopped)
	{
		dAngularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	//Call inverse inertia tensor function to recalculate inverse inertia tensor value with new values
	CalcInverseInertiaTensor();

	//Calculate and update dynamic objects angular velocity
	dAngularVelocity = dInertiaTensorInverse * dAngularMomentum;

	//Update angular velocity matrix based off of newly calculated angular velocity vector
	dAngVelocityMat = glm::mat3(0.0f, -dAngularVelocity.z, dAngularVelocity.y, dAngularVelocity.z, 0.0f, -dAngularVelocity.x, -dAngularVelocity.y, dAngularVelocity.x, 0.0f);

	//Calculate and update dynamic objects rotation matrix
	dRotationMatrix += dAngVelocityMat * dRotationMatrix * deltaTs;
}

void DynamicObject::UpdateModelMatrix()
{
	//Update model rotation based off of rotation matrix, visually changing the dynamic object model to represent the rotation it is experiencing
	glm::mat4 modelRotation = glm::mat4(dRotationMatrix);

	//Cast model rotation matrix into the rotation quaternion
	glm::quat rotation = glm::normalize(glm::quat_cast(modelRotation));

	//Cast the rotation quaternion back into a matrix
	dRotationMatrix = glm::mat3_cast(rotation);

	//Move the dynamic object model to its new calculated position using the identity matrix
	modelMatrix = glm::translate(glm::mat4(1.0f), dPosition);
	//Update the rotation of the model
	modelMatrix = modelMatrix * glm::mat4_cast(rotation);
	//Update the scale of the object in case that has changed between the last tick and this tick
	modelMatrix = glm::scale(modelMatrix, dScale);
	//Update the inverse model matrix based off of the new model matrix
	invModelMatrix = glm::inverse(modelMatrix);
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

	//------ROTATION PHYSICS HERE------//
	glm::vec3 tempTorque;

	tempTorque = dTorque;
	k0 = deltaTs * tempTorque;
	tempTorque = dTorque + k0 / 2.0f;
	k1 = deltaTs * tempTorque;

	dAngularMomentum += k1;

	if (dStopped)
	{
		dAngularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	CalcInverseInertiaTensor();

	dAngularVelocity = dInertiaTensorInverse * dAngularMomentum;

	dAngVelocityMat = glm::mat3(0.0f, -dAngularVelocity.z, dAngularVelocity.y, dAngularVelocity.z, 0.0f, -dAngularVelocity.x, -dAngularVelocity.y, dAngularVelocity.x, 0.0f);

	dRotationMatrix += dAngVelocityMat * dRotationMatrix * deltaTs;
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

	dAngVelocityMat = glm::mat3(0.0f, -dAngularVelocity.z, dAngularVelocity.y, dAngularVelocity.z, 0.0f, -dAngularVelocity.x, -dAngularVelocity.y, dAngularVelocity.x, 0.0f);

	dRotationMatrix += dAngVelocityMat * dRotationMatrix * deltaTs;
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
	dStopped = false;
	float elasticity = 0.7f;
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
	glm::vec3 zeroVector = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 thisObjCenter = dPosition;
	glm::vec3 otherObjCenter = dPosition + dVelocity * deltaTs;
	glm::vec3 q = otherObject->GetPosition();
	glm::vec3 contactPoint;
	const float radius = GetBoundingRadius();

	bool collision = PFG::MovingSphereToPlaneCollision(normal, thisObjCenter, otherObjCenter, q, radius, contactPoint);

	if (collision)
	{
		dPosition = contactPoint;

		float jLin = 0.0f;
		float jAng = 0.0f;

		glm::vec3 r1 = radius * normal;
		glm::vec3 relativeVel = dVelocity;
		float invMass = 1 / GetMass();
		
		//Zeroing out plane velocity vector, as the plane will not move
		glm::vec3 colliderVel = zeroVector; 
		//Zeroing out plane mass as plane would technically have infinite mass, which would break the equation, so zeroing the mass makes the equation work
		float invColliderMass = 0.0f; 

		//Linear impulse calculation
		jLin = ECoefficient(elasticity, dVelocity, normal) / (invMass + invColliderMass); 

		//Angular impulse calculation
		jAng = ECoefficient(elasticity, relativeVel, normal) / (invMass + invColliderMass + glm::dot(dInertiaTensorInverse * (r1 * normal), normal)); 

		//Calculation for collision impulse force
		glm::vec3 collisionImpulseForce = CalcCollisionImpulseForce(jLin, jAng, normal); 
		//Calcualtion for how much contact force to apply
		glm::vec3 contactForce = -(dForce) * normal; 

		//Adding up collision impulse force and contact force to get total force
		glm::vec3 totalForce = contactForce + collisionImpulseForce; 

		//Applying total force
		AddForce(totalForce);
		//Updating object velocity based of a = F/m equation 
		dVelocity += (collisionImpulseForce / dMass);

		glm::vec3 forwardRelativeVelocity = relativeVel - glm::dot(relativeVel, normal) * normal;

		glm::vec3 forwardRelativeDirection = zeroVector;
		if (forwardRelativeVelocity != zeroVector)
		{
			forwardRelativeDirection = glm::normalize(forwardRelativeVelocity);
		}

		float multiplier = 0.5f;
		glm::vec3 frictionDirection = forwardRelativeDirection * -1.0f;
		glm::vec3 frictionForce = frictionDirection * multiplier * glm::length(contactForce);

		float velocityCheck = glm::length(forwardRelativeVelocity) - (glm::length(frictionForce) / dMass * deltaTs);

		if (velocityCheck > 0.0f)
		{
			AddForce(frictionForce);
		}
		else
		{
			frictionForce = forwardRelativeVelocity * -1.0f;
			AddForce(frictionForce);
			dStopped = true;
		}

		glm::vec3 tempTorque = (glm::cross(r1, contactForce)) + (glm::cross(r1, frictionForce));

		tempTorque.x -= dAngularMomentum.x * 20.0f;
		tempTorque.z -= dAngularMomentum.z * 20.0f;

		AddTorque(tempTorque);
	}
}

void DynamicObject::DynamicObjectCollision(GameObject* otherObject, float deltaTs, float elasticity)
{
	glm::vec3 zeroVector = glm::vec3(0.0f, 0.0f, 0.0f);
	DynamicObject* otherDynamicObj = dynamic_cast<DynamicObject*>(otherObject);
	glm::vec3 thisObjCenter = dPosition + dVelocity * deltaTs;
	//glm::vec3 thisObjCentre1 = dPosition;
	glm::vec3 otherObjCenter = otherDynamicObj->GetPosition() + otherDynamicObj->GetVelocity() * deltaTs;
	//glm::vec3 otherObjCentre1 = otherDynamicObj->GetPosition();
	float radius1 = GetBoundingRadius();
	float radius2 = otherDynamicObj->GetBoundingRadius();
	glm::vec3 contactPoint;

	bool collision = PFG::SphereToSphereCollision(thisObjCenter, otherObjCenter, radius1, radius2, contactPoint);

	if (collision)
	{
		glm::vec3 ColliderVel = otherDynamicObj->GetVelocity();
		glm::vec3 relativeVel = dVelocity - ColliderVel;
		glm::vec3 normal = glm::normalize(thisObjCenter - otherObjCenter);

		glm::vec3 r1 = GetBoundingRadius() * normal;

		glm::vec3 contactPosition = radius1 * normal;

		float jLin = 0.0f;
		float jAng = 0.0f;

		float invMass = 1 / GetMass();
		float invColliderMass = 1 / otherDynamicObj->GetMass();

		jLin = ECoefficient(elasticity, relativeVel, normal) / (invMass + invColliderMass);

		jAng = ECoefficient(elasticity, relativeVel, normal) / (invMass + invColliderMass + glm::dot(dInertiaTensorInverse * (r1 * normal), normal)); 

		glm::vec3 collisionImpulseForce = CalcCollisionImpulseForce(jLin, jAng, normal);

		glm::vec3 acceleration = dForce / dMass + otherDynamicObj->GetMass();

		glm::vec3 contactForce = dForce - dMass * acceleration;

		glm::vec3 totalForce = contactForce + collisionImpulseForce;

		AddForce(totalForce);
		dVelocity += (collisionImpulseForce / dMass);

		glm::vec3 forwardRelativeVelocity = relativeVel - glm::dot(relativeVel, normal) * normal;

		glm::vec3 forwardRelativeDirection = zeroVector;
		if (forwardRelativeVelocity != zeroVector)
		{
			forwardRelativeDirection = glm::normalize(forwardRelativeVelocity);
		}

		float multiplier = 0.5f;
		glm::vec3 frictionDirection = forwardRelativeDirection * -1.0f;
		glm::vec3 frictionForce = frictionDirection * multiplier * glm::length(contactForce);

		float velocityCheck = glm::length(forwardRelativeVelocity) - (glm::length(frictionForce) / dMass * deltaTs);

		if (velocityCheck > 0.0f)
		{
			AddForce(frictionForce);
		}
		else
		{
			frictionForce = forwardRelativeVelocity * -1.0f;
			AddForce(frictionForce);
			dStopped = true;
		}

		glm::vec3 tempTorque = (glm::cross(r1, contactForce)) + (glm::cross(r1, frictionForce));

		tempTorque.x -= dAngularMomentum.x * 20.0f;
		tempTorque.z -= dAngularMomentum.z * 20.0f;

		AddTorque(tempTorque);
	}
}

float DynamicObject::ECoefficient(float elasticity, glm::vec3 velocity, glm::vec3 normal)
{
	float result = -(1.0f + elasticity) * glm::dot(velocity, normal);
	return result;
}

glm::vec3 DynamicObject::CalcCollisionImpulseForce(float linear, float angular, glm::vec3 normal)
{
	glm::vec3 result = (angular + linear) * normal;
	return result;
}