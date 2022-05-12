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
	//Local variable definition
	glm::vec3 localForce;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	//Calculate acceleration to evaluate velocity at current timestep
	localForce = dForce;
	acceleration = localForce / dMass;
	k0 = deltaTs * acceleration;

	//Calculate acceleration to evaluate velocity at the midpoint, using half of k0 in the evaluation
	localForce = dForce + k0 / 2.0f;
	acceleration = localForce / dMass;
	k1 = deltaTs * acceleration;

	//Update velocity at next timestep and use said velocity to update object position
	dVelocity += k1;
	dPosition += dVelocity * deltaTs;

	//Define local torque variable for use in calculations
	glm::vec3 tempTorque;

	//Set local torque value
	tempTorque = dTorque;

	//Calculate torque at current time step and evaluate what object torque would be at midpoint
	k0 = deltaTs * tempTorque;
	tempTorque = dTorque + k0 / 2.0f;
	k1 = deltaTs * tempTorque;

	//Update objects angular momentum based off of calculated torque at midpoint
	dAngularMomentum += k1;

	//Zero out angular momentum of object if the movement of the object has stopped
	if (dStopped)
	{
		dAngularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	//Call inverse inertia tensor calculation function 
	CalcInverseInertiaTensor();

	//Evaluate objects angular velocity using newly evaluated angular momentum and calculated inverse inertia tensor
	dAngularVelocity = dInertiaTensorInverse * dAngularMomentum;

	//Update objects angular velocity matrix with new values from newly evaluated angular velocity
	dAngVelocityMat = glm::mat3(0.0f, -dAngularVelocity.z, dAngularVelocity.y, dAngularVelocity.z, 0.0f, -dAngularVelocity.x, -dAngularVelocity.y, dAngularVelocity.x, 0.0f);

	//Update object rotation matrix using updated angular velocity matrix along with delta time and the current value of the rotation matrix
	dRotationMatrix += dAngVelocityMat * dRotationMatrix * deltaTs;
}

void DynamicObject::RungeKutta4(float deltaTs)
{
	//Local variable definition
	glm::vec3 localForce;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	//Evaluate once to find velocity and current time step
	localForce = dForce;
	acceleration = localForce / dMass;
	k0 = deltaTs * acceleration;

	//Evaluate twice at the current time step to find the values for the 2 midpoints, using half of the current time step and half of the calculated first midpoint
	//Calculating the first midpoint
	localForce = dForce + k0 / 2.0f;
	acceleration = localForce / dMass;
	k1 = deltaTs * acceleration;

	//Calculating the second midpoint
	localForce = dForce + k1 / 2.0f;
	acceleration = localForce / dMass;
	k2 = deltaTs * acceleration;

	//Calculate the endpoint using second midpoint
	localForce = dForce + k2;
	acceleration = localForce / dMass;
	k3 = deltaTs * acceleration;

	//Caclulate future velocity using the weighted sum of all the previous evaluations and update the velocity of the object
	dVelocity += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
	//Update object position based on calculated new veloci
	dPosition += dVelocity * deltaTs;

	//Define local torque variable for use in calculations
	glm::vec3 tempTorque;

	//Set local torque value
	tempTorque = dTorque;
	// Calculate torque at current time step and evaluate what object torque would be at first midpoint
	k0 = deltaTs * tempTorque;
	tempTorque = dTorque + k0 / 2.0f;
	k1 = deltaTs * tempTorque;

	//Evaluate torque at second midpoint based off of torque found at first midpoint
	tempTorque = dTorque + k1 / 2.0f;
	k2 = deltaTs * tempTorque;

	//Evaluate torque at endpoint based off of torque found at second midpoint
	tempTorque = dTorque + k2;
	k3 = deltaTs * tempTorque;

	//Update objects angular momentum using the weighted sum of all the previously calculated points
	dAngularMomentum += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;

	//Zero out angular momentum of object if the movement of the object has stoppe
	if (dStopped)
	{
		dAngularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	//Call inverse inertia tensor calculation function
	CalcInverseInertiaTensor();

	//Evaluate objects angular velocity using newly evaluated angular momentum and calculated inverse inertia tensor
	dAngularVelocity = dInertiaTensorInverse * dAngularMomentum;

	//Update objects angular velocity matrix with new values from newly evaluated angular velocity
	dAngVelocityMat = glm::mat3(0.0f, -dAngularVelocity.z, dAngularVelocity.y, dAngularVelocity.z, 0.0f, -dAngularVelocity.x, -dAngularVelocity.y, dAngularVelocity.x, 0.0f);

	//Update object rotation matrix using updated angular velocity matrix along with delta time and the current value of the rotation matrix
	dRotationMatrix += dAngVelocityMat * dRotationMatrix * deltaTs;
}

void DynamicObject::Verlet(float deltaTs)
{
	//Defining local variables
	glm::vec3 acceleration;
	
	//Calculate objects current acceleration
	acceleration = dForce / dMass;

	//Calculate objects previous position through the use of its current position and velocity, along with delta time and the newly calculkated acceleration
	dPreviousPosition = dPosition - dVelocity * deltaTs + 0.5f * acceleration * deltaTs * deltaTs;

	//Update object position using the negative calculated previous position vector along with the calculated acceleration, current value of the object position vector and delta time
	dPosition = -dPreviousPosition + 2.0f * dPosition + acceleration * deltaTs * deltaTs;

	//Set velocity at current time step to be a value calculated using both the previous and current position vectors, along with a division of 2 * delta time
	dVelocity = (dPosition - dPreviousPosition) / (2.0f * deltaTs);

	//Update object velocity using calculated acceleration value multiplied by delta times
	dVelocity += acceleration * deltaTs;
	
}

void DynamicObject::CollisionResponse(GameObject* otherObj, float deltaTs)
{
	//Set stopped bool to false to show object is colliding/moving
	dStopped = false;
	//Set elasticity value for impulse force calculations
	float elasticity = 0.6f;
	//Get object type of object being collided with to determine whether that object is a GameObject or DynamicObject
	int type = otherObj->GetType();

	//Call GameObject collision function when colliding with a GameObject 
	if (type == 0)
	{
		GameObjectCollision(otherObj, deltaTs, elasticity);
	}
	//Call DynamicObject collision function when colliding with a DynamicObject
	else if (type == 1)
	{
		DynamicObjectCollision(otherObj, deltaTs, elasticity);
	}
}

void DynamicObject::GameObjectCollision(GameObject* otherObject, float deltaTs, float elasticity)
{
	//Defining Zero Vector and plane normal vectors
	glm::vec3 zeroVector = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);

	//Getting the center of this DynamicObject and calculating the center of the other object using the position and the velocity of this DynamicObject multiplied by delta time
	glm::vec3 thisObjCenter = dPosition;
	glm::vec3 otherObjCenter = dPosition + dVelocity * deltaTs;

	//Get and store the position vector the collided with object
	glm::vec3 q = otherObject->GetPosition();

	//Define contact point vector
	glm::vec3 contactPoint;

	//Get and store radius of this DynamicObject
	const float radius = GetBoundingRadius();

	//Call function that calculates and determines whether collision is occuring
	bool collision = PFG::MovingSphereToPlaneCollision(normal, thisObjCenter, otherObjCenter, q, radius, contactPoint);

	//Execute collsion code if collision has occured
	if (collision)
	{
		//Set this DynamicObjects position to be the same as the calculated contact point between the 2 objects
		dPosition = contactPoint;

		//Define and zero out linear and angular impulse variables
		float jLin = 0.0f;
		float jAng = 0.0f;

		//Radius * normal result (Vector from contact point) stored for use in angular impulse and torque calculation
		glm::vec3 r1 = radius * normal;
		//Setting object relative velocity to be equal to objects actual velocity
		glm::vec3 relativeVel = dVelocity;
		//Calculating and storing object's inverse mass for use in both linear and angular impulse calculations
		float invMass = 1 / GetMass();
		
		//Zeroing out plane velocity vector, as the plane will not move
		glm::vec3 colliderVel = zeroVector; 
		//Zeroing out plane mass as plane would technically have infinite mass, which would break the equation, so zeroing the mass makes the equation work
		float invColliderMass = 0.0f; 

		//Linear impulse calculation, using the E Coefficient function to calculate top half of the equation
		jLin = ECoefficient(elasticity, dVelocity, normal) / (invMass + invColliderMass); 

		//Angular impulse calculation, using the E Coefficient function to calculate top half of the equation
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

		//Define forward relative velocity vector and calculate its vlaue for this DynamicObject using previously calculated relative velocity along with the normal of the plane
		glm::vec3 forwardRelativeVelocity = relativeVel - glm::dot(relativeVel, normal) * normal;

		//Define and zero out forward relative direction vector for future use
		glm::vec3 forwardRelativeDirection = zeroVector;

		//If the object still has forward velocity, and is therefore still moving, update its forward relative driection vector using the normalized value of the forward relative velocity vector
		if (forwardRelativeVelocity != zeroVector)
		{
			forwardRelativeDirection = glm::normalize(forwardRelativeVelocity);
		}

		//Define multiplier variable and set its value so it can be used in friction force calculation
		float multiplier = 0.5f;
		//Calculate the direction in which friction should act based off of the negative (opposite) of the forward direction of this object
		glm::vec3 frictionDirection = forwardRelativeDirection * -1.0f;
		//Calculate how much friction force should be applied to the object based on the direction the friction should act in, as well as the length of the contact force vector
		glm::vec3 frictionForce = frictionDirection * multiplier * glm::length(contactForce);

		//Define and calculate velocity check variable for use in determining whetehr this object is still moving or not, this is calculated using the lengths of both the forward relative velocity and friction force vectors, as well as this object's mass and delta time
		float velocityCheck = glm::length(forwardRelativeVelocity) - (glm::length(frictionForce) / dMass * deltaTs);

		//If the object still has a velocity, continue to apply frictional force to the object until it has no velocity
		if (velocityCheck > 0.0f)
		{
			AddForce(frictionForce);
		}
		else
		{
			//Update frictional force and apply it to ensure object has come to a complete stop
			frictionForce = forwardRelativeVelocity * -1.0f;
			AddForce(frictionForce);
			//Set stopped bool to true to inform the rest of the program that the object has stopped
			dStopped = true;
		}

		//Calculate objects torque vector using the total of the cross products of the contact force and friction force with the contact point vector
		glm::vec3 tempTorque = (glm::cross(r1, contactForce)) + (glm::cross(r1, frictionForce));

		//Update obejcts torque on the x and z axes based off of objects angular momentum on those axes
		tempTorque.x -= dAngularMomentum.x * 20.0f;
		tempTorque.z -= dAngularMomentum.z * 20.0f;

		//Add torque to object
		AddTorque(tempTorque);
	}
}

void DynamicObject::DynamicObjectCollision(GameObject* otherObject, float deltaTs, float elasticity)
{
	//Defining Zero Vector
	glm::vec3 zeroVector = glm::vec3(0.0f, 0.0f, 0.0f);
	//Cast collision GameObject into a DynamicObject in order to gain access to its DynamicObject associated variables
	DynamicObject* otherDynamicObj = dynamic_cast<DynamicObject*>(otherObject);

	//Calculate both objects center points using there respective positions and velocities
	glm::vec3 thisObjCenter = dPosition + dVelocity * deltaTs;
	glm::vec3 otherObjCenter = otherDynamicObj->GetPosition() + otherDynamicObj->GetVelocity() * deltaTs;

	//Get and store each objects radii
	float radius1 = GetBoundingRadius();
	float radius2 = otherDynamicObj->GetBoundingRadius();

	//Defining vector that stores collision contact point
	glm::vec3 contactPoint;

	//Call function that determines whether collision has occured
	bool collision = PFG::SphereToSphereCollision(thisObjCenter, otherObjCenter, radius1, radius2, contactPoint);

	//Only execute code if collision has occured
	if (collision)
	{
		//Get and store the velocity of the collision project
		glm::vec3 ColliderVel = otherDynamicObj->GetVelocity();
		//Calcualte the relative velocity of this object based off of its own velocity minus the collision object's velocity
		glm::vec3 relativeVel = dVelocity - ColliderVel;
		//Find the nromal between the 2 objects by normalizing between their center points
		glm::vec3 normal = glm::normalize(thisObjCenter - otherObjCenter);

		//Storing radius * normal calculation for use in both torque and angular impulse calculations
		glm::vec3 r1 = GetBoundingRadius() * normal;

		//Calculating the contact position using this objects radius and the calculated normal
		glm::vec3 contactPosition = radius1 * normal;

		//Define and zero out linear and angular impulse variables
		float jLin = 0.0f;
		float jAng = 0.0f;

		//Calcualte the inverse mass of both objects and store them for later use
		float invMass = 1 / GetMass();
		float invColliderMass = 1 / otherDynamicObj->GetMass();

		//Linear impulse calculation, using the E Coefficient function to calculate top half of the equation
		jLin = ECoefficient(elasticity, relativeVel, normal) / (invMass + invColliderMass);

		//Angular impulse calculation, using the E Coefficient function to calculate top half of the equation
		jAng = ECoefficient(elasticity, relativeVel, normal) / (invMass + invColliderMass + glm::dot(dInertiaTensorInverse * (r1 * normal), normal)); 

		//Calculation for collision impulse force
		glm::vec3 collisionImpulseForce = CalcCollisionImpulseForce(jLin, jAng, normal);
		//Calculation fo teh total acceleration to apply based off of this objects current force divided by the total mass of both objects
		glm::vec3 acceleration = dForce / dMass + otherDynamicObj->GetMass();
		//Calculation for how much contact force to apply
		glm::vec3 contactForce = dForce - dMass * acceleration;

		//Adding up collision impulse force and contact force to get total force
		glm::vec3 totalForce = contactForce + collisionImpulseForce;

		//Applying total force
		AddForce(totalForce);
		//Updating object velocity based of a = F/m equation 
		dVelocity += (collisionImpulseForce / dMass);

		//Define forward relative velocity vector and calculate its vlaue for this DynamicObject using previously calculated relative velocity along with the calculated normal
		glm::vec3 forwardRelativeVelocity = relativeVel - glm::dot(relativeVel, normal) * normal;

		//Define and zero out forward relative direction vector for future use
		glm::vec3 forwardRelativeDirection = zeroVector;

		//If the object still has forward velocity, and is therefore still moving, update its forward relative driection vector using the normalized value of the forward relative velocity vector
		if (forwardRelativeVelocity != zeroVector)
		{
			forwardRelativeDirection = glm::normalize(forwardRelativeVelocity);
		}

		//Define multiplier variable and set its value so it can be used in friction force calculation
		float multiplier = 0.5f;
		//Calculate the direction in which friction should act based off of the negative (opposite) of the forward direction of this object
		glm::vec3 frictionDirection = forwardRelativeDirection * -1.0f;
		//Calculate how much friction force should be applied to the object based on the direction the friction should act in, as well as the length of the contact force vector
		glm::vec3 frictionForce = frictionDirection * multiplier * glm::length(contactForce);

		//Define and calculate velocity check variable for use in determining whetehr this object is still moving or not, this is calculated using the lengths of both the forward relative velocity and friction force vectors, as well as this object's mass and delta time
		float velocityCheck = glm::length(forwardRelativeVelocity) - (glm::length(frictionForce) / dMass * deltaTs);

		//If the object still has a velocity, continue to apply frictional force to the object until it has no velocity
		if (velocityCheck > 0.0f)
		{
			AddForce(frictionForce);
		}
		else
		{
			//Update frictional force and apply it to ensure object has come to a complete stop 
			frictionForce = forwardRelativeVelocity * -1.0f;
			AddForce(frictionForce);
			//Set stopped bool to true to inform the rest of the program that the object has stopped 
			dStopped = true;
		}

		//Calculate objects torque vector using the total of the cross products of the contact force and friction force with the contact point vector  
		glm::vec3 tempTorque = (glm::cross(r1, contactForce)) + (glm::cross(r1, frictionForce));

		//Update obejcts torque on the x and z axes based off of objects angular momentum on those axes
		tempTorque.x -= dAngularMomentum.x * 20.0f;
		tempTorque.z -= dAngularMomentum.z * 20.0f;

		//Add torque to object
		AddTorque(tempTorque);
	}
}

float DynamicObject::ECoefficient(float elasticity, glm::vec3 velocity, glm::vec3 normal)
{
	//ECoefficient calculation used in both linear nad angular impulse equations
	float result = -(1.0f + elasticity) * glm::dot(velocity, normal);
	return result;
}

glm::vec3 DynamicObject::CalcCollisionImpulseForce(float linear, float angular, glm::vec3 normal)
{
	//Impulse force calculation
	glm::vec3 result = (angular + linear) * normal;
	return result;
}