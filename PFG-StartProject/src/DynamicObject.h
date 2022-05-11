#ifndef _DynamicObject_H_
#define _DynamicObject_H_

#include "GameObject.h"
#include <glm/gtc/quaternion.hpp>

/*! \brief Brief description.
*  This physics dynamic object class is derived from the GameObject class, as a one type/class of game objects
*  It sets up parameters for physics computation and calls certain physics algorithms.
*  It returns the position and orientation of the object for the visualisation engine to display.
*  It is important to not include any graphics drawings in this class. This is the principle of the separation
*  of physics computation from graphics
*
*/

class DynamicObject : public GameObject
{
public:

	/** DynamicObject constructor
	*/
	DynamicObject();
	/** DynamicObject destructor
	*/
	~DynamicObject();

	/** Update function to override the base class function
	*   for physics computation
	*   This function is typically organized that it calls each physics computation procedures/algorithms
	*   in a sequential order, so that each computational algorithm provides required information for the
	*   next procedure.
	*   @param float deltaTs simulation time step length
	*/
	virtual void Update(GameObject* otherObj, float deltaTs);
	//virtual void UpdateBefore(float deltaTs);

	//virtual void CollisionUpdate(GameObject* otherObj, float deltaTs);

	//virtual void UpdateAfter(float deltaTs);

	/** Add force that acts on the object to the total force for physics computation
	*  
	*   @param const glm::vec3 force 
	*/
	void AddForce(const glm::vec3 force) { dForce += force; }

	void AddTorque(glm::vec3 torque) { dTorque += torque; }

	void ClearForces() { dForce = glm::vec3(0.0f, 0.0f, 0.0f); }

	void ClearTorque() { dTorque = glm::vec3(0.0f, 0.0f, 0.0f); }
	/** Numerical integration function to compute the current velocity and the current position
	* based on the velocity and the position of the previous time step
	*   @param float deltaTs simulation time step length
	*/
	 void Euler(float deltaTs);
	 void RungeKutta2(float deltaTs);
	 void RungeKutta4(float deltaTs);
	 void Verlet(float deltaTs);

	 void CollisionResponse(GameObject* otherObject, float deltaTs);

	 void GameObjectCollision(GameObject* otherObject, float deltaTs, float elasticity);

	 void DynamicObjectCollision(GameObject* otherObejct, float deltaTs, float elasticity);

	/** Set force for the object
	* @param glm::vec3 force a 3D vector for the force acting on the object
	*/
	void SetForce(const glm::vec3 force) { dForce = force; }
	/** Set mass for the object
	* @param float mass a 3D vector for the mass of the object
	*/
	void SetMass(float mass) { dMass = mass; }
	/** Set a sphere bounding volume for the object
	* @param float r  the radius of the bounding sphere of the object
	*/
	void SetBoundingRadius(float r) { dBoundingRadius = r; }

	/** Set position for the object
	* @param glm::vec3 pos a 3D vector for the position of the object
	*/
	void SetPosition(const glm::vec3 pos) { dPosition = pos; }
	/** Set velocity for the object
	* @param glm::vec3 vel a 3D vector for the velocity of the object
	*/
	void SetVelocity(const glm::vec3 vel) { dVelocity = vel; }
	/** Set scale for the object
	* @param glm::vec3 vel a 3D vector for the scale of the object
	*/
	void SetScale(const glm::vec3 scale) { dScale = scale; }

	/** Get the force acting on the object
	* @return a 3D vector
	*/
	const glm::vec3 GetForce() const { return dForce; }
	/** Get the mass of the object
	* @return the result
	*/
	const float GetMass() const { return dMass; }
	/** Get the radius of the bounding sphere of the object
	* @return the result
	*/
	const float GetBoundingRadius() const { return dBoundingRadius; }
	/** Get the position of the object
	* @return a 3D vector
	*/
	const glm::vec3 GetPosition() const { return dPosition; }
	/** Get the orientation of the object
	* @return a 4x4 matrix
	*/
	const glm::mat4 GetOrientation() const { return dOrientation; }

	const glm::vec3 GetVelocity() const { return dVelocity; }

	void CalcInverseInertiaTensor();
	float ECoefficient(float elasticity, glm::vec3 velocity, glm::vec3 normal);
	glm::vec3 CalcCollisionImpulseForce(float linear, float angular, glm::vec3 normal);

	/** A boolean variable to control the start of the simulation This matrix is the camera's lens
	*/
	void StartSimulation(bool _start);

private:

	/**Update the model matrix with the current position, orientation and scale
	*
	*/
	void UpdateModelMatrix();

	/** Set up physics parameters for computation
	*  Specific parameters are determined by the physics simulation
	*/
	/** The total force on the object
	*/
	glm::vec3 dForce;
	/** Position of the object
	*/
	glm::vec3 dPosition;
	/** Previous position of the object
	*/
	glm::vec3 dPreviousPosition;
	/** Velocity of the object
	*/
	glm::vec3 dVelocity;
	/** The mass of the object
	*/
	float dMass;
	/** Scale of the object
	*/
	/** The radius of a bounding sphere of the object
	*/
	float dBoundingRadius;
	/** Scale of the object
	*/
	glm::vec3 dScale;
	/** Orientation of the object
	*/
	glm::mat4 dOrientation;

	/** Angular dynamics Troque
	*/
	glm::vec3 dTorque;
	/** Angular dynamics angular velocity
	*/
	glm::vec3 dAngularVelocity;
	/** Angular dynamics angular momentum
	*/
	glm::vec3 dAngularMomentum;
	/** Angular dynamics inverse inertia tensor
	*/
	glm::mat3 dInertiaTensorInverse;
	/** Angular dynamics inverse body inertia tensor
	*/
	glm::mat3 dBodyInertiaTensorInverse;
	/** Angular dynamics rotation matrix
	*/
	glm::mat3 dRotationMatrix;
	/** Quaterion
	*/
	glm::quat dRotationQuat;

	glm::mat3 dAngVelocityMat;

	/**
	* lerp
	*/
	float lerp(float a, float b, float t)
	{
		return a + (b - a) * t;
	}
	/** A boolean variable to control the start of the simulation This matrix is the camera's lens
	*/
	bool dStart;

	bool dStopped;
};

#endif //!_DynamicObject_H_

