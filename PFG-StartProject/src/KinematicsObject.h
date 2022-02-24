#ifndef _KinematicsObject_H_
#define _KinematicsObject_H_

#include "GameObject.h"

/*! \brief Brief description.
*  Physics object class is derived from the GameObject class, as a one type/class of game objects
*  It sets up parameters for physics computation and calls certain physics algorithms.
*  It returns the position and orientation of the object for the visualisation engine to display.
*  It is important to not include any graphics drawings in this class. This is the principle of the separation 
*  of physics computation from graphics
*
*/

class KinematicsObject : public GameObject
{
public:

	/** KinematicsObject constructor
	*/
	KinematicsObject();
	/** KinematicsObject destructor
	*/
	~KinematicsObject();

	/** Update function to override the base class function
	*   for physics computation
	*   This function is typically organized that it calls each physics computation procedures/algorithms
	*   in a sequential order, so that each computational algorithm provides required information for the 
	*   next procedure.
	*   @param float deltaTs simulation time step length
	*/
	virtual void Update(float deltaTs);

	/** Set position for the object
	* @param glm::vec3 pos a 3D vector for the position of the object
	*/
	void SetPosition(glm::vec3 pos) { _position = pos; }
	/** Set velocity for the object
	* @param glm::vec3 vel a 3D vector for the velocity of the object
	*/
	void SetVelocity(glm::vec3 vel) { _velocity = vel; }
	/** Set scale for the object
	* @param glm::vec3 vel a 3D vector for the scale of the object
	*/
	void SetScale(glm::vec3 scale) { _scale = scale; }
	/** Get the position of the object
	* @return a 3D vector 
	*/
	glm::vec3 GetPosition() { return _position; }
	/** Get the orientation of the object
	* @return a 4x4 matrix
	*/
	glm::mat4 GetOrientation() { return _orientation; }

	glm::vec3 Getvelocity() { return _velocity; }
	/** A boolean variable to control the start of the simulation This matrix is the camera's lens
	*/
	void StartSimulation(bool start) { _start = start; }

private:

	/**Update the model matrix with the current position, orientation and scale
	*
	*/
	void UpdateModelMatrix();

	/** Set up physics parameters for computation
	*  Specific parameters are determined by the physics simulation 
	*/
	/** Position of the object
	*/
	glm::vec3 _position;
	/** Velocity of the object
	*/
	glm::vec3 _velocity;
	/** Scale of the object
	*/
	glm::vec3 _scale;
	/** Orientation of the object
	*/
	glm::mat4 _orientation;
	
	/** A boolean variable to control the start of the simulation This matrix is the camera's lens
	*/
	bool _start;
};

#endif //!_KinematicsObject_H_
