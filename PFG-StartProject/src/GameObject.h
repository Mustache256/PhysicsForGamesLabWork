#ifndef __GAME_OBJECT__
#define __GAME_OBJECT__

#include "Mesh.h"
#include "Material.h"

/*! \brief Brief description.
*  GameObject class contains a mesh, a material, a position and an orientation information
*  about the game object. This should be a base class for different types of game object. 
*
*/

class GameObject
{
public:
	/** GameObject constructor
	* 
	*/
	GameObject();
	/** GameObject distructor
	*/
	~GameObject();

	/** Function for setting mesh geometry for the game object 
	* @param *input  a pointer to a mesh object
	*/
	void SetMesh(Mesh *input) {gMesh = input;}
	/** Function for setting material for the game object
	* @param *input  a pointer to a material object
	*/
	void SetMaterial(Material *input) {gMaterial = input;}
	/** Function for setting position for the game object
	* @param float posX x position
	* @param float posY y position
	* @param float posZ z position
	*/
	void SetPosition(float posX, float posY, float posZ) { gPosition.x = posX; gPosition.y = posY; gPosition.z = posZ; }
	/** Function for setting position for the game object
	* @param glm::vec3 value  a position 3D vector
	*/
	void SetPosition(glm::vec3 value) { gPosition = value; }
	/** Function for setting rotation for the game object
	* @param float rotX x rotation
	* @param float rotY y rotation
	* @param float rotZ z rotation
	*/
	void SetRotation(float rotX, float rotY, float rotZ) { gRotation.x = rotX; gRotation.y = rotY; gRotation.z = rotZ; }
	/** Function for setting scale for the game object
	* @param float sX x scale
	* @param float sY y scale
	* @param float sZ z scale
	*/
	void SetScale(float sX, float sY, float sZ) { gScale.x = sX; gScale.y = sY; gScale.z = sZ; }
	
	//Function for setting the objects velocity
	void SetInitialVelocity(glm::vec3 vel) { gInitialVelocity = vel; }

	//Function for getting the objects velocity
	glm::vec3 GetInitialVelocity() { return gInitialVelocity; }

	/** Function for getting position of the game object
	* @return The result
	*/
	glm::vec3 GetPosition() {return gPosition;}
	
	/** A virtual function for updating the simulation result at each time frame
	*   You need to expand this function 
	* @param float deltaTs the time intervel in second for the simulation frame
	*/
	virtual void Update( float deltaTs );
	/** A virtual function for drawing the simulation result
	*  The function takes viewing matrix and projection matrix 
	* @param glm::mat4 &viewMatrix a 4x4 matrix
	* @param glm::mat4 &projMatrix a 4x4 matrix
	*/
	virtual void Draw(glm::mat4 &viewMatrix, glm::mat4 &projMatrix);

	//Function for setting the objects mass
	void SetMass(float mass) { gMass = mass; }

	//Function for getting the objects mass
	float GetMass() { return gMass; }

	//Function for setting the objects type
	void SetType(int type);

	//Function for getting the objects type
	int GetType();

protected:

	/** The model geometry
	*/
	Mesh *gMesh;
	/** The material contains the shader
	*/
	Material *gMaterial;

	/** Matrix for the position of the game object
	*/ 
	glm::mat4 modelMatrix;
	/** Matrix for the orientation of the game object
	*/
	glm::mat4 invModelMatrix;
	
	/** Position of the model
	* The model matrix must be built from the position of the model geometry
	*/
	glm::vec3 gPosition;
	
	/** Orientation of the model
	* The model matrix must be built from the orientation of the model geometry
	*/
	glm::vec3 gRotation;
	/** Scale of the model
	* The model matrix must be built from the scale of the model geometry
	*/
	glm::vec3 gScale;

	//The mass of the object
	float gMass;

	//The type of the object
	int gObjectType;

	//The velocity of the object
	glm::vec3 gInitialVelocity;
};



#endif
