#pragma once
#ifndef _SCENE_H_
#define _SCENE_H_

#include "GameObject.h"
#include "Camera.h"
#include "KinematicsObject.h"
#include "DynamicObject.h"
#include "GameObject.h"

/*! \brief Brief description.
*  Scene class is a container for loading all the game objects in your simulation or your game. 
*  
*/

class Scene
{
public:

	/** Scene constructor
	* Currently the scene is set up in the constructor
	* This means the object(s) are loaded, given materials and positions as well as the camera and light
	*/
	Scene();
	/** Scene distructor
	*/
	~Scene();

	/** Scene update
	* This function is called for each simulation time step to
	* update on all objects in the scene
	*/
	void Update(float deltaTs, Input* input);

	/** 
	* Call this function to get a pointer to the camera
	* 
	*/
    Camera* GetCamera() { return camera; }

	/** Draw the scene from the camera's point of view
	*
	*/
	void Draw();

	//Defining function used to create spheres
	DynamicObject* CreateSphere(Material* mat, Mesh* modelMesh, glm::vec3 position, glm::vec3 scale, float mass, float boundingRadius);
	//Defining function used to create planes
	GameObject* CreatePlane(Material* mat, Mesh* modelMesh, glm::vec3 position, glm::vec3 rotation, glm::vec3 scale);

private:
	/** An example game level in the scene
	*/
	GameObject* _level; 
	/** The main camera in the scene 
	*/
	Camera* camera; 

	/**The position for a light that lits the scene
	*/
	glm::vec3 lightPosition; 
	/** This matrix represents the camera's position and orientation 
	*/
	glm::mat4 viewMatrix; 
	/** This matrix is the camera's lens
	*/
	glm::mat4 projMatrix; 
	/** A boolen variable to control the start of the simulation This matrix is the camera's lens
	*/
	bool simulationStart;
	//Temp variable for Exercise 5: Use kinematics equations to compute free fall
	//glm::vec3 _v_i; 

	//Defining vector used to store all DynamicObjects
	std::vector<DynamicObject*> sceneDynamicObjects;

	//Defining vector used to store all GameObjects
	std::vector<GameObject*> sceneGameObjects;

	//Defing fileread fucntion
	void fileRead(std::string fileName);

	//Defing vector that stores file input
	std::vector<std::string> fileInput;
};

#endif // !_SCENE_H_