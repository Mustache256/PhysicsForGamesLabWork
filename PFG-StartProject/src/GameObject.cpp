#include <GLM/gtc/type_ptr.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include "GameObject.h"

/*! \brief Brief description.
*  GameObject class contains a mesh, a material, a position and an orientation information
*  about the game object. This should be a base class for different types of game object.
*
*/
GameObject::GameObject()
{
	// Set default values formesh, material, rotation and scale
	gMesh = NULL;
	gMaterial = NULL;
	gScale = glm::vec3(1.0f, 1.0f, 1.0f);
	gRotation = glm::vec3(0.0f, 0.0f, 0.0f);
}

GameObject::~GameObject()
{
	
}

void GameObject::Update( float deltaTs )
{
	//Set objects scale, position and rotation or each axis based on associated variable values
	modelMatrix = glm::translate(glm::mat4(1.0f), gPosition);
	modelMatrix = glm::scale(modelMatrix, gScale);
	modelMatrix = glm::rotate(modelMatrix, gRotation.x, glm::vec3(1, 0, 0));
	invModelMatrix = glm::inverse(modelMatrix);
	modelMatrix = glm::rotate(modelMatrix, gRotation.y, glm::vec3(0, 1, 0));
	invModelMatrix = glm::inverse(modelMatrix);
	modelMatrix = glm::rotate(modelMatrix, gRotation.z, glm::vec3(0, 0, 1));
	invModelMatrix = glm::inverse(modelMatrix);
}

void GameObject::Draw(glm::mat4 &viewMatrix, glm::mat4 &projMatrix)
{
	if( gMesh != NULL )
	{
		if( gMaterial != NULL )
		{
			// Give all the matrices to the material

			// This makes sure they are sent to the shader
			gMaterial->SetMatrices(modelMatrix, invModelMatrix, viewMatrix, projMatrix);
			// This activates the shader
			gMaterial->Apply();
		}

		// Sends the mesh data down the pipeline
		gMesh->Draw();

	}
}

void GameObject::SetType(int type)
{
	//Set the objects type
	gObjectType = type;
}

int GameObject::GetType()
{
	//Return the objects type
	return gObjectType;
}