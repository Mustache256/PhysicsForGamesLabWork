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
	// Initialise everything here
	gMesh = NULL;
	gMaterial = NULL;
	// Set default value
	gScale = glm::vec3(1.0f, 1.0f, 1.0f);
	gRotation = glm::vec3(0.0f, 0.0f, 0.0f);
}

GameObject::~GameObject()
{
	// Do any clean up here
}

void GameObject::Update( float deltaTs )
{
	// Put any update code here
	// Make sure matrices are up to date (if you don't change them elsewhere, you can put this in the update function)
	//_modelMatrix = glm::rotate(_modelMatrix, _rotation.y, glm::vec3(0,1,0) );
	//_invModelMatrix = glm::rotate(glm::mat4(1.0f), -_rotation.y, glm::vec3(0,1,0) );
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
	gObjectType = type;
}

int GameObject::GetType()
{
	return gObjectType;
}