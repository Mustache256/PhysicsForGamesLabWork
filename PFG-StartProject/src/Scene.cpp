#include "Scene.h"

/*! \brief Brief description.
*  Scene class is a container for loading all the game objects in your simulation or your game.
*
*/
Scene::Scene()
{
	// Set up your scene here......
	// Set a camera
	_camera = new Camera();
	// Don't start simulation yet
	_simulation_start = false;

	// Position of the light, in world-space
	_lightPosition = glm::vec3(10, 10, 0);

	//Temp variable for Exercise 5: Une kinematics equations to compute the physics
	glm::vec3 _v_i = glm::vec3(0.0f, 0.5f, 0.0f);

	// Create a game object
	_physics_object = new KinematicsObject();
	_physics_object2 = new KinematicsObject();
	_physics_object->SetVelocity(_v_i);
	_physics_object2->SetVelocity(_v_i);
	// Create a game level object
	_level = new GameObject();

	// Create the material for the game object- level
	Material *modelMaterial = new Material();
	// Shaders are now in files
	modelMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	modelMaterial->SetDiffuseColour(glm::vec3(0.8, 0.8, 0.8));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	modelMaterial->SetTexture("assets/textures/diffuse.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	modelMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
	_level->SetMaterial(modelMaterial);

	// The mesh is the geometry for the object
	Mesh *groundMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	groundMesh->LoadOBJ("assets/models/woodfloor.obj");
	// Tell the game object to use this mesh
	_level->SetMesh(groundMesh);
	_level->SetPosition(0.0f, 0.0f, 0.0f);
	_level->SetRotation(3.141590f, 0.0f, 0.0f);


	// Create the material for the game object- level
	Material *objectMaterial = new Material();
	// Shaders are now in files
	objectMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	objectMaterial->SetDiffuseColour(glm::vec3(0.8, 0.1, 0.1));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	objectMaterial->SetTexture("assets/textures/default.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	objectMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
	_physics_object->SetMaterial(objectMaterial);
	_physics_object2->SetMaterial(objectMaterial);

	// Set the geometry for the object
	Mesh *modelMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	modelMesh->LoadOBJ("assets/models/sphere.obj");
	// Tell the game object to use this mesh
	_physics_object->SetMesh(modelMesh);
	_physics_object->SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));
	_physics_object->SetScale(glm::vec3(0.3f, 0.3f, 0.3f));

	_physics_object2->SetMesh(modelMesh);
	_physics_object2->SetPosition(glm::vec3(1.0f, 7.0f, 0.0f));
	_physics_object2->SetScale(glm::vec3(0.5f, 0.5f, 0.5f));
	_physics_object2->SetMass(2.0f);
}

Scene::~Scene()
{
	// You should neatly clean everything up here
	delete _physics_object;
	delete _physics_object2;
	delete _level;
	delete _camera;
}

void Scene::Update(float deltaTs, Input* input)
{
	// Update the game object (this is currently hard-coded motion)
	if (input->cmd_x)
	{
		_simulation_start = true;
	}
	if (_simulation_start == true)
	{
		/*glm::vec3 pos = _physics_object->GetPosition();
		//pos += glm::vec3(0.0, -deltaTs, 0.0);
		//pos.y = 0.5 * (-9.8) * deltaTs * deltaTs;
		glm::vec3 vel_temp;

		vel_temp.y = _v_i.y + (-9.8) * deltaTs;
		vel_temp.x = _v_i.x;
		vel_temp.z = _v_i.z;

		pos.y += (_v_i.y + vel_temp.y) / 2.0f * deltaTs;
		pos.x += vel_temp.x * deltaTs;
		pos.z += vel_temp.z * deltaTs;

		_v_i = vel_temp;

		if (pos.y <= 0.3f)
		{
			pos.y = 0.3f;
		}*/

		//_physics_object->SetPosition(pos);

		_physics_object->StartSimulation(_simulation_start);
		_physics_object2->StartSimulation(_simulation_start);
	}
	_physics_object->Update(deltaTs);
	_physics_object2->Update(deltaTs);
	_level->Update(deltaTs);
	_camera->Update(input);

	_viewMatrix = _camera->GetView();
	_projMatrix = _camera->GetProj();
														
}

void Scene::Draw()
{
	// Draw objects, giving the camera's position and projection
	_physics_object->Draw(_viewMatrix, _projMatrix);
	_physics_object2->Draw(_viewMatrix, _projMatrix);
	_level->Draw(_viewMatrix, _projMatrix);

}


