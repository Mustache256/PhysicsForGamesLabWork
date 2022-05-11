#include "Scene.h"

#include <fstream>
#include <string>

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

	// Create the material for the game object- level
	Material *gameObjectMaterial = new Material();
	// Shaders are now in files
	gameObjectMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	gameObjectMaterial->SetDiffuseColour(glm::vec3(0.8, 0.8, 0.8));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	gameObjectMaterial->SetTexture("assets/textures/diffuse.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	gameObjectMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material

	// The mesh is the geometry for the object
	Mesh *gameObjectMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	gameObjectMesh->LoadOBJ("assets/models/woodfloor.obj");

	// Create the material for the game object- level
	Material *dynamObjectMaterial = new Material();
	// Shaders are now in files
	dynamObjectMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	dynamObjectMaterial->SetDiffuseColour(glm::vec3(0.8, 0.1, 0.1));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	dynamObjectMaterial->SetTexture("assets/textures/default.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	dynamObjectMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material

	// Set the geometry for the object
	Mesh *dynamObjectMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	dynamObjectMesh->LoadOBJ("assets/models/sphere.obj");

	fileRead("objFile1.txt");

	//for (int j = 0; j < 3; j++)
	//{
		for (int i = 0; i < 3; i++)
		{
			DynamicObject* newObj = CreateSphere(dynamObjectMaterial, dynamObjectMesh, glm::vec3(std::stof(_fileInput.at(0)) + i, std::stof(_fileInput.at(1)), std::stof(_fileInput.at(2))), glm::vec3(std::stof(_fileInput.at(3)), std::stof(_fileInput.at(4)), std::stof(_fileInput.at(5))), std::stof(_fileInput.at(6)), std::stof(_fileInput.at(7)));

			_sceneDynamicObjects.push_back(newObj);
		}
	//}
	/*for (int j = 0; j < 2; j++)
	{
		for (int i = 0; i < 2; i++)
		{
			DynamicObject* newDynamObj = CreateSphere(dynamObjectMaterial, dynamObjectMesh, glm::vec3(0.2f + i, 25.0f, 0.0f + j), glm::vec3(0.3f, 0.3f, 0.3f), 2.0f, 0.3f);

			_sceneDynamicObjects.push_back(newDynamObj);
		}
	}*/

	GameObject* newGameObj = CreatePlane(gameObjectMaterial, gameObjectMesh, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(4.0f, 1.0f, 4.0f));

	_sceneGameObjects.push_back(newGameObj);

	DynamicObject* newDynamObj = CreateSphere(dynamObjectMaterial, dynamObjectMesh, glm::vec3(0.2f, 25.0f, 0.0f), glm::vec3(0.3f, 0.3f, 0.3f), 2.0f, 0.3f);

	_sceneDynamicObjects.push_back(newDynamObj);
}

Scene::~Scene()
{
	// You should neatly clean everything up here
	for (int i = 0; i < _sceneDynamicObjects.size(); i++)
	{
		delete _sceneDynamicObjects.at(i);
	}
	for (size_t i = 0; i < _sceneGameObjects.size(); i++)
	{
		delete _sceneGameObjects.at(i);
	}
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
		for (int i = 0; i < _sceneDynamicObjects.size(); i++)
		{
			_sceneDynamicObjects.at(i)->StartSimulation(_simulation_start);
		}
	}
	for (int i = 0; i < _sceneDynamicObjects.size(); i++)
	{
		for (size_t j = 0; j < _sceneGameObjects.size(); j++)
		{
		  	//_sceneDynamicObjects.at(i)->Update(_sceneGameObjects.at(j), deltaTs / 6);
			_sceneDynamicObjects.at(i)->Update(_sceneGameObjects.at(j), deltaTs / 6);
		}
		for (size_t k = 0; k < _sceneDynamicObjects.size(); k++)
		{
			if (k == i)
			{
				continue;
			}
			else
			{
				//_sceneDynamicObjects.at(i)->Update(_sceneDynamicObjects.at(k), deltaTs / 9);
				_sceneDynamicObjects.at(i)->Update(_sceneDynamicObjects.at(k), deltaTs / 9);
			}
		}
	}
	for (int j = 0; j < _sceneGameObjects.size(); j++)
	{
		_sceneGameObjects.at(j)->Update(deltaTs);
	}
	_camera->Update(input);

	_viewMatrix = _camera->GetView();
	_projMatrix = _camera->GetProj();												
}

void Scene::Draw()
{
	// Draw objects, giving the camera's position and projection

	for (int i = 0; i < _sceneDynamicObjects.size(); i++)
	{
		_sceneDynamicObjects.at(i)->Draw(_viewMatrix, _projMatrix);
	}

	for (int j = 0; j < _sceneGameObjects.size(); j++)
	{
		_sceneGameObjects.at(j)->Draw(_viewMatrix, _projMatrix);
	}
}

DynamicObject* Scene::CreateSphere(Material* mat, Mesh* modelMesh, glm::vec3 position, glm::vec3 scale, float mass, float boundingRadius)
{
	DynamicObject* dObject = new DynamicObject();
	dObject->SetMaterial(mat);
	dObject->SetMesh(modelMesh);
	dObject->SetPosition(position);
	dObject->SetScale(scale);
	dObject->SetMass(mass);
	dObject->SetBoundingRadius(boundingRadius);
	dObject->SetType(1);

	return dObject;
}

GameObject* Scene::CreatePlane(Material* mat, Mesh* modelMesh, glm::vec3 position, glm::vec3 rotation, glm::vec3 scale)
{
	GameObject* gObject = new GameObject();
	gObject->SetMaterial(mat);
	gObject->SetMesh(modelMesh);
	gObject->SetPosition(position.x, position.y, position.z);
	gObject->SetRotation(rotation.x, rotation.y, rotation.z);
	gObject->SetScale(scale.x, scale.y, scale.z);
	gObject->SetType(0);

	return gObject;
}

void Scene::fileRead(std::string fileName)
{
	std::string line;
	std::string fileString;
	std::ifstream fileRead(fileName);
	if (fileRead.is_open())
	{
		while (getline(fileRead, line))
		{
			std::cout << line << std::endl;
			fileString += line + "\n";

			_fileInput.push_back(line);
		}
	}
	else
	{
		std::cout << "File unable to open" << std::endl;
	}
	fileRead.close();
}


