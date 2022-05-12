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
	camera = new Camera();
	// Don't start simulation yet
	simulationStart = false;

	// Position of the light, in world-space
	lightPosition = glm::vec3(10, 10, 0);

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
	gameObjectMaterial->SetLightPosition(lightPosition);
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
	dynamObjectMaterial->SetLightPosition(lightPosition);
	// Tell the level object to use this material

	// Set the geometry for the object
	Mesh *dynamObjectMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	dynamObjectMesh->LoadOBJ("assets/models/sphere.obj");

	fileRead("objFile1.txt");

	//For loops used to create all the dynamic objects that appear in the scene
	for (int k = 0; k < 6; k += 2)
	{
		for (int j = 0; j < 6; j += 2)
		{
			for (int i = 0; i < 6; i += 2)
			{
				DynamicObject* newObj = CreateSphere(dynamObjectMaterial, dynamObjectMesh, glm::vec3(std::stof(fileInput.at(0)) + i, std::stof(fileInput.at(1)) + j, std::stof(fileInput.at(2)) + k), glm::vec3(std::stof(fileInput.at(3)), std::stof(fileInput.at(4)), std::stof(fileInput.at(5))), std::stof(fileInput.at(6)), std::stof(fileInput.at(7)));

				sceneDynamicObjects.push_back(newObj);
			}
		}
	}

	//Creating the plane GameObejct
	GameObject* newGameObj = CreatePlane(gameObjectMaterial, gameObjectMesh, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(4.0f, 1.0f, 4.0f));

	sceneGameObjects.push_back(newGameObj);
}

Scene::~Scene()
{
	//For loops used to delete all the Gameobjects and DynamicObjects within the scene
	for (int i = 0; i < sceneDynamicObjects.size(); i++)
	{
		delete sceneDynamicObjects.at(i);
	}
	for (size_t i = 0; i < sceneGameObjects.size(); i++)
	{
		delete sceneGameObjects.at(i);
	}
	//Deleting the camera object
	delete camera;
}

void Scene::Update(float deltaTs, Input* input)
{
	// Update the game object (this is currently hard-coded motion)
	if (input->cmd_x)
	{
		simulationStart = true;
	}
	if (simulationStart == true)
	{
		//Start the simulation for all the DynamicObjects 
		for (int i = 0; i < sceneDynamicObjects.size(); i++)
		{
			sceneDynamicObjects.at(i)->StartSimulation(simulationStart);
		}
	}
	//For loops that update all the objects within the scene, the loops exist to compare collision between every object within the scene
	for (int i = 0; i < sceneDynamicObjects.size(); i++)
	{
		for (int k = 0; k < sceneGameObjects.size(); k++)
		{
			sceneDynamicObjects.at(i)->Update(sceneGameObjects.at(k), deltaTs / (sceneGameObjects.size() * 6)); //Division used here as a hacky solution to the issue of each object updating multiple times per tick
		}
		for (size_t k = 0; k < sceneDynamicObjects.size(); k++)
		{
			if (k == i)
			{
				continue;
			}
			else
			{
				sceneDynamicObjects.at(i)->Update(sceneDynamicObjects.at(k), deltaTs / (sceneDynamicObjects.size() * 2));//Division used here as a hacky solution to the issue of each object updating multiple times per tick
			}
		}
	}
	for (int j = 0; j < sceneGameObjects.size(); j++)
	{
		sceneGameObjects.at(j)->Update(deltaTs);
	}
	//Updating the camera
	camera->Update(input);

	//Updating the camera's view and projection matrices
	viewMatrix = camera->GetView();
	projMatrix = camera->GetProj();												
}

void Scene::Draw()
{
	// Draw objects, giving the camera's position and projection

	for (int i = 0; i < sceneDynamicObjects.size(); i++)
	{
		sceneDynamicObjects.at(i)->Draw(viewMatrix, projMatrix);
	}

	for (int j = 0; j < sceneGameObjects.size(); j++)
	{
		sceneGameObjects.at(j)->Draw(viewMatrix, projMatrix);
	}
}

DynamicObject* Scene::CreateSphere(Material* mat, Mesh* modelMesh, glm::vec3 position, glm::vec3 scale, float mass, float boundingRadius)
{
	//Create a new DynamicObject and set all its default values to the variables that are being passed into the function
	DynamicObject* dObject = new DynamicObject();
	dObject->SetMaterial(mat);
	dObject->SetMesh(modelMesh);
	dObject->SetPosition(position);
	dObject->SetScale(scale);
	dObject->SetMass(mass);
	dObject->SetBoundingRadius(boundingRadius);
	dObject->SetType(1);
	//Return the newly created object
	return dObject;
}

GameObject* Scene::CreatePlane(Material* mat, Mesh* modelMesh, glm::vec3 position, glm::vec3 rotation, glm::vec3 scale)
{
	//Create a new GameObject and set all its default values to the variables that are being passed into the function
	GameObject* gObject = new GameObject();
	gObject->SetMaterial(mat);
	gObject->SetMesh(modelMesh);
	gObject->SetPosition(position.x, position.y, position.z);
	gObject->SetRotation(rotation.x, rotation.y, rotation.z);
	gObject->SetScale(scale.x, scale.y, scale.z);
	gObject->SetType(0);
	//Return the newly created object
	return gObject;
}

void Scene::fileRead(std::string fileName)
{
	//Defining loacal variables
	std::string line;
	std::string fileString;
	std::ifstream fileRead(fileName);
	//Read from the file ine by line and push the readings into the file input vector (only do this whilst the file is open)
	if (fileRead.is_open())
	{
		while (getline(fileRead, line))
		{
			std::cout << line << std::endl;
			fileString += line + "\n";

			fileInput.push_back(line);
		}
	}
	//Error catching
	else
	{
		std::cout << "File unable to open" << std::endl;
	}
	//Close the file
	fileRead.close();
}


