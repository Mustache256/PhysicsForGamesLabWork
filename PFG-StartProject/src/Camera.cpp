#include "Camera.h"

/*! \brief Brief description.
*  Camera class sets up parameters for camera. It controls camera's movement based on keyboard and mourse information.
*
*/
Camera::Camera()
{
	_cameraPos = glm::vec3(0.0f, 3.0f, 0.0f); //glm::vec3(4.0f, 3.0f, 3.0f);
	glm::vec3 initTarget = glm::vec3(-100.0, 0.0f, 0.0f);

	_cameraAngleX = 90.0f;
	_cameraAngleY = 0.0f;

	speed = 0.1f;
	mouseSpeed = 0.005f;

	oldMousePosition = glm::vec2(0.0, 0.0);

	_up = glm::vec3(0, 1, 0); 

	//Build  projection and viewing matrices
	_projMatrix = glm::perspective(45.0f, 4.0f/3.0f, 0.1f, 100.0f);
	_viewMatrix = glm::lookAt(_cameraPos, initTarget, _up);
}

Camera::~Camera() {}

void Camera::Update(Input* input)
{
	//int Mid_X = 0;
	//int Mid_Y = 0;
	// move the mouse back to the middle of the screen each frame
	//SDL_WarpMouseInWindow(NULL, 400, 300);
	//oldMousePosition.x = 400;
	//oldMousePosition.y = 300;

	// get the current position of the cursor
	//SDL_GetMouseState(&posx, &posy);
	//posx = input->mouse_x;
	//posy = input->mouse_y;

	//float diff_x = posx - oldMousePosition.x;
	//float diff_y = posy - oldMousePosition.y;

	//std::cout << "C: " << posx << " " << posy << std::endl;
	//std::cout << "O: " << oldMousePosition.x << " " << oldMousePosition.y << std::endl;
	//std::cout << "D: " << diff_x << " " << diff_y << std::endl;

	//std::cout << input->getMouseDelta().x << std::endl;

	_cameraAngleX -= input->getMouseDelta().x * mouseSpeed;
	_cameraAngleY -= input->getMouseDelta().y * mouseSpeed;

	//_cameraAngleX -= mouseSpeed * diff_x;
	//_cameraAngleY -= mouseSpeed * diff_y;
	oldMousePosition = glm::vec2(posx, posy);

	glm::vec3 direction(cos(_cameraAngleY) * sin(_cameraAngleX), sin(_cameraAngleY), cos(_cameraAngleY) * cos(_cameraAngleX));
	glm::vec3 right = glm::vec3(sin(_cameraAngleX - 3.14f / 2.0f), 0, cos(_cameraAngleX - 3.14f / 2.0f));
	glm::vec3 up = glm::cross(right, direction);

	// Move camera in a direction
	if (input->cmd_w)
	{
		_translation = direction * speed;
		_cameraPos += _translation;

	}
	else if (input->cmd_s)
	{
		_translation = -(direction * speed);
		_cameraPos += _translation;
	}
	if (input->cmd_a)
	{
		_translation = -(right * speed);
		_cameraPos += _translation;

	}
	else if (input->cmd_d)
	{
		_translation = right * speed;
		_cameraPos += _translation;
	}

	// the matrices used to see the models are changed.
	//_projMatrix = glm::perspective(45.0f, 800.0f / 600.0f, 0.1f, 10000.0f);
	_viewMatrix = glm::lookAt(_cameraPos, _cameraPos + direction, up);

}