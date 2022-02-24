#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "KinematicsObject.h"

KinematicsObject::KinematicsObject()
{
	_scale = glm::vec3(1.0f, 1.0f, 1.0f);
	_start = false;
}

KinematicsObject::~KinematicsObject()
{

}

void KinematicsObject::Update(float deltaTs)
{
	if (_start == true)
	{
		glm::vec3 v;
		v.x = _velocity.x;
		v.z = _velocity.z;
		v.y = _velocity.y + (-9.8) * deltaTs;
		_position.x += _velocity.x * deltaTs;
		_position.z += _velocity.z * deltaTs;
		_position.y += (_velocity.y + v.y) * 0.5f * deltaTs;
		_velocity = v;
		
		if (_position.y <= 0.3f)
		{
			_position.y = 0.3f;
		}
	}

	UpdateModelMatrix();
}

void KinematicsObject::UpdateModelMatrix()
{
	_modelMatrix = glm::translate(glm::mat4(1), _position);
	_modelMatrix = glm::scale(_modelMatrix, _scale);
	_invModelMatrix = glm::inverse(_modelMatrix);
}