#version 330

uniform mat4 matViewProjection;
uniform mat4 matModel;

in vec3 position; // POSITION

out vec4 ps_position;

void main()
{
    ps_position = matViewProjection * matModel * vec4(position, 1.0);
    gl_Position = ps_position;
}

