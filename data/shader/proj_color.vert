#version 330

uniform mat4 matViewProjection;
uniform mat4 matModel;

in vec3 position; // POSITION
in vec4 color;    // COLOR

out vec4 ps_position;
out vec4 ps_color;

void main()
{
    ps_position = matViewProjection * matModel * vec4(position, 1.0);
    ps_color = color;

    gl_Position = ps_position;
}

