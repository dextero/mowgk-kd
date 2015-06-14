#version 330

uniform vec4 color;

in vec4 ps_color;
out vec4 out_color;

void main()
{
    out_color = ps_color * color;
}

