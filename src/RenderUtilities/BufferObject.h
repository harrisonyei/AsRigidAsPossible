#pragma once
#include <glad\glad.h>

#define MAX_FBO_TEXTURE_AMOUNT 4
#define MAX_VAO_VBO_AMOUNT 3

struct VAO
{
	GLuint vao = 0;
	GLuint vbo[MAX_VAO_VBO_AMOUNT] = { 0 };
	GLuint ebo = 0;
	union
	{
		unsigned int element_amount = 0;//for draw element
		unsigned int count;			//for draw array
	};
};
struct UBO
{
	GLuint ubo = 0;
	GLsizeiptr size = 0;
};
struct FBO
{
	GLuint fbo = 0;	//frame buffer
	GLuint textures[MAX_FBO_TEXTURE_AMOUNT] = {0};	//attach to color buffer
	GLuint rbo = 0;	//attach to depth and stencil
};