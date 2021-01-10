/************************************************************************
	 File:        MyView.cpp (From MyView.cpp)

	 Author:
				  Michael Gleicher, gleicher@cs.wisc.edu

	 Modifier
				  Yu-Chi Lai, yu-chi@cs.wisc.edu
				  Maochinn, m10815023@gapps.ntust.edu

	 Comment:
						The MyView is the window that actually shows the
						train. Its a
						GL display canvas (Fl_Gl_Window).  It is held within
						a TrainWindow
						that is the outer window with all the widgets.
						The MyView needs
						to be aware of the window - since it might need to
						check the widgets to see how to draw

	  Note:        we need to have pointers to this, but maybe not know
						about it (beware circular references)

	 Platform:    Visio Studio 2019

*************************************************************************/

#include <iostream>
#include <Fl/fl.h>

// we will need OpenGL, and OpenGL needs windows.h
#include <windows.h>
//#include "GL/gl.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "GL/glu.h"

#include "MyView.h"
#include "MyWindow.h"
#include "Utilities/3DUtils.h"

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
typedef OpenMesh::PolyMesh_ArrayKernelT<>  PolyMesh;


//************************************************************************
//
// * Constructor to set up the GL window
//========================================================================
MyView::
MyView(int x, int y, int w, int h, const char* l)
	: Fl_Gl_Window(x, y, w, h, l)
	//========================================================================
{
	mode(FL_RGB | FL_ALPHA | FL_DOUBLE | FL_STENCIL);

	//Fl::add_idle(IdleCallback, this);
	translation = glm::vec3(0, 0, 0);
	resetArcball();
}

//************************************************************************
//
// * Reset the camera to look at the world
//========================================================================
void MyView::
resetArcball()
//========================================================================
{
	// Set up the camera to look at the world
	// these parameters might seem magical, and they kindof are
	// a little trial and error goes a long way
	arcball.setup(this, 40, 2.50, .2f, .4f, 0);
}

//************************************************************************
//
// * FlTk Event handler for the window
//########################################################################
// TODO: 
//       if you want to make the train respond to other events 
//       (like key presses), you might want to hack this.
//########################################################################
//========================================================================
int MyView::handle(int event)
{
	// see if the ArcBall will handle the event - if it does, 
	// then we're done
	// note: the arcball only gets the event if we're in world view
	if (mw->world_cam->value())
		if (arcball.handle(event)) {
			view_changed();
			return 1;
		}

	if (weight_mode) {
		if (handleWeightMode(event)) {
			return 1;
		}
	}
	// remember what button was used
	static int last_push;

	switch (event) {
		// Mouse button being pushed event
	case FL_PUSH:
		last_push = Fl::event_button();
		//// if the left button be pushed is left mouse button
		//if (last_push == FL_LEFT_MOUSE)
		//	if (!is_picking) {
		//		do_pick = true;
		//		pick_x = Fl::event_x();
		//		pick_y = Fl::event_y();
		//	}
		/*else */
		if (last_push == FL_RIGHT_MOUSE)
			doSelect(Fl::event_x(), Fl::event_y());

		damage(1);
		return 1;

		// Mouse button release event
	case FL_RELEASE: // button release
		if (last_push == FL_LEFT_MOUSE)
			doPick(Fl::event_x(), Fl::event_y());
		else if (last_push == FL_MIDDLE_MOUSE)
		{
			doSelect(Fl::event_x(), Fl::event_y());
			this->gl_mesh->remove_selected();
		}

		damage(1);
		last_push = 0;
		return 1;

		// Mouse button drag event
	case FL_DRAG:
		if (Fl::event_button() == FL_RIGHT_MOUSE) {
			doDrag(Fl::event_x(), Fl::event_y());
			damage(1);
			return 1;
		}
		break;

		// in order to get keyboard events, we need to accept focus
	case FL_FOCUS:
		view_changed();
		return 1;

		// every time the mouse enters this window, aggressively take focus
	case FL_ENTER:
		focus(this);
		break;

	case FL_MOUSEWHEEL:
		top_cam_range += (Fl::event_dy() < 0) ? -0.01f : 0.01f;
		if (top_cam_range <= 0.0f)
			top_cam_range = 0.001f;

		view_changed();

		return 1;
		break;

	case FL_KEYBOARD:
		float speed = 5.0f;
		switch (Fl::event_key()) {
		case FL_Down:
			translation.z -= speed;
			view_changed();
			return 1;
		case FL_Up:
			translation.z += speed;
			view_changed();
			return 1;
		case FL_Left:
			translation.x -= speed;
			view_changed();
			return 1;
		case FL_Right:
			translation.x += speed;
			view_changed();
			return 1;
		}
	}


	return Fl_Gl_Window::handle(event);
}

//************************************************************************
//
// * this is the code that actually draws the window
//   it puts a lot of the work into other routines to simplify things
//========================================================================
void MyView::draw()
{
	//*********************************************************************
	//
	// * Set up basic opengl informaiton
	//
	//**********************************************************************
	//initialized glad
	if (gladLoadGL())
	{
		//initiailize VAO, VBO, Shader...

		std::string common_lib = Shader::readCode("../MeshDeformation/src/shaders/common_lib.glsl");
		std::string material_lib = Shader::readCode("../MeshDeformation/src/shaders/material_lib.glsl");

		if (!this->commom_shader) {
			this->commom_shader = new Shader(
				common_lib + Shader::readCode("../MeshDeformation/src/shaders/simple.vert"),
				std::string(), std::string(), std::string(),
				Shader::readCode("../MeshDeformation/src/shaders/simple.frag"));
		}
		if (!this->commom_matrices) {
			this->commom_matrices = new UBO();
			this->commom_matrices->size = 3 * sizeof(glm::mat4);
			glGenBuffers(1, &this->commom_matrices->ubo);
			glBindBuffer(GL_UNIFORM_BUFFER, this->commom_matrices->ubo);
			glBufferData(GL_UNIFORM_BUFFER, this->commom_matrices->size, NULL, GL_STATIC_DRAW);
			glBindBuffer(GL_UNIFORM_BUFFER, 0);
		}

		if (!this->picking_shader) {
			this->picking_shader = new Shader(
				common_lib + Shader::readCode("../MeshDeformation/src/shaders/picking.vert"),
				std::string(), std::string(), std::string(),
				Shader::readCode("../MeshDeformation/src/shaders/picking.frag"));
			picking_tex.Init(w(), h());
		}

		if (!this->gl_mesh)
		{
			this->gl_mesh = new GLMesh();
			//this->gl_mesh->Init("D:/maochinn/NTUST/Course/DigitalMesh/project/build/output.obj");
		}
	}
	else
		throw std::runtime_error("Could not initialize GLAD!");


	// Set up the view port
	glViewport(0, 0, w(), h());


	// clear the window, be sure to clear the Z-Buffer too
	glClearColor(0.958, 0.909, 0.871, 0);		// background should be blue

	// we need to clear out the stencil buffer since we'll use
	// it for shadows
	glClearStencil(0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glEnable(GL_DEPTH);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// prepare for projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjection();		// put the code to set up matrices here
	//

	setUBO();
	glBindBufferRange(GL_UNIFORM_BUFFER, /*binding point*/0, this->commom_matrices->ubo, 0, this->commom_matrices->size);

	// model position (0,0,0)
	glm::mat4 model_matrix = glm::mat4();

	//bind shader
	this->commom_shader->Use();

	this->gl_mesh->checkUpdate();
	
	glUniformMatrix4fv(glGetUniformLocation(this->commom_shader->Program, "u_model"), 1, GL_FALSE, &model_matrix[0][0]);
	//glUniform3fv(glGetUniformLocation(this->commom_shader->Program, "u_color"), 1, &glm::vec3(.941f, .25f, .25f)[0]);
	glUniform3fv(glGetUniformLocation(this->commom_shader->Program, "u_color"), 1, &glm::vec3(1, 1, 1)[0]);

	this->gl_mesh->renderMesh();
	
	glDisable(GL_DEPTH_TEST);
	glUseProgram(0);

	//if (weight_mode) {
		//glUniform3fv(glGetUniformLocation(this->commom_shader->Program, "u_color"), 1, &glm::vec3(.25f, .941f, .25f)[0]);
		glColor4f(1.0f, 0, 0, 0.4f);
		this->gl_mesh->renderSelectedMesh();

		//glUniform3fv(glGetUniformLocation(this->commom_shader->Program, "u_color"), 1, &glm::vec3(.26f, .181f, .172f)[0]);
		glColor4f(.26f, .181f, .172f, 0.6f);
		glLineWidth(1.38f);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		this->gl_mesh->renderMesh();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//}
	glDisable(GL_BLEND);

	//unbind shader(switch to fixed pipeline)
	this->gl_mesh->renderControlPoints();
}

void MyView::resize(int x, int y, int w, int h)
{
	Fl_Gl_Window::resize(x, y, w, h);
	this->picking_tex.Resize(w, h);
}

void MyView::doPick(int mx, int my)
{
	UpdatePickTextrue();
	GLuint PrimID = this->picking_tex.ReadPixel(mx, h() - my - 1) - 1;
	if (this->gl_mesh->validID(PrimID)) {
		//std::cout << PrimID << std::endl;
		this->gl_mesh->select(PrimID, getWorldPos(mx, my));
	}
	else {
		std::cout << "Nope" << std::endl;
	}
}

void MyView::doPickWeightTri(int mx, int my, bool state)
{
	UpdatePickTextrue();
	GLuint PrimID = this->picking_tex.ReadPixel(mx, h() - my - 1) - 1;
	if (this->gl_mesh->validID(PrimID)) {
		//std::cout << PrimID << std::endl;
		this->gl_mesh->selectTri(PrimID, state);
	}
}

void MyView::doSelect(int mx, int my)
{
	this->gl_mesh->selectControlPoint(getWorldPos(mx, my));
}

void MyView::doDrag(int mx, int my)
{
	this->gl_mesh->dragControlPoint(getWorldPos(mx, my));
	view_changed();
}

MyMesh::Point MyView::getWorldPos(int mx, int my)
{
	float mouseX = mx / (w() * 0.5f) - 1.0f;
	float mouseY = my / (h() * 0.5f) - 1.0f;

	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjection();
	glm::mat4 proj, view;
	glGetFloatv(GL_MODELVIEW_MATRIX, &view[0][0]);
	glGetFloatv(GL_PROJECTION_MATRIX, &proj[0][0]);
	glPopMatrix();

	glm::mat4 invVP = glm::inverse(proj * view);
	glm::vec4 screenPos = glm::vec4(mouseX, -mouseY, 1.0f, 1.0f);
	glm::vec4 worldPos = invVP * screenPos;

	//std::cout << worldPos.x << " " << worldPos.y << " " << worldPos.z << " " << worldPos.w << std::endl;

	return MyMesh::Point(worldPos.x, 0, worldPos.z);
}

//************************************************************************
//
// * This sets up both the Projection and the ModelView matrices
//   HOWEVER: it doesn't clear the projection first (the caller handles
//   that) - its important for picking
//========================================================================
void MyView::
setProjection()
//========================================================================
{
	// Compute the aspect ratio (we'll need it)
	float aspect = static_cast<float>(w()) / static_cast<float>(h());

	// Check whether we use the world camp
	if (mw->world_cam->value())
		arcball.setProjection(false);
	// Or we use the top cam
	else if (mw->top_cam->value()) {
		float wi, he;
		if (aspect >= 1) {
			wi = w() * top_cam_range;
			he = wi / aspect;
		}
		else {
			he = h() * top_cam_range;
			wi = he * aspect;
		}

		// Set up the top camera drop mode to be orthogonal and set
		// up proper projection matrix
		glMatrixMode(GL_PROJECTION);
		glOrtho(-wi, wi, -he, he, 200, -200);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		// Use the transformation in the ArcBall
		glRotatef(-90, 1, 0, 0);
		glTranslatef(-translation.x, -translation.y, -translation.z);
	}
}

void MyView::setUBO()
{
	float wdt = this->pixel_w();
	float hgt = this->pixel_h();

	glm::mat4 view_matrix;
	glGetFloatv(GL_MODELVIEW_MATRIX, &view_matrix[0][0]);
	//HMatrix view_matrix; 
	//this->arcball.getMatrix(view_matrix);

	glm::mat4 projection_matrix;
	glGetFloatv(GL_PROJECTION_MATRIX, &projection_matrix[0][0]);
	//projection_matrix = glm::perspective(glm::radians(this->arcball.getFoV()), (GLfloat)wdt / (GLfloat)hgt, 0.01f, 1000.0f);


	glBindBuffer(GL_UNIFORM_BUFFER, this->commom_matrices->ubo);
	glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), &projection_matrix[0][0]);
	glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), &view_matrix[0][0]);
	glBufferSubData(GL_UNIFORM_BUFFER, 2 * sizeof(glm::mat4), sizeof(glm::mat4), &glm::inverse(view_matrix)[0][0]);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void MyView::UpdatePickTextrue()
{
	if (tex_is_outdated)
	{
		// bind picking
		this->picking_shader->Use();
		this->picking_tex.EnableWriting();
		glViewport(0, 0, w(), h());
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// prepare for projection
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		setProjection();		// put the code to set up matrices here

		//
		setUBO();
		glBindBufferRange(GL_UNIFORM_BUFFER, /*binding point*/0, this->commom_matrices->ubo, 0, this->commom_matrices->size);

		// model position (0,0,0)
		glm::mat4 model_matrix = glm::mat4();
		glUniformMatrix4fv(glGetUniformLocation(this->picking_shader->Program, "u_model"), 1, GL_FALSE, &model_matrix[0][0]);

		this->gl_mesh->renderMesh();

		this->picking_tex.DisableWriting();

		glUseProgram(0);
	}
	tex_is_outdated = false;
}

void MyView::setWeightMode(bool mode)
{
	if (!mode) {
		gl_mesh->applyTriangleWeights();
	}
	weight_mode = mode;
}

void MyView::view_changed()
{
	//std::cout << "CHANGED\n";
	tex_is_outdated = true;
}

bool MyView::handleWeightMode(int e)
{
	switch (e) {
	case FL_DRAG:
		if (Fl::event_button() == FL_LEFT_MOUSE) {
			doPickWeightTri(Fl::event_x(), Fl::event_y(), true);
			damage(1);
			return true;
		}
		else if (Fl::event_button() == FL_RIGHT_MOUSE) {
			doPickWeightTri(Fl::event_x(), Fl::event_y(), false);
			damage(1);
			return true;
		}
		break;
	}

	return true;
}
