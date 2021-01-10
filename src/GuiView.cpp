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

#include "GuiView.h"
#include "MyWindow.h"
#include "Utilities/3DUtils.h"

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
typedef OpenMesh::PolyMesh_ArrayKernelT<>  PolyMesh;


//************************************************************************
//
// * Constructor to set up the GL window
//========================================================================
GuiView::
GuiView(int x, int y, int w, int h, const char* l)
	: Fl_Gl_Window(x, y, w, h, l)
	//========================================================================
{
	mode(FL_RGB | FL_ALPHA | FL_DOUBLE | FL_STENCIL);

	//Fl::add_idle(IdleCallback, this);
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
int GuiView::handle(int event)
{
	// remember what button was used
	static int last_push;

	switch (event) {
		// Mouse button being pushed event
	case FL_PUSH:
		last_push = Fl::event_button();
		//// if the left button be pushed is left mouse button
		if (last_push == FL_LEFT_MOUSE)
		{
			doAdd(Fl::event_x(), Fl::event_y());
		}
		else if (last_push == FL_RIGHT_MOUSE) {
			doSelect(Fl::event_x(), Fl::event_y());
		}

		damage(1);
		return 1;

		// Mouse button release event
	case FL_RELEASE: // button release
		if (last_push == FL_LEFT_MOUSE) {

		}
		else if (last_push == FL_MIDDLE_MOUSE)
		{
			doSelect(Fl::event_x(), Fl::event_y());
			this->mw->myView->gl_mesh->removeKeyPoint();
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
		return 1;

		// every time the mouse enters this window, aggressively take focus
	case FL_ENTER:
		focus(this);
		break;
	case FL_KEYBOARD:
		int k = Fl::event_key();
		int ks = Fl::event_state();
		if (k == 'd') {
			this->mw->myView->gl_mesh->removeKeyPoint();
			damage(1);
			return 1;
		}
		break;
	}


	return Fl_Gl_Window::handle(event);
}

//************************************************************************
//
// * this is the code that actually draws the window
//   it puts a lot of the work into other routines to simplify things
//========================================================================
void GuiView::draw()
{
	//*********************************************************************
	//
	// * Set up basic opengl informaiton
	//
	//**********************************************************************
	//initialized glad
	if (gladLoadGL())
	{
	}
	else
		throw std::runtime_error("Could not initialize GLAD!");

	// Set up the view port
	glViewport(0, 0, w(), h());

	// clear the window, be sure to clear the Z-Buffer too
	//glClearColor(0.958, 0.909, 0.871, 0);
	glClearColor(0.4, 0.4, 0.4, 0);

	// we need to clear out the stencil buffer since we'll use
	// it for shadows
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// prepare for projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	setProjection();		// put the code to set up matrices here
	this->mw->myView->gl_mesh->renderKeyPoints();


}

void GuiView::doAdd(int mx, int my)
{
	this->mw->myView->gl_mesh->addKeyPoint(GetWorldPos(mx, my));
}

void GuiView::doSelect(int mx, int my)
{
	this->mw->myView->gl_mesh->selectKeyPoint(GetWorldPos(mx, my));
}

void GuiView::doDrag(int mx, int my)
{
	this->mw->myView->gl_mesh->dragKeyPoint(GetWorldPos(mx, my));
}

MyMesh::Point GuiView::GetWorldPos(int mx, int my)
{
	float mouseX = mx / (w() * 0.5f) - 1.0f;
	float mouseY = my / (h() * 0.5f) - 1.0f;

	mouseX = std::max(std::min(mouseX, 0.95f), -0.95f);
	mouseY = std::max(std::min(mouseY, 0.95f), -0.95f);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjection();

	glm::mat4 proj;
	glGetFloatv(GL_PROJECTION_MATRIX, &proj[0][0]);

	glm::mat4 invVP = glm::inverse(proj);
	glm::vec4 screenPos = glm::vec4(mouseX, -mouseY, 1.0f, 1.0f);
	glm::vec4 worldPos = invVP * screenPos;

	return MyMesh::Point(worldPos[0], worldPos[1], 0);
}

//************************************************************************
//
// * This sets up both the Projection and the ModelView matrices
//   HOWEVER: it doesn't clear the projection first (the caller handles
//   that) - its important for picking
//========================================================================
void GuiView::setProjection()
//========================================================================
{
	// Compute the aspect ratio (we'll need it)
	float aspect = static_cast<float>(w()) / static_cast<float>(h());
	float wi, he;
	if (aspect >= 1) {
		wi = 5;
		he = wi / aspect;
	}
	else {
		he = 5;
		wi = he * aspect;
	}
	glMatrixMode(GL_PROJECTION);
	glOrtho(-wi, wi, -he, he, 200, -200);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
