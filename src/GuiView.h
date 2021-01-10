#pragma once

#include "MyView.h"

class GuiView : public Fl_Gl_Window
{
public:
	// note that we keep the "standard widget" constructor arguments
	GuiView(int x, int y, int w, int h, const char* l = 0);

	// overrides of important window things
	virtual int handle(int);
	virtual void draw();

	void doAdd(int, int);
	void doSelect(int, int);
	void doDrag(int, int);
	MyMesh::Point GetWorldPos(int, int);

	// setup the projection - assuming that the projection stack has been
	// cleared for you
	void setProjection();
public:
	MyWindow* mw;				// The parent of this display window
};
