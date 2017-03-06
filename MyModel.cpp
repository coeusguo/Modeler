// Our model
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include <FL/gl.h>
#include "MarchingCube.h"
#include "modelerglobals.h"
#include "Metaball.h"
#include <iostream>
#include <vector>
using namespace std;
// To make a SampleModel, we inherit off of ModelerView
class MyModel : public ModelerView
{
private:
	vector<Metaball> metaballs;
public:
	MyModel(int x, int y, int w, int h, char *label)
		: ModelerView(x, y, w, h, label) {
		initMetaball();
	}

	virtual void draw();
	void drawUpperArm();//including elbow
	void drawLowerArm();//including head
	void drawThigh();//including knee
	void drawShank();//including foot
	void drawHead();//including neck
	void drawUpperTorso();//including shoulder
	void drawLowerTorso();//pelvis

	void initMetaball();
	void drawTorus();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char *label)
{
	return new MyModel(x, y, w, h, label);
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void MyModel::draw()
{
	// This call takes care of a lot of the nasty projection 
	// matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
	ModelerView::draw();
	
	// draw the floor

	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_RED);
	glPushMatrix();
	glTranslated(-5, 0, -5);
	drawBox(10, 0.01f, 10);
	glPopMatrix();

	
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GREEN);
	
	
	glPushMatrix();
	//x,y,z sliders
	glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));

	glPushMatrix();
	glTranslated(0, 5.5, 0);

	glPushMatrix();
	glTranslated(0, -1.3, 0);//before apply bow rotation slider
	glPushMatrix();
	glRotated(VAL(BOW), 1.0, 0.0, 0.0);
	glPushMatrix();// translate back
	glTranslated(0, 1.3, 0);
	

	drawUpperTorso();
		//head
		glPushMatrix();
		glTranslated(0, 0.2, 0);
		drawHead();
		glPopMatrix();

		//right arm
		glPushMatrix();
		glTranslated(-1.2, 0, 0);//shoulder width / 2

		glPushMatrix();
		//apply Rotate slider
		glRotated( - VAL(WAVE), 0.0, 0.0, 1.0);
		drawUpperArm();
			glPushMatrix();
			glTranslated(0, -1.0, 0);
			drawLowerArm();
			glPopMatrix();
		glPopMatrix();//pop rotate slider

		glPopMatrix();//pop right arm translate

		//left arm
		glPushMatrix();
		
		glTranslated(1.2, 0, 0);//shoulder width / 2
		glPushMatrix();

		//apply Rotate slider
		glRotated(VAL(WAVE), 0.0, 0.0, 1.0);
		drawUpperArm();
			glPushMatrix();
			glTranslated(0, -1, 0);
			drawLowerArm();
			glPopMatrix();
		glPopMatrix();//pop rotate slider

		glPopMatrix();//pop left arm translate
		
	glPopMatrix();//pop bow rotation related translate: glTranslated(0,-1.3,0)
	glPopMatrix();//pop bow rotation slider
	glPopMatrix();//pop bow rotation related translate: glTranslated(0,1.3,0)
		
		//lower torso
		glPushMatrix();
		glTranslated(0, -1.3, 0);
		drawLowerTorso();

			//left leg
			glPushMatrix();
			glTranslated(0.8, -1.0, 0);
			drawThigh();
				glPushMatrix();
				glTranslated(0, -1.5, 0);
				drawShank();
				glPopMatrix();
			glPopMatrix();

			//right leg
			glPushMatrix();
			glTranslated(-0.8, -1.0, 0);
			drawThigh();
				glPushMatrix();
				glTranslated(0, -1.5, 0);
				drawShank();
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();
		

	glPopMatrix();// pop s,y,z slider translate
	glPopMatrix();// pop first translate
	
	/*
	glPushMatrix();
	glScaled(0.1, 0.1, 0.1);
	glTranslated(0, 50, 0);
	//drawTorus();
	drawMetaball();
	glPopMatrix();
	*/
	//drawTorso();
	//drawThigh();
	//drawShank();
	//drawUpperArm();
	//drawLowerArm();
	//drawHead();
	/*
	
	
	// draw the sample model
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GREEN);
	glPushMatrix();
	glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
	
	glPushMatrix();
	glTranslated(-1.5, 0, -2);
	glScaled(3, 1, 4);
	drawBox(1, 1, 1);
	glPopMatrix();
	/*
	// draw cannon
	glPushMatrix();
	glRotated(VAL(ROTATE), 0.0, 1.0, 0.0);
	glRotated(-90, 1.0, 0.0, 0.0);
	drawCylinder(VAL(HEIGHT), 0.1, 0.1);

	glTranslated(0.0, 0.0, VAL(HEIGHT));
	drawCylinder(1, 1.0, 0.9);

	glTranslated(0.0, 0.0, 0.5);
	glRotated(90, 1.0, 0.0, 0.0);
	drawCylinder(4, 0.1, 0.2);
	glPopMatrix();

	glPopMatrix();
	*/
}

int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
	ModelerControl controls[NUMCONTROLS];
	controls[XPOS] = ModelerControl("X Position", -5, 5, 0.1f, 0);
	controls[YPOS] = ModelerControl("Y Position", 0, 5, 0.1f, 0);
	controls[ZPOS] = ModelerControl("Z Position", -5, 5, 0.1f, 0);
	controls[BOW] = ModelerControl("Bow", 0, 45, 1, 0);
	controls[WAVE] = ModelerControl("Wave", 0, 135, 1, 0);

	ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
	return ModelerApplication::Instance()->Run();
}

void MyModel::drawUpperTorso() {

	glPushMatrix();
				//glTranslated(0, 3, 0);
		//center
		drawSphere(0.2);
		
		//right shoulder
		glPushMatrix();
		glRotated(-90, 0, 1, 0);
		drawCylinder(1.2, 0.1, 0.1);
		glTranslated(0, 0, 1.2);
		drawSphere(0.2);
		glPopMatrix();
		//left shoulder
		glPushMatrix();
		glRotated(90, 0, 1, 0);
		drawCylinder(1.2, 0.1, 0.1);
		glTranslated(0, 0, 1.2);
		drawSphere(0.2);
		glPopMatrix();

		// upper spine
		glPushMatrix();
		glRotated(90, 1, 0, 0);
		drawCylinder(1.3, 0.1, 0.1);
		glPopMatrix();

	glPopMatrix();


}

void MyModel::drawLowerTorso() {
	
	//waist joint
	drawSphere(0.2);

	//lower spine
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(1.0, 0.1, 0.1);


	glPushMatrix();
	glTranslated(0, 0, 1.0);
	drawSphere(0.2);

		glPushMatrix();
		glRotated(90, 0, 1, 0);
		drawCylinder(0.8, 0.1, 0.1);
		glTranslated(0, 0, 0.8);
		drawSphere(0.2);
		glPopMatrix();

		glPushMatrix();
		glRotated(-90, 0, 1, 0);
		drawCylinder(0.8, 0.1, 0.1);
		glTranslated(0, 0, 0.8);
		drawSphere(0.2);
		glPopMatrix();
		glPopMatrix();

	glPopMatrix();
}

void MyModel::drawThigh() {
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(1.5, 0.1, 0.1);
	glTranslated(0, 0, 1.5);
	drawSphere(0.2);
	glPopMatrix();
}

void MyModel::drawShank() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(1.5, 0.1, 0.1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-0.3, -1.5, -0.2);
	drawBox(0.6, 0.2, 0.8);
	glPopMatrix();
	//glPopMatrix();
}

void MyModel::drawUpperArm() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(1, 0.1, 0.1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, -1, 0);
	drawSphere(0.2);
	glPopMatrix();
	//glPopMatrix();
}

void MyModel::drawLowerArm() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(1, 0.1, 0.1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, -1, 0);
	drawSphere(0.25);
	glPopMatrix();
	//glPopMatrix();
}

void MyModel::drawHead() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);

	glPushMatrix();
	glRotated(-90, 1, 0, 0);
	drawCylinder(0.3, 0.1, 0.1);
	glTranslated(-0.4, -0.3, 0.3);
	drawBox(0.8, 0.6, 1.0);
	glPopMatrix();
	//glPopMatrix();
}

void MyModel::drawTorus() {
	grid grid(50);
	
	float c = 7;
	float a = 3;
	for (int i = 0; i < grid.numVertices; i++) {
		float x = grid.vertices[i].position[0];
		float y = grid.vertices[i].position[1];
		float z = grid.vertices[i].position[2];
		
		float s0 = c - sqrt(x * x + y * y);
		if(s0 == 0 && z == 0)
			z == 0.0001;

		grid.vertices[i].value = (a * a) / (s0 * s0 + z * z);
		//grid.vertices[i].value = (a * a) / (x * x + y * y + z * z);
		//cout << grid.vertices[i].value << " ";
		grid.vertices[i].normal[0] = grid.vertices[i].position[0];
		grid.vertices[i].normal[1] = grid.vertices[i].position[1];
		grid.vertices[i].normal[2] = grid.vertices[i].position[2];
		grid.vertices[i].normal.normalize();
		//if(grid.vertices[i].value > 1)
			//cout << grid.vertices[i].value << " ";
	}

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_NORMALIZE);
	//cout << "i`m here" << endl;
	grid.drawSurface(1.0);

	
}

void MyModel::initMetaball() {
	//torso index = 1
	//metaballs.push_back(Metaball())
}
