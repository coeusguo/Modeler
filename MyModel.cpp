// Our model
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include <FL/gl.h>
#include "MarchingCube.h"
#include "InverseKinematics.h"
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
	InverseKinematics2* rightArm;
	InverseKinematics2* rightLeg;
public:
	MyModel(int x, int y, int w, int h, char *label)
		: ModelerView(x, y, w, h, label) {
		initMetaball();
		Vec3f rightPoint1(-1.2, 5.4, 0);
		Vec3f rightPoint2(-0.8, 3.2, 0);
		rightArm = new InverseKinematics2(rightPoint1, 1.0, 1.0);
		rightLeg = new InverseKinematics2(rightPoint2, 1.5, 1.7);
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

	void demo();
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
	Vec4f result1(0, 0, 0, 0);
	Vec4f result2(0, 0, 0, 0);
	ModelerView::draw();

	if (VAL(ENABLEIKARM)) {
		Vec3f destination(VAL(IKXARM), VAL(IKYARM), VAL(IKZARM));
		//cout << VAL(IKX) <<","<< VAL(IKY) << ","<< VAL(IKZ)<<endl;
		result1 = rightArm->getResult(destination);
		glPushMatrix();
		setDiffuseColor(COLOR_RED);
		glTranslated(VAL(IKXARM), VAL(IKYARM), VAL(IKZARM));
		drawBox(0.2, 0.2, 0.2);
		glPopMatrix();
	}

	if (VAL(ENABLEIKLEG)) {
		Vec3f destination(VAL(IKXLEG), VAL(IKYLEG), VAL(IKZLEG));
		//cout << VAL(IKX) <<","<< VAL(IKY) << ","<< VAL(IKZ)<<endl;
		result2 = rightLeg->getResult(destination);
		glPushMatrix();
		setDiffuseColor(COLOR_BLUE);
		glTranslated(VAL(IKXLEG), VAL(IKYLEG), VAL(IKZLEG));
		drawBox(0.2, 0.2, 0.2);
		rightLeg->setConstraint((bool)VAL(LEGCONSTRAINT));
		//rightLeg->setConstraint1(VAL(LEGCONSTRAINT1));
		//rightLeg->setConstraint2(VAL(LEGCONSTRAINT2));
		
		glPopMatrix();
	}
	// draw the floor
	/**/
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_RED);

	glPushMatrix();
	glTranslated(-5, 0, -5);
	drawBox(10, 0.01f, 10);
	glPopMatrix();


	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GREEN);

	glPushMatrix();
	glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
	//upper torso
	glPushMatrix();
	glTranslated(0, 4.2, 0);

	glPushMatrix();
	glRotated(VAL(BOW), 1, 0, 0);

	glPushMatrix();
	glTranslated(0, 1.2, 0);
	drawUpperTorso();

	//head
	drawHead();

	//right arm and ik part
	glPushMatrix();
	glTranslated(-1.2, 0, 0);
	if (VAL(ENABLEIKARM)) {
		glPushMatrix();
		glRotated(result1[1], 0, 1, 0);
		glPushMatrix();
		glRotated(result1[0], 1, 0, 0);
		drawUpperArm();
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();


		glPushMatrix();
		glTranslated(rightArm->joint[0], rightArm->joint[1] - 5.4, rightArm->joint[2]);
		glPushMatrix();
		glRotated(result1[3], 0, 1, 0);
		glPushMatrix();
		glRotated(result1[0], 1, 0, 0);
		drawLowerArm();
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}
	else {
		glPushMatrix();
		glRotated(-VAL(WAVE), 0, 0, 1);
		drawUpperArm();
		glPushMatrix();
		glTranslated(0, -1.0, 0);
		drawLowerArm();
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}
	//left arm,no ik
	glPushMatrix();
	glTranslated(1.2, 0, 0);
	glPushMatrix();
	glRotated(VAL(WAVE), 0, 0, 1);
	drawUpperArm();
	glPushMatrix();
	glTranslated(0, -1.0, 0);
	drawLowerArm();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();

	glPopMatrix();//upper torso related
	glPopMatrix();
	glPopMatrix();

	//lower torso
	glPushMatrix();
	glTranslated(0, 4.2, 0);// translate m1
	drawLowerTorso();

	//right leg and ik part
	glPushMatrix();
	glTranslated(-0.8, -1.0, 0);
	glPushMatrix();
	glRotated(result2[1], 0, 1, 0);
	glPushMatrix();
	glRotated(result2[0], 1, 0, 0);
	drawThigh();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();//pop translate m1

	glPushMatrix();
	glTranslated(rightLeg->joint[0], rightLeg->joint[1], rightLeg->joint[2]);
	glPushMatrix();
	glRotated(result2[3], 0, 1, 0);
	glPushMatrix();
	glRotated(result2[2], 1, 0, 0);
	drawShank();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();


	//left leg no ik
	glPushMatrix();
	glTranslated(0.8, 3.2, 0);
	drawThigh();
	glPushMatrix();
	glTranslated(0, -1.5, 0);
	drawShank();
	glPopMatrix();
	glPopMatrix();

	glPopMatrix();

	/*
	glPushMatrix();//m1
		//x,y,z sliders
		glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));

		glPushMatrix();//m2
			glTranslated(0, 5.5, 0);

			glPushMatrix();//m3
				glTranslated(0, -1.3, 0);//before apply bow rotation slider
				
				glPushMatrix();//m4
					glRotated(VAL(BOW), 1.0, 0.0, 0.0);
					
					glPushMatrix();//m5 translate back
						glTranslated(0, 1.3, 0);
						drawUpperTorso();
						
						//head
						glPushMatrix();//m6
							glTranslated(0, 0.2, 0);
							drawHead();
						glPopMatrix();

						//right arm
						glPushMatrix();
							glTranslated(-1.2, 0, 0);//shoulder width / 2

							glPushMatrix();
								//apply Rotate slider
								glRotated( - VAL(WAVE), 0.0, 0.0, 1.0);
								//ik part
								glPushMatrix();
								glRotated(result[1], 0, 1, 0);
								glPushMatrix();
								glRotated(result[0], 1, 0, 0);
								drawUpperArm();
								glPopMatrix();
								glPopMatrix();

								glPushMatrix();
								glTranslated(rightArm->joint[0], rightArm->joint[1] - 4.2, rightArm->joint[2]);
								glPushMatrix();
								glRotated(result[3], 0, 1, 0);
								glPushMatrix();
								glRotated(result[2], 1, 0, 0);
								drawLowerArm();
								glPopMatrix();
								glPopMatrix();
								glPopMatrix();

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
		
					glPopMatrix();//m5 pop bow rotation related translate: glTranslated(0,-1.3,0)
				glPopMatrix();//pop bow rotation slider
			glPopMatrix();//m3 pop bow rotation related translate: glTranslated(0,1.3,0)
		
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
	for (int i = 0; i < 4; i++)
		cout << result[i] << ",";
	cout << endl;
	*/
	/*
	glPushMatrix();
	glScaled(0.1, 0.1, 0.1);
	glTranslated(0, 30, 0);
	//drawTorus();
	//drawMetaball();
	initMetaball();
	glPopMatrix();
	*/
	/*
cout << result[0] << "," << result[1] << "," << result[2] << "," << result[3] << endl;
glPushMatrix();
glTranslated(0, 3.2, 0);

glPushMatrix();
glRotated(result[1], 0, 1, 0);

glPushMatrix();
glRotated(result[0],1, 0, 0);
drawThigh();

glPopMatrix();
glPopMatrix();
glPopMatrix();

glPushMatrix();
glTranslated(rightLeg->joint[0], rightLeg->joint[1], rightLeg->joint[2]);


glPushMatrix();
glRotated(result[3],0,1, 0);

glPushMatrix();
glRotated(result[2],1,0,0);
drawShank();

glPopMatrix();
glPopMatrix();
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
	controls[IKXARM] = ModelerControl("IK x arm", -3.2, 0.8, 0.1, -1.2);
	controls[IKYARM] = ModelerControl("IK y arm", 3.4, 7.4, 0.1, 3.4);
	controls[IKZARM] = ModelerControl("IK z arm", -2.0, 2.0, 0.1, 0);
	controls[ENABLEIKARM] = ModelerControl("Enable Arm IK", 0, 1, 1, 0);
	controls[IKXLEG] = ModelerControl("IK x leg", -4.0, 2.4, 0.1, -0.8);
	controls[IKYLEG] = ModelerControl("IK y leg", 0, 2.5, 0.1, 0);
	controls[IKZLEG] = ModelerControl("IK z leg", -3.2, 2.8, 0.1, 0);
	controls[ENABLEIKLEG] = ModelerControl("Enable Leg IK", 0, 1, 1, 0);
	controls[LEGCONSTRAINT] = ModelerControl("Leg IK Constraint", 0, 1, 1, 0);
	//controls[LEGCONSTRAINT1] = ModelerControl("Constraint angle1", 45, 135, 1, 135);
	//controls[LEGCONSTRAINT2] = ModelerControl("Constraint angle", 30, 180, 1, 30);
	
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
	

	glPushMatrix();
	glTranslated(0, 0, 1.5);
	
	drawCylinder(0.2, 0.4, 0.4);
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();

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
	ball ball1(0, 0, 0, 4);
	ball ball2(5, 5, 5, 3);
	Metaball balls(40);
	balls.addBalls(ball1);
	balls.addBalls(ball2);
	balls.drawMetaball();
}

void MyModel::demo() {
	IKmat mat(4,4);
	float m = 1.0;


	mat.setEntry(0, 0, 1); mat.setEntry(0, 1, 0); mat.setEntry(0, 2, 0); mat.setEntry(0, 3, 1);
	mat.setEntry(1, 0, 0); mat.setEntry(1, 1, 1); mat.setEntry(1, 2, 1); mat.setEntry(1, 3, 1);
	mat.setEntry(2, 0, 0); mat.setEntry(2, 1, 1); mat.setEntry(2, 2, 1); mat.setEntry(2, 3, 0);
	mat.setEntry(3, 0, 0); mat.setEntry(3, 1, 3); mat.setEntry(3, 2, 1); mat.setEntry(3, 3, 0);
	
	for (int i = 0; i < 12; i++)
		cout << mat.getEntry(i/4,i%4) << ",";

	cout << endl;

	//mat.transpose();
	for (int i = 0; i < 16; i++)
		cout << mat.getEntry(i/4,i%4)<< ",";
}
