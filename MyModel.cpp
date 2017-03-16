// Our model
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include <FL/gl.h>
#include "MarchingCube.h"
#include "InverseKinematics.h"
#include "modelerglobals.h"
#include "Metaball.h"
#include "mat.h"
#include "modelerui.h"
#include <iostream>
#include <vector>
using namespace std;

// To make a SampleModel, we inherit off of 
class MyModel : public ModelerView
{
private:
	vector<Metaball> metaballs;
	InverseKinematics2* rightArm;
	InverseKinematics2* rightLeg;
	int angle;
	bool isAnimationOn;
	int delta = 5;

	int textureWidth ;
	int textureHeight ;
	GLuint textureName;
	GLubyte texture[256][256][4];
	bool isTextureLoaded;

	Vec3f * calculateNewDir(Vec3f newDir, Vec3f lastDir);
	void recursionTree3D(Vec3f dir, Vec3f nextdir, Vec3f currentLocationint, float length);
	void changeAnimationAngle();
public:
	MyModel(int x, int y, int w, int h, char *label)
		: ModelerView(x, y, w, h, label) {
		initMetaball();
		
		Vec3f rightPoint1(-1.2, 5.4, 0);
		Vec3f rightPoint2(-0.8, 3.2, 0);
		rightArm = new InverseKinematics2(rightPoint1, 1.0, 1.0);
		rightLeg = new InverseKinematics2(rightPoint2, 1.5, 1.7);
		angle = 0;
		isAnimationOn = false;
		isTextureLoaded = false;
	}
	float* getRotateAngles(Vec3f target);//input an target vector,the funtion will return 2 float value,first is rotate from z coord about x,and second value is rotate about y

	virtual void draw();
	void drawUpperArm();//including elbow
	void drawLowerArm();//including head
	void drawThigh();//including knee
	void drawShank();//including foot
	void drawHead();//including neck
	void drawUpperTorso();//including shoulder
	void drawLowerTorso();//pelvis
	void drawGatling();
	void drawHandGun();
	void initRecurtionTree();
	void initMetaball();
	void drawTorus();
	void drwaComplexShape();
	void demo();
	void initTexture();
	void drawTexture();
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
	//change light source position
	if (VAL(TURNONLIGHT)) {
		GLfloat pos[4];
		pos[0] = VAL(LIGHTX);
		pos[1] = VAL(LIGHTY);
		pos[2] = VAL(LIGHTZ);
		pos[3] = 0.0f;
		glLightfv(GL_LIGHT1, GL_POSITION, pos);
		glPushMatrix();
		glTranslated(VAL(LIGHTX), VAL(LIGHTY), VAL(LIGHTZ));
		setDiffuseColor(1.0f, 1.0f, 1.0f);
		drawBox(0.2, 0.2, 0.2);
		glPopMatrix();
		setDiffuseColor(COLOR_GREEN);
	}
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
	else 
		rightArm->reset();


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
	else 
		rightLeg->reset();

	isAnimationOn = ModelerUserInterface::m_controlsAnimOnMenu->value();
 
	if (isAnimationOn)
		changeAnimationAngle();
	// draw the floor
	/**/
	
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_RED);
	
	glPushMatrix();
	glTranslated(-5, 0, -5);
	drawBox(10, 0.01f, 10);
	glPopMatrix();
	
	if (VAL(LSYSTEM)) {
		glPushMatrix();
		glTranslated(VAL(LSYSTEMX), 0, VAL(LSYSTEMZ));
		initRecurtionTree();
		glPopMatrix();
	}
		
	//character drawing procedure
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
		glRotated(result1[2], 1, 0, 0);
		drawLowerArm();
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}
	else {
		glPushMatrix();
		if(isAnimationOn)
			glRotated(angle, 1, 0, 0);
		else
			glRotated(-VAL(WAVE), 0, 0, 1);
		drawUpperArm();
		glPushMatrix();
		glTranslated(0, -1.0, 0);
		if (VAL(TERMINATER)) {
			glPushMatrix();
			glRotated(-90, 1, 0, 0);
		}
		drawLowerArm();
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
		if (VAL(TERMINATER))
			glPopMatrix();
	}
	//left arm,no ik
	glPushMatrix();
	glTranslated(1.2, 0, 0);
	glPushMatrix();
	if (isAnimationOn)
		glRotated(-angle, 1, 0, 0);
	else
		glRotated(VAL(WAVE), 0, 0, 1);
	drawUpperArm();
	glPushMatrix();
	glTranslated(0, -1.0, 0);
	if (VAL(TERMINATER)) {
		glPushMatrix();
		glRotated(-90, 1, 0, 0);
	}
	if(VAL(METABALL))
		initMetaball();
	drawLowerArm();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	if (VAL(TERMINATER))
		glPopMatrix();

	glPopMatrix();//upper torso related
	glPopMatrix();
	glPopMatrix();

	//lower torso
	glPushMatrix();
	glTranslated(0, 4.2, 0);// translate m1
	if (VAL(TORUS)) {
		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_RED);
		glPushMatrix();
		glTranslated(0, -0.5, 0);
		glPushMatrix();
		glScaled(0.1, 0.1, 0.1);
		glPushMatrix();
		glRotated(-90, 1, 0, 0);
		drawTorus();
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GREEN);
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
	controls[LSYSTEMX] = ModelerControl("L x", -5, 5, 1, -3);
	controls[LSYSTEMZ] = ModelerControl("L z", -5, 5, 1, -3);
	controls[LSYSTEM] = ModelerControl("Enable L-System", 0, 1, 1, 0);
	controls[LSYSTEMLEVEL] = ModelerControl("L-System Level", 1, 5, 1, 3);
	controls[TORUS] = ModelerControl("Draw Torus", 0, 1, 1, 0);
	controls[TERMINATER] = ModelerControl("Terminater", 0, 1, 1, 0);
	controls[COMPLEX] = ModelerControl("Show School Bag", 0, 1, 1, 0);
	controls[TURNONLIGHT] = ModelerControl("Custom Light Source", 0, 1, 1, 0);
	controls[LIGHTX] = ModelerControl("Light X", -5, 5, 0.1f, 0);
	controls[LIGHTY] = ModelerControl("Light Y", 0, 5, 0.1f, 0);
	controls[LIGHTZ] = ModelerControl("Light Z", -5, 5, 0.1f, 0);
	controls[LEVELOFDETAIL] = ModelerControl("Level of Detail", 0, 5, 1, 5);
	controls[METABALL] = ModelerControl("Show Metaball", 0, 1, 1, 0);
	controls[METABALLX] = ModelerControl("Metaball X", -5, 5, 0.1f, 0);
	controls[METABALLY] = ModelerControl("Metaball Y", -5, 5, 0.1f, 0);
	controls[METABALLZ] = ModelerControl("Metaball Z", -5, 5, 0.1f, 0);
	
	
	//controls[LEGCONSTRAINT1] = ModelerControl("Constraint angle1", 45, 135, 1, 135);
	//controls[LEGCONSTRAINT2] = ModelerControl("Constraint angle", 30, 180, 1, 30);

	ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
	return ModelerApplication::Instance()->Run();
}

void MyModel::drawUpperTorso() {
	if (VAL(LEVELOFDETAIL) >= 1) {

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

		if (VAL(TERMINATER)) {
			glPushMatrix();
			glTranslated(1.6, 0.6, -0.5);
			glPushMatrix();
			glRotated(-30, 0, 0, 1);
			glPushMatrix();
			glRotated(-30, 1, 0, 0);
			drawGatling();
			glPopMatrix();
			glPopMatrix();
			glPopMatrix();

			glPushMatrix();
			glTranslated(-1.6, 0.6, -0.5);
			glPushMatrix();
			glRotated(30, 0, 0, 1);
			glPushMatrix();
			glRotated(-30, 1, 0, 0);
			drawGatling();

			glPopMatrix();
			glPopMatrix();
			glPopMatrix();
			setDiffuseColor(COLOR_GREEN);
		}

		if (VAL(COMPLEX)) {
			glPushMatrix();
			glTranslated(0, -0.5, 0);
			glPushMatrix();
			glRotated(-90, 1, 0, 0);
			drwaComplexShape();
			glPopMatrix();
			glPopMatrix();
		}
	}
}

void MyModel::drawLowerTorso() {
	if (VAL(LEVELOFDETAIL) >= 2) {
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
}

void MyModel::drawThigh() {
	if (VAL(LEVELOFDETAIL) >= 3) {
		glPushMatrix();
		glRotated(90, 1, 0, 0);
		drawCylinder(1.5, 0.1, 0.1);
		glTranslated(0, 0, 1.5);
		drawSphere(0.2);
		glPopMatrix();
	}
}

void MyModel::drawShank() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);
	if (VAL(LEVELOFDETAIL) == 5) {
		glPushMatrix();
		glRotated(90, 1, 0, 0);
		drawCylinder(1.5, 0.1, 0.1);


		glPushMatrix();
		glTranslated(0, 0, 1.5);

		drawCylinder(0.2, 0.4, 0.4);
		glPopMatrix();
		glPopMatrix();

	}
}

void MyModel::drawUpperArm() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);
	if (VAL(LEVELOFDETAIL) >= 2) {
		glPushMatrix();
		glRotated(90, 1, 0, 0);
		drawCylinder(1, 0.1, 0.1);
		glPopMatrix();

		glPushMatrix();
		glTranslated(0, -1, 0);
		drawSphere(0.2);
		glPopMatrix();
	}
	//glPopMatrix();
}

void MyModel::drawLowerArm() {
	//glPushMatrix();
	if (VAL(LEVELOFDETAIL) >= 3) {
		//glTranslated(0, 3, 0);
		glPushMatrix();
		glRotated(90, 1, 0, 0);
		drawCylinder(1, 0.1, 0.1);
		if (VAL(TERMINATER))
			drawHandGun();
		glPopMatrix();

		glPushMatrix();
		glTranslated(0, -1, 0);
		drawSphere(0.25);
		glPopMatrix();
	}
	//glPopMatrix();
}

void MyModel::drawHead() {
	//glPushMatrix();
	//glTranslated(0, 3, 0);
	if (VAL(LEVELOFDETAIL) >= 2) {
		glPushMatrix();
		glRotated(-90, 1, 0, 0);
		drawCylinder(0.3, 0.1, 0.1);
		glPopMatrix();

		glPushMatrix();
		glTranslated(0.0, 0.9, -0.25);
		drawCylinder(0.5, 0.6, 0.6);
		initTexture();
		glPushMatrix();
		glTranslated(0.0, 0.0, 0.01);
		drawCylinder(0.50, 0.59, 0.59);
		glDisable(GL_TEXTURE_2D);
	
		glPopMatrix();

		glPopMatrix();
	}
	/*glPushMatrix();
	glTranslated(0, 0, 0.51);
	initTexture();
	drawTexture();
	glPopMatrix();
	*/
	
	
}

void MyModel::drawTorus() {
	grid grid(30);
	
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
	
	grid.drawSurface(1.0);

	
}

void MyModel::initMetaball() {
	ball ball3(0, -3, 0, 1.5);
	ball ball1(VAL(METABALLX), VAL(METABALLY) + 0.5,VAL(METABALLZ), 2);
	ball ball2(0,5, 0, 2 );
	Metaball balls(40);
	balls.addBalls(ball1);
	balls.addBalls(ball2);
	balls.addBalls(ball3);
	glPushMatrix();
	glScaled(0.2, 0.2, 0.2);
	balls.drawMetaball();
	glPopMatrix();
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

//L-system
void MyModel::recursionTree3D(Vec3f dir, Vec3f nextdir, Vec3f currentLocation, float length) {//dir and nextdir must be unit vector
	//cout << "dir:" << dir[0] << "," << dir[1] << "," << dir[2] << endl;
	//cout << "nextdir:" << nextdir[0] << "," << nextdir[1] << "," << nextdir[2] << endl;
	float width = length / 4.0f;
	//cout << length;
	nextdir *= length;

	if (length == 0.5) {
		glColor3ub(0, 255, 0);
		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_GREEN);
		//cout << nextdir[0] <<","<<nextdir[1]<<","<< nextdir[2] << endl;
		float* rotateAngle = getRotateAngles(nextdir);
		glPushMatrix();
		glTranslatef(currentLocation[0], currentLocation[1], currentLocation[2]);
		glPushMatrix();
		glRotatef(rotateAngle[1], 0, 1, 0);
		glPushMatrix();
		glRotatef(rotateAngle[0], 1, 0, 0);
		//cout << rotateAngle[0] << "," << rotateAngle[1] << endl;
		drawCylinder(length, 0.05, 0.05);
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
		return;
		
	}
	setAmbientColor(1.0f, 1.0f, 1.0f);
	setDiffuseColor(0.545,0.271,0.075);
	float* rotateAngle = getRotateAngles(nextdir);
	glPushMatrix();
	glTranslatef(currentLocation[0], currentLocation[1], currentLocation[2]);
	glPushMatrix();
	glRotatef(rotateAngle[1], 0, 1, 0);
	glPushMatrix();
	glRotatef(rotateAngle[0], 1, 0, 0);
	//cout << rotateAngle[0] << "," << rotateAngle[1] << endl;
	drawCylinder(length, 0.1, 0.1);
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	currentLocation += nextdir;//update current location
	
	nextdir.normalize();
	
	Vec3f* newDir = calculateNewDir(dir, nextdir);
	
	//cout << newDir[0][0] << "," << newDir[0][1] << "," << newDir[0][2] << endl;
	recursionTree3D(nextdir, newDir[0], currentLocation, length / 2.0f);
	recursionTree3D(nextdir, newDir[1], currentLocation, length / 2.0f);
	recursionTree3D(nextdir, newDir[2], currentLocation, length / 2.0f);
	recursionTree3D(nextdir, newDir[3], currentLocation, length / 2.0f);
}

Vec3f* MyModel::calculateNewDir(Vec3f newDir, Vec3f lastDir) {
	//cout << newDir[0] << "," << newDir[1] << "," << newDir[2] << endl;
	//cout << lastDir[0] << "," << lastDir[1] << "," << lastDir[2] << endl;
	float deltaValue = newDir * lastDir;
	Vec3f lastDirNormal = lastDir;
	lastDirNormal.normalize();
	Vec3f projVec = deltaValue * lastDirNormal;
	Vec3f deltaVec = projVec - newDir;
	Vec3f deltaVecRotate90 = newDir ^ lastDir;
	deltaVecRotate90.normalize();
	deltaVecRotate90 *= deltaValue;

	Vec3f * result = new Vec3f[4];
	result[0] = projVec - deltaVec;
	result[2] = projVec + deltaVec;
	result[3] = projVec + deltaVecRotate90;
	result[1] = projVec - deltaVecRotate90;

	for (int i = 0; i < 4; i++)
		result[i].normalize();
	//cout << result[0][0] << "," << result[0][1] << "," << result[0][2] << endl;
	return result;
}

void MyModel::initRecurtionTree() {
	Vec3f treeLocation(0, 0, 0);
	float trunkHeight = pow(2, VAL(LSYSTEMLEVEL)  - 2);
	Vec3f inidirVec(0, 1, 0);
	Vec3f nextDirSeed(0, 1, 1);
	nextDirSeed.normalize();
	Vec3f* newDir = calculateNewDir(nextDirSeed, inidirVec);
	Vec3f currentLocation = inidirVec * trunkHeight;
	setAmbientColor(1.0f, 1.0f, 1.0f);
	setDiffuseColor(0.545, 0.271, 0.075);
	glPushMatrix();
	glRotated(-90, 1, 0, 0);
	drawCylinder(trunkHeight, 0.1, 0.1);
	glPopMatrix();
	//cout << newDir[0][0] << "," << newDir[0][1] << "," << newDir[0][2] << endl;
	//cout << newDir[1][0] << "," << newDir[1][1] << "," << newDir[1][2] << endl;
	//cout << newDir[2][0] << "," << newDir[2][1] << "," << newDir[2][2] << endl;
	//cout << newDir[3][0] << "," << newDir[3][1] << "," << newDir[3][2] << endl;
	//cout << inidirVec[0] << "," << inidirVec[1] << "," << inidirVec[2] << endl;
	recursionTree3D(inidirVec,newDir[0], currentLocation, trunkHeight);
	recursionTree3D(inidirVec,newDir[1], currentLocation, trunkHeight);
	recursionTree3D(inidirVec,newDir[2], currentLocation, trunkHeight);
	recursionTree3D(inidirVec,newDir[3], currentLocation, trunkHeight);
}

float* MyModel::getRotateAngles(Vec3f target) {
	float* result = new float[2];
	Vec3f yaxis(0, 1, 0);
	Vec3f zaxis(0, 0, 1);
	Vec3f projVecY = target * yaxis * yaxis;
	Vec3f rotateVec = target - projVecY;
	Vec3f planeVec(0, 0, rotateVec.length());
	Vec3f planeTarget(0, projVecY.length(), rotateVec.length());
	
	result[0] = -acosf((zaxis * planeTarget) / planeTarget.length()) / 3.1416 * 180;
	if (rotateVec.length() != 0)
		result[1] = acosf((rotateVec * planeVec) / (rotateVec.length() * planeVec.length())) / 3.1416 * 180;
	else
		result[1] = 0;
	//
	if (target[0] < 0)
		result[1] = - result[1];
	if (target[1] < 0)
		result[0] = -result[0];
	//cout << "angle:" << result[1] << endl;
	return result;
}

void MyModel::drawGatling() {
	
	glPushMatrix();
	glTranslated(0, 0, -1.75);


	setDiffuseColor(0.2, 0.2, 0.2);
	drawCylinder(3.91, 0.15, 0.15);
	setDiffuseColor(0.5, 0.5, 0.5);
	glPushMatrix();
	
	glTranslated(0, 0, 3.7);
	drawCylinder(0.2, 0.55, 0.55);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, 3.3);
	drawCylinder(0.1, 0.55, 0.55);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, 3.0);
	drawCylinder(0.1, 0.55, 0.55);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, 1.5);
	drawCylinder(0.5, 0.55, 0.55);
	glPushMatrix();
	glTranslated(0, -0.3, 0.25);
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	drawCylinder(0.5, 0.2, 0.2);
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	//glPushMatrix();
	//glTranslated(0, 0, 0.2);
	drawCylinder(0.5, 0.55, 0.55);
	//glPopMatrix();

	glPushMatrix();
	glTranslated(0, -0.35, 0);
	setDiffuseColor(0.3, 0.3, 0.3);
	drawCylinder(4, 0.1, 0.1);
	
	setDiffuseColor(0, 0, 0);
	drawCylinder(4.01, 0.08, 0.08);
	glPopMatrix();

	
	glPushMatrix();
	glTranslated(0, 0.35, 0);
	setDiffuseColor(0.3, 0.3, 0.3);
	drawCylinder(4, 0.1, 0.1);
	setDiffuseColor(0, 0, 0);
	drawCylinder(4.01, 0.08, 0.08);
	glPopMatrix();


	glPushMatrix();
	glTranslated(-0.35, 0.202, 0);
	setDiffuseColor(0.3, 0.3, 0.3);
	drawCylinder(4, 0.1, 0.1);
	setDiffuseColor(0, 0, 0);
	drawCylinder(4.01, 0.08, 0.08);
	glPopMatrix();

	
	glPushMatrix();
	glTranslated(-0.35, -0.202, 0);
	setDiffuseColor(0.3, 0.3, 0.3);
	drawCylinder(4, 0.1, 0.1);
	setDiffuseColor(0, 0, 0);
	drawCylinder(4.01, 0.08, 0.08);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0.35, 0.202, 0);
	setDiffuseColor(0.3, 0.3, 0.3);
	drawCylinder(4, 0.1, 0.1);
	setDiffuseColor(0, 0, 0);
	drawCylinder(4.01, 0.08, 0.08);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0.35, -0.202, 0);
	setDiffuseColor(0.3, 0.3, 0.3);
	drawCylinder(4, 0.1, 0.1);
	setDiffuseColor(0, 0, 0);
	drawCylinder(4.01, 0.08, 0.08);
	glPopMatrix();
	glPopMatrix();
}

void MyModel::drawHandGun() {

	glPushMatrix();
	glTranslated(0, 0, 0.5);
	setDiffuseColor(0.2, 0.2, 0.2);
	drawCylinder(0.5, 0.5, 0.3);
	glPopMatrix();
	setDiffuseColor(0.3, 0.3, 0.3);
	glPushMatrix();
	glTranslated(0, 0.3, 0.5);
	drawCylinder(1, 0.1, 0.1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0.3, -0.15, 0.5);
	drawCylinder(1, 0.1, 0.1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-0.3, -0.15, 0.5);
	drawCylinder(1, 0.1, 0.1);
	glPopMatrix();
	setDiffuseColor(COLOR_GREEN);
}

void MyModel::drwaComplexShape() {
	int savemode;
	glGetIntegerv(GL_MATRIX_MODE, &savemode);
	float angle72 = 72.0f / 180.0f * 3.1416;
	float angle36 = 36.0f /180.0f * 3.1416;
	Mat3f rotate72(cosf(angle72),0,-sinf(angle72),0,1,0,sinf(angle72),0,cosf(angle72));
	Vec3f pointTop[5];
	
	Vec3f pointBottom[5];
	pointBottom[0][0] = 0; pointBottom[0][1] = 0; pointBottom[0][2] = 1;
	
	glBegin(GL_TRIANGLES);
	
	
	
	for (int i = 1; i <= 5; i++) {
		pointBottom[i%5] = rotate72 * pointBottom[i - 1];
		glVertex3d(0, 0, 0);
		glVertex3d(pointBottom[i-1][0], pointBottom[i-1][1], pointBottom[i-1][2]);
		//cout << pointBottom[i%5][0] << "," << pointBottom[i%5][1] << "," << pointBottom[i%5][2] << endl;
		glVertex3d(pointBottom[i%5][0], pointBottom[i%5][1], pointBottom[i%5][2]);
	}

	Mat3f rotate36(cosf(angle36), 0, -sinf(angle36), 0, 1, 0, sinf(angle36), 0, cosf(angle36));
	pointTop[0] = rotate36 * pointBottom[0];
	pointTop[0][1] = 1;
	glColor3ub(0, 0, 255);
	for (int i = 1; i <= 5; i++) {
		pointTop[i % 5] = rotate72 * pointTop[i - 1];
		glVertex3d(0, 1, 0);
		glVertex3d(pointTop[i - 1][0], pointTop[i - 1][1], pointTop[i - 1][2]);
		//cout << pointTop[i % 5][0] << "," << pointTop[i % 5][1] << "," << pointTop[i % 5][2] << endl;
		glVertex3d(pointTop[i % 5][0], pointTop[i % 5][1], pointTop[i % 5][2]);
	}
	glColor3ub(255, 0, 0);
	setDiffuseColor(COLOR_RED);
	for (int i = 1; i <= 5; i++) {
		glVertex3d(pointTop[i - 1][0], pointTop[i - 1][1], pointTop[i - 1][2]);
		glVertex3d(pointTop[i % 5][0], pointTop[i % 5][1], pointTop[i % 5][2]);
		glVertex3d(pointBottom[i % 5][0], pointBottom[i % 5][1], pointBottom[i % 5][2]);
	}

	setDiffuseColor(COLOR_BLUE);
	for (int i = 1; i <= 5; i++) {
		glVertex3d(pointBottom[i - 1][0], pointBottom[i - 1][1], pointBottom[i - 1][2]);
		glVertex3d(pointBottom[i % 5][0], pointBottom[i % 5][1], pointBottom[i % 5][2]);
		glVertex3d(pointTop[i - 1][0], pointTop[i - 1][1], pointTop[i - 1][2]);
	}
	glEnd();
	
	
}

void MyModel::changeAnimationAngle() {
	
	if (isAnimationOn) {
		if (angle >= 60 || angle <= -60)
			delta = -delta;
		angle += delta;
	}
	else
		angle = 0;

}

void MyModel::initTexture() {
	
	if (!isTextureLoaded) {
		char* name = "./huaji.bmp";

		unsigned char* temp = readBMP(name, textureWidth, textureHeight);
		/*
		texture = new GLubyte**[textureHeight];
		for (int i = 0; i < textureHeight; i++) {
			texture[i] = new GLubyte*[textureWidth];
			for (int j = 0; j < textureWidth; j++)
				texture[i][j] = new GLubyte[4];
		}
		*/
		//cout << "width:" << textureWidth << ",height:" << textureHeight;
		for (int row = 0; row < textureHeight; row++) {
			for (int col = 0; col < textureWidth; col++) {
				for (int i = 0; i < 3; i++)
					texture[row][col][i] = temp[(row * textureWidth + col) * 3 + i];
				//cout << "("<<texture[row][col][0] << "," << texture[row][col][1] << "," << texture[row][col][2]<<")";
				texture[row][col][3] = 255;
			}
		}
	}
		
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glGenTextures(1, &textureName);
	glBindTexture(GL_TEXTURE_2D, textureName);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, textureWidth,textureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE,texture);
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, textureName);
		
}

void MyModel::drawTexture() {
	
	
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0); glVertex3f(0.0, 0.0, 0.0);
	glTexCoord2f(0.0, 1.0); glVertex3f(0.0, 1.0, 0.0);
	glTexCoord2f(1.0, 1.0); glVertex3f(1.0, 1.0, 0.0);
	glTexCoord2f(1.0, 0.0); glVertex3f(1.0, 0.0, 0.0);


	glEnd();
	
	/*
	glBegin(GL_TRIANGLE_FAN);
	glTexCoord2f(0.5, 0.5); glVertex3f(0.0, 0.0, 0.0);
	glEnd();
	glFlush();
	*/
	glDisable(GL_TEXTURE_2D);
	
}