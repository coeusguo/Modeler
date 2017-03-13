#pragma once
#ifndef INVERSE_KINEMATICS
#define INVERSE_KINEMATICS
#include "vec.h"
class IKmat {
private:
	float* n;
	int numRow;
	int numCol;
	
	void swapRows(int i,int j);
	float* operator*(IKmat& m2);
public:
	IKmat(int row = 3,int col = 4);//row = 4 is used in calculating inverse matrix
	void setEntry(int row,int col,float value);//start from 0
	float getEntry(int row, int col);//start for 0
	void setZero();
	void operator=(IKmat& matrix);
	float operator[](int i)const;
	void pseudoInverse();
	void setMatrix(float* array,int numRows,int numCols);
	void copy(IKmat* source);
	float* getPointer();
	void setCol(Vec3f vector, int col);
	bool inverse44();
	bool inverse33();
	Vec4f operator*(Vec3f& deltaE);
	void transpose();
};


class InverseKinematics {//using jacobian matrix
private:
	enum jointType { ENDPOINT = 0, JOINT };
	enum axes { X = 0, Y, Z };
	//end point rotation constraint
	const int ALPHA_CONSTRAINT[2] = { -45,75 };//rotate about z axes
	const int BETA_CONSTRAINT[2] = { -45,45 };//rotate about y axes
	const int GAMMA_CONSTRAINT[2] = { -45,135 };//rotate about x axes
	//joint rotation constraint
	const int THETA_CONSTRAINT[2] = { -60,0 };//rotate about x axes

	Vec3f endPoint;
	Vec3f endPointCoord[3];
	Vec3f joint;
	Vec3f jointCoord[3];
	Vec3f effector;
	IKmat* jacobianPseudoInverse;

	float alpha = 0;
	float beta = 0;
	float gamma = 0;
	float theta = 0;

	float arm1;//the length of the arm between end point and joint
	float arm2;//the length of the arm between joint and effector

	void calculateJacobian();
	void resetCoords(int type, int axe, float delta);
	void resetJoint();
	void resetEffector();
public:
	InverseKinematics(Vec3f end, float arm1Length, float arm2Length);

	Vec4f getResult(Vec3f& destination);
	void resetAngle();
};

class InverseKinematics2{//using ccd method
private:
	Vec3f endPoint;
	Vec3f arm1;//the direction vection of arm1

	
	Vec3f arm2;//the dirction vector of arm2

	Vec3f effector;

	float arm1Length;//the length of the arm between end point and joint
	float arm2Length;//the length of the arm between joint and effector

	const float THRESHOLD = 0.05;
	bool enableConstraint = false;
	float constraint1;//store the constraint angle in sine, used to calculate vector
	float constraint2;

	bool largerThan901;//whether constraint1 is bigger than 90 
	bool largerThan902;//whether constraint2 is bigger than 90

public:
	Vec3f joint;
	InverseKinematics2(Vec3f end, float armLength1, float armLength2);

	Vec4f getResult(Vec3f& destination);

	//giving normal vector arm1 and arm2,return the rotation angle
	//rotation sequence: 
	//arm1: rotate about y ,then about x
	//arm2: rotate about x of arm1 coords
	Vec4f calculateAngle();
	void setConstraint(bool value);
	void setConstraint1(float value);
	void setConstraint2(float value);
	void reset();
};

#endif // !INVERSE_KINEMATICS
