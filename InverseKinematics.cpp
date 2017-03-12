#include "InverseKinematics.h"
#include "mat.h"
#include <cstdlib>
#include <iostream>
#include <math.h>
using namespace std;

IKmat::IKmat(int row,int col) {
	n = new float[row * col];
	numRow = row;//row can be 3 or 4
	numCol = col;
	setZero();
}

void IKmat::setEntry(int row, int col, float value) {
	if (row >= numRow || col >= numCol) {
		cout << "unable to set entry at [" << row << "," << col << "]" << endl;
		return;
	}
		
	n[row * numCol + col] = value;
}

float IKmat::getEntry(int row, int col) {
	if (row >= numRow || col >= numCol) {
		cout << "unable to get entry at [" << row << "," << col << "]" << endl;
		return 0.0;
	}
	return n[row * numCol + col];
}

void IKmat::setZero() {
	for (int i = 0; i < numRow * 4; i++)
		n[i] = 0;
}

void IKmat::transpose() {
	float* tn = new float[numRow * numCol];
	for (int row = 0; row < numRow; row++) {
		for (int col = 0; col < numCol; col++)
			tn[col * numRow + row] = n[row * numCol + col];
	}
	int temp = numRow;
	numRow = numCol;
	numCol = temp;
	delete[]n;
	n = tn;
}

void IKmat::pseudoInverse() {
	IKmat tran;//3*4
	tran.copy(this);
	/*
	cout << "testing copy" << endl;
	for (int i = 0; i < 12; i++)
		cout << tran[i] << ",";
	cout << endl;
	*/
	IKmat temp(3,3);//4*4 matrix
	tran.transpose();//now is a 4* 3 matrix
	//cout << tran.numRow;
	/*
	cout << "testing transpose" << endl;
	for (int i = 0; i < 12; i++)
		cout << tran[i] << ",";
	cout << endl;
	*/
	temp.setMatrix(tran * (*this),4,4);
	/*cout << "testing set" << endl;
	for (int i = 0; i < 16; i++)
		cout << temp[i] << ",";
	cout << endl;*/
	
	
	bool isInversed = temp.inverse44();
	if (!isInversed) {
		cout << "can`t be inversed!" << endl;
		return;
	}
	/*cout << "testing inverse" << endl;
	for (int i = 0; i < 16; i++)
		cout << temp[i] << ",";
	cout << endl;*/
	setMatrix(tran * temp, 4, 3);
	
}

void IKmat::copy(IKmat* source) {
	for (int i = 0; i < numRow * numCol; i++)
		n[i] = source->n[i];
}

//used int (4 * 4) * £¨4 * 3£© and (4 * 3) * (3 * 4) matrix only,private
float* IKmat::operator*(IKmat& m2) {
	float* m1 = getPointer();
	float* result;
	int index = 0;
	if (m2.numRow == 4) {//m2 is 4 * 3 matrix
		//result = new float[12];
		result = new float[9];
		for (int i = 0; i < 4; i++) {
		//for (int i = 0; i < 3; i++) {
			//cout << "second" << endl;
			result[i] = m1[0] * m2[i] + m1[1] * m2[3 + i] + m1[2] * m2[6 + i] + m1[3] * m2[9 + i];
			result[i + 3] = m1[4] * m2[i] + m1[5] * m2[3 + i] + m1[6] * m2[6 + i] + m1[7] * m2[9 + i];
			result[i + 6] = m1[8] * m2[i] + m1[9] * m2[3 + i] + m1[10] * m2[6 + i] + m1[11] * m2[9 + i];
			result[i + 9] = m1[12] * m2[i] + m1[13] * m2[3 + i] + m1[14] * m2[6 + i] + m1[15] * m2[9 + i];
		}
	}
	else {//m2 is 3 * 4 matrix
		//result = new float[16];
		result = new float[12];
		for (int i = 0; i < 4; i++) {
		//for (int i = 0; i < 3; i++) {
			// << "first"<<endl;
			
			result[i] = m1[0] * m2[i] + m1[1] * m2[4 + i] + m1[2] * m2[8 + i];
			result[i + 4] = m1[3] * m2[i] + m1[4] * m2[4 + i] + m1[5] * m2[8 + i];
			result[i + 8] = m1[6] * m2[i] + m1[7] * m2[4 + i] + m1[8] * m2[8 + i];
			result[i + 12] = m1[9] * m2[i] + m1[10] * m2[4 + i] + m1[11] * m2[8 + i];
			/*
			result[i] = m1[0] * m2[i] + m1[1] * m2[4 + i] + m1[2] * m2[8 + i];
			result[i + 4] = m1[3] * m2[i] + m1[4] * m2[4 + i] + m1[5] * m2[8 + i];
			result[i + 8] = m1[6] * m2[i] + m1[7] * m2[4 + i] + m1[8] * m2[8 + i];
			result[i + 12] = m1[9] * m2[i] + m1[10] * m2[4 + i] + m1[11] * m2[8 + i];
			*/
		}
		/*
		cout << "multiply test" << endl;
		for (int i = 0; i < 16; i++)
			cout << result[i] << ",";
		cout << endl;
		*/
	}
	
	return result;
}

float IKmat::operator[](int i)const {
	return n[i];
}
void IKmat::operator=(IKmat& matrix) {
	copy(&matrix);
}
/*
void IKmat::inverse() {
	IKmat		a;
	a = *this;
	IKmat		b;

	int	i, j, k, l;

	for (i = 0; i<4; i++) {
		j = i;
		for (k = i + 1; k<4; k++) {
			if (fabs(a.getEntry(k,i)) > fabs(a.getEntry(j,i)))
				j = k;
		}

		a.swapRows(i, j);
		b.swapRows(i, j);

		if (a.getEntry(i, i) == 0.0)
			return;

		float diag = a.getEntry(i, i);
		for (k = 0; k<4; k++) {
			a.setEntry(i, k, a.getEntry(i, k) / diag);
			a.setEntry(i, k, a.getEntry(i, k) / diag);
		}

		for (k = 0; k<4; k++) {
			if (k != i) {
				float aki = a.getEntry(k,i);
				for (l = 0; l<4; l++) {
					a.setEntry(k,l, a.getEntry(k, l) - aki * a.getEntry(i, l));
					if (fabs(a.getEntry(k, l)) < 0.00000001) a.setEntry(k, l, 0.0);
					 b.setEntry(k, l, b.getEntry(k, l) - aki * b.getEntry(i, l));
					if (fabs(b.getEntry(k,l)) < 0.00000001) b.setEntry(k,l,0.0);
				}
			}
		}
	}
	*this = b;
}*/

void IKmat::swapRows(int i,int j) {
	int a[4];//temp array
	a[0] = n[i * 4]; a[1] = n[i * 4 + 1]; a[2] = n[i * 4 + 2]; a[3] = n[i * 4 + 3];
	n[i * 4] = n[j * 4];n[i * 4 + 1] = n[j * 4 + 1]; n[i * 4 + 2] = n[j * 4 + 2]; n[i * 4 + 3] = n[j * 4 + 3];
	n[j * 4] = a[0]; n[j * 4 + 1] = a[1]; n[j * 4 + 2] = a[3]; n[j * 4 + 3] = a[3];
}

void IKmat::setMatrix(float* matrix,int numRows,int numCols) {
	delete[]n;
	n = matrix;
	numRow = numRows;
	numCol = numCols;
}

bool IKmat::inverse44()
{
	float* m = getPointer();
	float inv[16], det;
	int i;
	inv[0] = m[5] * m[10] * m[15] -
		m[5] * m[11] * m[14] -
		m[9] * m[6] * m[15] +
		m[9] * m[7] * m[14] +
		m[13] * m[6] * m[11] -
		m[13] * m[7] * m[10];

	inv[4] = -m[4] * m[10] * m[15] +
		m[4] * m[11] * m[14] +
		m[8] * m[6] * m[15] -
		m[8] * m[7] * m[14] -
		m[12] * m[6] * m[11] +
		m[12] * m[7] * m[10];
	
	inv[8] = m[4] * m[9] * m[15] -
		m[4] * m[11] * m[13] -
		m[8] * m[5] * m[15] +
		m[8] * m[7] * m[13] +
		m[12] * m[5] * m[11] -
		m[12] * m[7] * m[9];

	inv[12] = -m[4] * m[9] * m[14] +
		m[4] * m[10] * m[13] +
		m[8] * m[5] * m[14] -
		m[8] * m[6] * m[13] -
		m[12] * m[5] * m[10] +
		m[12] * m[6] * m[9];

	inv[1] = -m[1] * m[10] * m[15] +
		m[1] * m[11] * m[14] +
		m[9] * m[2] * m[15] -
		m[9] * m[3] * m[14] -
		m[13] * m[2] * m[11] +
		m[13] * m[3] * m[10];

	inv[5] = m[0] * m[10] * m[15] -
		m[0] * m[11] * m[14] -
		m[8] * m[2] * m[15] +
		m[8] * m[3] * m[14] +
		m[12] * m[2] * m[11] -
		m[12] * m[3] * m[10];

	inv[9] = -m[0] * m[9] * m[15] +
		m[0] * m[11] * m[13] +
		m[8] * m[1] * m[15] -
		m[8] * m[3] * m[13] -
		m[12] * m[1] * m[11] +
		m[12] * m[3] * m[9];

	inv[13] = m[0] * m[9] * m[14] -
		m[0] * m[10] * m[13] -
		m[8] * m[1] * m[14] +
		m[8] * m[2] * m[13] +
		m[12] * m[1] * m[10] -
		m[12] * m[2] * m[9];

	inv[2] = m[1] * m[6] * m[15] -
		m[1] * m[7] * m[14] -
		m[5] * m[2] * m[15] +
		m[5] * m[3] * m[14] +
		m[13] * m[2] * m[7] -
		m[13] * m[3] * m[6];

	inv[6] = -m[0] * m[6] * m[15] +
		m[0] * m[7] * m[14] +
		m[4] * m[2] * m[15] -
		m[4] * m[3] * m[14] -
		m[12] * m[2] * m[7] +
		m[12] * m[3] * m[6];

	inv[10] = m[0] * m[5] * m[15] -
		m[0] * m[7] * m[13] -
		m[4] * m[1] * m[15] +
		m[4] * m[3] * m[13] +
		m[12] * m[1] * m[7] -
		m[12] * m[3] * m[5];

	inv[14] = -m[0] * m[5] * m[14] +
		m[0] * m[6] * m[13] +
		m[4] * m[1] * m[14] -
		m[4] * m[2] * m[13] -
		m[12] * m[1] * m[6] +
		m[12] * m[2] * m[5];
	//for(int j = 0;j<16;j++)
		//cout << n[j] << endl;
	inv[3] = -m[1] * m[6] * m[11] +
		m[1] * m[7] * m[10] +
		m[5] * m[2] * m[11] -
		m[5] * m[3] * m[10] -
		m[9] * m[2] * m[7] +
		m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
		m[0] * m[7] * m[10] -
		m[4] * m[2] * m[11] +
		m[4] * m[3] * m[10] +
		m[8] * m[2] * m[7] -
		m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
		m[0] * m[7] * m[9] +
		m[4] * m[1] * m[11] -
		m[4] * m[3] * m[9] -
		m[8] * m[1] * m[7] +
		m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
		m[0] * m[6] * m[9] -
		m[4] * m[1] * m[10] +
		m[4] * m[2] * m[9] +
		m[8] * m[1] * m[6] -
		m[8] * m[2] * m[5];

	//for (int j = 0; j < 16; j++)
		//cout << inv[j];
	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (det == 0)
		return false;

	det = 1.0 / det;

	for (i = 0; i < 16; i++) {
		//cout << inv[j] * det << endl;
		n[i] = inv[i] * det;
	}
	
	return true;
}

bool IKmat::inverse33() {
	//float det = 
	return true;
}

float* IKmat::getPointer() {
	return n;
}

void IKmat::setCol(Vec3f vector, int col) {
	if (col >= numCol) {
		cout << col <<" does not exist" << endl;
		return;
	}
	n[col] = vector[0];
	n[col + 4] = vector[1];
	n[col + 8] = vector[2];
}

Vec4f IKmat::operator*(Vec3f& deltaE) {
	Vec4f result;
	if (numRow != 4 && numCol != 3) {
		cout << "the size of the matrix is not right!" << endl;
		return result;
	}

	result[0] = n[0] * deltaE[0] + n[1] * deltaE[1] + n[2] * deltaE[2];
	result[1] = n[3] * deltaE[0] + n[4] * deltaE[1] + n[5] * deltaE[2];
	result[2] = n[6] * deltaE[0] + n[7] * deltaE[1] + n[8] * deltaE[2];
	result[3] = n[9] * deltaE[0] + n[10] * deltaE[1] + n[11] * deltaE[2];
	
	return result;
}

InverseKinematics::InverseKinematics(Vec3f end, float arm1Length, float arm2Length) {
	endPoint = end;
	joint = endPoint;
	joint[1] -= arm1Length;
	effector = joint;
	effector[1] -= arm2Length;
	jacobianPseudoInverse = NULL;
	arm1 = arm1Length;
	arm2 = arm2Length;
	//using right hand rule
	endPointCoord[0][0] = -1.0f; endPointCoord[0][1] = 0.0f; endPointCoord[0][2] = 0.0f;//x coord
	endPointCoord[1][0] = 0.0f; endPointCoord[1][1] = 0.0f; endPointCoord[1][2] = -1.0f;//y coord
	endPointCoord[2][0] = 0.0f; endPointCoord[2][1] = -1.0f; endPointCoord[2][2] = 0.0f;//z coord

	jointCoord[0][0] = -1.0f; jointCoord[0][1] = 0.0f; jointCoord[0][2] = 0.0f;//x coord
	jointCoord[1][0] = 0.0f; jointCoord[1][1] = 0.0f; jointCoord[1][2] = -1.0f;//y coord
	jointCoord[2][0] = 0.0f; jointCoord[2][1] = -1.0f; jointCoord[2][2] = 0.0f;//z coord
}

void InverseKinematics::calculateJacobian() {

	if (!jacobianPseudoInverse)
		delete jacobianPseudoInverse;

	IKmat* iniMatrix = new IKmat;//3*4 matrix
	Vec3f col1, col2, col3, col4;
	//calculate column 1 ,2, 3
	//calculate moving direction
	Vec3f dir = effector - endPoint;
	col1 = endPointCoord[2] ^ dir;//rotate about z
	col2 = endPointCoord[1] ^ dir;//rotate about y
	col3 = endPointCoord[0] ^ dir;//rotate about x
	iniMatrix->setCol(col1, 0);
	iniMatrix->setCol(col2, 1);
	iniMatrix->setCol(col3, 2);

	//calculate column 4
	dir = effector - joint;
	col4 = jointCoord[0] ^ dir;
	iniMatrix->setCol(col4, 3);
	iniMatrix->pseudoInverse();
	/*
	for (int i = 0; i < 12; i++)
		cout << iniMatrix->getPointer()[i] << ",";
	cout << endl;
	*/
	jacobianPseudoInverse = iniMatrix;
}

void InverseKinematics::resetCoords(int type, int axe, float delta1) {
	Mat3f* rotation;
	//cout << delta1 << endl;
	float delta = (delta1 / 180.0f) * 3.1415926;
	if (type == ENDPOINT) {
		switch (axe)
		{
		case Z:
			if (delta1 + alpha > ALPHA_CONSTRAINT[1] || delta1 + alpha< ALPHA_CONSTRAINT[0])
				return;
			rotation = new Mat3f(cosf(delta), -sinf(delta), 0.0, sinf(delta), cosf(delta), 0.0, 0.0, 0.0, 1.0);
			endPointCoord[0] = *rotation * endPointCoord[0];
			endPointCoord[1] = *rotation * endPointCoord[1];
			endPointCoord[0].normalize();
			endPointCoord[1].normalize();
			alpha += delta1;
			//cout <<"alpha:" <<alpha << endl;
			break;
		case Y:
			if (delta1 + beta > BETA_CONSTRAINT[1] || delta1 + beta < BETA_CONSTRAINT[0])
				return;
			rotation = new Mat3f(cosf(delta), 0, -sinf(delta), 0.0, 1.0, 0.0, sinf(delta), 0.0, cosf(delta));
			endPointCoord[0] = *rotation * endPointCoord[0];//x
			endPointCoord[2] = *rotation * endPointCoord[2];//z
			endPointCoord[0].normalize();
			endPointCoord[2].normalize();
			beta += delta1;
			//cout <<"beta:"<< beta << endl;
			break;
		case X:
			if (delta1 + gamma > GAMMA_CONSTRAINT[1] || delta1 + gamma < GAMMA_CONSTRAINT[0])
				return;
			
			rotation = new Mat3f(1.0, 0.0, 0.0, 0.0, cosf(delta), -sinf(delta), 0, sinf(delta), cosf(delta));
			endPointCoord[1] = *rotation * endPointCoord[1];//y
			endPointCoord[2] = *rotation * endPointCoord[2];//z
			endPointCoord[1].normalize();
			endPointCoord[2].normalize();
			gamma += delta1;
			//cout <<"gamma:"<< gamma << endl;
			break;
		default:
			break;
		}
		resetJoint();
	}
	else if (type == JOINT) {
		
		if (delta1 + theta > THETA_CONSTRAINT[1] || delta1 + theta < GAMMA_CONSTRAINT[0])
			return;
		//cout << "here" << endl;
		rotation = new Mat3f(1.0, 0.0, 0.0, 0.0, cosf(delta), -sinf(delta), 0, sinf(delta), cosf(delta));
		jointCoord[1] = *rotation * jointCoord[1];//y
		jointCoord[2] =* rotation * jointCoord[2];//z
		jointCoord[1].normalize();
		jointCoord[2].normalize();
		theta += delta1;
		resetEffector();
		cout << "theta:" << theta << endl;
	}
	else {
		cout << "the type input is wrong!" << endl;
		return;
	}
	delete rotation;
}

void InverseKinematics::resetJoint() {
	joint = endPoint + arm1 * endPointCoord[2];//the length of thigh is 1.5
	resetEffector();
}

void InverseKinematics::resetEffector() {
	effector = joint + arm2 * jointCoord[2];//the length of shank + foot is 1.7
	cout << "(" << effector[0] << "," << effector[1] << "," << effector[2] << ")" << endl;
}

Vec4f InverseKinematics::getResult(Vec3f& destination) {
	Vec3f dir = destination - effector;
	//cout << "check dir vector:" << endl;
	/*
	for (int i = 0; i < 3; i++)
		cout << dir[i] << ",";
	cout << endl;
	*/
	float distance = dir.length();
	dir.normalize();
	/*
	cout << "check dir vector:" << endl;
	for (int i = 0; i < 3; i++)
		cout << dir[i] << ",";
	cout << endl;
	*/
	while (distance > 0.1) {
		//cout << "getResult(),current:" << distance << endl;
		
		dir *= 0.1;
		/*
		cout << "check dir vector:" << endl;
		for (int i = 0; i < 3; i++)
			cout << dir[i] << ",";
		cout << endl;
		*/
		calculateJacobian();
		Vec4f deltaTheta = *jacobianPseudoInverse * dir;
		/*
		cout << "check delta theta" << endl;
		for (int i = 0; i < 4; i++)
			cout << deltaTheta[i] << ",";
		cout << endl;
		*/
		resetCoords(ENDPOINT, X, deltaTheta[2]);
		resetCoords(ENDPOINT, Y, deltaTheta[1]);
		resetCoords(ENDPOINT, Z, deltaTheta[0]);
		resetCoords(JOINT, X, deltaTheta[3]);
		/*
		for (int i = 0; i < 4; i++)
			cout << "delta theta:" << deltaTheta[i] << ",";
		cout << endl;
		*/

		dir = destination - effector;
		distance = dir.length();
		dir.normalize();
	}

	Vec4f result(alpha, beta, gamma, theta);
	return result;
}

void InverseKinematics::resetAngle() {
	alpha = 0;
	beta = 0;
	gamma = 0;
	theta = 0;
}

InverseKinematics2::InverseKinematics2(Vec3f end, float armLength1, float armLength2) {
	endPoint = end;//set end point location

	//set arm length
	arm1Length = armLength1;
	arm2Length = armLength2;

	//set arm direction vector
	arm1[0] = 0; arm1[1] = -1; arm1[2] = 0;
	arm2[0] = 0; arm2[1] = -1; arm2[2] = 0;

	//set joint location and effector location
	joint = endPoint; joint[1] -= armLength1;
	effector = joint; effector[1] -= armLength2;
	//cout << "initial effector:(" << effector[0] << "," << effector[1] << "," << effector[2] << ")" << endl;
}

Vec4f InverseKinematics2::getResult(Vec3f& destination) {
	//cout << destination[0] << "," << destination[1] << "," << destination[2] << endl;
	Vec3f yaxes(0, 1, 0);
	if (enableConstraint&&destination[0] > -0.7&&destination[2] < 0)
		destination[0] = -0.7;
	/*
	if (enableConstraint) {
		//float value = ((arm1 + arm2) / 2)[1] - arm1[1];
		float value = arm1 * arm2;
		if (destination[2]<=0 && destination[2]>-1&value<=-0.866) {
			destination[2] = -1;
		}
		if (destination[2]>0 && destination[2]< 1 && value <= -0.7) {
			destination[2] = 1;
		}
	}
	*/
	for (int time = 0; time < 100; time++) {
		//1.calculate direction of joint and destination
		effector = destination;
		Vec3f dir0 = destination - joint;
		if (dir0 * arm2 == 0) {
			Vec3f delta(0.1, 0.1, 0.1);
			joint = joint + delta;
		}
		arm2 = destination - joint;
		arm2.normalize();
		
		

		if (enableConstraint) {
			float value = ((arm1 + arm2) / 2)[1];
			if ( arm1[1]<value&& destination[2] > 0) {
				Vec3f down = arm2 * arm1 * arm1 - arm2;
				arm2 += down * 2;
				arm2.normalize();
			}
			if (arm1[1] > value && destination[2] <= 0) {
				Vec3f up = arm2 * arm1 * arm1 - arm2;
				arm2 += up * 2;
				arm2.normalize();
			}
			//if(arm[2]<0&&&arm[0]>0)
		}
		
		
		//2.move the arm2 long the direction
		joint = destination - arm2Length * arm2;
		//3.calculate direction of endpoint and joint
		arm1 = joint - endPoint;
		arm1.normalize();
		/*
		if (enableConstraint) {
			Vec3f v = arm1 * yaxes * yaxes;
			if (v[1] > 0 && largerThan901) {
				Vec3f dir = arm1 - v;
				if (dir.length() < constraint1) {
					dir.normalize();
					dir *= constraint1;
					arm1 = v + dir;
					arm1.normalize();
				}
			}
			else if (v[1] < 0 && !largerThan901) {
				Vec3f dir = arm1 - v;
				if (dir.length() > constraint1) {
					dir.normalize();
					dir *= constraint1;
					arm1 = v + dir;
					arm1.normalize();
				}
			}
			else if (v[1] > 0 && !largerThan901) {
				arm1 -= v * 2;
				Vec3f dir = arm1 - v;
				dir.normalize();
				dir *= constraint1;
				arm1 = v + dir;
				arm1.normalize();
			}
		}
		*/
		//4.move arm1 along the direction
		//test constraint2
		bool boom = false;
		/*
		if (enableConstraint) {
			float value = -arm1 * arm2;
			if (value > cosf(constraint2)) {
				Vec3f v= arm1^arm2;
				Vec3f dir = arm2^v;
				dir.normalize();
				arm1 = -arm2 + dir * sinf(constraint2);
				boom = true;
			}
		}
		*/
		Vec3f fakeEndPoint = joint - arm1Length * arm1;
		//5.move whole things back to end point without changing the angle
		Vec3f dir = fakeEndPoint - endPoint;
		joint -= dir;
		effector -= dir;
		/*
		if (enableConstraint && boom) {
			return calculateAngle();
		}
		*/
		//6.calculate the distance between effector and destination and check whether distance smaller than threshold,if yes,done
		float currentDistance = dir.length();
		if (currentDistance < THRESHOLD) {
			//cout << "effector:(" << effector[0] << "," << effector[1] << "," << effector[2] << ")" << endl;
			//cout << "destination:"<<destination[0] << "," << destination[1] << "," << destination[2] << endl;
			return calculateAngle();
		}
		//7.if 6 doesn`t satisfied,check angle constraints and processing time constraints, if satisfied ,go to step1
		//if no, return
	
	}

	return calculateAngle();
}

Vec4f InverseKinematics2::calculateAngle() {
	//cout << "(" << effector[0] << "," << effector[1] << "," << effector[2] << ")" << endl;
	Vec4f result;
	float pi = 3.1415926;

	Vec3f yaxes(0, 1, 0);
	Vec3f zaxes(0, 0, 1);
	
	Vec3f axeVec = (arm1 * yaxes) * yaxes; //the projection of arm1 on y axes
	
	Vec3f rotationVec = arm1 - axeVec;
	//Vec3f rotationPlaneVec
	//cout <<"rotation vector:"<< rotationVec[0] << "," << rotationVec[1] << "," << rotationVec[2] << endl;

	Vec3f planeVec1(0, axeVec[1], rotationVec.length());

	if (axeVec.length() == 0)
		result[0] = -90;
	else
		result[0] = -(acosf(planeVec1 * (-yaxes)) / pi) * 180;

	if (rotationVec.length() == 0) 
		result[1] = 0;
	else 
		result[1] = (acosf((zaxes * rotationVec) / rotationVec.length()) / pi) * 180;

	if (arm1[0] < 0)
		result[1] = -result[1];
	//if (arm1[2] < 0)
		//result[0] = -result[0];

	axeVec = (arm2 * yaxes) * yaxes;
	rotationVec = arm2 - axeVec;
	Vec3f planeVec2(0, axeVec[1], rotationVec.length());
	
	if (rotationVec.length() == 0)
		result[3] = 0;
	else
		result[3] = (acosf((zaxes * rotationVec) / rotationVec.length()) / pi) * 180;
	//cout << result[3] << endl;
	if (axeVec.length() == 0)
		result[2] = -90;
	else
		result[2] = -(acosf(planeVec2 * (-yaxes)) / pi) * 180;

	if (arm2[0] < 0)
		result[3] = -result[3];
	//if (arm2[2] < 0)
		//result[2] = -result[2];


	return result;
}

void InverseKinematics2::setConstraint(bool value) {
	enableConstraint = value;
}

void InverseKinematics2::setConstraint1(float value) {
	float radian = value / 180 * 3.1415926;
	constraint1 = sinf(radian);
	if (value > 90)
		largerThan901 = true;
	else
		largerThan901 = false;
}

void InverseKinematics2::setConstraint2(float value) {
	constraint2 = value / 180 * 3.1415926;

	
}