#include "MarchingCube.h"
#include <FL/gl.h>
#include <math.h>
#include <iostream>
using namespace std;
grid::grid(int gridSize) {
	numVertices = pow(gridSize + 1, 3);
	numCubes = pow(gridSize, 3);
	vertices = new vertex[numVertices];
	cubes = new cube[numCubes];

	//initialize the position of the vertices
	int currentVertex = 0;
	for (float Z = 0.0; Z < gridSize + 1; Z++) {
		for (float Y = 0.0; Y < gridSize + 1; Y++) {
			for (float X = 0.0; X < gridSize + 1; X++) {
				vertices[currentVertex].normal.zeroElements();
				//cout << currentVertex << endl;
				//cout << X / gridSize << endl;
				//float* vec3 = vertices[currentVertex].position.getPointer();
				vertices[currentVertex].position[0] = (X / gridSize) * GRIDSIZE * 2 - GRIDSIZE;
				//cout << vertices[currentVertex].position[0] << endl;
				vertices[currentVertex].position[1] = (Y / gridSize) * GRIDSIZE * 2 - GRIDSIZE;
				vertices[currentVertex].position[2] = (Z / gridSize) * GRIDSIZE * 2 - GRIDSIZE;
				vertices[currentVertex].value = 0.0f;
				//cout << vertices[currentVertex].value << endl;
				//cout << "(" << vertices[currentVertex].position[0] << "," << vertices[currentVertex].position[1] << "," << vertices[currentVertex].position[2]<< ")" << endl;
				currentVertex++;
				//cout << "(" << (X / gridSize) * GRIDSIZE * 2 - GRIDSIZE << "," << (Y / gridSize) * GRIDSIZE * 2 - GRIDSIZE << "," << (Z / gridSize) * GRIDSIZE * 2 - GRIDSIZE << ")" << endl;
				
			}
		}
	}

	int currentCube = 0;
	int vertexSize = gridSize + 1;
	for (int Z = 1; Z <= gridSize; Z++) {
		for (int Y = 1; Y <= gridSize; Y++) {
			for (int X = 1; X <= gridSize; X++) {
				cubes[currentCube].vertices[0] = &vertices[(X - 1) + (Y - 1) * vertexSize + (Z - 1) * vertexSize * vertexSize];
				cubes[currentCube].vertices[1] = &vertices[X + (Y - 1) * vertexSize + (Z - 1) * vertexSize * vertexSize];
				cubes[currentCube].vertices[2] = &vertices[X + (Y - 1) * vertexSize + Z * vertexSize * vertexSize];
				cubes[currentCube].vertices[3] = &vertices[(X - 1) + (Y - 1) * vertexSize + Z * vertexSize * vertexSize];
				cubes[currentCube].vertices[4] = &vertices[(X - 1) + Y * vertexSize + (Z - 1) * vertexSize * vertexSize];
				cubes[currentCube].vertices[5] = &vertices[X + Y * vertexSize + (Z - 1) * vertexSize * vertexSize];
				cubes[currentCube].vertices[6] = &vertices[X + Y * vertexSize + Z * vertexSize * vertexSize];
				cubes[currentCube].vertices[7] = &vertices[(X - 1) + Y * vertexSize + Z * vertexSize * vertexSize];
				currentCube++;
			}
		}
	}
	/*
	cout << cubes[0].vertices[0]->position[1] << endl;
	cout << cubes[0].vertices[1]->position[1] << endl;
	cout << cubes[0].vertices[2]->position[1] << endl;
	cout << cubes[0].vertices[3]->position[1] << endl;
	cout << cubes[0].vertices[4]->position[1] << endl;
	cout << cubes[0].vertices[5]->position[1] << endl;
	cout << cubes[0].vertices[6]->position[1] << endl;
	cout << cubes[0].vertices[7]->position[1] << endl;
	*/
	
}

grid::~grid() {
	delete[]vertices;
	delete[]cubes;
}

void grid::drawSurface(float isolevel) {
	surfaceVertex onEdge[12];
	
	glBegin(GL_TRIANGLES);

	for (int i = 0; i < numCubes; i++) {
		
		int cubeindex = 0;
		if (cubes[i].vertices[0]->value < isolevel) cubeindex |= 1;
		if (cubes[i].vertices[1]->value < isolevel) cubeindex |= 2;
		if (cubes[i].vertices[2]->value < isolevel) cubeindex |= 4;
		if (cubes[i].vertices[3]->value < isolevel) cubeindex |= 8;
		if (cubes[i].vertices[4]->value < isolevel) cubeindex |= 16;
		if (cubes[i].vertices[5]->value < isolevel) cubeindex |= 32;
		if (cubes[i].vertices[6]->value < isolevel) cubeindex |= 64;
		if (cubes[i].vertices[7]->value < isolevel) cubeindex |= 128;

		if (cubeindex == 0 || cubeindex == 255) {
			//cout << cubeindex << endl;
			continue;

		}

		//cout << "triangle" << endl;
		int edges = edgeTable[cubeindex];
		int bit = 0;

		for (int n = 0; n < 12; n++) {
			bit = 1 << n;
			if (edges & bit) {
				vertex* p1 = cubes[i].vertices[endPoint[n * 2]];
				vertex* p2 = cubes[i].vertices[endPoint[n * 2 + 1]];
				//cout << endPoint[n * 2]<<","<<endPoint[n * 2 + 1] << endl;;
				//cout << p1->position[0] << "," << p2->position[0] << endl;
				float factor = (isolevel - p1->value) / (p2->value - p1->value);
				//cout << p2->value - p1->value << endl;
				onEdge[n].position[0] = p1->position[0] + factor * (p2->position[0] - p1->position[0]);
				//cout << p2->position[0] - p1->position[0] << endl;
				//cout << onEdge[n].position[0] << endl;
				//cout << factor << endl;
				onEdge[n].position[1] = p1->position[1] + factor * (p2->position[1] - p1->position[1]);
				onEdge[n].position[2] = p1->position[2] + factor * (p2->position[2] - p1->position[2]);

				onEdge[n].normal[0] = p1->normal[0] + factor * (p2->normal[0] - p1->normal[0]);
				onEdge[n].normal[1] = p1->normal[1] + factor * (p2->normal[1] - p1->normal[1]);
				onEdge[n].normal[2] = p1->normal[2] + factor * (p2->normal[2] - p1->normal[2]);
			}
		}
		
		for (int j = 0; triTable[cubeindex][j * 3] != -1; j++) {
			//cout << "hello" << endl;
			glNormal3fv(onEdge[triTable[cubeindex][j * 3]].normal.getPointer());
			glVertex3fv(onEdge[triTable[cubeindex][j * 3]].position.getPointer());
			//cout << onEdge[triTable[cubeindex][j * 3]].position.getPointer()[0] << endl;;
			glNormal3fv(onEdge[triTable[cubeindex][j * 3 + 1]].normal.getPointer());
			glVertex3fv(onEdge[triTable[cubeindex][j * 3 + 1]].position.getPointer());

			glNormal3fv(onEdge[triTable[cubeindex][j * 3 + 2]].normal.getPointer());
			glVertex3fv(onEdge[triTable[cubeindex][j * 3 + 2]].position.getPointer());
			
		}
		
	}

	glEnd();

}

