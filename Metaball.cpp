#include "Metaball.h"
#include <FL/gl.h>
#include <iostream>
using namespace std;
Metaball::Metaball(int gridSize) :grid(gridSize) {
	isInitialized = false;
}

void Metaball::addBalls(ball ball) {
	balls.push_back(ball);
	isInitialized = false;
}

void Metaball::drawMetaball() {
	if (!isInitialized) {
		isInitialized = true;
		for (int i = 0; i < balls.size(); i++) {
			float radius2 = balls[i].radius * balls[i].radius;
			float ballx = balls[i].position[0];
			float bally = balls[i].position[1];
			float ballz = balls[i].position[2];
			//cout << "hello" << endl;
			for (int k = 0; k < grid.numVertices; k++) {
				vertex* currentVertex = &grid.vertices[k];
				float x = currentVertex->position[0];
				float y = currentVertex->position[1];
				float z = currentVertex->position[2];
				float distance2 = (x - ballx) * (x - ballx) + (y - bally) * (y - bally) + (z - ballz) * (z - ballz);

				if (distance2 == 0)
					distance2 = 0.00001;

				currentVertex->value += radius2 / distance2;
				//cout << currentVertex->value << endl;
				float factor = radius2 / (distance2*distance2);
				//cubeGrid.vertices[j].normal+=ballToPoint*normalScale;
				currentVertex->normal[0] += (x - ballx) * factor;
				currentVertex->normal[1] += (y - bally) * factor;
				currentVertex->normal[2] += (z - ballz) * factor;
			}
		}
	}

	
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_NORMALIZE);
	grid.drawSurface(1.0);

}