#pragma once
#include "vec.h"
#include "MarchingCube.h"
#include <vector>
using namespace std;

class ball {
public:
	Vec3f position;
	float radius;
	ball(float x, float y, float z, float newRadius) {
		position[0] = x; position[1] = y; position[2] = z;
		radius = newRadius;
	}
};

class Metaball {
private:
	vector<ball> balls;
	grid grid;
	bool isInitialized;
public:
	Metaball(int gridSize);
	void addBalls(ball ball);
	void drawMetaball();
	void setBallPosition(int index, Vec3f newPosition);
};