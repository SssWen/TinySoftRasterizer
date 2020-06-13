#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "tgaimage.h"
#include "model.h"

#include "tinymathlib.h"
#include <algorithm>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor yellow   = TGAColor(255, 255,   0,   255);
const TGAColor blue   = TGAColor(255, 0, 255,   255);
Model *model = NULL;
const int width  = 100;
const int height = 100;
const int depth = 255;

void line(Vec3f p0, Vec3f p1, TGAImage &image, TGAColor color) {
	bool steep = false;
	if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y)) {
		std::swap(p0.x, p0.y);
		std::swap(p1.x, p1.y);
		steep = true;
	}
	if (p0.x > p1.x) {
		std::swap(p0, p1);
	}

	for (int x = p0.x; x <= p1.x; x++) {
		float t = (x - p0.x) / (float)(p1.x - p0.x);
		int y = p0.y*(1. - t) + p1.y*t + .5;
		if (steep) {
			image.set(y, x, color);
		}
		else {
			image.set(x, y, color);
		}
	}
}




Matrix4x4<float> translation(Vec3f v)
{
	return CreateTranslationMatrix4x4<float>(v.x, v.y, v.z);
}

// viewport 
Matrix4x4<float> viewport(int x,int y,int w, int h)
{
	Matrix4x4<float> m = CreateIdentityMatrix4x4<float>();
	m.m41 = x + w / 2.0f;
	m.m42 = y + h / 2.0f;
	m.m43 = depth / 2.0f;

	m.m11 = w / 2.0f;
	m.m22 = h / 2.0f;
	m.m33 = depth / 2.0f;
	return m;
}

int main(int argc, char** argv) {
    
	model = new Model("../../obj/african_head.obj");
   
    TGAImage image(width, height, TGAImage::RGB);
    
	// 绘制坐标轴
	
	Matrix VP = viewport(0, 0, width, height);
	{ 
		// 第一象限
		Matrix1x4<float> rowX(1, 0, 0, 1);
		rowX = rowX * VP;
		Matrix1x4<float> rowO(0, 0, 0, 1);
		rowO = rowO * VP;
		Matrix1x4<float> rowY(0, 1, 0, 1);
		rowY = rowY * VP;
		Vec3f x(rowX.m11, rowX.m12, rowX.m13);
		Vec3f o(rowO.m11, rowO.m12, rowO.m13);
		Vec3f y(rowY.m11, rowY.m12, rowY.m13);

		line(o, x, image, red);
		line(o, y, image, white);

		// 第三象限 
		Matrix1x4<float> NegRowX(-1, 0, 0, 1);
		NegRowX = NegRowX * VP;
		
		Matrix1x4<float> NegRowY(0, -1, 0, 1);
		NegRowY = NegRowY * VP;
		Vec3f x1(NegRowX.m11, NegRowX.m12, NegRowX.m13);
		
		Vec3f y1(NegRowY.m11, NegRowY.m12, NegRowY.m13);
		
		line(o, x1, image, yellow);
		line(o, y1, image, blue);
	}


    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image    
	image.write_tga_file("../../Output/TransfromationAxis.tga");
    delete model;
    return 0;
}

