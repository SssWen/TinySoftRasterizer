#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "geometry.h"
#include "model.h"
#include <algorithm>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
Model *model = NULL;
const int width = 200;
const int height = 200;

void line(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color) {
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
		int y = p0.y*(1. - t) + p1.y*t;
		if (steep) {
			image.set(y, x, color);
		}
		else {
			image.set(x, y, color);
		}
	}
}

// 连续的三角形线段
// void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
//     line(t0, t1, image, color);
//     line(t1, t2, image, color);
//     line(t2, t0, image, color);
// }

// 非连续线段
// void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) { 
//     // sort the vertices, t0, t1, t2 lower−to−upper (bubblesort!) 
//     if (t0.y>t1.y) std::swap(t0, t1); 
//     if (t0.y>t2.y) std::swap(t0, t2); 
//     if (t1.y>t2.y) std::swap(t1, t2); 
//     int total_height = t2.y-t0.y; 
//     for (int y=t0.y; y<=t1.y; y++) { 
//         int segment_height = t1.y-t0.y+1; 
//         float alpha = (float)(y-t0.y)/total_height; 
//         float beta  = (float)(y-t0.y)/segment_height; // be careful with divisions by zero 
//         Vec2i A = t0 + (t2-t0)*alpha; 
//         Vec2i B = t0 + (t1-t0)*beta; 
//         image.set(A.x, y, red); 
//         image.set(B.x, y, green); 
//     } 
// }

//// 暴力填充三角形 - 划分2个三角形对三角形进行栅格化
//void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
//	// sort the vertices, t0, t1, t2 lower−to−upper (bubblesort yay!) 
//	if (t0.y > t1.y) std::swap(t0, t1);
//	if (t0.y > t2.y) std::swap(t0, t2);
//	if (t1.y > t2.y) std::swap(t1, t2);
//	int total_height = t2.y - t0.y;
//	for (int y = t0.y; y <= t1.y; y++) {
//		int segment_height = t1.y - t0.y + 1;
//		float alpha = (float)(y - t0.y) / total_height;
//		float beta = (float)(y - t0.y) / segment_height; // be careful with divisions by zero 
//		Vec2i A = t0 + (t2 - t0)*alpha;
//		Vec2i B = t0 + (t1 - t0)*beta;
//		if (A.x > B.x) std::swap(A, B);
//		for (int j = A.x; j <= B.x; j++) {
//			image.set(j, y, color); // attention, due to int casts t0.y+i != A.y 
//		}
//	}
//	for (int y = t1.y; y <= t2.y; y++) {
//		int segment_height = t2.y - t1.y + 1;
//		float alpha = (float)(y - t0.y) / total_height;
//		float beta = (float)(y - t1.y) / segment_height; // be careful with divisions by zero 
//		Vec2i A = t0 + (t2 - t0)*alpha;
//		Vec2i B = t1 + (t2 - t1)*beta;
//		if (A.x > B.x) std::swap(A, B);
//		for (int j = A.x; j <= B.x; j++) {
//			image.set(j, y, color); // attention, due to int casts t0.y+i != A.y 
//		}
//	}
//}

//// 暴力填充三角形 - 优化2
//void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
//	if (t0.y == t1.y && t0.y == t2.y) return; // i dont care about degenerate triangles
//	if (t0.y > t1.y) std::swap(t0, t1);
//	if (t0.y > t2.y) std::swap(t0, t2);
//	if (t1.y > t2.y) std::swap(t1, t2);
//	int total_height = t2.y - t0.y;
//	for (int i = 0; i < total_height; i++) {
//		bool second_half = i > t1.y - t0.y || t1.y == t0.y;
//		int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
//		float alpha = (float)i / total_height;
//		float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height; // be careful: with above conditions no division by zero here
//		Vec2i A = t0 + (t2 - t0)*alpha;
//		Vec2i B = second_half ? t1 + (t2 - t1)*beta : t0 + (t1 - t0)*beta;
//		if (A.x > B.x) std::swap(A, B);
//		for (int j = A.x; j <= B.x; j++) {
//			image.set(j, t0.y + i, color); // attention, due to int casts t0.y+i != A.y
//		}
//	}
//}


// 重心法1
Vec3f barycentric(Vec2i A, Vec2i B, Vec2i C, Vec2i P) {
	Vec3f s[2];
	for (int i = 2; i--; ) {
		s[i][0] = C[i] - A[i];
		s[i][1] = B[i] - A[i];
		s[i][2] = A[i] - P[i];
	}	

	Vec3f u = s[0] ^ s[1];
	if (std::abs(u[2]) > 0.01) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
		return Vec3f(1.f - (u.x + u.y) / u.z, u.x / u.z, u.y / u.z);
	return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(Vec2i *pts, TGAImage &image, TGAColor color) {
	Vec2i bboxmin(image.get_width() - 1, image.get_height() - 1);
	Vec2i bboxmax(0, 0);
	Vec2i clamp(image.get_width() - 1, image.get_height() - 1);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			bboxmin[j] = std::max(0, std::min(bboxmin[j], pts[i][j]));
			bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
		}
	}
	Vec2i P;
	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts[0],pts[1],pts[2],P);
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
			image.set(P.x, P.y, color);
		}
	}
}

int main(int argc, char** argv) {
	TGAImage frame(200, 200, TGAImage::RGB);
	Vec2i pts[3] = { Vec2i(10,10), Vec2i(100, 30), Vec2i(190, 160) };
	triangle(pts, frame, TGAColor(255, 0, 0,255));
	frame.flip_vertically(); // to place the origin in the bottom left corner of the image 	
	frame.write_tga_file("../../Output/DrawtrianglesBarycentric.tga");
	return 0;
}
