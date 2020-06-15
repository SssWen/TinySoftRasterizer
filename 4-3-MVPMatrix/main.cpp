#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "tgaimage.h"
#include "model.h"

#include "tinymathlib.h"
#include <algorithm>
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor yellow   = TGAColor(255, 255,   0,   255);
const TGAColor blue   = TGAColor(255, 0, 255,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;
const int depth = 255;
Vec3f eye(-1, -1, -3);
Vec3f center(0, 0, 0);
TGAImage zbuffer(width, height, TGAImage::GRAYSCALE); // Zbuffer改成图片存放

Vec3f light_dir(0,0,-1);


// 重心坐标的特别版本,正常可以通过面积法 求出u.xyz,这里是特殊处理
Vec3f barycentric(Vec3i A, Vec3i B, Vec3i C, Vec3i P) {
	Vec3f u = Vec3f(C.x - A.x, B.x - A.x, A.x - P.x) ^ Vec3f(C.y - A.y, B.y - A.y, A.y - P.y);
	return std::abs(u.z) > .5 ? Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z) : Vec3f(-1, 1, 1); 
}

void triangle(Vec3i *pts, TGAImage &image,TGAImage &zbuffer,Vec2i *varying_uv) {
	Vec2i bboxmin(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
	Vec2i bboxmax(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			bboxmin[j] = std::min(bboxmin[j], pts[i][j]);
			bboxmax[j] = std::max(bboxmax[j], pts[i][j]);
		}
	}
	Vec3i P;
	TGAColor color;
	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
			//if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;			
			
			// 对深度进行插值
			P.z = std::max(0, std::min(255, int(pts[0].z*bc_screen.x + pts[1].z*bc_screen.y + pts[2].z*bc_screen.z + .5)));
						
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z<0 || zbuffer.get(P.x, P.y)[0]>P.z) continue;
			
			// UV进行插值
			Vec2i uv = varying_uv[0] * bc_screen.x + varying_uv[1] * bc_screen.y + varying_uv[2] * bc_screen.z;						
			color = model->diffuse(uv); //*inty;
			zbuffer.set(P.x, P.y, TGAColor(P.z));
			image.set(P.x, P.y, color);
			
		}
	}
}

Vec3f world2screen(Vec3f v) {
	return Vec3f(int((v.x + 1.)*width / 2. + .5), int((v.y + 1.)*height / 2. + .5), v.z);
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

// zaxis = normal(cameraTarget - cameraPosition)
// xaxis = normal(cross(cameraUpVector, zaxis))
// yaxis = cross(zaxis, xaxis)

//  xaxis.x           yaxis.x           zaxis.x          0
//  xaxis.y           yaxis.y           zaxis.y          0
//  xaxis.z           yaxis.z           zaxis.z          0
// -dot(xaxis, cameraPosition)  -dot(yaxis, cameraPosition)  -dot(zaxis, cameraPosition)  1

 Matrix4x4<float> lookat(Vec3f cameraPosition, Vec3f cameraTarget, Vec3f up) {
	Vec3f z = (cameraTarget-cameraPosition).normalize();
	Vec3f x = (up^z).normalize();
	Vec3f y = (z^x).normalize();	
	Matrix4x4<float> res = CreateIdentityMatrix4x4<float>();
	res.m11 = x.x;
	res.m21 = x.y;
	res.m31 = x.z;

	res.m12 = y.x;
	res.m22 = y.y;
	res.m32 = y.z;

	res.m13 = z.x;
	res.m23 = z.y;
	res.m33 = z.z;
	
	//res.m41 = -dotProduct(x, cameraPosition);
	//res.m42 = -dotProduct(y, cameraPosition);
	//res.m43 = -dotProduct(z, cameraPosition);

	// 匹配投影矩阵
	res.m41 = 0;
	res.m42 = 0;
	res.m43 = 0; 
	
	return res;
}

// 这里假设camera的Z方向平行world坐标系的Z方向，投影矫正简化版本
Matrix4x4<float> projection()
{
	Matrix4x4<float> proj = CreateIdentityMatrix4x4<float>();
	//proj.m34 = -1.f / (eye.z-center.z);
	proj.m34 = -1.f / 3;
	return proj;
}

int main(int argc, char** argv) {
    
	model = new Model("../../obj/african_head.obj");
   
    TGAImage image(width, height, TGAImage::RGB);    

	Matrix View = lookat(eye, center, Vec3f(0, 1, 0));
	Matrix4x4<float> proj = projection();	
	
	Matrix4x4<float> ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
	Vec2i varying_uv[3];
	// 先projection 再 viewport
	
	for (int i = 0; i < model->nfaces(); i++)
	{
		std::vector<int> face = model->face(i);
		Vec3i screen_coords[3];
		//Vec3f world_coords[3];
		for (int j = 0; j < 3; j++) {
			Vec3f v_point = model->vert(face[j]);			
			Matrix1x4<float> v(v_point);
			//v = v * proj*ViewPort;// 假设世界坐标和模型坐标中心重合，去掉世界坐标变换
			//v = v * View;
			v = v * View* proj*ViewPort;// 假设世界坐标和模型坐标中心重合，去掉世界坐标变换
		/*	if (i >= 38)
				std::cout << "i =" << i << " " << v.m14 << std::endl;*/
			screen_coords[j] = Vec3i(v.m11 / v.m14, v.m12 / v.m14, v.m13 / v.m14);
		
			//screen_coords[j] = Vec3i(v.m11, v.m12 , v.m13 );
						
		
			//world_coords[j] = v_point;
			varying_uv[j] = model->uv(i, j);
		}
		//std::cout << " " << i << std::endl;
		triangle(screen_coords, image, zbuffer, varying_uv);
		/*Vec3f n = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
		n = n.normalize();
		float intensity = dotProduct(n ,light_dir);
		if (intensity > 0) {

		}*/
		//triangle(screen_coords, image, zbuffer, varying_uv);
	}

    image.flip_vertically(); 
	image.write_tga_file("../../Output/MVPMatrix.tga");
	zbuffer.flip_vertically(); 
	zbuffer.write_tga_file("../../Output/zbufferTest.tga");
	std::cout << "输出图片完成！" << std::endl;
    delete model;
    return 0;
}

