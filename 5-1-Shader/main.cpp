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

Vec3f light_dir(1,1,1);
Matrix4x4<float> ViewPort;
Matrix4x4<float> View;
Matrix4x4<float> proj;
// struct 默认访问权限是public,class是private
struct Shader {
	~Shader() {}
	Vec2i varying_uv[3];	
	Vec3i vertex(int iface, int nthvert) {		
		// uv坐标在顶点着色器只进行保存
		varying_uv[nthvert] = model->uv(iface, nthvert); 
		Vec3f v = model->vert(iface, nthvert);		
		Matrix1x4<float> pos(v);
		// mvp * viewport 矩阵
		pos = pos * View* proj*ViewPort; 
		Vec3i gl_Position = Vec3i(pos.m11 / pos.m14, pos.m12 / pos.m14, pos.m13 / pos.m14);
		return gl_Position;
	}
	/*
		barycentry 传入frag，暂时在frag内进行插值uv
	*/
	bool fragment(Vec3f barycentry, TGAColor &color) {
		Vec2i uv = varying_uv[0] * barycentry.x + varying_uv[1] * barycentry.y + varying_uv[2] * barycentry.z;
		float inty = dotProduct(model->normal(uv), light_dir.normalize());
		color = model->diffuse(uv)*inty;
		bool discard = false; // 默认不进行剔除
		return discard;
	}
};

// 重心坐标的特别版本,正常可以通过面积法 求出u.xyz,这里是特殊处理
Vec3f barycentric(Vec3i A, Vec3i B, Vec3i C, Vec3i P) {
	Vec3f u = Vec3f(C.x - A.x, B.x - A.x, A.x - P.x) ^ Vec3f(C.y - A.y, B.y - A.y, A.y - P.y);
	return std::abs(u.z) > .5 ? Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z) : Vec3f(-1, 1, 1); 
}

void triangle(Vec3i *pts, Shader shader, TGAImage &image,TGAImage &zbuffer) {
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
			// 对深度进行插值
			P.z = std::max(0, std::min(255, int(pts[0].z*bc_screen.x + pts[1].z*bc_screen.y + pts[2].z*bc_screen.z + .5)));						
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z<0 || zbuffer.get(P.x, P.y)[0]>P.z) continue;			
			// UV进行插值
			bool discard = shader.fragment(bc_screen, color);
			// 默认不进行discard
			if (!discard)
			{
				zbuffer.set(P.x, P.y, TGAColor(P.z));
				image.set(P.x, P.y, color);
			}									
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

	View = lookat(eye, center, Vec3f(0, 1, 0));
	proj = projection();	
	
	ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);	
	
	Shader shader;
	for (int i = 0; i < model->nfaces(); i++)
	{
		std::vector<int> face = model->face(i);
		Vec3i screen_coords[3];		
		for (int j = 0; j < 3; j++) {
			// 对每个顶点进行坐标变换操作
			screen_coords[j] = shader.vertex(i, j);  
		}
		triangle(screen_coords, shader,image, zbuffer);
		
	}
    image.flip_vertically(); 
	image.write_tga_file("../../Output/Shader.tga");
	zbuffer.flip_vertically(); 
	zbuffer.write_tga_file("../../Output/zbufferTest.tga");
	std::cout << "输出图片完成！" << std::endl;
    delete model;
    return 0;
}

