#pragma once
// Our Matrix is RxC (rows by columns)
// An r x n matrix may be multiplied by an n x c matrix and the result is an r x c matrix.
// For now we only support 2x2, 3x3 and 4x4 matrices, so basically you can only multiply them by themselves.

// This library uses ROW VECTORS 


#include <sstream>
#include <math.h>
#include <cassert>

// Forward Declaration
template<typename T>
class Matrix2x2;

constexpr auto PI = 3.141592f;

template<typename T>
class Matrix4x4
{
public:
	Matrix4x4() : m11(0), m12(0), m13(0), m14(0), m21(0), m22(0), m23(0), m24(0), m31(0), m32(0), m33(0), m34(0), m41(0), m42(0), m43(0), m44(0)
	{
	}
	Matrix4x4(T _m11, T _m12, T _m13, T _m14, T _m21, T _m22, T _m23, T _m24, T _m31, T _m32, T _m33, T _m34, T _m41, T _m42, T _m43, T _m44)
		: m11(_m11), m12(_m12), m13(_m13), m14(_m14), m21(_m21), m22(_m22), m23(_m23), m24(_m24), m31(_m31), m32(_m32), m33(_m33), m34(_m34), m41(_m41), m42(_m42), m43(_m43), m44(_m44)
	{
	}
	Matrix4x4<T> transpose()
	{
		return Matrix4x4(m11, m21, m31, m41, m12, m22, m32, m42, m13, m23, m33, m43, m14, m24, m34, m44);
	}
	Matrix4x4<T> operator*(T scalar)
	{
		return Matrix4x4(m11 * scalar, m12 * scalar, m13 * scalar, m14 * scalar, m21 * scalar, m22 * scalar, m23 * scalar, m24 * scalar, m31 * scalar, m32 * scalar, m33 * scalar, m34 * scalar, m41 * scalar, m42 * scalar, m43 * scalar, m44 * scalar);
	}
	Matrix4x4<T> operator*(Matrix4x4<T> matrix)
	{
		return Matrix4x4(
			(m11 * matrix.m11 + m12 * matrix.m21 + m13 * matrix.m31 + m14 * matrix.m41),
			(m11 * matrix.m12 + m12 * matrix.m22 + m13 * matrix.m32 + m14 * matrix.m42),
			(m11 * matrix.m13 + m12 * matrix.m23 + m13 * matrix.m33 + m14 * matrix.m43),
			(m11 * matrix.m14 + m12 * matrix.m24 + m13 * matrix.m34 + m14 * matrix.m44),

			(m21 * matrix.m11 + m22 * matrix.m21 + m23 * matrix.m31 + m24 * matrix.m41),
			(m21 * matrix.m12 + m22 * matrix.m22 + m23 * matrix.m32 + m24 * matrix.m42),
			(m21 * matrix.m13 + m22 * matrix.m23 + m23 * matrix.m33 + m24 * matrix.m43),
			(m21 * matrix.m14 + m22 * matrix.m24 + m23 * matrix.m34 + m24 * matrix.m44),

			(m31 * matrix.m11 + m32 * matrix.m21 + m33 * matrix.m31 + m34 * matrix.m41),
			(m31 * matrix.m12 + m32 * matrix.m22 + m33 * matrix.m32 + m34 * matrix.m42),
			(m31 * matrix.m13 + m32 * matrix.m23 + m33 * matrix.m33 + m34 * matrix.m43),
			(m31 * matrix.m14 + m32 * matrix.m24 + m33 * matrix.m34 + m34 * matrix.m44),

			(m41 * matrix.m11 + m42 * matrix.m21 + m43 * matrix.m31 + m44 * matrix.m41),
			(m41 * matrix.m12 + m42 * matrix.m22 + m43 * matrix.m32 + m44 * matrix.m42),
			(m41 * matrix.m13 + m42 * matrix.m23 + m43 * matrix.m33 + m44 * matrix.m43),
			(m41 * matrix.m14 + m42 * matrix.m24 + m43 * matrix.m34 + m44 * matrix.m44)
		);
	}
public:
	T	m11, m12, m13, m14, 
		m21, m22, m23, m24, 
		m31, m32, m33, m34, 
		m41, m42, m43, m44;
};

template<typename T>
class Matrix3x3
{
public:
	Matrix3x3() : m11(0), m12(0), m13(0), m21(0), m22(0), m23(0), m31(0), m32(0), m33(0)
	{
	}
	Matrix3x3(T _m11, T _m12, T _m13, T _m21, T _m22, T _m23, T _m31, T _m32, T _m33)
		: m11(_m11), m12(_m12), m13(_m13), m21(_m21), m22(_m22), m23(_m23), m31(_m31), m32(_m32), m33(_m33)
	{
	}
	// Again, should I return or operate on myself??
	Matrix3x3<T> transpose()
	{
		return Matrix3x3(m11, m21, m31, m12, m22, m32, m13, m23, m33);
	}
	Matrix3x3<T> operator*(T scalar)
	{
		return Matrix3x3(
			m11 * scalar, m12 * scalar, m13 * scalar,
			m21 * scalar, m22 * scalar, m23 * scalar,
			m31 * scalar, m32 * scalar, m33 * scalar);
	}
	Matrix3x3<T> operator*(Matrix3x3<T> matrix)
	{
		return Matrix3x3(
			(m11 * matrix.m11 + m12 * matrix.m21 + m13 * matrix.m31),
			(m11 * matrix.m12 + m12 * matrix.m22 + m13 * matrix.m32),
			(m11 * matrix.m13 + m12 * matrix.m23 + m13 * matrix.m33),

			(m21 * matrix.m11 + m22 * matrix.m21 + m23 * matrix.m31),
			(m21 * matrix.m12 + m22 * matrix.m22 + m23 * matrix.m32),
			(m21 * matrix.m13 + m22 * matrix.m23 + m23 * matrix.m33),

			(m31 * matrix.m11 + m32 * matrix.m21 + m33 * matrix.m31),
			(m31 * matrix.m12 + m32 * matrix.m22 + m33 * matrix.m32),
			(m31 * matrix.m13 + m32 * matrix.m23 + m33 * matrix.m33)
		);
	}
	Matrix3x3<T> operator/(T scalar)
	{
		return Matrix3x3(m11 / scalar, m12 / scalar, m13 / scalar, m21 / scalar, m22 / scalar, m23 / scalar, m31 / scalar, m32 / scalar, m33 / scalar);
	}
	T determinant()
	{
		return m11 * (m22 * m33 - m23 * m32) + m12 * (m23 * m31 - m21 * m33) + m13 * (m21 * m32 - m22 * m31);
	}
	T minor(unsigned rowIndex, unsigned columnIndex)
	{
		Matrix2x2<T> subMatrix;
		if (rowIndex == 1)
		{
			if (columnIndex == 1)
			{
				subMatrix = Matrix2x2<T>(m22, m23, m32, m33);
			}
			else if (columnIndex == 2)
			{
				subMatrix = Matrix2x2<T>(m21, m23, m31, m33);
			}
			else if (columnIndex == 3)
			{
				subMatrix = Matrix2x2<T>(m21, m22, m31, m32);
			}
			else
			{
				// Handle error
			}
		}
		else if (rowIndex == 2)
		{
			if (columnIndex == 1)
			{
				subMatrix = Matrix2x2<T>(m12, m13, m32, m33);
			}
			else if (columnIndex == 2)
			{
				subMatrix = Matrix2x2<T>(m11, m13, m31, m33);
			}
			else if (columnIndex == 3)
			{
				subMatrix = Matrix2x2<T>(m11, m12, m31, m32);
			}
			else
			{
				// Handle error
			}
		}
		else if (rowIndex == 3)
		{
			if (columnIndex == 1)
			{
				subMatrix = Matrix2x2<T>(m12, m13, m22, m23);
			}
			else if (columnIndex == 2)
			{
				subMatrix = Matrix2x2<T>(m11, m13, m21, m23);
			}
			else if (columnIndex == 3)
			{
				subMatrix = Matrix2x2<T>(m11, m12, m21, m22);
			}
			else
			{
				// Handle error
			}
		}
		else
		{
			// Handle error
		}
		return subMatrix.determinant();
	}
	T cofactor(unsigned rowIndex, unsigned columnIndex)
	{
		return pow(-1, rowIndex + columnIndex) * minor(rowIndex, columnIndex);
	}
	Matrix3x3<T> adjoint()
	{
		return Matrix3x3<T>(
			cofactor(1, 1), cofactor(1, 2), cofactor(1, 3),
			cofactor(2, 1), cofactor(2, 2), cofactor(2, 3),
			cofactor(3, 1), cofactor(3, 2), cofactor(3, 3)
			);
	}
	Matrix3x3<T> classicalAdjoint()
	{		
		Matrix3x3<float> temp = adjoint();
		return temp.transpose();
	}
	Matrix3x3<T> inverse()
	{
		Matrix3x3<float> temp = classicalAdjoint();
		return temp / determinant();
	}
	Matrix4x4<T> extend()
	{
		return Matrix4x4<T>(
			m11, m12, m13, 0,
			m21, m22, m23, 0,
			m31, m32, m33, 0,
			0, 0, 0, 1
			);
	}
public:
	T m11, m12, m13, m21, m22, m23, m31, m32, m33;
};

template<typename T>
class Matrix2x2
{
public:
	Matrix2x2() : m11(0), m12(0), m21(0), m22(0)
	{
	}
	Matrix2x2(T _m11, T _m12, T _m21, T _m22)
		: m11(_m11), m12(_m12), m21(_m21), m22(_m22)
	{
	}
	Matrix2x2<T> transpose()
	{
		return Matrix2x2(m11, m21, m12, m22);
	}
	Matrix2x2<T> operator*(T scalar)
	{
		return Matrix2x2(m11 * scalar, m12 * scalar, m21 * scalar, m22 * scalar);
	}
	Matrix2x2<T> operator*(Matrix2x2<T> matrix)
	{
		return Matrix2x2((m11 * matrix.m11 + m12 * matrix.m21), (m11 * matrix.m12 + m12 * matrix.m22), (m21 * matrix.m11 + m22 * matrix.m21), (m21 * matrix.m12 + m22 * matrix.m22));
	}
	T determinant()
	{
		return (m11 * m22) - (m12 * m21);
	}
public:
	T m11, m12, m21, m22;
};
template<typename T>
class Matrix1x4
{
public:
	Matrix1x4() : m11(0), m12(0), m13(0), m14(0)
	{
	}
	Matrix1x4(T _m11, T _m12, T _m21, T _m22)
		: m11(_m11), m12(_m12), m13(_m21), m14(_m22)
	{
	}
	//Matrix1x4<T> transpose()
	//{
	//	return Matrix2x2(m11, m21, m12, m22);
	//}
	Matrix1x4<T> operator*(T scalar)
	{
		return Matrix2x2(m11 * scalar, m12 * scalar, m13 * scalar, m14 * scalar);
	}
	Matrix1x4<T> operator*(Matrix4x4<T> matrix)
	{
		return Matrix1x4(
			(m11 * matrix.m11 + m12 * matrix.m21 + m13 * matrix.m31 + m14 * matrix.m41),
			(m11 * matrix.m12 + m12 * matrix.m22 + m13 * matrix.m32 + m14 * matrix.m42),
			(m11 * matrix.m13 + m12 * matrix.m23 + m13 * matrix.m33 + m14 * matrix.m43),
			(m11 * matrix.m14 + m12 * matrix.m24 + m13 * matrix.m34 + m14 * matrix.m44),
			
		);
	}

public:
	T m11, m12, m13, m14;
};
template<typename T>
class Vector2
{
public:
	Vector2() : x(0), y(0)
	{
	}
	Vector2(T _x, T _y) : x(_x), y(_y)
	{
	}
	Vector2 operator-()
	{
		// Vector Negation
		return Vector2(-x, -y);
	}
	Vector2 operator*(T scalar) const
	{
		return Vector2(x * scalar, y * scalar);
	}
	friend Vector2<T> operator*(T scalar, Vector2<T> vector)
	{
		return Vector2(vector.x * scalar, vector.y * scalar);
	}
	Vector2 operator*(const Vector2& vector) const
	{
		// Vector Multiplcation by Vector
		return Vector2(x * vector.x, y * vector.y);
	}
	Vector2 operator*(const Matrix2x2<T>& matrix)
	{
		// Vector Multiplication by Matrix
		return Vector2((x * matrix.m11 + y * matrix.m21), (x * matrix.m12 + y * matrix.m22));
	}
	Vector2 operator/(T scalar) const
	{
		// Vector Division by Scalar
		return Vector2(x / scalar, y / scalar);
	}
	Vector2 operator/(const Vector2& vector) const
	{
		// Vector Division by Vector
		return Vector2(x / vector.x, y / vector.y);
	}
	Vector2 operator+(const Vector2& vector)
	{
		// Vector Addition
		return Vector2(x + vector.x, y + vector.y);
	}
	Vector2 operator-(const Vector2& vector) const
	{
		// Vector Substraction
		return Vector2(x - vector.x, y - vector.y);
	}
	T dotProduct(const Vector2& vector) const
	{
		// Dot Product
		return (x * vector.x) + (y * vector.y);
	}
	float length() const
	{
		// Vector Magnitude (length)
		// - why float and not double??
		return sqrt((x * x) + (y * y));
	}
	Vector2 normalize()
	{
		// Normalizing a Vector
		return *this / this->length();
	}
	T& operator[](const size_t i)  { assert(i < 2); return i <= 0 ? x : y; }
	friend std::ostream& operator<< (std::ostream& out, const Vector2& obj)
	{
		out << "[" << obj.x << ", " << obj.y << "]";
		return out;
	}
public:
	T x, y;
};

template<typename T>
class Vector3
{
public:
	Vector3() : x(0), y(0), z(0)
	{
	}
	Vector3(T _x, T _y, T _z) : x(_x), y(_y), z(_z)
	{
	}
	// Vector Negation
	Vector3 operator-()
	{
		return Vector3(-x, -y, -z);
	}
	// Vector Multiplicaton by Scalar
	Vector3 operator*(T scalar) const
	{
		return Vector3(x * scalar, y * scalar, z * scalar);
	}
	// Different Multiplication by Scalar (commutative property)
	friend Vector3<T> operator*(T scalar, Vector3<T> vector)
	{
		return Vector3(vector.x * scalar, vector.y * scalar, vector.z * scalar);
	}
	// Vector Multiplication by Vector
	Vector3 operator*(const Vector3& vector) const
	{
		return Vector3(x * vector.x, y * vector.y, z * vector.z);
	}
	// Vector Multiplication by Matrix3x3
	Vector3 operator*(const Matrix3x3<T>& matrix)
	{
		return Vector3((x * matrix.m11 + y * matrix.m21 + z * matrix.m31), (x * matrix.m12 + y * matrix.m22 + z * matrix.m32), (x * matrix.m13 + y * matrix.m23 + z * matrix.m33));
	}
	// Vector Division by Scalar
	Vector3 operator/(T scalar) const
	{
		return Vector3(x / scalar, y / scalar, z / scalar);
	}
	// Vector Division by Vector
	Vector3 operator/(const Vector3& vector) const
	{
		return Vector3(x / vector.x, y / vector.y, z / vector.z);
	}
	// Vector Addition
	Vector3 operator+(const Vector3& vector)
	{
		return Vector3(x + vector.x, y + vector.y, z + vector.z);
	}
	// Vector Substraction
	Vector3 operator-(const Vector3& vector) const
	{
		return Vector3(x - vector.x, y - vector.y, z - vector.z);
	}
	// Dot Product
	T dotProduct(const Vector3& vector)
	{
		return (x * vector.x) + (y * vector.y) + (z * vector.z);
	}
	// Cross Product
	Vector3 cross(const Vector3& vector) const
	{
		return Vector3((y * vector.z) - (z * vector.y), (z * vector.x) - (x * vector.z), (x * vector.y) - (y * vector.x));
	}
	// Vector Magnitude (length)
	// - why float and not double??
	float length()
	{
		return sqrt((x * x) + (y * y) + (z * z));
	}
	// Normalize a Vector
	// - do we return or operate on the vector itself??
	Vector3 normalize()
	{
		return *this / this->length();
	}

	T& operator[](const size_t i) { assert(i < 3); return i <= 0 ? x : (1 == i ? y : z); }

	friend std::ostream& operator<< (std::ostream& out, const Vector3& obj)
	{
		out << "[" << obj.x << ", " << obj.y << ", " << obj.z << "]";
		return out;
	}
public:
	T x, y, z;
};

template<typename T>
class Vector4
{
public:
	Vector4() : x(0), y(0), z(0), w(0)
	{
	}
	Vector4(T _x, T _y, T _z, T _w) : x(_x), y(_y), z(_z), w(_w)
	{
	}
	Vector4 operator-()
	{
		return Vector4(-x, -y, -z, -w);
	}
	Vector4 operator*(T scalar) const
	{
		return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
	}
	friend Vector4<T> operator*(T scalar, Vector4<T> vector)
	{
		return Vector4(vector.x * scalar, vector.y * scalar, vector.z * scalar, vector.w * scalar);
	}
	Vector4 operator*(const Vector4& vector) const
	{
		return Vector4(x * vector.x, y * vector.y, z * vector.z, w * vector.w);
	}
	Vector4 operator*(const Matrix4x4<T>& matrix)
	{
		return Vector4(
			(x * matrix.m11 + y * matrix.m21 + z * matrix.m31 + w * matrix.m41),
			(x * matrix.m12 + y * matrix.m22 + z * matrix.m32 + w * matrix.m42),
			(x * matrix.m13 + y * matrix.m23 + z * matrix.m33 + w * matrix.m43),
			(x * matrix.m14 + y * matrix.m24 + z * matrix.m34 + w * matrix.m44)
		);
	}
	Vector4 operator+(const Vector4& vector)
	{
		return Vector4(x + vector.x, y + vector.y, z + vector.z, w + vector.w);
	}
	T dotProduct(const Vector4& vector)
	{
		return (x * vector.x) + (y * vector.y) + (z * vector.z) + (w * vector.w);
	}

	T& operator[](const size_t i) {
		assert(i < 4);

		if (i == 0)
			return x;
		else if (i == 1)
			return y;
		else if (i == 2)
			return z;
		else if (i == 3)
			return w;
	}
	friend std::ostream& operator<< (std::ostream& out, const Vector4& obj)
	{
		out << "[" << obj.x << ", " << obj.y << ", " << obj.z << ", " << obj.w << "]";
		return out;
	}
public:
	T x, y, z, w;
};

// Converts Degrees to Radians
inline float convertToRadians(float degrees)
{
	return degrees * (PI / 180.0f);
}

inline float convertToDegrees(float radians)
{
	return radians * (180 / PI);
}

// Same thing here - do we return or operate on the vector??
template<typename T>
Vector2<T> normalize(const Vector2<T>& vector)
{
	return vector / vector.length();
}

template<typename T>
Vector3<T> normalize(const Vector3<T>& vector)
{
	return vector / vector.length();
}

template<typename T>
float length(const Vector2<T>& vector)
{
	return sqrt((vector.x * vector.x) + (vector.y * vector.y));
}

template<typename T>
float length(const Vector3<T>& vector)
{
	return sqrt((vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z));
}

template<typename T>
float distance(const Vector2<T>& a, const Vector2<T>& b)
{
	return length(b - a);
}

template<typename T>
float distance(const Vector3<T>& a, const Vector3<T>& b)
{
	return length(b - a);
}

// Calculates dot product for two 2D vectors.
// Dot product of two unit vectors is equal to the cosine of the angle between them.
// The dot product of two vectors is equal to the cosine of the angle between them, multiplied by the lenghts of them.
// The dot product measures the length of the projection of second vector onto the first one, multiplied by the lenght of the first one.
template<typename T>
T dotProduct(const Vector2<T>& a, const Vector2<T>& b)
{
	return (a.x * b.x) + (a.y * b.y);
}

// Calculates dot product for two 3D vectors.
// Dot product of two unit vectors is equal to the cosine of the angle between them.
// The dot product of two vectors is equal to the cosine of the angle between them, multiplied by the lenghts of them.
// The dot product measures the length of the projection of second vector onto the first one, multiplied by the lenght of the first one.
template<typename T>
T dotProduct(const Vector3<T>& a, const Vector3<T>& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

// Calculated cross product for two 3D vectors.
// Cross product yields a vector that is parpendicular to the orginal two vectors.
template<typename T>
Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b)
{
	return Vector3<T>((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x));
}

template<typename T>
Vector3<T> CreateZeroVector3D()
{
	return Vector3<T>(0, 0, 0);
}

template<typename T>
Vector2<T> CreateZeroVector2D()
{
	return Vector2<T>(0, 0);
}

template<typename T>
Matrix2x2<T> CreateIdentityMatrix2x2()
{
	return Matrix2x2<T>(
		1.0f, 0.0f,
		0.0f, 1.0f);
}

template<typename T>
Matrix3x3<T> CreateIdentityMatrix3x3()
{
	return Matrix3x3<T>(
		1.0f, 0.0f, 0.0f, 
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f);
}

template<typename T>
Matrix4x4<T> CreateIdentityMatrix4x4()
{
	return Matrix4x4<T>(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f, 
		0.0f, 0.0f, 1.0f, 0.0f, 
		0.0f, 0.0f, 0.0f, 1.0f);
}

// Consider counterclockwise rotation positive and clockwise rotation negative.
template<typename T>
Matrix2x2<T> CreateRotationMatrix(float angle)
{
	return Matrix2x2<T>(cos(angle), sin(angle), -sin(angle), cos(angle));
}

// This library is using left-hand rule for determining "positive" and "negative" rotation
template<typename T>
Matrix3x3<T> CreateRotationMatrix3x3AxisX(float angle)
{
	return Matrix3x3<T>(1, 0, 0, 0, cos(angle), sin(angle), 0, -sin(angle), cos(angle));
}

template<typename T>
Matrix3x3<T> CreateRotationMatrix3x3AxisY(float angle)
{
	return Matrix3x3<T>(cos(angle), 0, -sin(angle), 0, 1, 0, sin(angle), 0, cos(angle));
}

template<typename T>
Matrix3x3<T> CreateRotationMatrix3x3AxisZ(float angle)
{
	return Matrix3x3<T>(cos(angle), sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, 1);
}

template<typename T>
Matrix3x3<T> CreateRotationMatrix3x3(float angle, Vector3<T> unitAxis)
{
	return Matrix3x3<T>(
		(unitAxis.x * unitAxis.x) * (1 - cos(angle)) + cos(angle), (unitAxis.x * unitAxis.y) * (1 - cos(angle)) + (unitAxis.z * sin(angle)), (unitAxis.x * unitAxis.z) * (1 - cos(angle)) - (unitAxis.y * sin(angle)),
		(unitAxis.x * unitAxis.y) * (1 - cos(angle)) - (unitAxis.z * sin(angle)), (unitAxis.y * unitAxis.y) * (1 - cos(angle)) + cos(angle), (unitAxis.y * unitAxis.z) * (1 - cos(angle)) + (unitAxis.x * sin(angle)),
		(unitAxis.x * unitAxis.z) * (1 - cos(angle)) + (unitAxis.y * sin(angle)), (unitAxis.y * unitAxis.z) * (1 - cos(angle)) - (unitAxis.x * sin(angle)), (unitAxis.z * unitAxis.z) * (1 - cos(angle)) + cos(angle)
		);
}

template<typename T>
Matrix4x4<T> CreateRotationMatrix4x4AxisX(float angle)
{
	return Matrix4x4<T>(
		1, 0, 0, 0, 0, 
		cos(angle), sin(angle), 0, 0,
		-sin(angle), cos(angle),   
		0, 0, 0, 0, 1);
}

template<typename T>
Matrix4x4<T> CreateRotationMatrix4x4AxisY(float angle)
{
	return Matrix4x4<T>(cos(angle), 0, -sin(angle), 0, 0, 1, 0, 0, sin(angle), 0, cos(angle), 0, 0, 0, 0, 1);
}

template<typename T>
Matrix4x4<T> CreateRotationMatrix4x4AxisZ(float angle)
{
	return Matrix4x4<T>(cos(angle), sin(angle), 0, 0, -sin(angle), cos(angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
}

template<typename T>
Matrix4x4<T> CreateRotationMatrix4x4(float angle, Vector3<T> unitAxis)
{
	return Matrix4x4<T>(
		(unitAxis.x * unitAxis.x) * (1 - cos(angle)) + cos(angle), (unitAxis.x * unitAxis.y) * (1 - cos(angle)) + (unitAxis.z * sin(angle)), (unitAxis.x * unitAxis.z) * (1 - cos(angle)) - (unitAxis.y * sin(angle)), 0,
		(unitAxis.x * unitAxis.y) * (1 - cos(angle)) - (unitAxis.z * sin(angle)), (unitAxis.y * unitAxis.y) * (1 - cos(angle)) + cos(angle), (unitAxis.y * unitAxis.z) * (1 - cos(angle)) + (unitAxis.x * sin(angle)), 0,
		(unitAxis.x * unitAxis.z) * (1 - cos(angle)) + (unitAxis.y * sin(angle)), (unitAxis.y * unitAxis.z) * (1 - cos(angle)) - (unitAxis.x * sin(angle)), (unitAxis.z * unitAxis.z) * (1 - cos(angle)) + cos(angle), 0,
		0, 0, 0, 1
		);
}

template<typename T>
Matrix2x2<T> CreateScaleMatrix2x2(float scaleX, float scaleY)
{
	return Matrix2x2<T>(scaleX, 0, 0, scaleY);
}

template<typename T>
Matrix3x3<T> CreateScaleMatrix3x3(float scaleX, float scaleY, float scaleZ)
{
	return Matrix3x3<T>(
		scaleX, 0, 0,
		0, scaleY, 0,
		0, 0, scaleZ
		);
}

template<typename T>
Matrix4x4<T> CreateScaleMatrix4x4(float scaleX, float scaleY, float scaleZ)
{
	return Matrix4x4<T>(
		scaleX, 0, 0, 0,
		0, scaleY, 0, 0,
		0, 0, scaleZ, 0,
		0, 0, 0, 1
		);
}

template<typename T>
Matrix2x2<T> CreateScaleMatrix2x2(float scale, Vector2<T> direction)
{
	return Matrix2x2<T>(
		1 + (scale - 1) * direction.x * direction.x, (scale - 1) * direction.x * direction.y,
		(scale - 1) * direction.x * direction.y, 1 + (scale - 1) * direction.y * direction.y
		);
}

template<typename T>
Matrix3x3<T> CreateScaleMatrix3x3(float scale, Vector3<T> direction)
{
	return Matrix3x3<T>(
		1 + (scale - 1) * direction.x * direction.x, (scale - 1) * direction.x * direction.y, (scale - 1) * direction.x * direction.z,
		(scale - 1) * direction.y * direction.x, 1 + (scale - 1) * direction.y * direction.y, (scale - 1) * direction.y * direction.z,
		(scale - 1) * direction.z * direction.x, (scale - 1) * direction.z * direction.y, 1 + (scale - 1) * direction.z * direction.z
		);
}

template<typename T>
Matrix4x4<T> CreateTranslationMatrix4x4(float dx, float dy, float dz)
{
	return Matrix4x4<T>(
		1, 0, 0, 0,
		0, 1, 0, 0, 
		0, 0, 1, 0, 
		dx, dy, dz, 1);
}

template<typename T>
Matrix2x2<T> CreateOrtographicProjectionMatrixX2x2()
{
	return Matrix2x2<T>(1, 0, 0, 0);
}

template<typename T>
Matrix2x2<T> CreateOrtographicProjectionMatrixY2x2()
{
	return Matrix2x2<T>(0, 0, 0, 1);
}

template<typename T>
Matrix2x2<T> CreateOrtographicProjectionMatrix2x2(Vector2<T> line)
{
	return Matrix2x2<T>(
		1 - line.x * line.x, -line.x * line.y,
		-line.x * line.y, 1 - line.y * line.y
		);
}

template<typename T>
Matrix3x3<T> CreateOrtographicProjectionMatrixXY3x3()
{
	return Matrix3x3<T>(1, 0, 0, 0, 1, 0, 0, 0, 0);
}

template<typename T>
Matrix3x3<T> CreateOrtographicProjectionMatrixXZ3x3()
{
	return Matrix3x3<T>(1, 0, 0, 0, 0, 0, 0, 0, 1);
}

template<typename T>
Matrix3x3<T> CreateOrtographicProjectionMatrixYZ3x3()
{
	return Matrix3x3<T>(
		0, 0, 0,
		0, 1, 0,
		0, 0, 1
		);
}

template<typename T>
Matrix3x3<T> CreateOrtographicProjectionMatrix3x3(Vector3<T> normal)
{
	return Matrix3x3<T>(
		+1 - normal.x * normal.x, -1 * normal.x * normal.y, -1 * normal.x * normal.z,
		-1 * normal.y * normal.x, +1 - normal.y * normal.y, -1 * normal.y * normal.z,
		-1 * normal.z * normal.x, -1 * normal.z * normal.y, +1 - normal.z * normal.z
		);
}

template<typename T>
Matrix2x2<T> CreateReflectionMatrix2x2(Vector2<T> axis)
{
	return Matrix2x2<T>(
		1 - 2 * axis.x * axis.x, -2 * axis.x * axis.y,
		-2 * axis.x * axis.y, 1 - 2 * axis.y * axis.y
		);
}

template<typename T>
Matrix3x3<T> CreateReflectionMatrix3x3(Vector3<T> plane)
{
	return Matrix3x3<T>(
		1 - 2 * plane.x * plane.x, -2 * plane.y * plane.x, -2 * plane.z * plane.x,
		-2 * plane.x * plane.y, 1 - 2 * plane.y * plane.y, -2 * plane.z * plane.y,
		-2 * plane.x * plane.z, -2 * plane.y * plane.z, 1 - 2 * plane.z * plane.z
		);
}

template<typename T>
Matrix2x2<T> CreateShearingMatrixX2x2(T s)
{
	return Matrix2x2<T>(1, 0, s, 1);
}

template<typename T>
Matrix2x2<T> CreateShearingMatrixY2x2(T s)
{
	return Matrix2x2<T>(1, s, 0, 1);
}

template<typename T>
Matrix3x3<T> CreatingShearingMatrixXY3x3(T s, T t)
{
	return Matrix3x3<T>(
		1, 0, 0,
		0, 1, 0,
		s, t, 1
		);
}

template<typename T>
Matrix3x3<T> CreatingShearingMatrixXZ3x3(T s, T t)
{
	return Matrix3x3<T>(
		1, 0, 0,
		s, 1, t,
		0, 0, 1
		);
}

template<typename T>
Matrix3x3<T> CreatingShearingMatrixYZ3x3(T s, T t)
{
	return Matrix3x3<T>(
		1, s, t,
		0, 1, 0,
		0, 0, 1
		);
}

template<typename T>
Matrix4x4<T> CreateLookAtMatrix4x4(Vector3<T> position, Vector3<T> target, Vector3<T> up)
{
	Vector3<T> viewDirection = (target - position).normalize();
	Vector3<T> viewSide = viewDirection.cross(up).normalize();
	Vector3<T> viewUp = (viewSide.cross(viewDirection)).normalize();

	Matrix4x4<T> orientation(
		viewSide.x, viewUp.x, viewDirection.x, 0,
		viewSide.y, viewUp.y, viewDirection.y, 0,
		viewSide.z, viewUp.z, viewDirection.z, 0,
		0, 0, 0, 1
	);

	Matrix4x4<T> translation(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		-position.x, -position.y, -position.z, 1
	);

	// Somehow this order is very important, otherwise it won't work. Again, this has something to do with inverse/transposition/column-row stuff
	return (translation * orientation);
}

template<typename T>
Matrix4x4<T> CreateClipMatrixForOrtographicProjection()
{
	// The first goal of the clip matrix is to get the correct value into 'w'
	// such that the division caused the desired projection.
	// The projection doesn't take place during the multiplication by this matrix,
	// it happens when we divide 'x', 'y' and 'z' by 'w'.
	// For ortographic projection, 'w' ought to be '1'.
	return Matrix4x4<T>(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		);
}

template<typename T>
Matrix4x4<T> CreateClipMatrixForPerspectiveProjection()
{
	// The first goal of the clip matrix is to get the correct value into 'w'
	// such that the division caused the desired projection.
	// The projection doesn't take place during the multiplication by this matrix,
	// it happens when we divide 'x', 'y' and 'z' by 'w'.
	// For ortographic projection, 'w' ought to be 'z'.
	return Matrix4x4<T>(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 1,
		0, 0, 0, 0
		);
}


typedef Vector2<float> Vec2f;
typedef Vector2<int> Vec2i;

typedef Vector3<float> Vec3f;
typedef Vector3<int>   Vec3i;
typedef Vector4<float> Vec4f;
typedef Matrix4x4<float> Matrix;