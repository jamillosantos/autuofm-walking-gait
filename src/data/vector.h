/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#ifndef WALKING_DATA_VECTOR_H
#define WALKING_DATA_VECTOR_H

namespace mote
{
namespace walking
{
namespace data
{

template<typename T>
class Vector2
{
public:
	T x;
	T y;

	Vector2()
		: x(0), y(0)
	{ }

	Vector2(T x, T y)
		: x(x), y(y)
	{ }

	Vector2(Vector2& vector)
		: x(vector.x), y(vector.y)
	{ }
};

template<typename T>
class Vector3
	: public Vector2<T>
{
public:
	T z;

	Vector3(T x, T y, T z)
		: Vector2<T>(x, y), z(z)
	{ }

	Vector3()
		: Vector2<T>(), z(0)
	{ }

	Vector3(Vector3<T> &vector)
		: Vector2<T>(vector), z(vector.z)
	{ }

	Vector3(Vector2<T> &vector)
		: Vector2<T>(vector), z(0)
	{ }

	void zero()
	{
		this->x = this->y = this->z = 0;
	}
};

typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;
}
}
}

#endif //WALKING_DATA_VECTOR_H