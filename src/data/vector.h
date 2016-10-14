/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#ifndef WALKING_DATA_VECTOR_H
#define WALKING_DATA_VECTOR_H

#include <json/json.h>

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

	virtual void zero()
	{
		this->x = this->y = 0;
	}

	virtual void fromJson(const Json::Value &json)
	{
		const Json::Value &xJson = json["x"];
		const Json::Value &yJson = json["y"];
		if (!xJson.isNull())
			this->x = xJson.asDouble();
		if (!yJson.isNull())
			this->y = yJson.asDouble();
	}
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

	virtual void zero() override
	{
		this->x = this->y = this->z = 0;
	}

	virtual void fromJson(const Json::Value &json) override
	{
		Vector2<T>::fromJson(json);
		const Json::Value &zJson = json["z"];
		this->z = zJson.asDouble();
	}
};

template <typename T>
class Vector2t:
	public Vector2<T>
{
public:
	T theta;

	Vector2t()
		: Vector2<T>()
	{ }

	Vector2t(T x, T y)
		: Vector2<T>(x, y)
	{ }
	Vector2t(T x, T y, T theta)
		: Vector2<T>(x, y), theta(theta)
	{ }

	Vector2t(Vector2<T> &vector)
		: Vector2<T>(vector)
	{ }

	Vector2t(Vector2t<T> &vector)
		: Vector2<T>(vector)
	{
		this->theta = vector.theta;
	}

	virtual void zero() override
	{
		Vector2<T>::zero();
		this->theta = 0;
	}

	virtual void fromJson(const Json::Value &json) override
	{
		Vector2<T>::fromJson(json);
		const Json::Value &thetaJson = json["theta"];
		this->theta = thetaJson.asDouble();
	}
};

typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

typedef Vector2t<double> Vector2td;

typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;
}
}
}

#endif //WALKING_DATA_VECTOR_H