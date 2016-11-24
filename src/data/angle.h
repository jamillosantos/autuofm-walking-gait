/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#ifndef WALKING_DATA_ANGLE_H
#define WALKING_DATA_ANGLE_H

#include <json/json.h>

namespace mote
{
namespace walking
{
namespace data
{
template<typename T>
class Angle2
{
public:
	T pitch;
	T roll;

	Angle2()
		: pitch(0), roll(0)
	{ }

	Angle2(T pitch, T roll)
		: pitch(pitch), roll(roll)
	{ }

	Angle2(const Angle2<T>& angle)
		: pitch(angle.pitch), roll(angle.roll)
	{ }

	virtual void set(T value)
	{
		this->pitch = this->roll = value;
	}

	virtual void zero()
	{
		this->pitch = 0;
		this->roll = 0;
	}

	virtual void fromJson(const Json::Value &json)
	{
		for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
		{
			if (it.name() == "pitch")
				this->pitch = it->asDouble();
			else if (it.name() == "roll")
				this->roll = it->asDouble();
		}
	}
};

template<typename T>
class Angle3
	: public Angle2<T>
{
public:
	T yaw;

	Angle3(T pitch, T roll, T yaw)
		: Angle2<T>(pitch, roll), yaw(yaw)
	{ }

	Angle3()
	{ }

	Angle3(const Angle3<T> &angle)
		: Angle2<T>(angle), yaw(angle.yaw)
	{ }

	Angle3(const Angle2<T>& angle)
		: Angle2<T>(angle), yaw(0)
	{ }

	virtual void set(T value) override
	{
		this->pitch = this->roll = this->yaw = value;
	}

	virtual void zero() override
	{
		this->pitch = this->roll = this->yaw = 0;
	}

	virtual void fromJson(const Json::Value &json) override
	{
		Angle2<T>::fromJson(json);
		const Json::Value &jsonYaw = json["yaw"];
		if (!jsonYaw.isNull())
			this->yaw = jsonYaw.asDouble();
	}
};

typedef Angle2<float> Angle2f;
typedef Angle2<double> Angle2d;

typedef Angle3<float> Angle3f;
typedef Angle3<double> Angle3d;

class HeadData
{
public:
	double pan;
	double tilt;

	HeadData();
	HeadData(double pan, double tilt);
};
}
}
}

#endif //WALKING_DATA_ANGLE_H
