#ifndef _VEC2F_H_
#define _VEC2F_H_

#include <math.h>

class Float2
{
public:
	/// Construct a zero vector.
	Float2() :
		x(0.0f),
		y(0.0f)
	{
	}

	/// Construct from coordinates.
	Float2(float x, float y) :
		x(x),
		y(y)
	{
	}

	/// Construct from a float array.
	explicit Float2(const float* data) :
		x(data[0]),
		y(data[1])
	{
	}

	/// Test for equality with another vector without epsilon.
	bool operator ==(const Float2& rhs) const { return x == rhs.x && y == rhs.y; }

	/// Test for inequality with another vector without epsilon.
	bool operator !=(const Float2& rhs) const { return x != rhs.x || y != rhs.y; }

	/// Add a vector.
	Float2 operator +(const Float2& rhs) const { return Float2(x + rhs.x, y + rhs.y); }

	/// Return negation.
	Float2 operator -() const { return Float2(-x, -y); }

	/// Subtract a vector.
	Float2 operator -(const Float2& rhs) const { return Float2(x - rhs.x, y - rhs.y); }

	/// Multiply with a scalar.
	Float2 operator *(float rhs) const { return Float2(x * rhs, y * rhs); }

	/// Multiply with a vector.
	Float2 operator *(const Float2& rhs) const { return Float2(x * rhs.x, y * rhs.y); }

	/// Divide by a scalar.
	Float2 operator /(float rhs) const { return Float2(x / rhs, y / rhs); }

	/// Divide by a vector.
	Float2 operator /(const Float2& rhs) const { return Float2(x / rhs.x, y / rhs.y); }

	/// Add-assign a vector.
	Float2& operator +=(const Float2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	/// Subtract-assign a vector.
	Float2& operator -=(const Float2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	/// Multiply-assign a scalar.
	Float2& operator *=(float rhs)
	{
		x *= rhs;
		y *= rhs;
		return *this;
	}

	/// Multiply-assign a vector.
	Float2& operator *=(const Float2& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		return *this;
	}

	/// Divide-assign a scalar.
	Float2& operator /=(float rhs)
	{
		float invRhs = 1.0f / rhs;
		x *= invRhs;
		y *= invRhs;
		return *this;
	}

	/// Divide-assign a vector.
	Float2& operator /=(const Float2& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;
		return *this;
	}

	/// Normalize to unit length.
	void Normalize()
	{
		float lenSquared = LengthSquared();
		if (lenSquared > 0.0f)
		{
			float invLen = 1.0f / sqrtf(lenSquared);
			x *= invLen;
			y *= invLen;
		}
	}

	/// Return length.
	float Length() const { return sqrtf(x * x + y * y); }

	/// Return squared length.
	float LengthSquared() const { return x * x + y * y; }

	/// Calculate dot product.
	float DotProduct(const Float2& rhs) const { return x * rhs.x + y * rhs.y; }

	/// Project vector onto axis.
	float ProjectOntoAxis(const Float2& axis) const { return DotProduct(axis.Normalized()); }

	/// Returns the angle between this vector and another vector
	float Angle(const Float2& rhs) const { return acos(DotProduct(rhs) / (Length() * rhs.Length())); }

	/// Return absolute vector.
	Float2 Abs() const { return Float2(fabs(x), fabs(y)); }

	/// Linear interpolation with another vector.
	Float2 Lerp(const Float2& rhs, float t) const { return *this * (1.0f - t) + rhs * t; }

	/// Return normalized to unit length.
	Float2 Normalized() const
	{
		float lenSquared = LengthSquared();
		if (lenSquared > 0.0f)
		{
			float invLen = 1.0f / sqrtf(lenSquared);
			return *this * invLen;
		}
		else
			return *this;
	}

	/// Return float data.
	const float* Data() const { return &x; }

	float x;
	float y;
};

/// Multiply Float2 with a scalar
inline Float2 operator *(float lhs, const Float2& rhs) { return rhs * lhs; }

#endif
