#ifndef _VEC2F_H_
#define _VEC2F_H_

#include <math.h>

class Vector2
{
public:
	/// Construct a zero vector.
	Vector2() :
		x(0.0f),
		y(0.0f)
	{
	}

	/// Construct from coordinates.
	Vector2(float x, float y) :
		x(x),
		y(y)
	{
	}

	/// Construct from a float array.
	explicit Vector2(const float* data) :
		x(data[0]),
		y(data[1])
	{
	}

	/// Test for equality with another vector without epsilon.
	bool operator ==(const Vector2& rhs) const { return x == rhs.x && y == rhs.y; }

	/// Test for inequality with another vector without epsilon.
	bool operator !=(const Vector2& rhs) const { return x != rhs.x || y != rhs.y; }

	/// Add a vector.
	Vector2 operator +(const Vector2& rhs) const { return Vector2(x + rhs.x, y + rhs.y); }

	/// Return negation.
	Vector2 operator -() const { return Vector2(-x, -y); }

	/// Subtract a vector.
	Vector2 operator -(const Vector2& rhs) const { return Vector2(x - rhs.x, y - rhs.y); }

	/// Multiply with a scalar.
	Vector2 operator *(float rhs) const { return Vector2(x * rhs, y * rhs); }

	/// Multiply with a vector.
	Vector2 operator *(const Vector2& rhs) const { return Vector2(x * rhs.x, y * rhs.y); }

	/// Divide by a scalar.
	Vector2 operator /(float rhs) const { return Vector2(x / rhs, y / rhs); }

	/// Divide by a vector.
	Vector2 operator /(const Vector2& rhs) const { return Vector2(x / rhs.x, y / rhs.y); }

	/// Add-assign a vector.
	Vector2& operator +=(const Vector2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	/// Subtract-assign a vector.
	Vector2& operator -=(const Vector2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	/// Multiply-assign a scalar.
	Vector2& operator *=(float rhs)
	{
		x *= rhs;
		y *= rhs;
		return *this;
	}

	/// Multiply-assign a vector.
	Vector2& operator *=(const Vector2& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		return *this;
	}

	/// Divide-assign a scalar.
	Vector2& operator /=(float rhs)
	{
		float invRhs = 1.0f / rhs;
		x *= invRhs;
		y *= invRhs;
		return *this;
	}

	/// Divide-assign a vector.
	Vector2& operator /=(const Vector2& rhs)
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
	float DotProduct(const Vector2& rhs) const { return x * rhs.x + y * rhs.y; }

	/// Project vector onto axis.
	float ProjectOntoAxis(const Vector2& axis) const { return DotProduct(axis.Normalized()); }

	/// Returns the angle between this vector and another vector
	float Angle(const Vector2& rhs) const { return acos(DotProduct(rhs) / (Length() * rhs.Length())); }

	/// Return absolute vector.
	Vector2 Abs() const { return Vector2(fabs(x), fabs(y)); }

	/// Linear interpolation with another vector.
	Vector2 Lerp(const Vector2& rhs, float t) const { return *this * (1.0f - t) + rhs * t; }

	/// Return normalized to unit length.
	Vector2 Normalized() const
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

/// Multiply Vector2 with a scalar
inline Vector2 operator *(float lhs, const Vector2& rhs) { return rhs * lhs; }

#endif
