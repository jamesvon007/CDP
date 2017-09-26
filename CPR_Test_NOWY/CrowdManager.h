#pragma once

#include "CPR_Framework.h"
#include <list>

static const float EPSILON = 0.000001f;

struct DebugPoint
{
	D3DXVECTOR3 pos;
	D3DXVECTOR4 color;

	DebugPoint()
		: pos(D3DXVECTOR3(0.f, 0.f, 0.f))
		, color(D3DXVECTOR4(0.f, 0.f, 1.f, 1.f))
	{

	}
};

/** pi */
const float M_PI = 3.14159265358979323846f;
const float M_2PI = 6.28318530717958647692f;

// The number of boids in the simulation
#define BOIDS 8

#define OBSTACLES 40

#define WORLD_SIZE 25.f

// The relative blend weights
#define SEPARATION_WEIGHT (1.0f)
#define COHESION_WEIGHT (1.0f)
#define VMA_WEIGHT (2.0f)

#define real_atan2 atan2f
#define real_pow powf
#define real_mod_real fmodf
#define real_sin sinf
#define real_cos cosf

#define SIMPLE_INTEGRATION(duration, velocity, rotation) \
        position.x += (velocity).x*duration; \
        position.y += (velocity).y*duration; \
        position.z += (velocity).z*duration; \
        orientation += (rotation)*duration; \
        orientation = real_mod_real(orientation, M_2PI);

struct Sphere
{
	/**
	* The position of the geometric centre of the sphere.
	*/
	D3DXVECTOR3 position;

	/**
	* The radius of the sphere.
	*/
	float radius;
};

struct SteeringOutput
{

	D3DXVECTOR3 linear;

	float angular;


	SteeringOutput() : angular(0.f)
		, linear(D3DXVECTOR3(0.f, 0.f, 0.f))
	{}

	SteeringOutput(const D3DXVECTOR3& linear, float angular = 0.f)
		: linear(linear), angular(angular)
	{}

	void clear()
	{
		linear.x = linear.y = linear.z = 0.f;
		angular = 0.f;
	}

	bool operator == (const SteeringOutput& other) const
	{
		return linear == other.linear &&  angular == other.angular;
	}

	bool operator != (const SteeringOutput& other) const
	{
		return linear != other.linear || angular != other.angular;
	}

	float squareMagnitude()
	{
		return D3DXVec3LengthSq(&linear) + angular*angular;
	}

	float magnitude()
	{
		return sqrt(squareMagnitude());
	}
};

struct Location
{
	D3DXVECTOR3 position;

	float orientation;

	Location() : orientation(0.f)
		, position(D3DXVECTOR3(0.f, 0.f, 0.f))
	{}

	Location(const D3DXVECTOR3& position)
		: position(position), orientation(0.f)
	{}

	Location(const D3DXVECTOR3& position, float orientation)
		: position(position), orientation(orientation)
	{}

	Location(float x, float y, float z, float orientation)
		: position(x, y, z), orientation(orientation)
	{}

	Location& operator = (const Location& other)
	{
		position = other.position;
		orientation = other.orientation;
		return *this;
	}

	void clear()
	{
		position.x = position.y = position.z = 0.f;
		orientation = 0.f;
	}

	bool operator == (const Location& other) const
	{
		return position == other.position &&
			orientation == other.orientation;
	}

	bool operator != (const Location& other) const
	{
		return position != other.position ||
			orientation != other.orientation;
	}

	void integrate(const SteeringOutput& steer, float duration);

	void setOrientationFromVelocity(const D3DXVECTOR3& velocity);

	D3DXVECTOR3 getOrientationAsVector() const;

};

struct Kinematic : public Location
{
	D3DXVECTOR3 velocity;

	float rotation;

	Kinematic()
		: Location(), velocity(), rotation(0.f)
	{}

	Kinematic(const D3DXVECTOR3& position, const D3DXVECTOR3& velocity)
		: Location(position), velocity(velocity), rotation(0.f)
	{}

	Kinematic(const Location& loc, const D3DXVECTOR3& velocity)
		: Location(loc), velocity(velocity), rotation(0.f)
	{}

	Kinematic(const Location& loc)
		: Location(loc), velocity(), rotation(0.f)
	{}

	Kinematic(const D3DXVECTOR3& position, float orientation,
		const D3DXVECTOR3& velocity, float avel)
		: Location(position, orientation),
		velocity(velocity), rotation(avel)
	{}

	void clear()
	{
		Location::clear();
		velocity.x = velocity.y = velocity.z = 0.f;
		rotation = 0.f;
	}

	bool operator == (const Kinematic& other) const
	{
		return position == other.position &&
			orientation == other.orientation &&
			velocity == other.velocity &&
			rotation == other.rotation;
	}

	bool operator != (const Kinematic& other) const
	{
		return position != other.position ||
			orientation != other.orientation ||
			velocity != other.velocity ||
			rotation != other.rotation;
	}

	bool operator < (const Kinematic& other) const
	{
		return position.x < other.position.x;
	}

	Kinematic& operator = (const Location& other)
	{
		orientation = other.orientation;
		position = other.position;
		return *this;
	}

	Kinematic& operator = (const Kinematic& other)
	{
		orientation = other.orientation;
		position = other.position;
		velocity = other.velocity;
		rotation = other.rotation;
		return *this;
	}

	void operator += (const Kinematic&);

	void operator -= (const Kinematic&);

	void operator *= (float f);

	void integrate(float duration);

	void integrate(const SteeringOutput& steer, float duration);

	void integrate(const SteeringOutput& steer,
		float drag, float duration);

	void integrate(const SteeringOutput& steer,
		const SteeringOutput& drag,
		float duration);

	void trimMaxSpeed(float speed);

	void setOrientationFromVelocity();
};

class SteeringBehaviour
{
public:
	Kinematic *mCharacter;

	virtual void getSteering(SteeringOutput* output) = 0;
};

/**
* This class stores a flock of creatures.
*/
class Flock
{
public:
	std::list<Kinematic*> mBoids;
	bool *mInNeighbourhood;
	unsigned int mArraySize;

	Flock();

	/**
	* Sets up the boolean flags of boids in the neighbourhood of the given boid.
	*/
	unsigned prepareNeighourhood(
		const Kinematic* of,
		float size,
		float minDotProduct = -1.f
	);

	/**
	* Returns the geometric center of the flock.
	*/
	D3DXVECTOR3 getNeighbourhoodCenter();

	/**
	* Returns the average velocity of the flock.
	*/
	D3DXVECTOR3 getNeighbourhoodAverageVelocity();
};

class BoidSteeringBehaviour : public SteeringBehaviour
{
public:
	Flock *mFlock;
	float mNeighbourhoodSize;
	float mNeighbourhoodMinDP;
	float mMaxAcceleration;
};

class Seek : public SteeringBehaviour
{
public:
	const D3DXVECTOR3 *target;

	float maxAcceleration;

	virtual void getSteering(SteeringOutput* output);
};

class Flee : public Seek
{
public:
	virtual void getSteering(SteeringOutput* output);
};

class Separation : public BoidSteeringBehaviour
{
	Flee flee;

public:
	virtual void getSteering(SteeringOutput* output);
};

class Cohesion : public BoidSteeringBehaviour
{
	Seek seek;

public:
	virtual void getSteering(SteeringOutput* output);
};

class VelocityMatchAndAlign : public BoidSteeringBehaviour
{
public:
	virtual void getSteering(SteeringOutput* output);
};

class BlendedSteering : public SteeringBehaviour
{
public:
	/**
	* Holds a steering behaviour with its associated weight.
	*/
	struct BehaviourAndWeight
	{
		SteeringBehaviour *behaviour;
		float weight;

		BehaviourAndWeight(SteeringBehaviour *behaviour, float weight = 1.f)
			:
			behaviour(behaviour), weight(weight)
		{}
	};

	/**
	* Holds the list of behaviour and their corresponding blending
	* weights.
	*/
	std::vector<BehaviourAndWeight> behaviours;

	/**
	* Works out the desired steering and writes it into the given
	* steering output structure.
	*/
	virtual void getSteering(SteeringOutput* output);
};

class SeekWithInternalTarget : public Seek
{
protected:

	D3DXVECTOR3 internal_target;

	SeekWithInternalTarget();
};

class AvoidSphere : public SeekWithInternalTarget
{
public:

	Sphere *obstacle;

	float avoidMargin;

	float maxLookahead;

	/**
	* Works out the desired steering and writes it into the given
	* steering output structure.
	*/
	virtual void getSteering(SteeringOutput* output);
};

class CCrowdManager
{
	/** Holds the kinematic of all the boids. */
	Kinematic *kinematic;

	/** Holds the flock */
	Flock flock;

	/** Holds the steering behaviours. */
	Separation *separation;
	Cohesion *cohesion;
	VelocityMatchAndAlign *vMA;
	BlendedSteering *steering;

	AvoidSphere *avoid;

public:
	CCrowdManager();
	CCrowdManager(Sphere *obstacles);
	virtual ~CCrowdManager();

	void init();

	void update(float dt);

	Kinematic* getKinematic() const { return kinematic; }
};
