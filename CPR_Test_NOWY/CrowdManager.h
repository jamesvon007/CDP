#pragma once

#include "CPR_Framework.h"
#include <list>

/** pi */
const float M_PI = 3.14159265358979323846f;
const float M_2PI = 6.28318530717958647692f;

// The number of boids in the simulation
#define BOIDS 10

#define WORLD_SIZE 25

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

struct SteeringOutput
{

	D3DXVECTOR3 linear;

	float angular;


	SteeringOutput() : angular(0.f)
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
	Kinematic *character;

	virtual void getSteering(SteeringOutput* output) = 0;
};

/**
* This class stores a flock of creatures.
*/
class Flock
{
public:
	std::list<Kinematic*> boids;
	bool *inNeighbourhood;
	unsigned arraySize;

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
	Flock *theFlock;
	float neighbourhoodSize;
	float neighbourhoodMinDP;
	float maxAcceleration;
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

public:
	CCrowdManager();
	virtual ~CCrowdManager();

	void update(float dt);

	Kinematic* getKinematic() const { return kinematic; }
};
