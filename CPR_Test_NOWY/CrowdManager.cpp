#include "CrowdManager.h"

DebugPoint g_pointToAvoid;

bool IsLesserOrEqualWithEpsilon(float x, float y) { return ((x)-(y) < EPSILON); }

void Location::integrate(const SteeringOutput& steer, float duration)
{
	SIMPLE_INTEGRATION(duration, steer.linear, steer.angular);
}

void Location::setOrientationFromVelocity(const D3DXVECTOR3& velocity)
{
	// If we haven't got any velocity, then we can do nothing.
	if (D3DXVec3LengthSq(&velocity) > 0) {
		orientation = real_atan2(velocity.x, velocity.z);
	}
}

D3DXVECTOR3 Location::getOrientationAsVector() const
{
	return D3DXVECTOR3(real_sin(orientation),
		0,
		real_cos(orientation));
}

/*
* Uses SIMPLE_INTEGRATION(duration), defined above.
*/
void Kinematic::integrate(float duration)
{
	SIMPLE_INTEGRATION(duration, velocity, rotation);
}

/*
* Uses SIMPLE_INTEGRATION(duration), defined above.
*/
void Kinematic::integrate(const SteeringOutput& steer,
	float duration)
{
	SIMPLE_INTEGRATION(duration, velocity, rotation);
	velocity.x += steer.linear.x*duration;
	velocity.y += steer.linear.y*duration;
	velocity.z += steer.linear.z*duration;
	rotation += steer.angular*duration;
}

/*
* Uses SIMPLE_INTEGRATION(duration), defined above.
*/
void Kinematic::integrate(const SteeringOutput& steer,
	float drag,
	float duration)
{
	SIMPLE_INTEGRATION(duration, velocity, rotation);

	// Slowing velocity and rotational velocity
	drag = real_pow(drag, duration);
	velocity *= drag;
	rotation *= drag*drag;

	velocity.x += steer.linear.x*duration;
	velocity.y += steer.linear.y*duration;
	velocity.z += steer.linear.z*duration;
	rotation += steer.angular*duration;
}

/*
* Uses SIMPLE_INTEGRATION(duration), defined above.
*/
void Kinematic::integrate(const SteeringOutput& steer,
	const SteeringOutput& drag,
	float duration)
{
	SIMPLE_INTEGRATION(duration, velocity, rotation);

	velocity.x *= real_pow(drag.linear.x, duration);
	velocity.y *= real_pow(drag.linear.y, duration);
	velocity.z *= real_pow(drag.linear.z, duration);
	rotation *= real_pow(drag.angular, duration);

	velocity.x += steer.linear.x*duration;
	velocity.y += steer.linear.y*duration;
	velocity.z += steer.linear.z*duration;
	rotation += steer.angular*duration;
}

/* Add and divide used in finding Kinematic means. */
void Kinematic::operator += (const Kinematic& other)
{
	position += other.position;
	velocity += other.velocity;
	rotation += other.rotation;
	orientation += other.orientation;
}

void Kinematic::operator -= (const Kinematic& other)
{
	position -= other.position;
	velocity -= other.velocity;
	rotation -= other.rotation;
	orientation -= other.orientation;
}

void Kinematic::operator *= (float f)
{
	position *= f;
	velocity *= f;
	rotation *= f;
	orientation *= f;
}

void Kinematic::trimMaxSpeed(float maxSpeed)
{
	if (D3DXVec3LengthSq(&velocity) > maxSpeed*maxSpeed) {
		D3DXVec3Normalize(&velocity, &velocity);
		velocity *= maxSpeed;
	}
}

void Kinematic::setOrientationFromVelocity()
{
	// If we haven't got any velocity, then we can do nothing.
	if (D3DXVec3LengthSq(&velocity) > 0) {
		orientation = real_atan2(velocity.x, velocity.z);
	}
}

Flock::Flock()
	:
	mInNeighbourhood(0), mArraySize(0)
{}

unsigned Flock::prepareNeighourhood(
	const Kinematic* of,
	float size,
	float minDotProduct /* = -1.0 */)
{
	// Make sure the array is of the correct size
	if (mArraySize != mBoids.size())
	{
		if (mArraySize)
		{
			delete[] mInNeighbourhood;
		}

		mArraySize = mBoids.size();
		if (mArraySize)
		{
			mInNeighbourhood = new bool[mArraySize];
		}

		memset(mInNeighbourhood, 0, sizeof(bool)*mArraySize);
	}

	// Compile the look vector if we need it
	D3DXVECTOR3 look = D3DXVECTOR3(0.f, 0.f, 0.f);
	if (minDotProduct > -1.0f)
	{
		look = of->getOrientationAsVector();
	}

	Flock result;
	std::list<Kinematic*>::iterator bi;
	unsigned i = 0, count = 0;;
	for (bi = mBoids.begin(); bi != mBoids.end(); bi++, i++)
	{
		Kinematic *k = *bi;
		mInNeighbourhood[i] = false;

		// Ignore ourself
		if (k == of) continue;

		// Check for maximum distance
		if(D3DXVec3Length(&D3DXVECTOR3(k->position - of->position)) > size) continue;

		// Check for angle
		if (minDotProduct > -1.0)
		{
			D3DXVECTOR3 offset = k->position - of->position;
			D3DXVec3Normalize(&offset, &offset);
			if (D3DXVec3Dot(&look, &offset)  < minDotProduct)
			{
				continue;
			}
		}

		// If we get here we've passed all tests
		mInNeighbourhood[i] = true;
		count++;
	}
	return count;
}

D3DXVECTOR3 Flock::getNeighbourhoodCenter()
{
	D3DXVECTOR3 center = D3DXVECTOR3(0.f, 0.f, 0.f);
	std::list<Kinematic*>::iterator bi;
	int i = 0;
	int count = 0;
	for (bi = mBoids.begin(); bi != mBoids.end(); bi++, i++)
	{
		if (mInNeighbourhood[i])
		{
			center += (*bi)->position;
			count++;
		}
	}
	center *= 1.f / count;

	return center;
}

D3DXVECTOR3 Flock::getNeighbourhoodAverageVelocity()
{
	D3DXVECTOR3 center = D3DXVECTOR3(0.f, 0.f, 0.f);
	std::list<Kinematic*>::iterator bi;
	int i = 0;
	int count = 0;
	for (bi = mBoids.begin(); bi != mBoids.end(); bi++, i++)
	{
		if (mInNeighbourhood[i])
		{
			center += (*bi)->velocity;
			count++;
		}
	}
	center *= 1.f / count;

	return center;
}

void Seek::getSteering(SteeringOutput* output)
{
	// First work out the direction
	output->linear = *target;
	output->linear -= mCharacter->position;

	// If there is no direction, do nothing
	if (D3DXVec3LengthSq(&output->linear) > 0)
	{
		D3DXVec3Normalize(&output->linear, &output->linear);
		output->linear *= maxAcceleration;
	}
}

void Flee::getSteering(SteeringOutput* output)
{
	// First work out the direction
	output->linear = mCharacter->position;
	output->linear -= *target;

	// If there is no direction, do nothing
	if (D3DXVec3LengthSq(&output->linear) > 0)
	{
		D3DXVec3Normalize(&output->linear, &output->linear);
		output->linear *= maxAcceleration;
	}
}

void Separation::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	unsigned count = mFlock->prepareNeighourhood(mCharacter, mNeighbourhoodSize, mNeighbourhoodMinDP);
	if (count <= 0) 
		return;

	// Work out their center of mass
	D3DXVECTOR3 cofm = mFlock->getNeighbourhoodCenter();

	// Steer away from it.
	flee.maxAcceleration = mMaxAcceleration;
	flee.mCharacter = mCharacter;
	flee.target = &cofm;
	
	flee.getSteering(output);

}

void Cohesion::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	unsigned count = mFlock->prepareNeighourhood(
		mCharacter, mNeighbourhoodSize, mNeighbourhoodMinDP
	);
	if (count <= 0) 
		return;

	// Work out their center of mass
	D3DXVECTOR3 cofm = mFlock->getNeighbourhoodCenter();

	// Steer away from it.
	seek.maxAcceleration = mMaxAcceleration;
	seek.mCharacter = mCharacter;
	seek.target = &cofm;
	g_pointToAvoid.pos = cofm;
	seek.getSteering(output);
}

void VelocityMatchAndAlign::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	unsigned count = mFlock->prepareNeighourhood(
		mCharacter, mNeighbourhoodSize, mNeighbourhoodMinDP
	);
	if (count <= 0) 
		return;

	// Work out their center of mass
	D3DXVECTOR3 vel = mFlock->getNeighbourhoodAverageVelocity();

	// Try to match it
	output->linear = vel - mCharacter->velocity;
	if (D3DXVec3LengthSq(&output->linear) > mMaxAcceleration*mMaxAcceleration)
	{
		D3DXVec3Normalize(&output->linear, &output->linear);
		output->linear *= mMaxAcceleration;
	}
}

void BlendedSteering::getSteering(SteeringOutput *output)
{
	// Clear the output to start with
	output->clear();

	// Go through all the behaviours in the list
	std::vector<BehaviourAndWeight>::iterator baw;
	float totalWeight = 0.f;
	SteeringOutput temp;
	for (baw = behaviours.begin(); baw != behaviours.end(); baw++)
	{
		// Make sure the children's character is set
		baw->behaviour->mCharacter = mCharacter;

		// Get the behaviours steering and add it to the accumulator
		baw->behaviour->getSteering(&temp);
		output->linear += temp.linear * baw->weight;
		output->angular += temp.angular * baw->weight;

		totalWeight += baw->weight;
	}

	// Divide the accumulated output by the total weight
	if (totalWeight > 0.f)
	{
		totalWeight = 1.f / totalWeight;
		output->linear *= totalWeight;
		output->angular *= totalWeight;
	}
}

SeekWithInternalTarget::SeekWithInternalTarget()
{
	// Make the target pointer point at our internal target.
	target = &internal_target;
}

void AvoidSphere::getSteering(SteeringOutput* output)
{
	// Clear the output, in case we don't write to it later.
	output->clear();

	// Make sure we're moving
	if (D3DXVec3LengthSq(&mCharacter->velocity) > 0)
	{
		// Find the distance from the line we're moving along to the obstacle.
		D3DXVec3Normalize(&mCharacter->velocity, &mCharacter->velocity);
		D3DXVECTOR3 movementNormal = mCharacter->velocity;
		D3DXVECTOR3 characterToObstacle = obstacle->position - mCharacter->position;

		float distanceSquared = D3DXVec3Dot(&characterToObstacle, &movementNormal);
		distanceSquared = D3DXVec3LengthSq(&characterToObstacle) -
			distanceSquared*distanceSquared;

		// Check for collision
		float radius = obstacle->radius + avoidMargin;
		if (distanceSquared < radius*radius)
		{
			// Find how far along our movement vector the closest pass is
			float distanceToClosest = D3DXVec3Dot(&characterToObstacle, &movementNormal);

			// Make sure this isn't behind us and is closer than our lookahead.
			if (distanceToClosest > 0 && distanceToClosest < maxLookahead)
			{
				// Find the closest point
				D3DXVECTOR3 closestPoint =
					mCharacter->position + movementNormal*distanceToClosest;

				// Find the point of avoidance
				D3DXVECTOR3 dir = closestPoint - obstacle->position;
				dir.y = 0.f;
				D3DXVec3Normalize(&dir, &dir);
				if (IsLesserOrEqualWithEpsilon(D3DXVec3Length(&dir), 0.f))
				{
					D3DXMATRIX R;
					D3DXMatrixRotationAxis(&R, &D3DXVECTOR3(0.f, 1.f, 0.f), M_PI/2.f);
					D3DXVec3TransformNormal(&dir, &movementNormal, &R);
				}
				
				internal_target = obstacle->position + dir * (obstacle->radius + avoidMargin);

				// Seek this point
				Seek::getSteering(output);
			}
		}
	}
}

CCrowdManager::CCrowdManager()
{
	init();
}

CCrowdManager::CCrowdManager(Sphere *obstacles)
{
	init();

	avoid = new AvoidSphere[OBSTACLES];
	for (unsigned i = 0; i < OBSTACLES; i++)
	{
		avoid[i].obstacle = &obstacles[i];
		avoid[i].mCharacter = kinematic;
		avoid[i].maxAcceleration = 30.f;
		avoid[i].avoidMargin = 1.f;
		avoid[i].maxLookahead = 6.f;

		steering->behaviours.push_back(BlendedSteering::BehaviourAndWeight(avoid + i));
	}
}

void CCrowdManager::init()
{

	// Set up the kinematics and all individual behaviours
	kinematic = new Kinematic[BOIDS];

	for (unsigned i = 0; i < BOIDS; i++)
	{
		kinematic[i].position.x = 1.f + i;
		kinematic[i].position.y = 0.5f;
		kinematic[i].position.z = 1.f;
		kinematic[i].orientation = M_PI / 4.f;
		kinematic[i].velocity.x = 0.1f;
		kinematic[i].velocity.y = 0.f;
		kinematic[i].velocity.z = 0.1f;
		kinematic[i].rotation = 0.f;

		flock.mBoids.push_back(kinematic + i);
	}

	// Set up the steering behaviours (we use one for all)
	separation = new Separation;
	separation->mMaxAcceleration = 15.f;
	separation->mNeighbourhoodSize = 10.f;
	separation->mNeighbourhoodMinDP = -1.f;
	separation->mFlock = &flock;

	cohesion = new Cohesion;
	cohesion->mMaxAcceleration = 20.f;
	cohesion->mNeighbourhoodSize = 20.f;
	cohesion->mNeighbourhoodMinDP = 0.f;
	cohesion->mFlock = &flock;

	vMA = new VelocityMatchAndAlign;
	vMA->mMaxAcceleration = 10.f;
	vMA->mNeighbourhoodSize = 20.f;
	vMA->mNeighbourhoodMinDP = 0.f;
	vMA->mFlock = &flock;

	steering = new BlendedSteering;
	steering->behaviours.push_back(BlendedSteering::BehaviourAndWeight(
		separation, SEPARATION_WEIGHT
	));
	steering->behaviours.push_back(BlendedSteering::BehaviourAndWeight(
		cohesion, COHESION_WEIGHT
	));
	steering->behaviours.push_back(BlendedSteering::BehaviourAndWeight(
		vMA, VMA_WEIGHT
	));
}

CCrowdManager::~CCrowdManager()
{
	delete[] kinematic;
	delete[] avoid;

	delete separation;
	delete cohesion;
	delete vMA;
}

#define TRIM_WORLD(var) \
    if (var < -WORLD_SIZE) var = WORLD_SIZE; \
    if (var > WORLD_SIZE) var = -WORLD_SIZE;


void CCrowdManager::update(float dt)
{
	SteeringOutput steer;
	SteeringOutput temp;

	for (unsigned i = 0; i < BOIDS; i++)
	{
		// Get the steering output
		steering->mCharacter = kinematic + i;
		steering->getSteering(&steer);

		// Update the kinematic
		kinematic[i].integrate(steer, 0.7f, dt);
		kinematic[i].setOrientationFromVelocity();

		// Check for maximum speed
		kinematic[i].trimMaxSpeed(20.f);

		// Keep in bounds of the world
		TRIM_WORLD(kinematic[i].position.x);
		TRIM_WORLD(kinematic[i].position.z);
		kinematic[i].position.y = 0.5f;
	}
}
