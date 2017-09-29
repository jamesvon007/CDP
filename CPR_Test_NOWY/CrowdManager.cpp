#include "CrowdManager.h"
#include <algorithm>
#include <random>

DebugPoint g_pointToAvoid;
DebugPoint g_pointDest;

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

int Flock::prepareNeighourhood(
	const Kinematic* self,
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
		look = self->getOrientationAsVector();
	}

	Flock result;
	std::list<Kinematic*>::iterator bi;
	int i = 0, count = 0;;
	for (bi = mBoids.begin(); bi != mBoids.end(); bi++, i++)
	{
		Kinematic* k = *bi;
		mInNeighbourhood[i] = false;

		// Ignore ourself
		if (k == self)
			continue;

		// Check for maximum distance
		if(D3DXVec3Length(&D3DXVECTOR3(k->position - self->position)) > size) continue;

		// Check for angle
		if (minDotProduct > -1.0)
		{
			D3DXVECTOR3 offset = k->position - self->position;
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
		output->linear *= getAccelaration();
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
		output->linear *= getAccelaration();
	}
}

void Separation::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	int count = mFlock->prepareNeighourhood(mCharacter, mNeighbourhoodSize, mNeighbourhoodMinDP);
	if (count <= 0) 
		return;

	// Work out their center of mass
	D3DXVECTOR3 cofm = mFlock->getNeighbourhoodCenter();

	// Steer away from it.
	flee.setAccelaration(mMaxAcceleration);
	flee.mCharacter = mCharacter;
	flee.target = &cofm;
	
	flee.getSteering(output);
}

void Separation2::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	int count = mFlock->prepareNeighourhood(mCharacter, mNeighbourhoodSize, mNeighbourhoodMinDP);
	if (count <= 0)
		return;

	// Work out their center of mass
	D3DXVECTOR3 cofm = mFlock->getNeighbourhoodCenter();

	// Steer away from it.
	flee.setAccelaration(mMaxAcceleration);
	flee.mCharacter = mCharacter;
	flee.target = &cofm;

	flee.getSteering(output);

}

void Cohesion::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	int count = mFlock->prepareNeighourhood(
		mCharacter, mNeighbourhoodSize, mNeighbourhoodMinDP
	);
	if (count <= 0) 
		return;

	// Work out their center of mass
	D3DXVECTOR3 cofm = mFlock->getNeighbourhoodCenter();

	// Steer away from it.
	seek.setAccelaration(mMaxAcceleration);
	seek.mCharacter = mCharacter;
	seek.target = &cofm;
	g_pointToAvoid.pos = cofm;
	seek.getSteering(output);
}

void Cohesion2::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	int count = mFlock->prepareNeighourhood(mCharacter, 1000.f, -1);

	// Work out their center of mass
	D3DXVECTOR3 cofm = mFlock->getNeighbourhoodCenter();
	D3DXVECTOR3 centerToChar = mCharacter->position - cofm;
	if (D3DXVec3LengthSq(&centerToChar) > mTolerence*mTolerence)
	{
		// Steer away from it.
		seek.setAccelaration(mMaxAcceleration);
		seek.mCharacter = mCharacter;
		seek.target = &cofm;
		g_pointToAvoid.pos = cofm;
		seek.getSteering(output);
	}

}

void VelocityMatchAndAlign::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	int count = mFlock->prepareNeighourhood(
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

void PrioritySteering::getSteering(SteeringOutput* output)
{
	// We'll need epsilon squared later.
	float epSquared = epsilon*epsilon;

	// Start with zero output, so if there are no behaviours in 
	// the list, we'll output zero.
	output->clear();

	// Go through all the behaviours in the list
	std::vector<SteeringBehaviour*>::iterator b;
	for (b = behaviours.begin(); b != behaviours.end(); b++)
	{
		// Make sure the children's character is set
		(*b)->mCharacter = mCharacter;

		// Get the steering result from this behaviour
		(*b)->getSteering(output);

		// Check if it is non zero.
		if (output->squareMagnitude() > epSquared)
		{
			// We have a result, so store it and exit.
			lastUsed = *b;
			return;
		}
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

void Wander::getSteering(SteeringOutput* output)
{
	D3DXVECTOR3 characterToObstacle = mCurrentTarget - mCharacter->position;
	if (D3DXVec3LengthSq(&characterToObstacle) < 1.f || 
		(IsLesserOrEqualWithEpsilon(mCurrentTarget.x, 0.f) && IsLesserOrEqualWithEpsilon(mCurrentTarget.z, 0.f)))
	{
		mCurrentTarget = DestinationManager::Get()->Query();
	}

	mWanderPoint.pos = mCurrentTarget;

	internal_target = mCurrentTarget;
	Seek::getSteering(output);
}

PursueService* PursueService::mInstance = nullptr;

PursueService::PursueService()
{
	init();
	steering->behaviours.push_back(wander);
}

PursueService* PursueService::Get()
{
	if (mInstance == nullptr)
	{
		mInstance = new PursueService;
	}
	return mInstance;
}

void PursueService::init()
{
	// Set up the steering behaviours (we use one for all)
	wander = new Wander;
	wander->setAccelaration(15.f);
	wander->mCurrentTarget = D3DXVECTOR3(0.f, 0.5f, 0.f);
	wander->mFlock = nullptr;

	steering = new PrioritySteering();
	steering->epsilon = 0.01f;
}

PursueService::~PursueService()
{
	for (std::vector<SteeringBehaviour*>::iterator it = steering->behaviours.begin(); it != steering->behaviours.end(); ++it)
	{
		if (*it != nullptr)
		{
			delete *it;
		}
	}
	steering->behaviours.clear();
}

void PursueService::update(float dt)
{
	SteeringOutput steer;

	for (std::list<LevelService::Simulation*>::iterator it = pursueAgents.begin(); it != pursueAgents.end(); ++it)
	{
		Kinematic kinematic((*it)->position, (*it)->velocity);

		wander->setAccelaration(D3DXVec3Length(&((*it)->acceleration)));

		// Get the steering output
		steering->mCharacter = &kinematic;
		steering->getSteering(&steer);

		// Update the kinematic
		kinematic.integrate(steer, 0.7f, dt);
		kinematic.setOrientationFromVelocity();

		// Keep in bounds of the world
		if (kinematic.position.x < -WORLD_SIZE)
		{
			kinematic.position.x = -WORLD_SIZE + 0.5f;
			kinematic.velocity *= -1.f;
		}
		else if (kinematic.position.z < -WORLD_SIZE)
		{
			kinematic.position.z = -WORLD_SIZE + 0.5f;
			kinematic.velocity *= -1.f;
		}
		else if (kinematic.position.x > WORLD_SIZE)
		{
			kinematic.position.x = WORLD_SIZE - 0.5f;
			kinematic.velocity *= -1.f;
		}
		else if (kinematic.position.z > WORLD_SIZE)
		{
			kinematic.position.z = WORLD_SIZE - 0.5f;
			kinematic.velocity *= -1.f;
		}

		kinematic.position.y = 0.5f;

		(*it)->position = kinematic.position;
		(*it)->velocity = kinematic.velocity;
	}
}

CCrowdManager::CCrowdManager()
{
	init();
}

CCrowdManager::CCrowdManager(std::vector<Sphere>& obstacles)
{
	init();
	for (std::vector<Sphere>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
	{
		AvoidSphere* avoid = new AvoidSphere;
		avoid->obstacle = &(*it);
		avoid->setAccelaration(30.f);
		avoid->avoidMargin = 0.5f;
		avoid->maxLookahead = 2.5f;
		steering->behaviours.push_back(avoid);
	}

	steering->behaviours.push_back(separation);
	steering->behaviours.push_back(cohesion);

	steering->behaviours.push_back(wander);
	steering->behaviours.push_back(vMA);
	
}

void CCrowdManager::init()
{
	// Set up the kinematics and all individual behaviours
	for (int i = 0; i < BOIDS; i++)
	{
		Kinematic kine(D3DXVECTOR3(1.f + i, 0.5f, 20.f), M_PI / 4.f, D3DXVECTOR3(0.1f, 0.f, 0.1f), 0.f);
		kinematic.push_back(kine);
		flock.mBoids.push_back(&kinematic.back());
	}

	// Set up the steering behaviours (we use one for all)
	separation = new Separation2;
	separation->mMaxAcceleration = 5.f;
	separation->mNeighbourhoodSize = 2.f;
	separation->mTolerence = 3.f;
	separation->mNeighbourhoodMinDP = -1.f;
	separation->mFlock = &flock;

	cohesion = new Cohesion2;
	cohesion->mMaxAcceleration = 10.f;
	cohesion->mNeighbourhoodSize = 5.f;
	cohesion->mTolerence = 5.f;
	cohesion->mNeighbourhoodMinDP = 0.f;
	cohesion->mFlock = &flock;

	vMA = new VelocityMatchAndAlign;
	vMA->mMaxAcceleration = 10.f;
	vMA->mNeighbourhoodSize = 20.f;
	vMA->mNeighbourhoodMinDP = 0.f;
	vMA->mFlock = &flock;

	wander = new Wander;
	wander->setAccelaration(15.f);
	wander->mCurrentTarget = D3DXVECTOR3(0.f, 0.5f, 0.f);
	wander->mFlock = &flock;

	steering = new PrioritySteering();
	steering->epsilon = 0.01f;
}

CCrowdManager::~CCrowdManager()
{
	kinematic.clear();

	for (std::vector<SteeringBehaviour*>::iterator it = steering->behaviours.begin(); it != steering->behaviours.end(); ++it)
	{
		if (*it != nullptr)
		{
			delete *it;
		}
	}
	steering->behaviours.clear();
}

#define TRIM_WORLD(var) \
    if (var < -WORLD_SIZE) var = WORLD_SIZE; \
    if (var > WORLD_SIZE) var = -WORLD_SIZE;


void CCrowdManager::update(float dt)
{
	SteeringOutput steer;
	SteeringOutput temp;

	for (std::list<Kinematic>::iterator it = kinematic.begin(); it != kinematic.end(); )
	{
		Kinematic& kine = *it;
		bool selfErased = false;

		// Check red balls
		for (std::list<LevelService::Simulation>::iterator rit = LevelService::Get()->ModifyRedBalls().begin(); rit != LevelService::Get()->ModifyRedBalls().end(); ++rit)
		{
			LevelService::Simulation& redball = *rit;
			if (D3DXVec3Length(&D3DXVECTOR3(redball.position - kine.position)) < 0.6f)
			{
				redball.AddFuel(0.01f);
				it = kinematic.erase(it);
				selfErased = true;
			}
		}

		if (selfErased)
		{
			continue;
		}

		// Get the steering output
		steering->mCharacter = &kine;
		steering->getSteering(&steer);

		// Update the kinematic
		kine.integrate(steer, 0.7f, dt);
		kine.setOrientationFromVelocity();

		// Check for maximum speed
		kine.trimMaxSpeed(20.f);

		// Keep in bounds of the world
		if (kine.position.x < -WORLD_SIZE)
		{
			kine.position.x = -WORLD_SIZE + 0.5f;
			kine.velocity *= -1.f;
		}
		else if (kine.position.z < -WORLD_SIZE)
		{
			kine.position.z = -WORLD_SIZE + 0.5f;
			kine.velocity *= -1.f;
		}
		else if (kine.position.x > WORLD_SIZE)
		{
			kine.position.x = WORLD_SIZE - 0.5f;
			kine.velocity *= -1.f;
		}
		else if (kine.position.z > WORLD_SIZE)
		{
			kine.position.z = WORLD_SIZE - 0.5f;
			kine.velocity *= -1.f;
		}

		kine.position.y = 0.5f;

		++it;
	}
}

DestinationManager* DestinationManager::mInstance = nullptr;

DestinationManager* DestinationManager::Get()
{
	if (mInstance == nullptr)
	{
		mInstance = new DestinationManager;
	}
	return mInstance;
}

void DestinationManager::Push(const SSkycraper& data)
{

	mReachableRangeX.push_back(data.position.x - data.scale / 2.f);
	mReachableRangeX.push_back(data.position.x + data.scale / 2.f);

	mReachableRangeZ.push_back(data.position.z - data.scale / 2.f);
	mReachableRangeZ.push_back(data.position.z + data.scale / 2.f);
}

void DestinationManager::Finalize(D3DXVECTOR2& worldBoundX, D3DXVECTOR2& worldBoundZ)
{
	std::sort(mReachableRangeX.begin(), mReachableRangeX.end());
	auto last = std::unique(mReachableRangeX.begin(), mReachableRangeX.end());
	mReachableRangeX.erase(last, mReachableRangeX.end());
	if (mReachableRangeX.front() > worldBoundX.x)
	{
		mReachableRangeX.insert(mReachableRangeX.begin(), worldBoundX.x);
	}

	if (mReachableRangeX.back() < worldBoundX.y)
	{
		mReachableRangeX.push_back(worldBoundX.y);
	}

	std::sort(mReachableRangeZ.begin(), mReachableRangeZ.end());
	last = std::unique(mReachableRangeZ.begin(), mReachableRangeZ.end());
	mReachableRangeZ.erase(last, mReachableRangeZ.end());
	if (mReachableRangeZ.front() > worldBoundZ.x)
	{
		mReachableRangeZ.insert(mReachableRangeZ.begin(), worldBoundZ.x);
	}

	if (mReachableRangeZ.back() < worldBoundZ.y)
	{
		mReachableRangeZ.push_back(worldBoundZ.y);
	}

}

D3DXVECTOR3 DestinationManager::Query() const
{
	D3DXVECTOR3 dest;
	std::random_device rd;
	std::mt19937 gen(rd());

	std::uniform_int_distribution<> disX(0, mReachableRangeX.size()/2-1);
	int xIndex = disX(gen);
	std::uniform_real_distribution<float> xDest(mReachableRangeX.at(2*xIndex), mReachableRangeX.at(2 * xIndex + 1));
	dest.x = xDest(gen);

	std::uniform_int_distribution<> disZ(0, mReachableRangeZ.size() / 2 - 1);
	int zIndex = disZ(gen);
	std::uniform_real_distribution<float> zDest(mReachableRangeZ.at(2 * zIndex), mReachableRangeZ.at(2 * zIndex + 1));
	dest.z = zDest(gen);
	
	dest.y = 0.5f;

	return dest;
}

LevelService* LevelService::mInstance = nullptr;

LevelService* LevelService::Get()
{
	if (mInstance == nullptr)
	{
		mInstance = new LevelService;
	}
	return mInstance;
}

void LevelService::Push(SSkycraper& data, SNode* node)
{
	if (node == nullptr)
	{
		node = &mSkycraperRoot;
	}

	if (IsLesserOrEqualWithEpsilon(node->halfWidth.x, 4.f) || IsLesserOrEqualWithEpsilon(node->halfWidth.y, 4.f))
	{
		node->data.push_back(&data);
		return;
	}

	if (data.position.x > node->center.x)
	{
		if (data.position.z > node->center.z)
		{
			if (node->rightHigh == nullptr)
			{
				node->rightHigh = new SNode(D3DXVECTOR3((node->center.x + node->center.x + node->halfWidth.x) / 2.f, 
					node->center.y, (node->center.z + node->center.z + node->halfWidth.y) / 2.f),
					D3DXVECTOR2(node->halfWidth.x / 2.f, node->halfWidth.y / 2.f));
			}
			Push(data, node->rightHigh);
		}
		else
		{
			if (node->rightLow == nullptr)
			{
				node->rightLow = new SNode(D3DXVECTOR3((node->center.x + node->center.x + node->halfWidth.x) / 2.f, 
					node->center.y, (node->center.z + node->center.z - node->halfWidth.y) / 2.f),
					D3DXVECTOR2(node->halfWidth.x / 2.f, node->halfWidth.y / 2.f));
			}
			Push(data, node->rightLow);
		}
	}
	else
	{
		if (data.position.z > node->center.z)
		{
			if (node->leftHigh == nullptr)
			{
				node->leftHigh = new SNode(D3DXVECTOR3((node->center.x + node->center.x - node->halfWidth.x) / 2.f, 
					node->center.y, (node->center.z + node->center.z + node->halfWidth.y) / 2.f),
					D3DXVECTOR2(node->halfWidth.x / 2.f, node->halfWidth.y / 2.f));
			}
			Push(data, node->leftHigh);
		}
		else
		{
			if (node->leftLow == nullptr)
			{
				node->leftLow = new SNode(D3DXVECTOR3((node->center.x + node->center.x - node->halfWidth.x) / 2.f, 
					node->center.y, (node->center.z + node->center.z - node->halfWidth.y) / 2.f),
					D3DXVECTOR2(node->halfWidth.x / 2.f, node->halfWidth.y / 2.f));
			}
			Push(data, node->leftLow);
		}
		
	}
}

void LevelService::AddRedBall(D3DXVECTOR3& pos, D3DXVECTOR3& vel, D3DXVECTOR3& accel, float r)
{
	mRedBalls.push_back(Simulation(pos, vel, accel, r));
}

void LevelService::Update(float dt)
{
	for (std::list<Simulation>::iterator it = mRedBalls.begin(); it != mRedBalls.end(); )
	{
		Simulation& redBall = *it;
		if (IsLesserOrEqualWithEpsilon(D3DXVec3LengthSq(&redBall.velocity), 0.00000001f))
		{
			it = mRedBalls.erase(it);
			continue;
		}

		if (redBall.position.x < -WORLD_SIZE || redBall.position.x > WORLD_SIZE
			|| redBall.position.z > WORLD_SIZE || redBall.position.z < -WORLD_SIZE)
		{
			it = mRedBalls.erase(it);
			continue;
		}

		++it;

		D3DXVECTOR3 normalized;
		D3DXVec3Normalize(&normalized, &redBall.acceleration);
		redBall.acceleration -= dt * normalized;
		if (IsLesserOrEqualWithEpsilon(redBall.position.y, 0.5f))
		{
			// landed on the ground
			redBall.velocity.y = 0.f;
			redBall.acceleration.y = 0.f;
			PursueService::Get()->AddPursueAgent(&redBall);
		}
		else
		{
			redBall.velocity += redBall.acceleration * dt;
			redBall.position += redBall.velocity * dt;
		}

		std::vector<SSkycraper*> skycrapers;
		GetNearestSkycraper(skycrapers, D3DXVECTOR3(redBall.position.x-4, redBall.position.y, redBall.position.z-4), 
			D3DXVECTOR3(redBall.position.x + 4, redBall.position.y, redBall.position.z + 4), &mSkycraperRoot);

		for (std::vector<SSkycraper*>::const_iterator sIt = skycrapers.begin(); sIt != skycrapers.end(); ++sIt)
		{
			SSkycraper* skycraper = *sIt;
			skycraper->color = D3DXVECTOR4(1.f, 1.f, 1.f, 1.f);
			if (InSkycraper(redBall.position, skycraper))
			{
				redBall.velocity *= -1.f;
				redBall.position += redBall.velocity*dt;
				break;
			}
		}
	}
}

bool LevelService::InSkycraper(D3DXVECTOR3& pos, const SSkycraper* skycraper) const
{
	return (pos.x > skycraper->minBoundingBox.x && pos.x < skycraper->maxBoundingBox.x && pos.y > skycraper->minBoundingBox.y
		&& pos.y < skycraper->maxBoundingBox.y && pos.z > skycraper->minBoundingBox.z && pos.z < skycraper->maxBoundingBox.z);
}

void LevelService::GetNearestSkycraper(std::vector<SSkycraper*>& skycrapers, D3DXVECTOR3& lowerBound, D3DXVECTOR3& upperBound, SNode* node)
{
	if (node == nullptr)
	{
		return;
	}

	if (lowerBound.x < -WORLD_SIZE || upperBound.x > WORLD_SIZE
		|| lowerBound.z < -WORLD_SIZE || upperBound.z > WORLD_SIZE
		|| lowerBound.x >= upperBound.x || lowerBound.z >= upperBound.z )
	{
		return;
	}

	if (lowerBound.x <= node->center.x && upperBound.x <= node->center.x)
	{
		if (lowerBound.z <= node->center.z && upperBound.z <= node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftLow);
		}
		else if (lowerBound.z > node->center.z && upperBound.z > node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftHigh);
		}
		else if (lowerBound.z <= node->center.z && upperBound.z > node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftLow);
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftHigh);
		}
	}
	else if (lowerBound.x <= node->center.x && upperBound.x > node->center.x)
	{
		if (lowerBound.z <= node->center.z && upperBound.z <= node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftLow);
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightLow);
		}
		else if (lowerBound.z > node->center.z && upperBound.z > node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftHigh);
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightHigh);
		}
		else if (lowerBound.z <= node->center.z && upperBound.z > node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftLow);
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightLow);
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->leftHigh);
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightHigh);

			for (std::vector<SSkycraper*>::const_iterator it = node->data.begin(); it != node->data.end(); ++it)
			{
				skycrapers.push_back(*it);
			}
			
		}
	}
	else if (lowerBound.x > node->center.x && upperBound.x > node->center.x)
	{
		if (lowerBound.z <= node->center.z && upperBound.z <= node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightLow);
		}
		else if (lowerBound.z > node->center.z && upperBound.z > node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightHigh);
		}
		else if (lowerBound.z <= node->center.z && upperBound.z > node->center.z)
		{
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightLow);
			GetNearestSkycraper(skycrapers, lowerBound, upperBound, node->rightHigh);
		}
	}
}
