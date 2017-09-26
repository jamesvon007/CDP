#include "CrowdManager.h"

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
	inNeighbourhood(0), arraySize(0)
{}

unsigned Flock::prepareNeighourhood(
	const Kinematic* of,
	float size,
	float minDotProduct /* = -1.0 */)
{
	// Make sure the array is of the correct size
	if (arraySize != boids.size())
	{
		if (arraySize) delete[] inNeighbourhood;
		arraySize = boids.size();
		if (arraySize) inNeighbourhood = new bool[arraySize];
		memset(inNeighbourhood, 0, sizeof(bool)*arraySize);
	}

	// Compile the look vector if we need it
	D3DXVECTOR3 look;
	if (minDotProduct > -1.0f)
	{
		look = of->getOrientationAsVector();
	}

	Flock result;
	std::list<Kinematic*>::iterator bi;
	unsigned i = 0, count = 0;;
	for (bi = boids.begin(); bi != boids.end(); bi++, i++)
	{
		Kinematic *k = *bi;
		inNeighbourhood[i] = false;

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
		inNeighbourhood[i] = true;
		count++;
	}
	return count;
}

D3DXVECTOR3 Flock::getNeighbourhoodCenter()
{
	D3DXVECTOR3 center;
	std::list<Kinematic*>::iterator bi;
	int i = 0;
	int count = 0;
	for (bi = boids.begin(); bi != boids.end(); bi++, i++)
	{
		if (inNeighbourhood[i])
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
	D3DXVECTOR3 center;
	std::list<Kinematic*>::iterator bi;
	int i = 0;
	int count = 0;
	for (bi = boids.begin(); bi != boids.end(); bi++, i++)
	{
		if (inNeighbourhood[i])
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
	output->linear -= character->position;

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
	output->linear = character->position;
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
	unsigned count = theFlock->prepareNeighourhood(
		character, neighbourhoodSize, neighbourhoodMinDP
	);
	if (count <= 0) return;

	// Work out their center of mass
	D3DXVECTOR3 cofm = theFlock->getNeighbourhoodCenter();

	// Steer away from it.
	flee.maxAcceleration = maxAcceleration;
	flee.character = character;
	flee.target = &cofm;
	flee.getSteering(output);

}

void Cohesion::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	unsigned count = theFlock->prepareNeighourhood(
		character, neighbourhoodSize, neighbourhoodMinDP
	);
	if (count <= 0) return;

	// Work out their center of mass
	D3DXVECTOR3 cofm = theFlock->getNeighbourhoodCenter();

	// Steer away from it.
	seek.maxAcceleration = maxAcceleration;
	seek.character = character;
	seek.target = &cofm;
	seek.getSteering(output);
}

void VelocityMatchAndAlign::getSteering(SteeringOutput* output)
{
	// Get the neighbourhood of boids
	unsigned count = theFlock->prepareNeighourhood(
		character, neighbourhoodSize, neighbourhoodMinDP
	);
	if (count <= 0) return;

	// Work out their center of mass
	D3DXVECTOR3 vel = theFlock->getNeighbourhoodAverageVelocity();

	// Try to match it
	output->linear = vel - character->velocity;
	if (D3DXVec3LengthSq(&output->linear) > maxAcceleration*maxAcceleration)
	{
		D3DXVec3Normalize(&output->linear, &output->linear);
		output->linear *= maxAcceleration;
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
		baw->behaviour->character = character;

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

CCrowdManager::CCrowdManager()
{
	static const float accel = 25.f;

	// Set up the kinematics and all individual behaviours
	kinematic = new Kinematic[BOIDS];

	for (unsigned i = 0; i < BOIDS; i++)
	{
		kinematic[i].position.x = 1.f + i;
		kinematic[i].position.y = 2.f;
		kinematic[i].position.z = 1.f + i;
		kinematic[i].orientation = M_PI/4.f;
		kinematic[i].velocity.x = i + 0.1f;
		kinematic[i].velocity.y = 0.f;
		kinematic[i].velocity.z = 1.f;
		kinematic[i].rotation = 0.f;

		flock.boids.push_back(kinematic + i);
	}

	// Set up the steering behaviours (we use one for all)
	separation = new Separation;
	separation->maxAcceleration = accel;
	separation->neighbourhoodSize = 5.f;
	separation->neighbourhoodMinDP =  -1.f;
	separation->theFlock = &flock;

	cohesion = new Cohesion;
	cohesion->maxAcceleration = accel;
	cohesion->neighbourhoodSize = 5.f;
	cohesion->neighbourhoodMinDP = 0.f;
	cohesion->theFlock = &flock;

	vMA = new VelocityMatchAndAlign;
	vMA->maxAcceleration = accel;
	vMA->neighbourhoodSize = 5.f;
	vMA->neighbourhoodMinDP = 0.f;
	vMA->theFlock = &flock;

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
		steering->character = kinematic + i;
		steering->getSteering(&steer);

		// Update the kinematic
		kinematic[i].integrate(steer, 0.7f, dt);
		kinematic[i].setOrientationFromVelocity();

		// Check for maximum speed
		kinematic[i].trimMaxSpeed(20.f);

		// Keep in bounds of the world
		TRIM_WORLD(kinematic[i].position.x);
		TRIM_WORLD(kinematic[i].position.z);
		kinematic[i].position.y = 2.f;
	}
}