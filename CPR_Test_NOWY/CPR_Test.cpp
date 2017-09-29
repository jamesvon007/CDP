#include "CPR_Framework.h"
#include <vector>
#include <string>
#include <ctype.h>
#include <chrono>
#include <iostream>
#include "CrowdManager.h"


float PlacePosY(float height) { return height / 2.f + 0.1f; }

Mesh*	g_sphere = 0;
float	g_angle = 0.0f;
Mesh*	g_unitBox = 0;
CCrowdManager* g_crowdManager;

CCrowdManager* g_crowdManagerB;
std::vector<Sphere> g_obstacles;

bool g_pause = true;

extern DebugPoint g_pointToAvoid;
extern DebugPoint g_pointDest;
extern bool IsLesserOrEqualWithEpsilon(float x, float y);

std::vector<SSkycraper> g_skycrapers;

std::ofstream g_ofs;

struct CameraInfo
{
	CameraInfo()
		: mPosition(0.0f, 5.0f, 0.0f),
		mRight(1.0f, 0.0f, 0.0f),
		mUp(0.0f, 1.0f, 0.0f),
		mLook(0.0f, 0.0f, 1.0f),
		mLastMousePos(0.0f, 0.0f)
	{

	}

	void Walk(float d)
	{
		mPosition += d*mLook;
	}

	void Rise(float d)
	{
		mPosition += d*mUp;
	}

	void Strafe(float d)
	{
		mPosition += d*mRight;
	}

	void Pitch(float angle)
	{
		// Rotate up and look vector about the right vector.
		D3DXMATRIX R;
		D3DXMatrixRotationAxis(&R, &mRight, angle);
		D3DXVec3TransformNormal(&mUp, &mUp, &R);
		D3DXVec3TransformNormal(&mLook, &mLook, &R);
	}

	void RotateY(float angle)
	{
		// Rotate the basis vectors about the world y-axis.
		D3DXMATRIX R;
		D3DXMatrixRotationY(&R, angle);
		D3DXVec3TransformNormal(&mRight, &mRight, &R);
		D3DXVec3TransformNormal(&mUp, &mUp, &R);
		D3DXVec3TransformNormal(&mLook, &mLook, &R);
	}

	D3DXVECTOR3 mPosition;
	D3DXVECTOR3 mLook;
	D3DXVECTOR3 mRight;
	D3DXVECTOR3 mUp;
	D3DXVECTOR2 mLastMousePos;
};

CameraInfo g_camera;

void UpdateCamera(float _deltaTime);
void ReadData();
void RenderSkycrapers();
void CalculateFrameStats();
void RenderUI();
void RenderBalls();
void RenderRedBalls();
void UpdateRedBalls(float dt);


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	This file contains the main loop of the program and works as follows:

		OnInit();

		while( !shouldShutDown )
		{
			OnUpdate( deltaTime );
			OnRender();
		}

		OnShutdown();

	For vector & matrix math we're using the D3DX Library. 
	Here are some useful classes & functions (that may or may not be handy):

		D3DXVECTOR3		- x, y, z (floats)
		D3DXMATRIX		- 16 float values
		D3DXQUATERNION	- x, y, z, w (floats)

		D3DXVECTOR3*	D3DXVec3Normalize( D3DXVECTOR3 *pOut, const D3DXVECTOR3 *pV );
		FLOAT			D3DXVec3Length( const D3DXVECTOR3 *pV );
		FLOAT			D3DXVec3Dot( const D3DXVECTOR3 *pV1, const D3DXVECTOR3 *pV2 );
		D3DXVECTOR3*	D3DXVec3Cross( D3DXVECTOR3 *pOut, const D3DXVECTOR3 *pV1, const D3DXVECTOR3 *pV2 );

	You can find these and more in the DX SDK documentation.

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//----------------------------------------------------------------------------
void OnInit()
{
	// NOTE: there is also unitbox.x, unitsphere.x & unitcylinder.x to use.
	g_sphere = Mesh::LoadFromFile( "resources/meshes/unitsphere.x" );
	g_unitBox = Mesh::LoadFromFile("resources/meshes/unitbox.x");

	ReadData();

	for (std::vector<SSkycraper>::iterator it = g_skycrapers.begin(); it != g_skycrapers.end(); ++it)
	{
		(*it).minBoundingBox = D3DXVECTOR3((*it).position.x - (*it).scale / 2.f, (*it).position.y - (*it).height / 2.f, (*it).position.z - (*it).scale / 2.f);
		(*it).maxBoundingBox = D3DXVECTOR3((*it).position.x + (*it).scale / 2.f, (*it).position.y + (*it).height / 2.f, (*it).position.z + (*it).scale / 2.f);
		g_obstacles.push_back(Sphere(M_HALFSQRT2*(*it).scale, D3DXVECTOR3((*it).position.x, 0.5f, (*it).position.z)));
		DestinationManager::Get()->Push(*it);

		LevelService::Get()->Push(*it);
	}

	DestinationManager::Get()->Finalize(D3DXVECTOR2(-WORLD_SIZE, WORLD_SIZE), D3DXVECTOR2(-WORLD_SIZE, WORLD_SIZE));
	g_crowdManager = new CCrowdManager(g_obstacles);

	g_crowdManagerB = new CCrowdManager(g_obstacles);
	int increment = 0;
	for (std::list<Kinematic>::iterator it = g_crowdManagerB->getKinematic().begin(); it != g_crowdManagerB->getKinematic().end(); ++it)
	{
		Kinematic& agent = *it;
		agent.position.x = 10.f + (increment++);
		agent.position.y = 0.5f;
		agent.position.z = 20.f;
	}

	LevelService::Get()->GetYellowBalls().push_back(g_crowdManager);
	LevelService::Get()->GetYellowBalls().push_back(g_crowdManagerB);

	PursueService::Get()->init();

	g_ofs.open("fps.txt", std::ofstream::out | std::ofstream::trunc);
}

//----------------------------------------------------------------------------
void OnShutdown()
{
	delete g_crowdManager;
	g_crowdManager = nullptr;
	delete g_crowdManagerB;
	g_crowdManagerB = nullptr;
	g_obstacles.clear();

	g_ofs.close();
}

//----------------------------------------------------------------------------
void OnUpdate( float _deltaTime )
{
	UpdateCamera(_deltaTime);
	CalculateFrameStats();
	UpdateRedBalls(_deltaTime);

	if (Keyboard::IsKeyPressed(Keyboard::KEY_SPACE))
	{
		g_pause = !g_pause;
	}

	if (!g_pause)
	{
		g_crowdManager->update(_deltaTime);
		g_crowdManagerB->update(_deltaTime);
		PursueService::Get()->update(_deltaTime);
	}
	
}

//----------------------------------------------------------------------------
void OnRender()
{
	// render mesh
	D3DXVECTOR3 rot(0.0f, 0.0f, 0.0f);

	RenderSkycrapers();
	RenderUI();
	RenderBalls();
	RenderRedBalls();

// 	for (unsigned i = 0; i < OBSTACLES; i++)
// 	{
// 		g_sphere->Render(D3DXVECTOR3(g_obstacles[i].position.x, g_obstacles[i].position.y, g_obstacles[i].position.z), rot,
// 			D3DXVECTOR3(1.f, 1.f, 1.f), D3DXVECTOR4(1.0f, 0.f, 0.f, 1.0f));
// 	}
	
	g_sphere->Render(g_pointToAvoid.pos, rot, D3DXVECTOR3(0.5f, 0.5f, 0.5f), g_pointToAvoid.color);

	g_sphere->Render(g_crowdManager->getWander()->mWanderPoint.pos, rot, 
		D3DXVECTOR3(1.f, 1.f, 1.f), D3DXVECTOR4(1.f, 1.f, 1.f, 1.f));

	g_sphere->Render(g_crowdManagerB->getWander()->mWanderPoint.pos, rot,
		D3DXVECTOR3(1.f, 1.f, 1.f), D3DXVECTOR4(0.5f, 0.5f, 0.5f, 1.f));
	

	// Render ground
	g_unitBox->Render(D3DXVECTOR3(0.0f, 0.0f, 0.0f), rot, D3DXVECTOR3(50.0f, 0.1f, 50.0f), D3DXVECTOR4 (0.0f, 0.5f, 0.7f, 1.0f));
}

void UpdateCamera(float _deltaTime)
{
	if (Keyboard::IsKeyPressed(Keyboard::KEY_W))
	{
		g_camera.Walk(10.f * _deltaTime);
	}
	else if (Keyboard::IsKeyPressed(Keyboard::KEY_S))
	{
		g_camera.Walk(-10.f * _deltaTime);
	}
	else if (Keyboard::IsKeyPressed(Keyboard::KEY_A))
	{
		g_camera.Strafe(-10.f * _deltaTime);
	}
	else if (Keyboard::IsKeyPressed(Keyboard::KEY_D))
	{
		g_camera.Strafe(10.f * _deltaTime);
	}
	else if (Keyboard::IsKeyPressed(Keyboard::KEY_UP))
	{
		g_camera.Rise(10.f * _deltaTime);
	}
	else if (Keyboard::IsKeyPressed(Keyboard::KEY_DOWN))
	{
		g_camera.Rise(-10.f * _deltaTime);
	}
	else if (Keyboard::IsKeyPressed(Keyboard::KEY_LEFT))
	{
		g_camera.RotateY(-5.f * _deltaTime);
	}
	else if (Keyboard::IsKeyPressed(Keyboard::KEY_RIGHT))
	{
		g_camera.RotateY(5.f * _deltaTime);
	}

	D3DXVECTOR2 newPos = Mouse::GetPosition();
	float dx = D3DXToRadian(0.1f*(newPos.x - g_camera.mLastMousePos.x));
	float dy = D3DXToRadian(0.1f*(newPos.y - g_camera.mLastMousePos.y));
	g_camera.Pitch(dy);
	g_camera.RotateY(dx);
	g_camera.mLastMousePos = newPos;

	D3DXVec3Normalize(&g_camera.mLook, &g_camera.mLook);
	D3DXVec3Cross(&g_camera.mUp, &g_camera.mLook, &g_camera.mRight);
	D3DXVec3Normalize(&g_camera.mUp, &g_camera.mUp);
	D3DXVec3Cross(&g_camera.mRight, &g_camera.mUp, &g_camera.mLook);

	// update camera
	g_angle += _deltaTime;
	Camera::LookAt(g_camera.mPosition, g_camera.mPosition + 10.f * g_camera.mLook);
}

void UpdateRedBalls(float dt)
{
	static std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	static std::chrono::steady_clock::time_point end = start;

	std::chrono::milliseconds diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	if (diff.count() >= 1000)
	{
		if (Mouse::LeftMouseButton())
		{
			LevelService::Get()->AddRedBall(g_camera.mPosition, 5.f*g_camera.mLook, 0.1f*g_camera.mLook, 0.2f);
			start = std::chrono::steady_clock::now();
		}
	}

	LevelService::Get()->Update(dt);
	end = std::chrono::steady_clock::now();
}

void RenderUI()
{
	D3DXVECTOR4 color(1.0f, 0.0f, 0.0f, 1.0f);
	D3DXVECTOR3 rot(0.0f, 0.0f, 0.0f);

	D3DXVECTOR3 trans = g_camera.mPosition + 0.2f * g_camera.mLook;

	g_sphere->Render(trans, rot, D3DXVECTOR3(0.001f, 0.001f, 0.001f), color);
}

void RenderBalls()
{
	for (std::list<Kinematic>::iterator it = g_crowdManager->getKinematic().begin(); it != g_crowdManager->getKinematic().end(); ++it)
	{
		g_sphere->Render(D3DXVECTOR3((*it).position.x, (*it).position.y, (*it).position.z),
			D3DXVECTOR3(0.f, (*it).orientation, 0.f), D3DXVECTOR3(0.2f, 0.2f, 0.2f),
			D3DXVECTOR4(1.0f, 1.0f, 0.0f, 1.0f));
	}
	for (std::list<Kinematic>::iterator it = g_crowdManagerB->getKinematic().begin(); it != g_crowdManagerB->getKinematic().end(); ++it)
	{
		g_sphere->Render(D3DXVECTOR3((*it).position.x, (*it).position.y, (*it).position.z),
			D3DXVECTOR3(0.f, (*it).orientation, 0.f), D3DXVECTOR3(0.2f, 0.2f, 0.2f),
			D3DXVECTOR4(0.0f, 1.0f, 1.0f, 1.0f));
	}
}

void RenderRedBalls()
{
	for (std::list<LevelService::Simulation>::const_iterator it = LevelService::Get()->GetRedBalls().begin(); it != LevelService::Get()->GetRedBalls().end(); ++it)
	{
		g_sphere->Render(D3DXVECTOR3((*it).position.x, (*it).position.y, (*it).position.z),
			D3DXVECTOR3(0.f, 0.f, 0.f), D3DXVECTOR3(0.2f, 0.2f, 0.2f), D3DXVECTOR4(1.0f, 0.0f, 0.0f, 1.0f));
	}
}

void RenderSkycrapers()
{
	D3DXVECTOR3 rot(0.0f, 0.0f, 0.0f);

	for (std::vector<SSkycraper>::iterator it = g_skycrapers.begin(); it != g_skycrapers.end(); ++it)
	{
		SSkycraper skycraper = *it;

		g_unitBox->Render(skycraper.position, rot, D3DXVECTOR3(skycraper.scale, skycraper.height, skycraper.scale), skycraper.color);
	}
}

void ReadData()
{
	std::string line;
	std::fstream city("Resources\\city.txt", std::fstream::in);
	if (city.is_open())
	{
		while (city.good())
		{
			std::getline(city, line);
			int first = 0;
			int last = 0;
			for (std::string::iterator it = line.begin(); it != line.end(); ++it)
			{
				if (*it == '/')
				{
					break;
				}

				if (!isdigit(line.at(first)))
				{
					++first;
				}
				else if (*it == ' ' || *it == ',' || *it == ';')
				{
					float height = std::stof(line.substr(first, last - first));
					if (!IsLesserOrEqualWithEpsilon(height, 0.f))
					{
						g_skycrapers.push_back(SSkycraper(height));
					}
					first = last + 1;
				}
				++last;
			}

			//take care the edge case: last number
			if (!line.empty() && line.at(first) && last - first > 0)
			{
				float height = std::stof(line.substr(first, last - first));
				if (!IsLesserOrEqualWithEpsilon(height, 0.f))
				{
					g_skycrapers.push_back(SSkycraper(height));
				}
			}
		}

		city.close();
	}

	int rowColMax = static_cast<int>((sqrt(g_skycrapers.size())));
	int row = 0;
	int col = 0;

	for (std::vector<SSkycraper>::iterator it = g_skycrapers.begin(); it != g_skycrapers.end(); ++it)
	{
		SSkycraper& skycraper = *it;
		skycraper.position = D3DXVECTOR3(7.0f * (col - rowColMax / 2), PlacePosY(skycraper.height), 7.0f * (row - rowColMax / 2));
		skycraper.scale = 4.f;

		if (col++ == rowColMax)
		{
			col = 0;
			row++;
		}
	}
}

void CalculateFrameStats()
{
	static int frameCnt = 0;
	static std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	static std::chrono::steady_clock::time_point end = start;

	frameCnt++;

	std::chrono::milliseconds diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	if (diff.count() >= 1000)
	{
		float fps = (float)frameCnt; // fps = frameCnt / 1
		
		g_ofs << "FPS: " << fps << std::endl;

		// Reset for next average.
		frameCnt = 0;
		start = std::chrono::steady_clock::now();
	}

	end = std::chrono::steady_clock::now();
}
