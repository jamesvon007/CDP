#include "CPR_Framework.h"
#include <vector>
#include <fstream>
#include <string>
#include <ctype.h>
#include <chrono>
#include <iostream>
#include "CrowdManager.h"

static const float EPSILON = 0.000001f;
bool IsLesserOrEqualWithEpsilon(float x, float y) { return ((x)-(y) < EPSILON); }
float PlacePosY(float height) { return height / 2.f + 0.1f; }

Mesh*	g_sphere = 0;
float	g_angle = 0.0f;
Mesh*	g_unitBox = 0;
CCrowdManager gCrowdManager;


std::vector<float> g_skycrapers;

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
}

//----------------------------------------------------------------------------
void OnShutdown()
{
}

//----------------------------------------------------------------------------
void OnUpdate( float _deltaTime )
{
	UpdateCamera(_deltaTime);
	CalculateFrameStats();

	gCrowdManager.update(_deltaTime);
}

//----------------------------------------------------------------------------
void OnRender()
{
	// render mesh
	D3DXVECTOR3 rot(0.0f, 0.0f, 0.0f);

	//RenderSkycrapers();
	RenderUI();
	RenderBalls();

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

void RenderUI()
{
	D3DXVECTOR4 color(1.0f, 0.0f, 0.0f, 1.0f);
	D3DXVECTOR3 rot(0.0f, 0.0f, 0.0f);

	D3DXVECTOR3 trans = g_camera.mPosition + 0.2f * g_camera.mLook;

	g_sphere->Render(trans, rot, D3DXVECTOR3(0.001f, 0.001f, 0.001f), color);
}

void RenderBalls()
{
	D3DXVECTOR4 color(1.0f, 1.0f, 0.0f, 1.0f);

	for (unsigned i = 0; i < BOIDS; i++) 
	{
		Kinematic agent = gCrowdManager.getKinematic()[i];
		g_sphere->Render(D3DXVECTOR3(agent.position.x, agent.position.y, agent.position.z),
			D3DXVECTOR3(0.f, agent.orientation, 0.f), D3DXVECTOR3(0.5f, 0.5f, 0.5f), color);
	}
}

void RenderSkycrapers()
{
	float width = 4.f;
	D3DXVECTOR4 color(1.0f, 0.5f, 0.0f, 1.0f);
	D3DXVECTOR3 rot(0.0f, 0.0f, 0.0f);
	int rowColMax = static_cast<int>((sqrt(g_skycrapers.size())));
	int row = 0;
	int col = 0;

	for (std::vector<float>::iterator it = g_skycrapers.begin(); it != g_skycrapers.end(); ++it)
	{
		float height = *it;

		g_unitBox->Render(D3DXVECTOR3(7.0f * (col - rowColMax/2), PlacePosY(height), 7.0f * (row - rowColMax/2)),
			rot, D3DXVECTOR3(width, height, width), color);
		if (col++ == rowColMax)
		{
			col = 0;
			row++;
		}
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
						g_skycrapers.push_back(height);
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
					g_skycrapers.push_back(height);
				}
			}
		}

		city.close();
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
		
		std::cout << "FPS: " << fps << std::endl;

		// Reset for next average.
		frameCnt = 0;
		start = std::chrono::steady_clock::now();
	}

	end = std::chrono::steady_clock::now();
}
