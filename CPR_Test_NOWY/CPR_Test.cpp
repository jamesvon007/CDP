#include "CPR_Framework.h"

Mesh*	g_mesh = 0;
float	g_angle = 0.0f;
Mesh*	g_unitBox = 0;

struct CameraInfo
{
	CameraInfo()
		: mPosition(0.0f, 0.0f, 0.0f),
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
	g_mesh = Mesh::LoadFromFile( "resources/meshes/unitcylinder.x" );
	g_unitBox = Mesh::LoadFromFile("resources/meshes/unitbox.x");
}

//----------------------------------------------------------------------------
void OnShutdown()
{
}

//----------------------------------------------------------------------------
void OnUpdate( float _deltaTime )
{
	UpdateCamera(_deltaTime);
}

//----------------------------------------------------------------------------
void OnRender()
{
	// render mesh
	D3DXVECTOR3 pos( 0.0f, 0.0f, 0.0f );
	D3DXVECTOR3 rot( 0.0f, 0.0f, 0.0f );
	D3DXVECTOR3 sca( 1.0f, 1.0f, 1.0f );
	D3DXVECTOR4 color( 1.0f, 0.5f, 0.0f, 1.0f );
	g_mesh->Render( pos, rot, sca, color );

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