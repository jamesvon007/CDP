#pragma once

#include <d3dx9.h>
#include <vector>

//----------------------------------------------------------------------------
//	Simple class for handling mouse input
//----------------------------------------------------------------------------
class Mouse
{
public:
	static D3DXVECTOR2 GetPosition();
	static bool LeftMouseButton();
	static bool RightMouseButton();
};

//----------------------------------------------------------------------------
//	Simple class for handling keyboard input
//----------------------------------------------------------------------------
class Keyboard
{
public:
	enum Key
	{
		KEY_UP,
		KEY_DOWN,
		KEY_LEFT,
		KEY_RIGHT,
		KEY_SPACE,
		KEY_RETURN,
		KEY_W,
		KEY_A,
		KEY_S,
		KEY_D
	};

	static bool IsKeyPressed( Key key );
};

//----------------------------------------------------------------------------
//	A class for controlling the camera
//----------------------------------------------------------------------------
class Camera
{
public:
	static void LookAt( const D3DXVECTOR3& _eye, const D3DXVECTOR3& _target );
};

//----------------------------------------------------------------------------
//	Mesh resource handling class
//----------------------------------------------------------------------------
class Mesh
{
protected:
	Mesh();

public:
	~Mesh();

	static Mesh* LoadFromFile( char filename[] );

	void Render( const D3DXVECTOR3& _position, const D3DXVECTOR3& _rotation, const D3DXVECTOR3& _scale, D3DXVECTOR4 _color );

private:
	ID3DXMesh*			m_mesh;
	int					m_numSubsets;
};