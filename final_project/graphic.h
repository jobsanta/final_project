#include <PxPhysicsAPI.h>
#include <PxExtensionsAPI.h>
#include <PxDefaultErrorCallback.h>
#include <PxDefaultAllocator.h>
#include <PxDefaultSimulationFilterShader.h>
#include <PxDefaultCpuDispatcher.h>
#include <PxShapeExt.h>
#include <PxMat33.h>
#include <PxSimpleFactory.h>
#include <vector>
#include <PxVisualDebuggerExt.h>

#include <stdio.h>
#include <d3d10.h>
#include <D3DX10.h>
#include <DxErr.h>
#include "d3dUtil.h"
#include <iostream>

#include "Box.h"


#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d10.lib")
#pragma comment(lib, "d3dx10.lib")

#include "nvapi.h"
#pragma comment(lib, "nvapi.lib")
#include "terrainclass.h"

#include <tchar.h>
#include "nvStereo.h"

using namespace physx;
using namespace std;


class Graphic
{
public:

	Graphic();
	~Graphic();
	void Initialize(HWND);
	void Render();

private:

	// NVAPI Stereo Handle
	StereoHandle                 g_StereoHandle;
	float                        g_EyeSeparation;
	float                        g_Separation;
	float                        g_Convergence;
	nv::StereoParametersD3D10    g_StereoParamD3D10;

	// Directx and Effect
	ID3D10Device*             g_D3D10Device;
	DXGI_SWAP_CHAIN_DESC      g_DXGISwapChainDesc;
	IDXGISwapChain*           g_DXGISwapChain;
	ID3D10RenderTargetView*   g_D3D10BackBufferRTV;
	ID3D10DepthStencilView*   g_D3D10DepthBufferDSV;
	D3D10_VIEWPORT            g_D3D10MainViewport;
	ID3D10ShaderResourceView* g_D3D10DepthBufferSRV;

	ID3D10Effect*               mFX;
	ID3D10EffectTechnique*      mTech;
	ID3D10InputLayout*          mVertexLayout;
	ID3D10EffectMatrixVariable* mfxWVPVar;

	D3DXMATRIX mView;
	D3DXMATRIX mProj;
	D3DXMATRIX mWVP;


	PxPhysics*                gPhysicsSDK = NULL;
	PxDefaultErrorCallback    gDefaultErrorCallback;
	PxDefaultAllocator        gDefaultAllocatorCallback;
	PxSimulationFilterShader  gDefaultFilterShader = PxDefaultSimulationFilterShader;
	PxFoundation*             gFoundation = NULL;

	PxScene*              gScene = NULL;
	PxReal                myTimestep = 1.0f / 1000.0f;
	vector<PxRigidActor*> boxes;
	PxDistanceJoint*      gMouseJoint = NULL;
	PxRigidDynamic*       gMouseSphere = NULL;
	PxReal                gMouseDepth = 0.0f;
	PxRigidDynamic*       gSelectedActor = NULL;

	Box          mBox;
	TerrainClass terrain;

	void       buildFX();
	void       buildVertexLayouts();
	void       InitializePhysX();
	void       ShutdownPhysX();
	void       StepPhysX();
	D3DXMATRIX PxtoXMMatrix(PxTransform input);
	void       DrawBox(PxShape* pShape, PxRigidActor* actor);
	void       DrawShape(PxShape* shape, PxRigidActor* actor);
	void       DrawActor(PxRigidActor* actor);
	void       RenderActors();
	bool       CreateDevice(HWND);
	void       FreeDevice();




};
