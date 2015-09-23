#include "graphic.h"


Graphic::Graphic()
{
}

Graphic::~Graphic()
{
	ShutdownPhysX();
	FreeDevice();
}

//Defining a custome filter shader 
PxFilterFlags customFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eCONTACT_DEFAULT
		| PxPairFlag::eTRIGGER_DEFAULT
		| PxPairFlag::eNOTIFY_CONTACT_POINTS
		| PxPairFlag::eCCD_LINEAR; //Set flag to enable CCD (Continuous Collision Detection) 

	return PxFilterFlag::eDEFAULT;
}



bool Graphic::Initialize(HWND g_hWnd)
{
	if (!CreateDevice(g_hWnd))
	{
		return false;
	}
	InitializePhysX();
}

void Graphic::buildFX()
{
	DWORD shaderFlags = D3D10_SHADER_ENABLE_STRICTNESS;
#if defined( DEBUG ) || defined( _DEBUG )
	shaderFlags |= D3D10_SHADER_DEBUG;
	shaderFlags |= D3D10_SHADER_SKIP_OPTIMIZATION;
#endif

	ID3D10Blob* compilationErrors = 0;
	HRESULT hr = 0;
	hr = D3DX10CreateEffectFromFile(L"color.fx", 0, 0,
		"fx_4_0", shaderFlags, 0, g_D3D10Device, 0, 0, &mFX_Box, &compilationErrors, 0);
	if (FAILED(hr))
	{
		if (compilationErrors)
		{
			MessageBoxA(0, (char*)compilationErrors->GetBufferPointer(), 0, 0);
			ReleaseCOM(compilationErrors);
		}
		DXTrace(__FILE__, (DWORD)__LINE__, hr, L"D3DX10CreateEffectFromFile", true);
	}

	mTech_Box = mFX_Box->GetTechniqueByName("ColorTech");
	mfxWVPVar_Box = mFX_Box->GetVariableByName("gWVP")->AsMatrix();
	mfxCameraDistance_Box = mFX_Box->GetVariableByName("cameraDepth")->AsScalar();


	hr = D3DX10CreateEffectFromFile(L"sphere.fx", 0, 0,
		"fx_4_0", shaderFlags, 0, g_D3D10Device, 0, 0, &mFX_Sphere, &compilationErrors, 0);
	if (FAILED(hr))
	{
		if (compilationErrors)
		{
			MessageBoxA(0, (char*)compilationErrors->GetBufferPointer(), 0, 0);
			ReleaseCOM(compilationErrors);
		}
		DXTrace(__FILE__, (DWORD)__LINE__, hr, L"D3DX10CreateEffectFromFile", true);
	}

	mTech_Sphere = mFX_Sphere->GetTechniqueByName("ColorTech");
	mfxWVPVar_Sphere = mFX_Sphere->GetVariableByName("gWVP")->AsMatrix();
	mfxCameraDistance_Sphere = mFX_Sphere->GetVariableByName("cameraDepth")->AsScalar();
}

void Graphic::buildVertexLayouts()
{
	// Create the vertex input layout.
	D3D10_INPUT_ELEMENT_DESC vertexDesc[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0 }
	};

	// Create the input layout
	D3D10_PASS_DESC PassDesc;
	mTech_Box->GetPassByIndex(0)->GetDesc(&PassDesc);
	HR(g_D3D10Device->CreateInputLayout(vertexDesc, 2, PassDesc.pIAInputSignature,
		PassDesc.IAInputSignatureSize, &mVertexLayout));

	mTech_Sphere->GetPassByIndex(0)->GetDesc(&PassDesc);
	HR(g_D3D10Device->CreateInputLayout(vertexDesc, 2, PassDesc.pIAInputSignature,
		PassDesc.IAInputSignatureSize, &mVertexLayout));
}

bool Graphic::CreateDevice(HWND g_hWnd)
{
	D3DXMatrixIdentity(&mView);
	D3DXMatrixIdentity(&mProj);
	D3DXMatrixIdentity(&mWVP);

	DEVMODE devMode;

	if (!EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &devMode))
		return false;

	HRESULT hr = S_OK;

	// Prepare NVAPI for use in this application
	NvAPI_Status status;
	status = NvAPI_Initialize();
	if (status != NVAPI_OK)
	{
		NvAPI_ShortString errorMessage;
		NvAPI_GetErrorMessage(status, errorMessage);
		MessageBoxA(NULL, errorMessage, "Unable to initialize NVAPI", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
	}
	else
	{
		// Check the Stereo availability
		NvU8 isStereoEnabled;
		status = NvAPI_Stereo_IsEnabled(&isStereoEnabled);

		// Stereo status report an error
		if (status != NVAPI_OK)
		{
			// GeForce Stereoscopic 3D driver is not installed on the system
			MessageBoxA(NULL, "Stereo is not available\nMake sure the stereo driver is installed correctly", "Stereo not available", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);

		}
		// Stereo is available but not enabled, let's enable it
		else if (NVAPI_OK == status && !isStereoEnabled)
		{
			MessageBoxA(NULL, "Stereo is available but not enabled\nLet's enable it", "Stereo not enabled", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
			status = NvAPI_Stereo_Enable();
		}

		NvAPI_Stereo_CreateConfigurationProfileRegistryKey(NVAPI_STEREO_DEFAULT_REGISTRY_PROFILE);
	}


	IDXGIAdapter* capableAdapter = NULL;
	{
		IDXGIFactory *pFactory;
		hr = CreateDXGIFactory(__uuidof(IDXGIFactory), (void**)(&pFactory));

		for (UINT adapter = 0; !capableAdapter; ++adapter)
		{
			// get a candidate DXGI adapter
			IDXGIAdapter* pAdapter = NULL;
			hr = pFactory->EnumAdapters(adapter, &pAdapter);
			if (FAILED(hr))
			{
				break;
			}
			// query to see if there exists a corresponding compute device

			// if so, mark it as the one against which to create our d3d10 device
			capableAdapter = pAdapter;
			capableAdapter->AddRef();

			pAdapter->Release();
		}
		pFactory->Release();
	}

	// Create device and swapchain
	ZeroMemory(&g_DXGISwapChainDesc, sizeof(g_DXGISwapChainDesc));
	g_DXGISwapChainDesc.BufferCount = 1;
	g_DXGISwapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	g_DXGISwapChainDesc.BufferDesc.RefreshRate.Denominator = 1;
	g_DXGISwapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	g_DXGISwapChainDesc.OutputWindow = g_hWnd;
	g_DXGISwapChainDesc.SampleDesc.Count = 1;
	g_DXGISwapChainDesc.SampleDesc.Quality = 0;


	//RECT rect;
	//GetWindowRect(g_hWnd, &rect);

	//g_DXGISwapChainDesc.Windowed = true;
	//g_DXGISwapChainDesc.BufferDesc.RefreshRate.Numerator = devMode.dmDisplayFrequency;
	//g_DXGISwapChainDesc.BufferDesc.Width = rect.right - rect.left;
	//g_DXGISwapChainDesc.BufferDesc.Height = rect.bottom - rect.top;
	//// Fullscreen mode mode
	g_DXGISwapChainDesc.Windowed = FALSE;
	//g_DXGISwapChainDesc.BufferDesc.RefreshRate.Numerator = devMode.dmDisplayFrequency;
	g_DXGISwapChainDesc.BufferDesc.Width = devMode.dmPelsWidth;
	g_DXGISwapChainDesc.BufferDesc.Height = devMode.dmPelsHeight;

	hr = D3D10CreateDeviceAndSwapChain(
		capableAdapter,
		D3D10_DRIVER_TYPE_HARDWARE,
		NULL,
		0, //D3D10_CREATE_DEVICE_DEBUG,
		D3D10_SDK_VERSION,
		&g_DXGISwapChainDesc,
		&g_DXGISwapChain,
		&g_D3D10Device);

	capableAdapter->Release();

	status = NvAPI_Stereo_CreateHandleFromIUnknown(g_D3D10Device, &g_StereoHandle);
	if (NVAPI_OK != status)
	{
		MessageBoxA(NULL, "Couldn't create the StereoHandle", "NvAPI_Stereo_CreateHandleFromIUnknown failed", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
	}

	if (NVAPI_OK != NvAPI_Stereo_GetEyeSeparation(g_StereoHandle, &g_EyeSeparation))
	{
		MessageBoxA(NULL, "Couldn't get the hardware eye separation", "NvAPI_Stereo_GetEyeSeparation failed", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
	}

	// Create the RenderTarget View
	ID3D10Texture2D* lBuffer;
	g_DXGISwapChain->GetBuffer(0, __uuidof(ID3D10Texture2D), (LPVOID*)&lBuffer);
	if (FAILED(g_D3D10Device->CreateRenderTargetView(lBuffer,
		NULL,
		&g_D3D10BackBufferRTV)))
	{
		MessageBox(NULL, _T("Failed to create RenderTargetView for D3D10."), _T("Test App Error"), MB_SETFOREGROUND | MB_OK | MB_SYSTEMMODAL | MB_TOPMOST);
		return false;
	}
	lBuffer->Release();

	// Create a depthStencil buffer and view
	D3D10_TEXTURE2D_DESC texDesc;
	memset(&texDesc, 0, sizeof(D3D10_TEXTURE2D_DESC));
	texDesc.Width = g_DXGISwapChainDesc.BufferDesc.Width;
	texDesc.Height = g_DXGISwapChainDesc.BufferDesc.Height;
	texDesc.MipLevels = 1;
	texDesc.ArraySize = 1;
	texDesc.SampleDesc.Count = 1;
	texDesc.Usage = D3D10_USAGE_DEFAULT;
	texDesc.BindFlags = D3D10_BIND_DEPTH_STENCIL | D3D10_BIND_SHADER_RESOURCE;
	texDesc.Format = DXGI_FORMAT_R32_TYPELESS;

	ID3D10Texture2D* lSurface = 0;
	g_D3D10Device->CreateTexture2D(&texDesc, 0, &lSurface);

	D3D10_DEPTH_STENCIL_VIEW_DESC dsvDesc;
	memset(&dsvDesc, 0, sizeof(D3D10_DEPTH_STENCIL_VIEW_DESC));
	dsvDesc.Format = DXGI_FORMAT_D32_FLOAT;
	dsvDesc.ViewDimension = D3D10_DSV_DIMENSION_TEXTURE2D;
	dsvDesc.Texture2D.MipSlice = 0;
	g_D3D10Device->CreateDepthStencilView(lSurface, &dsvDesc, &g_D3D10DepthBufferDSV);

	D3D10_SHADER_RESOURCE_VIEW_DESC srvDesc;
	memset(&srvDesc, 0, sizeof(D3D10_SHADER_RESOURCE_VIEW_DESC));
	srvDesc.Format = DXGI_FORMAT_R32_FLOAT;
	srvDesc.ViewDimension = D3D10_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Texture2D.MipLevels = 1;
	srvDesc.Texture2D.MostDetailedMip = 0;
	g_D3D10Device->CreateShaderResourceView(lSurface, &srvDesc, &g_D3D10DepthBufferSRV);

	lSurface->Release();

	// Setup the viewport
	g_D3D10MainViewport.Width = g_DXGISwapChainDesc.BufferDesc.Width;
	g_D3D10MainViewport.Height = g_DXGISwapChainDesc.BufferDesc.Height;
	g_D3D10MainViewport.MinDepth = 0.0f;
	g_D3D10MainViewport.MaxDepth = 1.0f;
	g_D3D10MainViewport.TopLeftX = 0;
	g_D3D10MainViewport.TopLeftY = 0;


	ID3D10RasterizerState* rs;
	D3D10_RASTERIZER_DESC rasterizerState;
	rasterizerState.FillMode = D3D10_FILL_SOLID;
	rasterizerState.CullMode = D3D10_CULL_NONE;
	rasterizerState.FrontCounterClockwise = false;
	rasterizerState.DepthBias = false;
	rasterizerState.DepthBiasClamp = 0;
	rasterizerState.SlopeScaledDepthBias = 0;
	rasterizerState.DepthClipEnable = false;
	rasterizerState.ScissorEnable = false;
	rasterizerState.MultisampleEnable = false;
	rasterizerState.AntialiasedLineEnable = false;
	g_D3D10Device->CreateRasterizerState(&rasterizerState, &rs);
	g_D3D10Device->RSSetState(rs);


	D3DXVECTOR3 pos(0.0f, -0.3f, -0.8f);
	D3DXVECTOR3 target(0.0f, 0.3f, 0.0f);
	D3DXVECTOR3 up(0.0f, 1.0f, 0.0f);
	D3DXMatrixLookAtLH(&mView, &pos, &target, &up);

	float Near = 2.f;
	float Far = 1000.0f;
	//float fAspectRatio = (FLOAT)g_D3D10MainViewport.Width / (FLOAT)g_D3D10MainViewport.Height;
	//D3DXMatrixPerspectiveFovLH(&mProj, D3DX_PI / 4, fAspectRatio, Near, Far);

	float fAspectRatio = (FLOAT)g_D3D10MainViewport.Width / (FLOAT)g_D3D10MainViewport.Height;
	D3DXMatrixPerspectiveFovLH(&mProj, D3DX_PI / 6, fAspectRatio, Near, Far);


	

	// Create 3D models that will use
	mBox.init(g_D3D10Device, BoxSize);
	terrain.Initialize(g_D3D10Device);

	DXUTCreateSphere(g_D3D10Device, 0.05f, 20, 20, &sphere);

	sphere->CommitToDevice();


	// Simple FX and layout
	buildFX();
	buildVertexLayouts();

	return true;
}

void Graphic::FreeDevice()
{
	if (g_DXGISwapChain)
	{
		BOOL isFullScreen;
		g_DXGISwapChain->GetFullscreenState(&isFullScreen, 0);
		if (isFullScreen)
		{
			g_DXGISwapChain->SetFullscreenState(FALSE, 0);
		}
	}

	if (g_D3D10DepthBufferDSV)
	{
		g_D3D10DepthBufferDSV->Release();
		g_D3D10DepthBufferDSV = 0;
	}
	if (g_D3D10DepthBufferSRV)
	{
		g_D3D10DepthBufferSRV->Release();
		g_D3D10DepthBufferSRV = 0;
	}
	if (g_D3D10BackBufferRTV)
	{
		g_D3D10BackBufferRTV->Release();
		g_D3D10BackBufferRTV = 0;
	}
	if (g_DXGISwapChain)
	{
		g_DXGISwapChain->Release();
		g_DXGISwapChain = 0;
	}

	g_StereoParamD3D10.destroyGraphics();

	if (g_D3D10Device)
	{
		g_D3D10Device->Release();
		g_D3D10Device = 0;
	}
	ReleaseCOM(mFX_Sphere);
	ReleaseCOM(mFX_Box);
	ReleaseCOM(mVertexLayout);
}

void Graphic::Render(float x, float y, float z)
{
	D3DXVECTOR3 pos(x,y,z );
	D3DXVECTOR3 target(0.0f, y, 0.0f);
	D3DXVECTOR3 up(0.0f, 1.0f, 0.0f);
	D3DXMatrixLookAtLH(&mView, &pos, &target, &up);

	float Near = 2.f;
	float Far = 1000.0f;
	float fAspectRatio = (FLOAT)g_D3D10MainViewport.Width / (FLOAT)g_D3D10MainViewport.Height;
	float angle = 2*atan2(1.7, abs(z));
	D3DXMatrixPerspectiveFovLH(&mProj, angle, fAspectRatio, Near, Far);
	
	static unsigned int frameNb = 0;

	g_D3D10Device->RSSetViewports(1, &g_D3D10MainViewport);

	float color[] = { 0.0f, 0.0f, 0.0f, 1.f };
	g_D3D10Device->ClearRenderTargetView(g_D3D10BackBufferRTV, color);
	g_D3D10Device->ClearDepthStencilView(g_D3D10DepthBufferDSV, D3D10_CLEAR_DEPTH, 1, 0);

	g_D3D10Device->OMSetRenderTargets(1, &g_D3D10BackBufferRTV, g_D3D10DepthBufferDSV);

	if (frameNb > 2)
	{

		if (gScene && frameNb > 20)
		{
			StepPhysX();
		}

		if (boxesJoint.size() != 0)
		{
			for (int i = 0; i < boxesJoint.size(); i++)
			{
				PxVec3 offset(0, 0.01, 0);
				if (boxesJoint[i] != NULL && particleJoint[i] != NULL)
				{
					PxDistanceJoint* joint = PxDistanceJointCreate(*gPhysicsSDK, boxesJoint[i], PxTransform(-offset), particleJoint[i], PxTransform(offset));
					if (joint != NULL)
					{
						joint->setMaxDistance(0.5f);
						joint->setDamping(0.5);
						joint->setStiffness(2000.0f);
						joint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
						joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
					}
				}
			}
			boxesJoint.clear();
			particleJoint.clear();

		}


		float sep, conv;
		if (NVAPI_OK != NvAPI_Stereo_SetConvergence(g_StereoHandle, abs(z)*10))
		{
			MessageBoxA(NULL, "Couldn't set the convergence", "NvAPI_Stereo_SetConvergence failed", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
		}
		if (NVAPI_OK != NvAPI_Stereo_GetSeparation(g_StereoHandle, &sep))
		{
			MessageBoxA(NULL, "Couldn't get the separation", "NvAPI_Stereo_GetSeparation failed", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
		}
		if (NVAPI_OK != NvAPI_Stereo_GetConvergence(g_StereoHandle, &conv))
		{
			MessageBoxA(NULL, "Couldn't get the convergence", "NvAPI_Stereo_GetConvergence failed", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);

		}
		if (sep * 0.01 != g_Separation || conv != g_Convergence)
		{
			g_Separation = sep * 0.01;
			g_Convergence = conv;
			//g_pStereoSeparation->SetFloat(g_Separation);
			//g_pStereoConvergence->SetFloat(g_Convergence);

			g_StereoParamD3D10.updateStereoParamsMap(g_D3D10Device, g_EyeSeparation, g_Separation, g_Convergence);
		}
		D3DXMATRIX mat;
		D3D10_TECHNIQUE_DESC techDesc;



		D3DXMatrixTranslation(&mat, -50.0, 0.0, -50.0);
		mWVP = mat* mView*mProj;
		mfxWVPVar_Box->SetMatrix((float*)&mWVP);
		mfxCameraDistance_Box->SetFloat(z);
		mfxCameraDistance_Sphere->SetFloat(z);
		terrain.Render(g_D3D10Device);
		mTech_Box->GetDesc(&techDesc);
		for (UINT p = 0; p < techDesc.Passes; ++p)
		{
			mTech_Box->GetPassByIndex(p)->Apply(0);

			g_D3D10Device->DrawIndexed(terrain.GetIndexCount(), 0, 0);
		}



			g_D3D10Device->OMSetDepthStencilState(0, 0);
			float blendFactors[] = { 0.0f, 0.0f, 0.0f, 0.0f };
			g_D3D10Device->OMSetBlendState(0, blendFactors, 0xffffffff);
			g_D3D10Device->IASetInputLayout(mVertexLayout);
			g_D3D10Device->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			RenderActors(true);
		
			// Stop drawing in the depth buffer so we can fetch from it
			g_D3D10Device->OMSetRenderTargets(1, &g_D3D10BackBufferRTV, 0);
	

	}

	g_DXGISwapChain->Present(0, 0);


	frameNb++;
}

void Graphic::InitializePhysX() 
{

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);

	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());
	if (gPhysicsSDK == NULL) {
		cerr << "Error creating PhysX3 device." << endl;
		cerr << "Exiting..." << endl;
		exit(1);
	}

	if (!PxInitExtensions(*gPhysicsSDK))
		cerr << "PxInitExtensions failed!" << endl;

	if (gPhysicsSDK->getPvdConnectionManager() == NULL)
		return;
	const char* pvd_host_ip = "127.0.0.1";
	int port = 5425;
	unsigned int timeout = 100;


	//--- Debugger
	PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();
	theConnection = PxVisualDebuggerExt::createConnection(gPhysicsSDK->getPvdConnectionManager(),
		pvd_host_ip, port, timeout, connectionFlags);


	//Create the scene
	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);

	if (!sceneDesc.cpuDispatcher) {
		PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(3);
		if (!mCpuDispatcher)
			cerr << "PxDefaultCpuDispatcherCreate failed!" << endl;
		sceneDesc.cpuDispatcher = mCpuDispatcher;
	}
	if (!sceneDesc.filterShader)
		sceneDesc.filterShader = customFilterShader;//gDefaultFilterShader;
	sceneDesc.simulationEventCallback = this;

	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;


	gScene = gPhysicsSDK->createScene(sceneDesc);
	if (!gScene)
		cerr << "createScene failed!" << endl;

	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);


	PxMaterial* mMaterial = gPhysicsSDK->createMaterial(0.5, 0.5, 0.5);

	//Create actors 
	//1) Create ground plane
	PxReal d = 0.0f;
	PxTransform pose = PxTransform(PxVec3(0.0f, 0, 0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));

	PxRigidStatic* plane = gPhysicsSDK->createRigidStatic(pose);
	if (!plane)
		cerr << "create plane failed!" << endl;

	PxShape* shape = plane->createShape(PxPlaneGeometry(), *mMaterial);
	if (!shape)
		cerr << "create shape failed!" << endl;
	gScene->addActor(*plane);

	pose = PxTransform(PxVec3(0.0f, 0, 0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));

	PxRigidStatic* plane2 = gPhysicsSDK->createRigidStatic(pose);
	if (!plane)
		cerr << "create plane failed!" << endl;

	PxShape* shape2 = plane2->createShape(PxPlaneGeometry(), *mMaterial);
	if (!shape)
		cerr << "create shape failed!" << endl;
	gScene->addActor(*plane2);
	
	//2)           Create cube	 
	PxReal         density = 1.0f;
	PxTransform    transform(PxVec3(0.0f, 10.0f, 0.0f), PxQuat::createIdentity());
	PxVec3         dimensions(BoxSize,BoxSize,BoxSize);
	PxBoxGeometry  geometry(dimensions);


	for (int i = 0; i < 2; i++)
	{
		transform.p = PxVec3(0.0f, 2.0f + 2 * i, -3.0f);
		PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);

		actor->setAngularDamping(0.75);
		actor->setMass(100.0);
		actor->setLinearVelocity(PxVec3(0, 0, 0));
		if (!actor)
			cerr << "create actor failed!" << endl;
		gScene->addActor(*actor);
		boxes.push_back(actor);
		//PxDistanceJoint* joint = PxDistanceJointCreate(*gPhysicsSDK, actor, PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), pole, PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
		//joint->setMaxDistance(2.0f);
		//joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
		//joint->setDamping(1.0f);
		//joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
	}
	
}

void Graphic::ShutdownPhysX() {

	if (gScene != NULL)
	{
		for (int i = 0; i < boxes.size(); i++)
			gScene->removeActor(*boxes[i]);

		gScene->release();

		for (int i = 0; i < boxes.size(); i++)
			boxes[i]->release();
	}
	if (gPhysicsSDK != NULL)
		gPhysicsSDK->release();

	//if (theConnection!=NULL)
	//	theConnection->release();
}

void Graphic::StepPhysX()
{
	gScene->simulate(myTimestep);
	while (!gScene->fetchResults())
	{

	}
}


D3DXMATRIX Graphic::PxtoXMMatrix(PxTransform input)
{
	PxMat33 quat = PxMat33(input.q);
	D3DXMATRIX start;
	start._11 = quat.column0[0];
	start._12 = quat.column0[1];
	start._13 = quat.column0[2];
	start._14 = 0;


	start._21 = quat.column1[0];
	start._22 = quat.column1[1];
	start._23 = quat.column1[2];
	start._24 = 0;

	start._31 = quat.column2[0];
	start._32 = quat.column2[1];
	start._33 = quat.column2[2];
	start._34 = 0;

	start._41 = input.p.x;
	start._42 = input.p.y;
	start._43 = input.p.z;
	start._44 = 1;

	return start;
}

void Graphic::DrawBox(PxShape* pShape, PxRigidActor* actor)
{
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	D3DXMATRIX mat = PxtoXMMatrix(pT);
	mWVP = mat* mView*mProj;
	mfxWVPVar_Box->SetMatrix((float*)&mWVP);

	D3D10_TECHNIQUE_DESC techDesc;
	mTech_Box->GetDesc(&techDesc);
	for (UINT p = 0; p < techDesc.Passes; ++p)
	{
		mTech_Box->GetPassByIndex(p)->Apply(0);

		mBox.draw();
	}
}

void Graphic::DrawSphere(PxShape* pShape, PxRigidActor* actor)
{
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	D3DXMATRIX mat = PxtoXMMatrix(pT);

	mWVP = mat* mView*mProj;
	mfxWVPVar_Sphere->SetMatrix((float*)&mWVP);

	D3D10_TECHNIQUE_DESC techDesc;
	mTech_Sphere->GetDesc(&techDesc);
	for (UINT p = 0; p < techDesc.Passes; ++p)
	{
		mTech_Sphere->GetPassByIndex(p)->Apply(0);

		sphere->DrawSubset(0);
	}
}

void Graphic::DrawShape(PxShape* shape, PxRigidActor* actor)
{
	PxGeometryType::Enum type = shape->getGeometryType();
	switch (type)
	{
	case PxGeometryType::eBOX:
		DrawBox(shape, actor);
		break;
	case PxGeometryType::eSPHERE:
		DrawSphere(shape, actor);
		break;
	}
}

void Graphic::DrawActor(PxRigidActor* actor)
{
	PxU32 nShapes = actor->getNbShapes();
	PxShape** shapes = new PxShape*[nShapes];

	actor->getShapes(shapes, nShapes);
	while (nShapes--)
	{
		DrawShape(shapes[nShapes], actor);
	}
	delete[] shapes;
}

void Graphic::RenderActors(bool proxy)
{
	for (int i = 0; i < boxes.size(); i++)
		DrawActor(boxes[i]);

	if (proxy)
	for (int i = 0; i < proxyParticle.size(); i++)
		DrawActor(proxyParticle[i]);
}

PxPhysics* Graphic::getPhysicsSDK()
{
	return gPhysicsSDK;
}

PxScene* Graphic::getScene()
{
	return gScene;
}

void Graphic::SetProxyActor(vector<PxRigidActor*> proxyParticleActor)
{
	proxyParticle = proxyParticleActor;
}

void Graphic::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
{
	//cout << nbPairs << " Contact pair(s) detected\n";

	const PxU32 buff = 64; //buffer size
	PxContactPairPoint contacts[buff];

	//loop through all contact pairs of PhysX simulation
	for (PxU32 i = 0; i < nbPairs; i++)
	{
		//extract contant info from current contact-pair 
		const PxContactPair& curContactPair = pairs[i];
		PxU32 nbContacts = curContactPair.extractContacts(contacts, buff);
		if (curContactPair.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
		{
			if (pairs->shapes[0]->getGeometryType() == PxGeometryType::eBOX && pairs->shapes[1]->getGeometryType() == PxGeometryType::eSPHERE)
			{
				PxRigidActor* box = pairHeader.actors[0];
				PxRigidActor* sphere = pairHeader.actors[1];
				if (std::find(boxesJoint.begin(), boxesJoint.end(), box) == boxesJoint.end())
				{
					boxesJoint.push_back(box);
					particleJoint.push_back(sphere);
				}
			}
			else if (pairs->shapes[1]->getGeometryType() == PxGeometryType::eBOX && pairs->shapes[0]->getGeometryType() == PxGeometryType::eSPHERE)
			{
				PxRigidActor* box = pairHeader.actors[1];
				PxRigidActor* sphere = pairHeader.actors[0];
				if (std::find(boxesJoint.begin(), boxesJoint.end(), box) == boxesJoint.end())
				{
					boxesJoint.push_back(box);
					particleJoint.push_back(sphere);
				}
			}
		}

		//for (PxU32 j = 0; j < nbContacts; j++)
		//{
		//	//print all positions of contact.   
		//	PxVec3 point = contacts[j].position;
		//	
		//	//cout << "Contact point (" << point.x << " " << point.y << " " << point.x << ")\n";
		//}
	}
}