//-- Physx Library

#ifndef PHYSXHELPER
#define PHYSXHELPER
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
using namespace physx;

struct FilterGroup
{
	enum Enum
	{
		ePARTICLE = (1 << 0),
		eBox = (1 << 1),
		eInteract = (1 << 2),
	};
};



void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask);
#endif
