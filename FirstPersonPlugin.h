#pragma once
#pragma comment(lib, "PluginSDK.lib")
#include "bakkesmod/plugin/bakkesmodplugin.h"

class FirstPersonPlugin : public BakkesMod::Plugin::BakkesModPlugin
{
private:
	bool enabled = false;
	Vector FOCUS;
	Rotator ROTATION, SWIVEL;
	float DISTANCE, FOV;
	bool overrideValue[8];
	bool isInBallCam = false;
	bool isInRearCam = false;
	bool wasSupersonic = false;
	float interpStepsIntoSupersonic = 0;
	float interpStepsOutOfSupersonic = 0;

public:
	void onLoad() override;
	void onUnload() override;

	void CreateValues();
	Vector project_v1_on_v2(Vector vec1, Vector vec2);
	Quat AngleAxisRotation(float angle, Vector axis);
	Quat GetQuatFromMatrix(Vector fwd, Vector right, Vector up);
	Quat rotToQuat(Rotator rot);

	ServerWrapper GetCurrentGameState();
	void Initialize();
	bool CanCreateValues();
	bool IsCVarNull(std::string cvarName);
	void Enable();
	void Disable();
	void HandleValues();
	Rotator GetSwivel();
};
