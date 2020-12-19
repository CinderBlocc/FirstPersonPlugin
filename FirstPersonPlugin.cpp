#include "FirstPersonPlugin.h"
#include "bakkesmod\wrappers\includes.h"
#include "HelperFunctions.h"
#include <sstream>
#include <filesystem>
#define _USE_MATH_DEFINES
#include <math.h>


BAKKESMOD_PLUGIN(FirstPersonPlugin, "First Person Plugin", "1.0", PLUGINTYPE_FREEPLAY)


void FirstPersonPlugin::onLoad()
{
	Initialize();
	cvarManager->registerNotifier("FirstPersonEnable", [this](std::vector<std::string> params){Enable();}, "Enables first person plugin", PERMISSION_ALL);
	cvarManager->registerNotifier("FirstPersonDisable", [this](std::vector<std::string> params){Disable();}, "Disables first person plugin", PERMISSION_ALL);

	cvarManager->registerCvar("FirstPerson_UseCustomBallCam", "1", "Use true first person ball cam");
}


void FirstPersonPlugin::CreateValues()
{
	ServerWrapper server = GetCurrentGameState();
	CameraWrapper camera = gameWrapper->GetCamera();
	if(server.IsNull() || camera.IsNull())
		return;

	CarWrapper car = gameWrapper->GetLocalCar();			
	if (!car.IsNull())
	{
		int rearCamMult = 1;
		if(isInRearCam)
			rearCamMult = -1;

		Vector carLocation = car.GetLocation();
		Quat carQuat = rotToQuat(car.GetRotation());
		Quat carQuat2 = rotToQuat(Rotator{rearCamMult * car.GetRotation().Pitch, car.GetRotation().Yaw, rearCamMult * car.GetRotation().Roll});
			
		//FOCUS
		Vector baseCameraPosition = {42, 0, 35};
		Vector alignedCameraPosition = sp::rotateVectorWithQuat(baseCameraPosition, carQuat);
		FOCUS = car.GetLocation() + alignedCameraPosition;
		
		//ROTATION
		Rotator baseCameraRotation = {-2500, 0, 0};
		Quat baseCameraQuat = rotToQuat(Rotator{SWIVEL.Pitch * 2, SWIVEL.Yaw, SWIVEL.Roll} + baseCameraRotation);
		Rotator alignedCameraQuat = sp::quatToRot(carQuat2 * baseCameraQuat);

		BallWrapper ball = server.GetBall();
		if(!ball.IsNull() && isInBallCam && !isInRearCam)
		{
			Vector newQuatFwd = sp::quatToFwd(carQuat);
			Vector newQuatRight = sp::quatToRight(carQuat);
			Vector newQuatUp = sp::quatToUp(carQuat);

			Vector ballLocation = ball.GetLocation();
			Vector cameraLocation = camera.GetLocation();

			Vector targetDirection = {ballLocation.X - cameraLocation.X, ballLocation.Y - cameraLocation.Y, ballLocation.Z - cameraLocation.Z};
			targetDirection.normalize();
			Vector targetDirectionUpComponent = project_v1_on_v2(targetDirection, newQuatUp);
			Vector rejectedTargetDirection = targetDirection - targetDirectionUpComponent;
			rejectedTargetDirection.normalize();

			//rotate newQuatFwd and newQuatRight around newQuatUp so that newQuatFwd is aligned to rejectedTargetDirection
			float a = (rejectedTargetDirection - newQuatFwd).magnitude();
			float b = rejectedTargetDirection.magnitude();
			float c = newQuatFwd.magnitude();
			float upRot = acos((b*b + c*c - a*a)/2*b*c);
			if(Vector::dot(rejectedTargetDirection, newQuatRight) <= 0)
				upRot *= -1;
			Quat upRotQuat = AngleAxisRotation(upRot, newQuatUp);
			newQuatFwd = sp::rotateVectorWithQuat(newQuatFwd, upRotQuat);
			newQuatRight = sp::rotateVectorWithQuat(newQuatRight, upRotQuat);

			//rotate newQuatFwd and newQuatUp around newQuatRight so that newQuatFwd is aligned to targetDirection
			a = (targetDirection - newQuatFwd).magnitude();
			b = targetDirection.magnitude();
			c = newQuatFwd.magnitude();
			float rightRot = acos((b*b + c*c - a*a)/2*b*c);
			if(Vector::dot(targetDirection, newQuatUp) >= 0)
				rightRot *= -1;
			Quat rightRotQuat = AngleAxisRotation(rightRot, newQuatRight);
			newQuatFwd = sp::rotateVectorWithQuat(newQuatFwd, rightRotQuat);
			newQuatUp = sp::rotateVectorWithQuat(newQuatUp, rightRotQuat);

			Quat newQuat = GetQuatFromMatrix(newQuatFwd, newQuatRight, newQuatUp);
			alignedCameraQuat = sp::quatToRot(newQuat * baseCameraQuat);
		}
		ROTATION = alignedCameraQuat - SWIVEL;

		if(isInBallCam && !cvarManager->getCvar("FirstPerson_UseCustomBallCam").getBoolValue())
		{
			overrideValue[3] = false;//Rotation Pitch
			overrideValue[4] = false;//Rotation Yaw
			overrideValue[5] = false;//Rotation Roll
		}

		//DISTANCE
		DISTANCE = 0;

		//FOV
		CameraWrapper camera = gameWrapper->GetCamera();
		ProfileCameraSettings playerCamSettings = camera.GetCameraSettings();
		float baseFOV = playerCamSettings.FOV;
		Vector carSpeedVec = car.GetVelocity();
		float carSpeed = sqrt(carSpeedVec.X * carSpeedVec.X + carSpeedVec.Y * carSpeedVec.Y + carSpeedVec.Z * carSpeedVec.Z);
		float supersonic = 2200;
		float speedPerc = carSpeed / supersonic;
		float newFOV = 0;
		
		bool isSupersonic = false;
		if(car.GetbSuperSonic() == 1)
			isSupersonic = true;

		float interpsteps = 25;
		if(isSupersonic)
		{
			if(interpStepsIntoSupersonic < interpsteps)
				interpStepsIntoSupersonic++;
			newFOV = baseFOV + 5 + (5 * (interpStepsIntoSupersonic / interpsteps));
			wasSupersonic = true;
			interpStepsOutOfSupersonic = 0;
		}
		else
		{
			interpStepsIntoSupersonic = 0;
			if(wasSupersonic)
			{
				if(interpStepsOutOfSupersonic < interpsteps)
					interpStepsOutOfSupersonic++;
				newFOV = baseFOV + 10 - (5 * (interpStepsOutOfSupersonic / interpsteps));
				if(interpStepsOutOfSupersonic >= interpsteps)
					wasSupersonic = false;
			}
			else
				newFOV = baseFOV + 5 * speedPerc;
		}
		FOV = newFOV + 15;
	}
	else
	{
		overrideValue[0] = false;//Focus X
		overrideValue[1] = false;//Focus Y
		overrideValue[2] = false;//Focus Z
		overrideValue[3] = false;//Rotation Pitch
		overrideValue[4] = false;//Rotation Yaw
		overrideValue[5] = false;//Rotation Roll
		overrideValue[6] = false;//Distance
		overrideValue[7] = false;//FOV
	}
}



//MATH
Vector FirstPersonPlugin::project_v1_on_v2(Vector vec1, Vector vec2)
{
	float dot = Vector::dot(vec1, vec2);
	float vec2magnitude = vec2.magnitude();
	return (vec2 * dot/(vec2magnitude * vec2magnitude));
}
Quat FirstPersonPlugin::AngleAxisRotation(float angle, Vector axis)
{
	//DEGREES
	Quat result;
	float angDiv2 = angle * 0.5f;
	result.W = cos(angDiv2);
	result.X = axis.X * sin(angDiv2);
	result.Y = axis.Y * sin(angDiv2);
	result.Z = axis.Z * sin(angDiv2);

	return result;
}
Quat FirstPersonPlugin::GetQuatFromMatrix(Vector fwd, Vector right, Vector up)
{
	//https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	//W, X, Y, Z
	Quat q(1,0,0,0);

	//{ fwd.X, right.X, up.X }
	//{ fwd.Y, right.Y, up.Y }  ------->  newQuat
	//{ fwd.Z, right.Z, up.Z }
	float a[3][3];

	a[0][0] = fwd.X, a[0][1] = right.X, a[0][2] = up.X;
	a[1][0] = fwd.Y, a[1][1] = right.Y, a[1][2] = up.Y;
	a[2][0] = fwd.Z, a[2][1] = right.Z, a[2][2] = up.Z;


	float trace = a[0][0] + a[1][1] + a[2][2];
	if( trace > 0 )
	{
		float s = 0.5f / sqrtf(trace+ 1.0f);
		q.W = 0.25f / s;
		q.X = ( a[2][1] - a[1][2] ) * s;
		q.Y = ( a[0][2] - a[2][0] ) * s;
		q.Z = ( a[1][0] - a[0][1] ) * s;
	}
	else
	{
		if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] )
		{
			float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
			q.W = (a[2][1] - a[1][2] ) / s;
			q.X = 0.25f * s;
			q.Y = (a[0][1] + a[1][0] ) / s;
			q.Z = (a[0][2] + a[2][0] ) / s;
		}
		else if (a[1][1] > a[2][2])
		{
			float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
			q.W = (a[0][2] - a[2][0] ) / s;
			q.X = (a[0][1] + a[1][0] ) / s;
			q.Y = 0.25f * s;
			q.Z = (a[1][2] + a[2][1] ) / s;
		}
		else
		{
			float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
			q.W = (a[1][0] - a[0][1] ) / s;
			q.X = (a[0][2] + a[2][0] ) / s;
			q.Y = (a[1][2] + a[2][1] ) / s;
			q.Z = 0.25f * s;
		}
	}

	return q;
}
Quat FirstPersonPlugin::rotToQuat(Rotator rot)
{
	float uRotToDeg = 182.044449;
	float DegToRadDiv2 = (M_PI/180)/2;
	float sinPitch = sin((rot.Pitch/uRotToDeg)*DegToRadDiv2);
	float cosPitch = cos((rot.Pitch/uRotToDeg)*DegToRadDiv2);
	float sinYaw = sin((rot.Yaw/uRotToDeg)*DegToRadDiv2);
	float cosYaw = cos((rot.Yaw/uRotToDeg)*DegToRadDiv2);
	float sinRoll = sin((rot.Roll/uRotToDeg)*DegToRadDiv2);
	float cosRoll = cos((rot.Roll/uRotToDeg)*DegToRadDiv2);
	
	Quat convertedQuat;
	convertedQuat.X = (cosRoll*sinPitch*sinYaw) - (sinRoll*cosPitch*cosYaw);
	convertedQuat.Y = ((-cosRoll)*sinPitch*cosYaw) - (sinRoll*cosPitch*sinYaw);
	convertedQuat.Z = (cosRoll*cosPitch*sinYaw) - (sinRoll*sinPitch*cosYaw);
	convertedQuat.W = (cosRoll*cosPitch*cosYaw) + (sinRoll*sinPitch*sinYaw);

	return convertedQuat;
}







//LEAVE THESE UNCHANGED

ServerWrapper FirstPersonPlugin::GetCurrentGameState()
{
	if(gameWrapper->IsInReplay())
		return gameWrapper->GetGameEventAsReplay().memory_address;
	else if(gameWrapper->IsInOnlineGame())
		return gameWrapper->GetOnlineGame();
	else
		return gameWrapper->GetGameEventAsServer();
}
void FirstPersonPlugin::onUnload(){}
void FirstPersonPlugin::Initialize()
{
	//Install parent plugin if it isn't already installed. Ensure parent plugin is loaded.
	if(!std::filesystem::exists(gameWrapper->GetBakkesModPath() / "plugins" / "CameraControl.dll"))
		cvarManager->executeCommand("bpm_install 71");
	cvarManager->executeCommand("plugin load CameraControl", false);

	//Hook events
	gameWrapper->HookEvent("Function ProjectX.Camera_X.ClampPOV", std::bind(&FirstPersonPlugin::HandleValues, this));
	gameWrapper->HookEvent("Function TAGame.PlayerController_TA.PressRearCamera", [&](std::string eventName){isInRearCam = true;});
	gameWrapper->HookEvent("Function TAGame.PlayerController_TA.ReleaseRearCamera", [&](std::string eventName){isInRearCam = false;});
	gameWrapper->HookEvent("Function TAGame.CameraState_BallCam_TA.BeginCameraState", [&](std::string eventName){isInBallCam = true;});
	gameWrapper->HookEvent("Function TAGame.CameraState_BallCam_TA.EndCameraState", [&](std::string eventName){isInBallCam = false;});
}
bool FirstPersonPlugin::CanCreateValues()
{
	if(!enabled || IsCVarNull("CamControl_Swivel_READONLY") || IsCVarNull("CamControl_Focus") || IsCVarNull("CamControl_Rotation") || IsCVarNull("CamControl_Distance") || IsCVarNull("CamControl_FOV"))
		return false;
	else
		return true;
}
bool FirstPersonPlugin::IsCVarNull(std::string cvarName)
{
    struct CastStructOne
    {
        struct CastStructTwo{void* address;};
        CastStructTwo* casttwo;
    };

	CVarWrapper cvar = cvarManager->getCvar(cvarName);
    CastStructOne* castone = (CastStructOne*)&cvar;
    return castone->casttwo->address == nullptr;
}
void FirstPersonPlugin::Enable()
{
	cvarManager->executeCommand("CamControl_Enable 1", false);
	enabled = true;
}
void FirstPersonPlugin::Disable()
{
	enabled = false;
	cvarManager->executeCommand("CamControl_Enable 0", false);
}
void FirstPersonPlugin::HandleValues()
{
	if(!CanCreateValues())
		return;
	
	//Reset values so that the game won't crash if the developer doesn't assign values to variables
	overrideValue[0] = true;//Focus X
	overrideValue[1] = true;//Focus Y
	overrideValue[2] = true;//Focus Z
	overrideValue[3] = true;//Rotation Pitch
	overrideValue[4] = true;//Rotation Yaw
	overrideValue[5] = true;//Rotation Roll
	overrideValue[6] = true;//Distance
	overrideValue[7] = true;//FOV

	SWIVEL = GetSwivel();
	FOCUS = Vector{0,0,0};
	ROTATION = Rotator{0,0,0};
	DISTANCE = 100;
	FOV = 90;

	//Get values from the developer
	CreateValues();

	//Send value requests to the parent mod
	std::string values[8];
	values[0] = std::to_string(FOCUS.X);
	values[1] = std::to_string(FOCUS.Y);
	values[2] = std::to_string(FOCUS.Z);
	values[3] = std::to_string(ROTATION.Pitch);
	values[4] = std::to_string(ROTATION.Yaw);
	values[5] = std::to_string(ROTATION.Roll);
	values[6] = std::to_string(DISTANCE);
	values[7] = std::to_string(FOV);
	
	for(int i=0; i<8; i++)
	{
		if(!overrideValue[i])
			values[i] = "NULL";
	}

	cvarManager->getCvar("CamControl_Focus").setValue(values[0] + "," + values[1] + "," + values[2]);
	cvarManager->getCvar("CamControl_Rotation").setValue(values[3] + "," + values[4] + "," + values[5]);
	cvarManager->getCvar("CamControl_Distance").setValue(values[6]);
	cvarManager->getCvar("CamControl_FOV").setValue(values[7]);
}
Rotator FirstPersonPlugin::GetSwivel()
{
	if(IsCVarNull("CamControl_Swivel_READONLY"))
		return Rotator{0,0,0};

	std::string readSwivel = cvarManager->getCvar("CamControl_Swivel_READONLY").getStringValue();
	std::string swivelInputString;
	std::stringstream ssSwivel(readSwivel);

	Rotator SWIVEL = {0,0,0};

	getline(ssSwivel, swivelInputString, ',');
	SWIVEL.Pitch = stof(swivelInputString);
	getline(ssSwivel, swivelInputString, ',');
	SWIVEL.Yaw = stof(swivelInputString);
	getline(ssSwivel, swivelInputString, ',');
	SWIVEL.Roll = stof(swivelInputString);

	return SWIVEL;
}