# WsRobotClient
This is the communications control client for the robot. It is a variant of the wsClient.

## Create a new client
First, make sure you already have the server running. Now create a new wsClientClient:

```

sClient = DosClient::WsClient::startClient<DosClient::WsRobotClient>(mServerAddress, mServerPort);

```

where **mServerAddress** is the addres of your server for the robot and **mServerPort** is the port of the server.

## Power On/Off

To power On/Off the robot:

```

sClient->powerRobot(flag); // true or false

```

## Basic Commands

You can issue basic commands to the robot ( stand, sit etc. ) via:

```

sClient->commandRobot(cmd);

```

The **cmd** is defined as a **RobotCommandPacket::ROBOT_CMD** enum:

```
enum class ROBOT_CMD {
    STAND,
    SIT,
    SELF_RIGHT,
    SELF_LEFT
};

```

## Moving the Robot

There are 2 ways to move the Robot, via a basic **moveVelRobot** method or the preferred **freeMoveVelRobot** which gives you more flexibility.

For **moveVelRobot**:

```

sClient->moveVelRobot(cmd, magnitude);

```

The **cmd** is defined as a **MoveVelCommandPacket::VEL_CMD** enum:

```
enum class VEL_CMD
{
    MOVE_FORWARD,
    MOVE_BACKWARD,
    STRAFE_LEFT,
    STRAFE_RIGHT,
    TURN_LEFT,
    TURN_RIGHT
};
```

Of course if you are actually piloting the robot with something like joystick, you should use **freeMoveVelRobot** instead:

```

sClient->freeMoveVelRobot(xMag, yMag, turnMag, maxMagnitude);

```

Let's look at a more concrete example from our actual controller code. Here we want to move the robot via our joystick. Code is running in Win32:

```
Magnum::Vector3 bvlosPanel::getNormalizedJoystickPos(unsigned int jStick, int minVal, int maxVal, float minCutoffToZero) const
{
#if defined(_WIN32)
	JOYINFOEX  joyinfoex;
	joyinfoex.dwSize = sizeof(joyinfoex);
	joyinfoex.dwFlags = JOY_RETURNALL;
	MMRESULT bDev1Attached = false;
	bDev1Attached = joyGetPosEx(jStick, &joyinfoex);
	if ((bDev1Attached == JOYERR_PARMS) || (bDev1Attached == JOYERR_NOCANDO) || (bDev1Attached == JOYERR_UNPLUGGED))
	{
		return { 0.0f, 0.0f, 0.0f };
	}

	float midVal = float(minVal + maxVal) * 0.5f;
	float rangeVal = float(maxVal - minVal) * 0.5f;
	float normX = (float(joyinfoex.dwXpos) - midVal) / rangeVal;
	float normY = (float(joyinfoex.dwYpos) - midVal) / rangeVal;
	float normZ = 1.0f - (float(joyinfoex.dwZpos) / 65535.0f);

	if (std::abs(normX) <= minCutoffToZero)
	{
		normX = 0;
	}

	if (std::abs(normY) <= minCutoffToZero)
	{
		normY = 0;
	}

	//_printToDebug("Joystick [" + std::to_string(normX) + ", " + std::to_string(normY) + "]");
	return { normX, normY, normZ };
#else
	return { 0, 0 };
#endif
}
```

After that in our event tick ( part of the game engine event loop), we actually move the robot:
```
void bvlosPanel::updateJoystick()
{
	// Rate limit so we do not throttle the connection
	auto cTime = CanvasPanelManager::getTimeMS();
	auto diffTime = cTime.count() - mJoystickMoveTime.count();
	auto constexpr cLimitVal = 80;
	if (diffTime > cLimitVal) {
		mJoystickMoveTime = cTime;
	}
	else {
		return;
	}

	// Process Joystick
	if (mMovementMode == MOVEMENT_MODE::MOVE) {
		mJoystickPos = getNormalizedJoystickPos(JOYSTICKID1);

		if (std::abs(mJoystickPos.z() - mJoystickPrevZ) > 0.01f)
		{
			mConstantVelMag = mJoystickPos.z();
			mJoystickPrevZ = mJoystickPos.z();
		}

		if (mConstantVelMag > 0.0f)
		{
			if (mJoystickPos.y() > 0.0f)
			{
				mConstantVelMag = 0.0f;
			}
			sClient->freeMoveVelRobot(mConstantVelMag, 0.0f, -mJoystickPos.x(), mMoveSpeed);
		}
		else {
			if ((std::abs(mJoystickPos.x()) > 0) || (std::abs(mJoystickPos.y()) > 0))
			{
				sClient->freeMoveVelRobot(-mJoystickPos.y(), 0.0f, -mJoystickPos.x(), mMoveSpeed);
			}
		}
	}
```

***Remember not to continuously send the freeMoveVelRobot command with too short a latency or you will throttle the network connection and cause the packets sent over to be delayed.***

## Receiving Robot Odometery ( Robot Transform )

You can choose to register a callback to receive the current world transform of the robot:

```
void bvlosPanel::pktXformFn(DosClient::XformCommandPacket& msgIn)
{
	if (msgIn.subType() == 0)
	{
		mRobotFrameSet = true;
		// Root world frame with position and quaternion
		mRobotPos = Magnum::Vector3(msgIn.pX(), msgIn.pY(), msgIn.pZ());
		mRobotQuat = Magnum::Quaternion({ msgIn.qX(), msgIn.qY(), msgIn.qZ() }, msgIn.qW());
		mRobotLVel = Magnum::Vector3(msgIn.lVelX(), msgIn.lVelY(), msgIn.lVelZ());
		mRobotAVel = Magnum::Vector3(msgIn.aVelX(), msgIn.aVelY(), msgIn.aVelZ());
	}
}

sClient->registerPacketRecvFn(
    DosClient::COMMAND_XFORM_TYPE,
    [this](DosClient::BaseCommandPacket* pktIn) -> void
    {
        if (auto cPkt = dynamic_cast<DosClient::XformCommandPacket*>(pktIn))
        {
            pktXformFn(*cPkt);
        }
    }
);
```

The transform is packed as your standard Vector3 and Quaternion ( Position + Orientation ). One thing to remember is on your server end, make sure the **spotServerConfig.json** has the following set:

```
    "alwaysPublishOdom" : true,
```

This means the robot will always broadcast its position + orientation as it moves to your wsRobotClient.