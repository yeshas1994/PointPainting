#include <dosCore/commandPacket.h>
#include <iostream>
#include <dosCore/commandPacketParse.h>

namespace DosClient
{
	// BaseCommandPacket
	std::string BaseCommandPacket::encode()
	{
		msgpack11::MsgPack::array newArray;
		newArray.push_back(getPacketTypeID());
		encodeToArray(newArray);
		msgpack11::MsgPack result = newArray;
		return result.dump();
	}

	// MsgStrCommandPacket
	void MsgStrCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_str = dataArrayIn[1].string_value();
	}

	void MsgStrCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_str);
	}

	REGISTER_PACKET_FACTORY(MsgStrCommandPacket)
	
	// StatusCommandPacket
	nlohmann::json StatusCommandPacket::getStatusJSON() const
	{
		return nlohmann::json::parse(m_str); // Clearly parse() exists, VS2019 intellisense bug
	}

	REGISTER_PACKET_FACTORY(StatusCommandPacket)

	// SettingsCommandPacket
	nlohmann::json SettingsCommandPacket::getStatusJSON() const
	{
		return nlohmann::json::parse(m_str); // Clearly parse() exists, VS2019 intellisense bug
	}

	REGISTER_PACKET_FACTORY(SettingsCommandPacket)

	// SpotGraphNavCommandPacket
	nlohmann::json SpotGraphNavCommandPacket::getStatusJSON() const
	{
		return nlohmann::json::parse(m_str); // Clearly parse() exists, VS2019 intellisense bug
	}

	REGISTER_PACKET_FACTORY(SpotGraphNavCommandPacket)

	// ControlPermissionsPacket
	nlohmann::json ControlPermissionsPacket::getStatusJSON() const
	{
		return nlohmann::json::parse(m_str); // Clearly parse() exists, VS2019 intellisense bug
	}

	REGISTER_PACKET_FACTORY(ControlPermissionsPacket)

	// MotionParamsCommandPacket
	nlohmann::json MotionParamsCommandPacket::getStatusJSON() const
	{
		return nlohmann::json::parse(m_str); // Clearly parse() exists, VS2019 intellisense bug
	}

	REGISTER_PACKET_FACTORY(MotionParamsCommandPacket)

	// BytesCommandPacket
	void BytesCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_bytes = dataArrayIn[1].binary_items();
	}

	void BytesCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_bytes);
	}

	REGISTER_PACKET_FACTORY(BytesCommandPacket)

	// MoveVelCommandPacket
	void MoveVelCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_descrip = dataArrayIn[1].string_value();
		m_x = dataArrayIn[2].float32_value();
		m_y = dataArrayIn[3].float32_value();
		m_rot = dataArrayIn[4].float32_value();
	}

	void MoveVelCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_descrip);
		dataArrayIn.push_back(m_x);
		dataArrayIn.push_back(m_y);
		dataArrayIn.push_back(m_rot);
	}

	void MoveVelCommandPacket::setVelCmd(VEL_CMD velCmdIn, float magnitude)
	{
		m_x = 0;
		m_y = 0;
		m_rot = 0;

		if (velCmdIn == VEL_CMD::MOVE_BACKWARD)
		{
			m_descrip = "move_backward";
			m_x = -magnitude;
		}
		else if (velCmdIn == VEL_CMD::MOVE_FORWARD)
		{
			m_descrip = "move_forward";
			m_x = magnitude;
		}
		else if (velCmdIn == VEL_CMD::STRAFE_LEFT)
		{
			m_descrip = "strafe_left";
			m_y = magnitude;
		}
		else if (velCmdIn == VEL_CMD::STRAFE_RIGHT)
		{
			m_descrip = "strafe_right";
			m_y = -magnitude;
		}
		else if (velCmdIn == VEL_CMD::TURN_LEFT)
		{
			m_descrip = "turn_left";
			m_rot = magnitude;
		}
		else if (velCmdIn == VEL_CMD::TURN_RIGHT)
		{
			m_descrip = "turn_right";
			m_rot = -magnitude;
		}
	}

	void MoveVelCommandPacket::setVelFreeMove(float xMag, float yMag, float turnMag, float magRange)
	{
		auto clampVal = [this](float valIn) -> float
		{
			return std::min(std::max(valIn, -1.0f), 1.0f);
		};

		auto scaleRangeVal = [this, magRange](float valIn) -> float
		{
			float aVal = 1.0f;
			return (valIn - 0.0f) / aVal * magRange;
		};

		xMag = clampVal(xMag);
		yMag = clampVal(yMag);
		turnMag = clampVal(turnMag);

		m_descrip = "free_move";
		m_x = scaleRangeVal(xMag);
		m_y = scaleRangeVal(yMag);
		m_rot = scaleRangeVal(turnMag);
	}

	REGISTER_PACKET_FACTORY(MoveVelCommandPacket)

	// PowerRobotCommandPacket
	void PowerRobotCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_state = dataArrayIn[1].int32_value();
	}

	void PowerRobotCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_state);
	}

	REGISTER_PACKET_FACTORY(PowerRobotCommandPacket)

	// RobotCommandPacket
	void RobotCommandPacket::setRobotCommand(ROBOT_CMD cmdIn)
	{
		if (cmdIn == ROBOT_CMD::SELF_LEFT)
		{
			m_str = "self_left";
		}
		else if (cmdIn == ROBOT_CMD::SELF_RIGHT)
		{
			m_str = "self_right";
		}
		else if (cmdIn == ROBOT_CMD::SIT)
		{
			m_str = "sit";
		}
		else if (cmdIn == ROBOT_CMD::STAND)
		{
			m_str = "stand";
		}
	}

	REGISTER_PACKET_FACTORY(RobotCommandPacket)

	// ImageCommandPacket
	void ImageCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		BytesCommandPacket::parseDataArray(dataArrayIn);
		m_compressionType = dataArrayIn[2].int32_value();
	}

	void ImageCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		BytesCommandPacket::encodeToArray(dataArrayIn);
		dataArrayIn.push_back(m_compressionType);
	}

	REGISTER_PACKET_FACTORY(ImageCommandPacket)

	// ImageStreamTogglePacket
	void ImageStreamTogglePacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_toggle = dataArrayIn[1].int32_value();
		
		int numNames = dataArrayIn[2].int32_value();
		m_imageNames.resize(numNames);
		for (int i = 0; i < numNames; i++)
		{
			m_imageNames[i] = dataArrayIn[3 + i].string_value();
		}
	}

	void ImageStreamTogglePacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_toggle);
		dataArrayIn.push_back(static_cast<int>(m_imageNames.size()));
		for (auto& cName : m_imageNames)
		{
			dataArrayIn.push_back(cName);
		}
	}

	REGISTER_PACKET_FACTORY(ImageStreamTogglePacket)

	// ImageListCommandPacket
	void ImageListCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_compressionType = dataArrayIn[1].int32_value();
		int numImages = dataArrayIn[2].int32_value();
		m_images.clear();
		for (int i = 0; i < numImages; i++)
		{
			int baseIdx = 3 + (i * 2);
			SubImage newImage;
			newImage.bytes = dataArrayIn[baseIdx].binary_items();
			newImage.label = dataArrayIn[baseIdx + 1].string_value();
			m_images.push_back(std::move(newImage));
		}
	}

	void ImageListCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_compressionType);
		dataArrayIn.push_back(static_cast<int32_t>(m_images.size()));
		for (auto& sImage : m_images)
		{
			dataArrayIn.push_back(sImage.bytes);
			dataArrayIn.push_back(sImage.label);
		}
	}

	REGISTER_PACKET_FACTORY(ImageListCommandPacket)

		// ImageListCommandPacket
	void ImageListWithMetaCommandPacket::parseDataArray( const msgpack11::MsgPack::array& dataArrayIn )
	{
		auto offset = 0;
		//ImageListCommandPacket::parseDataArray( dataArrayIn );
		m_compressionType = dataArrayIn[ 1 ].int32_value();
		int numImages = dataArrayIn[ 2 ].int32_value();
		m_images.clear();
		for ( int i = 0; i < numImages; i++ )
		{
			int baseIdx = 3 + ( i * 2 );
			SubImage newImage;
			newImage.bytes = dataArrayIn[ baseIdx ].binary_items();
			newImage.label = dataArrayIn[ baseIdx + 1 ].string_value();
			m_images.push_back( std::move( newImage ) );
		}
		offset = 3 + numImages * 2;
		auto numInferred = dataArrayIn[ offset ].int32_value();
		offset++;
		constexpr auto stride = 5;
		m_meta.Inferred.clear();
		for ( auto i = 0; i < numInferred; ++i )
		{
			auto idx = offset + i * stride;
			m_meta.Inferred.push_back( InferResult{
				dataArrayIn[ idx + 1 ].float32_value(),
				dataArrayIn[ idx + 2 ].float32_value(),
				dataArrayIn[ idx + 3 ].float32_value(),
				dataArrayIn[ idx + 4 ].float32_value(),
				dataArrayIn[ idx + 0 ].int32_value()
				} );
		}
	}

	void ImageListWithMetaCommandPacket::encodeToArray( msgpack11::MsgPack::array& dataArrayIn )
	{
		ImageListCommandPacket::encodeToArray( dataArrayIn );
		dataArrayIn.push_back( m_meta.Inferred.size() );
		for ( auto const r : m_meta.Inferred )
		{
			dataArrayIn.push_back( r.classIndex );
			dataArrayIn.push_back( r.x1 );
			dataArrayIn.push_back( r.y1 );
			dataArrayIn.push_back( r.x2 );
			dataArrayIn.push_back( r.y2 );
		}
	}
	REGISTER_PACKET_FACTORY( ImageListWithMetaCommandPacket )
	// BytesListCommandPacket
	void BytesListCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		int numBlobs = dataArrayIn[1].int32_value();
		m_blobs.clear();
		for (int i = 0; i < numBlobs; i++)
		{
			int baseIdx = 2 + (i * 3);
			SubBlob newBlob;
			newBlob.bytes = dataArrayIn[baseIdx].binary_items();
			newBlob.subType = dataArrayIn[baseIdx + 1].int32_value();
			newBlob.label = dataArrayIn[baseIdx + 2].string_value();
			m_blobs.push_back(std::move(newBlob));
		}
	}

	void BytesListCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)	
	{
		dataArrayIn.push_back(static_cast<int32_t>(m_blobs.size()));
		for (auto& sBlob : m_blobs)
		{
			dataArrayIn.push_back(sBlob.bytes);
			dataArrayIn.push_back(sBlob.subType);
			dataArrayIn.push_back(sBlob.label);
		}
	}

	REGISTER_PACKET_FACTORY(BytesListCommandPacket)

	// XformCommandPacket
	void XformCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_subType = dataArrayIn[1].int32_value();

		m_posX = dataArrayIn[2].float32_value();
		m_posY = dataArrayIn[3].float32_value();
		m_posZ = dataArrayIn[4].float32_value();

		m_quatX = dataArrayIn[5].float32_value();
		m_quatY = dataArrayIn[6].float32_value();
		m_quatZ = dataArrayIn[7].float32_value();
		m_quatW = dataArrayIn[8].float32_value();

		m_lVelX = dataArrayIn[9].float32_value();
		m_lVelY = dataArrayIn[10].float32_value();
		m_lVelZ = dataArrayIn[11].float32_value();

		m_aVelX = dataArrayIn[12].float32_value();
		m_aVelY = dataArrayIn[13].float32_value();
		m_aVelZ = dataArrayIn[14].float32_value();

		m_lAccX = dataArrayIn[15].float32_value();
		m_lAccY = dataArrayIn[16].float32_value();
		m_lAccZ = dataArrayIn[17].float32_value();

		m_aAccX = dataArrayIn[18].float32_value();
		m_aAccY = dataArrayIn[19].float32_value();
		m_aAccZ = dataArrayIn[20].float32_value();
	}

	void XformCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_subType);
		
		dataArrayIn.push_back(m_posX);
		dataArrayIn.push_back(m_posY);
		dataArrayIn.push_back(m_posZ);

		dataArrayIn.push_back(m_quatX);
		dataArrayIn.push_back(m_quatY);
		dataArrayIn.push_back(m_quatZ);
		dataArrayIn.push_back(m_quatW);

		dataArrayIn.push_back(m_lVelX);
		dataArrayIn.push_back(m_lVelY);
		dataArrayIn.push_back(m_lVelZ);

		dataArrayIn.push_back(m_aVelX);
		dataArrayIn.push_back(m_aVelY);
		dataArrayIn.push_back(m_aVelZ);

		dataArrayIn.push_back(m_lAccX);
		dataArrayIn.push_back(m_lAccY);
		dataArrayIn.push_back(m_lAccZ);

		dataArrayIn.push_back(m_aAccX);
		dataArrayIn.push_back(m_aAccY);
		dataArrayIn.push_back(m_aAccZ);
	}

	void XformCommandPacket::setSubType(int valIn)
	{
		m_subType = valIn;
	}

	void XformCommandPacket::setPos(float xIn, float yIn, float zIn)
	{
		m_posX = xIn;
		m_posY = yIn;
		m_posZ = zIn;
	}

	void XformCommandPacket::setQuat(float xIn, float yIn, float zIn, float wIn)
	{
		m_quatX = xIn;
		m_quatY = yIn;
		m_quatZ = zIn;
		m_quatW = wIn;
	}

	REGISTER_PACKET_FACTORY(XformCommandPacket)

	//WaypointsVectorPacket
	void SetWaypointsPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		int num = dataArrayIn[1].int32_value();
		m_waypoints.resize(num);
		for (int i = 0; i < num; i++) {
			m_waypoints[i][0] = dataArrayIn[i*2+2].float32_value();
			m_waypoints[i][1] = dataArrayIn[i*2+3].float32_value();
		}
	}

	void SetWaypointsPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(int(m_waypoints.size()));
		for (const auto& wp : m_waypoints) {
			dataArrayIn.push_back(wp[0]);
			dataArrayIn.push_back(wp[1]);
		}
	}

	void SetWaypointsPacket::setWaypointsVector(const float* pFloats, unsigned num)
	{
		m_waypoints.resize(num);
		if (pFloats) {
			for (unsigned i = 0; i < num; ++i) {
				m_waypoints[i][0] = *(pFloats++);
				m_waypoints[i][1] = *(pFloats++);
			}
		}
	}

	REGISTER_PACKET_FACTORY(SetWaypointsPacket)

	void MoveToWaypointPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_waypointIndex = dataArrayIn[1].int32_value();
	}

	void MoveToWaypointPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_waypointIndex);
	}

	REGISTER_PACKET_FACTORY(MoveToWaypointPacket)

	// UpdatePosePacket
	void UpdatePosePacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_posX = dataArrayIn[1].float32_value();
		m_posY = dataArrayIn[2].float32_value();
		m_posZ = dataArrayIn[3].float32_value();

		m_quatX = dataArrayIn[4].float32_value();
		m_quatY = dataArrayIn[5].float32_value();
		m_quatZ = dataArrayIn[6].float32_value();
		m_quatW = dataArrayIn[7].float32_value();
	}

	void UpdatePosePacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_posX);
		dataArrayIn.push_back(m_posY);
		dataArrayIn.push_back(m_posZ);

		dataArrayIn.push_back(m_quatX);
		dataArrayIn.push_back(m_quatY);
		dataArrayIn.push_back(m_quatZ);
		dataArrayIn.push_back(m_quatW);
	}

	void UpdatePosePacket::set(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW)
	{
		m_posX = pX;
		m_posY = pY;
		m_posZ = pZ;
		m_quatX = qX;
		m_quatY = qY;
		m_quatZ = qZ;
		m_quatW = qW;
	}

	REGISTER_PACKET_FACTORY(UpdatePosePacket)

	// SetInitialPositionPacket
	void InitialPositionPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_x = dataArrayIn[1].float32_value();
		m_y = dataArrayIn[2].float32_value();
		m_angle = dataArrayIn[3].float32_value();
	}

	void InitialPositionPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_x);
		dataArrayIn.push_back(m_y);
		dataArrayIn.push_back(m_angle);
	}

	void InitialPositionPacket::set(float pX, float pY, float angle)
	{
		m_x = pX;
		m_y = pY;
		m_angle = angle;
	}

	REGISTER_PACKET_FACTORY(InitialPositionPacket)

	//Send Pointcloud Over
	void PointCloudCommandPacket::setPts(const std::vector<PointCloudCommandPacket::PtData>& ptsIn)
	{
		m_pts = ptsIn;
	}

	const std::vector<PointCloudCommandPacket::PtData>& PointCloudCommandPacket::getPts() const
	{
		return m_pts;
	}

	void PointCloudCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_pts.clear();
		const int numPts = dataArrayIn[1].int32_value();
		m_pts.reserve(numPts);
		for(int i = 0; i < numPts; i++)
		{
			float cX = dataArrayIn[i * 3 + 2].float32_value();
			float cY = dataArrayIn[i * 3 + 3].float32_value();
			float cZ = dataArrayIn[i * 3 + 4].float32_value();
			m_pts.push_back(PtData(cX, cY, cZ));
		}
	}

	void PointCloudCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(static_cast<int>(m_pts.size()));
		for(const auto& cPt : m_pts)
		{
			dataArrayIn.push_back(cPt.x);
			dataArrayIn.push_back(cPt.y);
			dataArrayIn.push_back(cPt.z);
		}
	}

	REGISTER_PACKET_FACTORY(PointCloudCommandPacket)

} // end of namespace SpotClient