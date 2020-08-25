#pragma once
#include <msgpack11.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <nlohmann/json.hpp>
#include "imageMeta.hxx"

namespace DosClient
{
	// Base class that takes in a MsgPack array. Take note the first element of the array is
	// ALWAYS an integer denoting the type of CommandPacket it actually is. Use this to instantiate
	// the appropriate command packet
	class BaseCommandPacket {
	public:
		BaseCommandPacket() {}
		virtual ~BaseCommandPacket() {}

		std::string encode();

		// Override this method to return the integer type of this command packet
		virtual int getPacketTypeID() const = 0;

		// Override this method to encode the data into an array, APPEND to this array but
		// do not assign to it since there are other items already inside this array required
		// for the encoding format to work
		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) = 0;

		// Override this method to form your data structure
		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) = 0;

	protected:
	};

	constexpr int COMMAND_MSG_TYPE = 1;
	constexpr int COMMAND_BYTES_TYPE = 2;
	constexpr int COMMAND_MOVE_VEL_TYPE = 3;
	constexpr int COMMAND_POWER_ROBOT_TYPE = 4;
	constexpr int COMMAND_ROBOT_CMD_TYPE = 5;
	constexpr int COMMAND_IMG_TYPE = 6;
	constexpr int COMMAND_IMG_STREAM_TOGGLE_TYPE = 7;
	constexpr int COMMAND_IMG_LIST_TYPE = 8;
	constexpr int COMMAND_STATUS_TYPE = 9;
	constexpr int COMMAND_MOTION_PARAMS_TYPE = 10;
	constexpr int COMMAND_SETTINGS_TYPE = 11;
	constexpr int COMMAND_BYTES_LIST_TYPE = 12;
	constexpr int COMMAND_XFORM_TYPE = 13;
	constexpr int COMMAND_CONTROL_PERMISSIONS_TYPE = 14;
	constexpr int COMMAND_WAYPOINTS_TYPE = 15;
	constexpr int COMMAND_MOVE_WAYPOINT_TYPE = 16;
	constexpr int COMMAND_SPOT_NAVGRAPH_TYPE = 17;
	constexpr int COMMAND_IMG_LIST_WITH_META_TYPE = 18;
	constexpr int COMMAND_POSE_UPDATE_TYPE = 19;
	constexpr int COMMAND_INITIAL_POSITION_TYPE = 20;
	constexpr int COMMAND_POINT_CLOUD_TYPE = 21;

	// A packet that represents a string message: [ typeID, str]
	class MsgStrCommandPacket : public BaseCommandPacket {
	public:
		MsgStrCommandPacket() : BaseCommandPacket() {}
		virtual ~MsgStrCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;
		
		virtual int getPacketTypeID() const override { return COMMAND_MSG_TYPE; }

		const std::string& getStrMsg() const { return m_str; }
		std::string& getStrMsg() { return m_str; }

	protected:
		std::string m_str;
	};

	// A packet that encodes the status in a json string: [typeID, jsonStr]
	class StatusCommandPacket : public MsgStrCommandPacket {
	public:
		StatusCommandPacket() : MsgStrCommandPacket() {}
		virtual ~StatusCommandPacket() {}

		virtual int getPacketTypeID() const override { return COMMAND_STATUS_TYPE; }

		nlohmann::json getStatusJSON() const;
	};

	// A packet that encodes the settings in a json string: [typeID, jsonStr]
	class SettingsCommandPacket : public MsgStrCommandPacket {
	public:
		SettingsCommandPacket() : MsgStrCommandPacket() {}
		virtual ~SettingsCommandPacket() {}

		virtual int getPacketTypeID() const override { return COMMAND_SETTINGS_TYPE; }

		nlohmann::json getStatusJSON() const;
	};

	// A packet that encodes the Spot GraphNav Autonomy data as a json string: [typeID, jsonStr]
	class SpotGraphNavCommandPacket : public MsgStrCommandPacket {
	public:
		SpotGraphNavCommandPacket() : MsgStrCommandPacket() {}
		virtual ~SpotGraphNavCommandPacket() {}

		virtual int getPacketTypeID() const override { return COMMAND_SPOT_NAVGRAPH_TYPE; }

		nlohmann::json getStatusJSON() const;
	};

	// A packet that encodes control permissions in a json string: [typeID, jsonStr]
	class ControlPermissionsPacket : public MsgStrCommandPacket {
	public:
		ControlPermissionsPacket() : MsgStrCommandPacket() {}
		virtual ~ControlPermissionsPacket() {}

		virtual int getPacketTypeID() const override { return COMMAND_CONTROL_PERMISSIONS_TYPE; }

		nlohmann::json getStatusJSON() const;
	};

	// A packet that encodes the motion params in a json string: [typeID, jsonStr]
	class MotionParamsCommandPacket : public MsgStrCommandPacket {
	public:
		MotionParamsCommandPacket() : MsgStrCommandPacket() {}
		virtual ~MotionParamsCommandPacket() {}

		virtual int getPacketTypeID() const override { return COMMAND_MOTION_PARAMS_TYPE; }

		nlohmann::json getStatusJSON() const;

	};

	// A packet that represents a byte stream [typeID, bytes]
	class BytesCommandPacket : public BaseCommandPacket {
	public:
		BytesCommandPacket() : BaseCommandPacket() {}
		virtual ~BytesCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_BYTES_TYPE; }

		const std::vector<uint8_t>& getBytes() const { return m_bytes; }
		std::vector<uint8_t>& getBytes() { return m_bytes; }

	protected:
		std::vector<uint8_t> m_bytes;
	};

	// A packet that represents a list of generic binary blobs sent back from the robot [typeID, numBlobs, [bytes, subType, label] ...]
	class BytesListCommandPacket : public BaseCommandPacket {
	public:
		struct SubBlob
		{
			std::vector<uint8_t> bytes;
			int subType = 0;
			std::string label;
		};

	public:
		BytesListCommandPacket() : BaseCommandPacket() {}
		virtual ~BytesListCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_BYTES_LIST_TYPE; }

		const std::vector<SubBlob>& getBlobs() const { return m_blobs; }
		std::vector<SubBlob>& getBlobs() { return m_blobs; }

	protected:
		std::vector<SubBlob> m_blobs;
	};

	// A packet that represents a Transformation ( position + quaternion ) [typeID, subType, posX, posY, posZ, quatX, quatY, quatZ, quatW]
	class XformCommandPacket : public BaseCommandPacket {
	public:
		XformCommandPacket() : BaseCommandPacket() {}
		virtual ~XformCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_XFORM_TYPE; }

		int subType() const { return m_subType; }
		void setSubType(int valIn);

		void setPos(float xIn, float yIn, float zIn);
		void setQuat(float xIn, float yIn, float zIn, float wIn);

		float pX() const { return m_posX; }
		float pY() const { return m_posY; }
		float pZ() const { return m_posZ; }

		float qX() const { return m_quatX; }
		float qY() const { return m_quatY; }
		float qZ() const { return m_quatZ; }
		float qW() const { return m_quatW; }

		float lVelX() const { return m_lVelX; }
		float lVelY() const { return m_lVelY; }
		float lVelZ() const { return m_lVelZ; }

		float aVelX() const { return m_aVelX; }
		float aVelY() const { return m_aVelY; }
		float aVelZ() const { return m_aVelZ; }

		float lAccX() const { return m_lAccX; }
		float lAccY() const { return m_lAccY; }
		float lAccZ() const { return m_lAccZ; }

		float aAccX() const { return m_aAccX; }
		float aAccY() const { return m_aAccY; }
		float aAccZ() const { return m_aAccZ; }

	protected:
		int m_subType = 0;
		float m_posX = 0;
		float m_posY = 0;
		float m_posZ = 0;
		float m_quatX = 0;
		float m_quatY = 0;
		float m_quatZ = 0;
		float m_quatW = 0;

		float m_lVelX = 0;
		float m_lVelY = 0;
		float m_lVelZ = 0;
		float m_aVelX = 0;
		float m_aVelY = 0;
		float m_aVelZ = 0;

		float m_lAccX = 0;
		float m_lAccY = 0;
		float m_lAccZ = 0;
		float m_aAccX = 0;
		float m_aAccY = 0;
		float m_aAccZ = 0;
	};

	// A packet that represents powering ON/OFF the robot [typeID, int]
	class PowerRobotCommandPacket : public BaseCommandPacket {
	public:
		PowerRobotCommandPacket() : BaseCommandPacket() {}
		virtual ~PowerRobotCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_POWER_ROBOT_TYPE; }

		void setPower(bool on) {
			m_state = (on == true) ? 1 : 0;
		}

		bool isPowered() const { return (m_state == 1) ? true : false; }

	protected:
		int m_state = 0;
	};

	// A packet that represents a Robot Command ( Stand, Sit etc. ): [typeID, Description]
	class RobotCommandPacket : public MsgStrCommandPacket
	{
	public:
		enum class ROBOT_CMD {
			STAND,
			SIT,
			SELF_RIGHT,
			SELF_LEFT
		};

	public:
		RobotCommandPacket()
			: MsgStrCommandPacket()
		{}
		virtual ~RobotCommandPacket() {}

		virtual int getPacketTypeID() const override { return COMMAND_ROBOT_CMD_TYPE; }

		void setRobotCommand(ROBOT_CMD cmdIn);
	};

	// A packet that represents a veloctiy move command: [typeID, Description, X, Y, Rot]
	class MoveVelCommandPacket : public BaseCommandPacket {
	public:
		enum class VEL_CMD
		{
			MOVE_FORWARD,
			MOVE_BACKWARD,
			STRAFE_LEFT,
			STRAFE_RIGHT,
			TURN_LEFT,
			TURN_RIGHT
		};

	public:
		MoveVelCommandPacket() {}
		virtual ~MoveVelCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_MOVE_VEL_TYPE; }

		const std::string& getDescrip() const { return m_descrip; }
		std::string& getDescrip() { return m_descrip; }

		void setVelCmd(VEL_CMD velCmdIn, float magnitude);
		void setVelFreeMove(float xMag, float yMag, float turnMag, float magRange);

		float getX() const { return m_x; }
		float& getX() { return m_x; }

		float getY() const { return m_y; }
		float& getY() { return m_y; }

		float getRot() const { return m_rot; }
		float& getRot() { return m_rot; }

	protected:
		std::string m_descrip;
		float m_x = 0;
		float m_y = 0;
		float m_rot = 0;
	};

	// A packet that represents an image sent back from the robot [typeID, bytes, compressionType]
	constexpr int ROBOT_IMG_COMPRESSION_NONE = 0;
	constexpr int ROBOT_IMG_COMPRESSION_JPG = 1;
	constexpr int ROBOT_IMG_COMPRESSION_JPG_BW = 2;
	class ImageCommandPacket : public BytesCommandPacket
	{
	public:
		ImageCommandPacket() : BytesCommandPacket() {}
		virtual ~ImageCommandPacket() {}

		virtual int getPacketTypeID() const override { return COMMAND_IMG_TYPE; }

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		int getCompressionType() const { return m_compressionType; }

	protected:
		int m_compressionType = ROBOT_IMG_COMPRESSION_NONE;
	};

	// A packet that represents a list of images sent back from the robot [typeID, compressionType, numImages, [bytes, label] ...]
	class ImageListCommandPacket : public BaseCommandPacket
	{
	public:
		struct SubImage
		{
			std::vector<uint8_t> bytes;
			std::string label;
		};

	public:
		ImageListCommandPacket() : BaseCommandPacket(), m_id(0) {}
		virtual ~ImageListCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_IMG_LIST_TYPE; }
		size_t getFrameId() const { return m_id; }
		void setFrameId( size_t id ) { m_id = id; }
		void setImage(std::vector<SubImage> imageIn) { m_images = imageIn; }
	
		int getCompressionType() const { return m_compressionType; }
		
		const std::vector<SubImage>& getImages() const { return m_images; }
		std::vector<SubImage>& getImages() { return m_images; }

	protected:
		int m_compressionType = ROBOT_IMG_COMPRESSION_NONE;
		std::vector<SubImage> m_images;
		
		size_t m_id;
	};

	class ImageListWithMetaCommandPacket : public ImageListCommandPacket
	{
	public:
		ImageListWithMetaCommandPacket()
			: ImageListCommandPacket()
		{}
		virtual void parseDataArray( const msgpack11::MsgPack::array& dataArrayIn ) override;
		virtual void encodeToArray( msgpack11::MsgPack::array& dataArrayIn ) override;
		virtual int getPacketTypeID() const override { return COMMAND_IMG_LIST_WITH_META_TYPE; }
		void setMeta( const ImageMeta & meta ) { m_meta =meta;}
		const ImageMeta& getMeta() const { return m_meta; }
	private:
		ImageMeta m_meta;
	};
	
	// A packet that represents a command to start/stop image streaming from the robot [typeID, int]
	class ImageStreamTogglePacket : public BaseCommandPacket {
	public:
		ImageStreamTogglePacket() : BaseCommandPacket() {}
		virtual ~ImageStreamTogglePacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;
		
		virtual int getPacketTypeID() const override { return COMMAND_IMG_STREAM_TOGGLE_TYPE; }

		void setStream(int valIn)
		{
			m_toggle = valIn;
		}

		void setImageNames(const std::vector<std::string>& namesIn)
		{
			m_imageNames = namesIn;
		}

	protected:
		int m_toggle = 0;
		std::vector<std::string> m_imageNames;
	};	

	// A packet that send a sends a vector of waypoints to the robot
	class SetWaypointsPacket : public BaseCommandPacket {
	public:
		SetWaypointsPacket() : BaseCommandPacket() {}
		
		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_WAYPOINTS_TYPE; }

		void setWaypointsVector(const float *start = nullptr, unsigned num = 0);

		const std::vector<std::array<float, 2>>& getWaypoints() const { return m_waypoints; }

	protected:
		std::vector<std::array<float, 2>> m_waypoints;
	};

	// A packet that send a sends a vector of waypoints to the robot
	class MoveToWaypointPacket : public BaseCommandPacket {
	public:
		MoveToWaypointPacket() : BaseCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_MOVE_WAYPOINT_TYPE; }

		void setWaypointIndex(unsigned index) { m_waypointIndex = index; }

		int getWaypointIndex() const { return m_waypointIndex; }

	protected:
		int m_waypointIndex;
	};

	// A packet that updates the pose calculated by MotionPlanner for visualization
	class UpdatePosePacket : public BaseCommandPacket {
	public:
		UpdatePosePacket() : BaseCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_POSE_UPDATE_TYPE; }

		void set(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW);

		float pX() const { return m_posX; }
		float pY() const { return m_posY; }
		float pZ() const { return m_posZ; }

		float qX() const { return m_quatX; }
		float qY() const { return m_quatY; }
		float qZ() const { return m_quatZ; }
		float qW() const { return m_quatW; }

	protected:
		float m_posX = 0;
		float m_posY = 0;
		float m_posZ = 0;
		float m_quatX = 0;
		float m_quatY = 0;
		float m_quatZ = 0;
		float m_quatW = 0;
	};

	// A packet that tell a spot its current position so that it can localize itself
	class InitialPositionPacket : public BaseCommandPacket {
	public:
		InitialPositionPacket() : BaseCommandPacket() {}

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_INITIAL_POSITION_TYPE; }

		void set(float pX, float pY, float angle);

		float x() const { return m_x; }
		float y() const { return m_y; }
		float a() const { return m_angle; }

	protected:
		float m_x = 0;
		float m_y = 0;
		float m_angle = 0;
	};

	// A packet that sends pointcloud over once requested.
	class PointCloudCommandPacket : public BaseCommandPacket {
	public:
		struct PtData
		{
			PtData(float xIn, float yIn, float zIn)
			: x(xIn), y(yIn), z(zIn)
			{}

			PtData() = default; 

			float x = 0;
			float y = 0;
			float z = 0;
		};

	public:
		PointCloudCommandPacket() : BaseCommandPacket() {}
		virtual ~PointCloudCommandPacket() {}

		void setPts(const std::vector<PtData>& ptsIn);
		const std::vector<PtData>& getPts() const;

		virtual void parseDataArray(const msgpack11::MsgPack::array& dataArrayIn) override;

		virtual void encodeToArray(msgpack11::MsgPack::array& dataArrayIn) override;

		virtual int getPacketTypeID() const override { return COMMAND_POINT_CLOUD_TYPE; }    

	protected:
		std::vector<PtData> m_pts;
	};
}