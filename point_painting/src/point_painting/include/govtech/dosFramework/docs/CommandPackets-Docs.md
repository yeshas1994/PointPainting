# CommandPackets Documentation
This describes the CommandPackets framework for DOSS ( Digital Operations Smart Systems ). CommandPackets are the base layer that represents structs you send around different nodes/clients across the network. They allow you to encode a struct to a binary byte stream and back ( Serialization + Deserialization ). 

## Dependencies
- MsgPack
- nlohmann JSON ( C++11 JSON )
- C++14

MsgPack is the underlying binary encoding/decoding layer. We could have also gone for other solutions as well ( ProtoBuf, Flatbuffers, Cap'n'Proto ). Flatbuffers and Cap'n'Proto are much more performant. However, MsgPack is the simplest and has the least amount of dependencies ( and performance wise it isn't too bad, though it still lags behind Flatbuffers/Cap'n'n Proto). MsgPack does not require a schema, it functions more like binary JSON. To be able to do something more dynamic ( read in a a byte stream and create a generic object as opposed to alread knowing the type ), FlatBuffers is a bit more involved for that sort of thing. The advantage of the current approach is it allows us to use regular classes with custom functionality while allowing the reading/writing of incoming/outgoing byte data with a lot more flexibility. The disadvantage is at a small cost of performance and having to write a bit more custom code for serialization/deserialization.


## BaseCommandPacket
This is the base class you should inherit from:
```
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
```

The first is to override **getPacketTypeID()** to return your custom type. You can see some types already defined:

```
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
```

Next, you override **encodeToArray** to serialize data from your class. And the override **parseDataArray** to deseralize. Let's look at a more concrete example:

```
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
```

and the implementation:

```
	// MsgStrCommandPacket
	void MsgStrCommandPacket::parseDataArray(const msgpack11::MsgPack::array& dataArrayIn)
	{
		m_str = dataArrayIn[1].string_value(); // When parsing, always start from index 1 ( not 0 !) because the 0th element always encodes the object type
	}

	void MsgStrCommandPacket::encodeToArray(msgpack11::MsgPack::array& dataArrayIn)
	{
		dataArrayIn.push_back(m_str); // Push the string property into the MsgPack array
	}

	REGISTER_PACKET_FACTORY(MsgStrCommandPacket)
```

Don't forget to call **REGISTER_PACKET_FACTORY(MyCustomCommandPacket)** at the end to register your class.

Now let's look at another more involved example, **MoveVelCommandPacket**. This is a commandPacket that sends a move command with velocities etc. for robot movement:

```
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
```

And its implementation:

```
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
```

## Using CommandPackets
To get the binary byte representation on a **CommandPacket**, just call:
```
    std::string byteData = myPacket.encode(); // The String encodes a pure binary representation, NOT ascii
```
That's basically it, you can now use it to save to a file or pass it across the network.

## Getting CommandPacket from ByteStream
Because all CommandPackets are auto-registered, you can get a CommandPacket of the correct type from a raw byte stream like so:
```
std::unique_ptr<BaseCommandPacket> myPkt = CommandPacketMgr::getCommandPacket(rawByteData);
```
Then proceed to **dynamic_cast** it into the correct type for further processing.






