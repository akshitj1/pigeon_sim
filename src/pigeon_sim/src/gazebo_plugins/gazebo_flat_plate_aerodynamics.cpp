#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

#include "common.h"
#include "Float32.pb.h"
#include "CommandMotorSpeed.pb.h"

namespace igmath = ignition::math;

namespace gazebo
{
	/**
	 * @brief Defines aerodynamic forces for tailsitter body including control surfaces
	 * 
	 * sdf usage example:
	 * <plugin name="foo_aero_forces" filename="libgazebo_flat_plate_aerodynamics.so">
	 * <lift_surfaces>
	 * 	<lift_surface type="fixed/revolute">
	 * 		<surface_link></surface_link_name>
	 * 		<wetted_surface_area></wetted_surface_area>
	 * 		// optional
	 * 		<propwash>
	 * 			<prop_link></propwash_source_rotor_link>
	 * 			<prop_speed_sub_topic></prop_speed_sub_topic>
	 * 		</propwash>
	 * 		// only for type revolute type
	 * 		<joint_actuator_command_sub_topic></joint_actuator_command_sub_topic>
	 * 	</lift_surface>
	 * </lift_surfaces>
	 * </plugin>
	 */
	class GazeboFlatPlateAerodynamics : public ModelPlugin
	{
	private:
		physics::ModelPtr model_;
		physics::LinkPtr lift_surface;
		double surface_area_wet;
		igmath::Vector3d surface_normal;
		// Center of gravity to aerodynamic center vector
		igmath::Vector3d cog_coa;

		// Pointer to the update event connection
		event::ConnectionPtr updateConnection_;
		gazebo::transport::NodePtr node_handle_;
		std::string namespace_ = "tailsitter";
		gazebo::transport::SubscriberPtr command_sub_;
		std::string command_sub_topic_;
		typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> GzCommandMotorInputMsgPtr;

	public:
		GazeboFlatPlateAerodynamics() : ModelPlugin() {}

		virtual void Load(physics::ModelPtr _model,
						  sdf::ElementPtr _sdf)
		{
			model_ = _model;

			// node_handle_ = gazebo::transport::NodePtr(new transport::Node());

			// Initialise with default namespace (typically /gazebo/default/)
			// node_handle_->Init();

			// we will follow lowercase naming convention
			// getSdfParam<std::string>(
			// 	_sdf, "command_sub_topic", command_sub_topic_, command_sub_topic_);

			if (!_sdf->HasElement("lift_surface_link"))
				gzerr << "lift_surface_link unspecified" << std::endl;
			std::string lift_surface_name = _sdf->Get<std::string>("lift_surface_link");
			lift_surface = model_->GetLink(lift_surface_name);

			if (!_sdf->HasElement("surface_area_wet"))
				gzerr << "surface_area_wet unspecified" << std::endl;
			surface_area_wet = _sdf->Get<double>("surface_area_wet");

			surface_normal = igmath::Vector3d::UnitZ;
			if (_sdf->HasElement("surface_normal_axis"))
			{
				std::string surface_normal_axis = _sdf->Get<std::string>("surface_normal_axis");
				if (surface_normal_axis == "x")
					surface_normal = igmath::Vector3d::UnitX;
				else if (surface_normal_axis == "y")
					surface_normal = igmath::Vector3d::UnitY;
				else if (surface_normal_axis == "z")
					surface_normal = igmath::Vector3d::UnitZ;
				else
					gzwarn << "surface_normal_axis not in x/y/z. Defaulting to z-axis" << std::endl;
			}
			else
			{
				gzwarn << "surface_normal_axis not specified. Defaulting to z-axis" << std::endl;
			}
			surface_area_wet = _sdf->Get<double>("surface_area_wet");

			cog_coa = igmath::Vector3d::Zero;
			if (_sdf->HasElement("cog_coa"))
			{	
				std::string cog_coa_s = _sdf->Get<std::string>("cog_coa");
				std::istringstream(cog_coa_s) >> cog_coa;
				gzdbg << "cog to coa vector set to: "<< cog_coa << std::endl; 
			}

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			updateConnection_ = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&GazeboFlatPlateAerodynamics::OnUpdate, this, _1));
		}

		/// \brief Override this method for custom plugin initialization behavior.
		virtual void Init()
		{
			CreatePubsAndSubs();
		}

		void CreatePubsAndSubs()
		{
			// Create temporary "ConnectGazeboToRosTopic" publisher and message
			// gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
			// 	node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
			// 		"~/" + kConnectGazeboToRosSubtopic, 1);
			// gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

			// // Create temporary "ConnectRosToGazeboTopic" publisher and message
			// gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
			// 	node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
			// 		"~/" + kConnectRosToGazeboSubtopic, 1);
			// gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

			// ============================================ //
			// = ELEVON SERVO CONTROL COMMAND MSG SETUP (ROS->GAZEBO) = //
			// ============================================ //
			// command_sub_ = node_handle_->Subscribe(
			// 	"~/" + namespace_ + "/" + command_sub_topic_,
			// 	&ControlCommandCallback, this);

			// connect_ros_to_gazebo_topic_msg.set_ros_topic(
			// 	namespace_ + "/" + command_sub_topic_);
			// connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
			// 	"~/" + namespace_ + "/" + command_sub_topic_);
			// connect_ros_to_gazebo_topic_msg.set_msgtype(
			// 	gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED);
			// gz_connect_ros_to_gazebo_topic_pub->Publish(
			// 	connect_ros_to_gazebo_topic_msg, true);

			gzdbg << "[GazeboFlatPlateAerodynamics] pubs and subs created" << std::endl;
		}

		/**
		 * @brief called on every simulation update
		 */
		virtual void OnUpdate(const common::UpdateInfo &_info)
		{
			UpdateForcesAndMoments();
		}

		void UpdateForcesAndMoments()
		{
			const auto &pose_Ao_W = lift_surface->WorldCoGPose();
			const auto &R_WA = pose_Ao_W.Rot();
			const auto &p_dot_A0_W = lift_surface->WorldCoGLinearVel();
			double sin_attack_angle = -p_dot_A0_W.Normalized().Dot(R_WA.RotateVector(surface_normal));
			const double atmospheric_density = 1.204;
			double aero_force_magnitude = atmospheric_density * p_dot_A0_W.SquaredLength() * surface_area_wet * sin_attack_angle;
			auto aero_force = aero_force_magnitude * surface_normal;
			if (false)//(aero_force_magnitude > 0.01)
			{
				gzdbg << "link vel: " << p_dot_A0_W << std::endl;
				gzdbg << "attack angle:" << sin_attack_angle << std::endl;
				gzdbg << "aero force: " << aero_force << std::endl;
			}
			auto aero_force_W = R_WA.RotateVector(aero_force);
			// todo: change for aerodynamic center
			lift_surface->AddForceAtRelativePosition(aero_force_W, cog_coa);
		}

		/// \brief Override this method for custom plugin reset behavior.
		// void Reset() {}

		// void ControlCommandCallback(GzCommandMotorInputMsgPtr &command_motor_input_msg) {}
	};

	GZ_REGISTER_MODEL_PLUGIN(GazeboFlatPlateAerodynamics);
} // namespace gazebo