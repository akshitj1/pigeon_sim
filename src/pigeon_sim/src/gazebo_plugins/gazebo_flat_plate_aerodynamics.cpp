#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

#include "gazeb_ros_transport.hpp"
#include "common.h"
#include "Float32.pb.h"
#include "CommandMotorSpeed.pb.h"
#include "WindSpeed.pb.h"

namespace igmath = ignition::math;

namespace gazebo
{
	/**
	 * @brief Defines aerodynamic forces for tailsitter body including control surfaces
	 */
	class GazeboFlatPlateAerodynamics : public ModelPlugin
	{
	private:
		physics::ModelPtr model_;
		physics::LinkPtr lift_surface;
		physics::JointPtr propeller_joint;
		physics::LinkPtr downwash_propeller_link;

		double surface_area_wet;
		igmath::Vector3d surface_normal;
		// Center of gravity to aerodynamic center vector
		igmath::Vector3d cog_coa;
		double downwash_speed = 0.;
		double lift_force = 0.;

		// Pointer to the update event connection
		event::ConnectionPtr updateConnection_;
		gazebo::transport::NodePtr node_handle_;
		std::string namespace_ = "tailsitter";

		std::string downwash_sub_topic_;
		gazebo::transport::SubscriberPtr downwash_speed_sub;

		gazebo::transport::PublisherPtr force_pub;

		std::string command_sub_topic_;
		typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> GzCommandMotorInputMsgPtr;

	public:
		GazeboFlatPlateAerodynamics() : ModelPlugin() {}

	protected:
		void Load(physics::ModelPtr _model,
				  sdf::ElementPtr _sdf)
		{
			model_ = _model;

			node_handle_ = gazebo::transport::NodePtr(new transport::Node());

			// Initialise with default namespace (typically /gazebo/default/)
			node_handle_->Init();

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

			if (_sdf->HasElement("propeller_joint") && _sdf->HasElement("downwash_sub_topic"))
			{
				std::string propeller_joint_name = _sdf->Get<std::string>("propeller_joint");
				propeller_joint = _model->GetJoint(propeller_joint_name);
				downwash_sub_topic_ = _sdf->Get<std::string>("downwash_sub_topic");
				lift_surface->GetParentJoints().front()->SetPosition(0, igmath::Angle::Pi.Radian() / 6.1);
			}
			else
			{
				gzwarn << "propeller_joint and downwash_sub_topic not spedified. Ignoring downwash." << std::endl;
			}

			cog_coa = igmath::Vector3d::Zero;
			if (_sdf->HasElement("cog_coa"))
			{
				std::string cog_coa_s = _sdf->Get<std::string>("cog_coa");
				std::istringstream(cog_coa_s) >> cog_coa;
				gzdbg << "cog to coa vector set to: " << cog_coa << std::endl;
			}

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			updateConnection_ = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&GazeboFlatPlateAerodynamics::OnUpdate, this, _1));
		}

	private:
		/// \brief Override this method for custom plugin initialization behavior.
		virtual void Init()
		{
			CreatePubsAndSubs();
		}

		void CreatePubsAndSubs()
		{
			GazeboRosTransport gz_ros(node_handle_, namespace_);
			if (!downwash_sub_topic_.empty())
				downwash_speed_sub = gz_ros.getSubscriber(
					downwash_sub_topic_,
					gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED,
					&GazeboFlatPlateAerodynamics::UpdateDownwashSpeed,
					this);

			std::string force_pub_topic = lift_surface->GetName() + "/lift_force";
			force_pub = gz_ros.getPublisher<gz_std_msgs::Float32>(force_pub_topic, gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);

			gzdbg << "[GazeboFlatPlateAerodynamics] pubs and subs created" << std::endl;
		}

	protected:
		/**
		 * @brief called on every simulation update
		 */
		void OnUpdate(const common::UpdateInfo &_info)
		{
			UpdateForcesAndMoments();
			Publish();
		}

	private:
		void UpdateForcesAndMoments()
		{
			const auto &pose_Ao_W = lift_surface->WorldCoGPose();
			const auto &R_WA = pose_Ao_W.Rot();
			const auto &p_dot_A0_W = lift_surface->WorldCoGLinearVel();
			auto wind_vel_W = -p_dot_A0_W;

			if (!downwash_sub_topic_.empty())
			{
				const auto &propeller_axis = propeller_joint->GlobalAxis(0);
				wind_vel_W += std::max(downwash_speed, 0.) * -propeller_axis;
			}

			double sin_attack_angle = wind_vel_W.Normalized().Dot(R_WA.RotateVector(surface_normal));
			const double atmospheric_density = 1.225; // standard atm. density
			double aero_force_magnitude = atmospheric_density * wind_vel_W.SquaredLength() * surface_area_wet * sin_attack_angle;
			lift_force = aero_force_magnitude;
			auto aero_force = aero_force_magnitude * surface_normal;
			if (false) //(aero_force_magnitude > 0.01)
			{
				gzdbg << "\nlink vel: " << p_dot_A0_W << "\nwind vel: " << wind_vel_W << "\nattack angle:" << sin_attack_angle << "\naero force: " << aero_force << std::endl;
			}
			auto aero_force_W = R_WA.RotateVector(aero_force);
			// todo: change for aerodynamic center
			lift_surface->AddForceAtRelativePosition(aero_force_W, cog_coa);
		}

		void UpdateDownwashSpeed(const boost::shared_ptr<const gz_mav_msgs::WindSpeed> &downwash_speed)
		{
			this->downwash_speed = downwash_speed->velocity().x();
			// gzdbg << "Downwash speed received: " << downwash_speed->DebugString() << std::endl;
			// this->downwash_speed = downwash_speed->data();
		}

		void Publish()
		{
			gz_std_msgs::Float32 lift_force_msg;
			lift_force_msg.set_data(lift_force);
      		force_pub->Publish(lift_force_msg);
		}

		/// \brief Override this method for custom plugin reset behavior.
		// void Reset() {}

		// void ControlCommandCallback(GzCommandMotorInputMsgPtr &command_motor_input_msg) {}
	};

	GZ_REGISTER_MODEL_PLUGIN(GazeboFlatPlateAerodynamics);
} // namespace gazebo