#include "gazebo_client.hpp"
#include <tinyxml2.h>
#include <ignition/math.hh>
#include <cmath>

namespace igmath = ignition::math;
using std::string;

enum propeller_type
{
    CW,
    CCW
};
/**
 * @brief wing is in x-z plane. with thrust in z. y is surface normal 
 * 
 */
class TailsitterSpawner
{
private:
    const string tailsitter_name = "tailsitter";

    const igmath::Vector3d body_dims = igmath::Vector3d(1.0, 0.4, 0.5);
    const igmath::Vector3d body_mesh_dims = igmath::Vector3d(1000, 400, 500);
    const double body_mass = 1.6;
    const double body_mesh_scale = body_dims.X() / body_mesh_dims.X();
    const string body_mesh_name = "tailsitter_body_wo_elevons";

    const double propeller_radius = 0.1, propeller_thickness = 0.005, propeller_mass = 0.005;
    const string propeller_mesh_name = "iris_prop";
    const igmath::Vector3d propeller_mesh_dims = igmath::Vector3d(0.2577, 0.0280, 0.0103);
    const double propeller_mesh_scale_factor = (2 * propeller_radius) / propeller_mesh_dims.X();
    const igmath::Vector3d propeller_distance = igmath::Vector3d(
        body_dims.X() / 3,
        0.0,
        body_dims.Z() / 2 + 2 * propeller_thickness);

    const igmath::Vector3d elevon_dims = igmath::Vector3d(body_dims.X() / 2, 0.005, 0.135);
    const double elevon_mass = 0.01;
    const string elevon_mesh = "elevon";
    // body mesh and elevon mesh are at same scale
    const double elevon_mesh_scale = body_mesh_scale;
    const igmath::Vector3d elevon_distance = igmath::Vector3d(
        elevon_dims.X() / 2,
        0,
        0.313 - body_dims.Z() / 2 + elevon_dims.Z() / 2);

    GazeboClient gazebo_client;
    tinyxml2::XMLDocument doc;

    static const string toString(const igmath::Pose3d pose)
    {
        std::stringstream pose_str;
        pose_str << pose;
        return pose_str.str();
    }

    template <typename T>
    static const string toString(const igmath::Vector3<T> &x)
    {
        std::stringstream stream;
        stream << x;
        return stream.str();
    }

    static const string toString(const tinyxml2::XMLDocument &doc)
    {
        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);
        string s(printer.CStr());
        return s;
    }

    /**
     * @brief push child to parent. Order kept for readability
     * 
     */
    static void pushChild(tinyxml2::XMLElement *child, tinyxml2::XMLElement *parent)
    {
        parent->InsertEndChild(child);
    }

    tinyxml2::XMLElement *getInertial(
        const double &mass,
        const igmath::Vector3d &principal_inertia)
    {
        auto inertial = doc.NewElement("inertial");
        inertial->InsertNewChildElement("mass")->SetText(mass);
        auto inertia = inertial->InsertNewChildElement("inertia");
        inertia->InsertNewChildElement("ixx")->SetText(principal_inertia.X());
        inertia->InsertNewChildElement("ixy")->SetText(0.0);
        inertia->InsertNewChildElement("ixz")->SetText(0.0);
        inertia->InsertNewChildElement("iyy")->SetText(principal_inertia.Y());
        inertia->InsertNewChildElement("iyz")->SetText(0.0);
        inertia->InsertNewChildElement("izz")->SetText(principal_inertia.Z());
        return inertial;
    }

    tinyxml2::XMLElement *getCollision(tinyxml2::XMLElement *shape_geometry, const string &parent_name)
    {
        auto collision = doc.NewElement("collision");
        collision->SetAttribute("name", (parent_name + "_collision").c_str());
        auto collision_geometry = collision->InsertNewChildElement("geometry");
        pushChild(shape_geometry, collision_geometry);

        auto surface = collision->InsertNewChildElement("surface");
        surface->InsertNewChildElement("contact")->InsertNewChildElement("ode");
        surface->InsertNewChildElement("friction")->InsertNewChildElement("ode");
        return collision;
    }

    tinyxml2::XMLElement *getCylindricalCollision(
        const double &length,
        const double &radius, const string &parent_name)
    {
        auto cylinder = doc.NewElement("cylinder");
        cylinder->InsertNewChildElement("length")->SetText(length);
        cylinder->InsertNewChildElement("radius")->SetText(radius);
        return getCollision(cylinder, parent_name);
    }

    tinyxml2::XMLElement *getBoxCollision(igmath::Vector3d dims, const string &parent_name)
    {
        auto box = doc.NewElement("box");
        box->InsertNewChildElement("size")->SetText(toString(dims).c_str());
        return getCollision(box, parent_name);
    }

    tinyxml2::XMLElement *getMeshVisual(const string &parent_name,
                                        const string &mesh_name,
                                        const string &color = "Blue",
                                        const double scale = 1.,
                                        const igmath::Pose3d pose = igmath::Pose3d::Zero)
    {
        auto visual = doc.NewElement("visual");
        visual->SetAttribute("name", (parent_name + "_visual").c_str());

        visual->InsertNewChildElement("pose")->SetText(toString(pose).c_str());
        auto body_mesh = visual->InsertNewChildElement("geometry")->InsertNewChildElement("mesh");
        body_mesh->InsertNewChildElement("scale")->SetText(toString(scale * igmath::Vector3d::One).c_str());
        body_mesh->InsertNewChildElement("uri")->SetText(("file://pigeon_sim/meshes/" + mesh_name + ".dae").c_str());
        visual->InsertNewChildElement("material")
            ->InsertNewChildElement("script")
            ->InsertNewChildElement("name")
            ->SetText(("Gazebo/" + color).c_str());
        return visual;
    }
    igmath::Vector3d boxInertia(const double &mass, const igmath::Vector3d &dim)
    {
        return 1 / 12. * mass * igmath::Vector3d((dim * igmath::Vector3d(0, 1, 1)).SquaredLength(), (dim * igmath::Vector3d(1, 0, 1)).SquaredLength(), (dim * igmath::Vector3d(1, 1, 0)).SquaredLength());
    }

    tinyxml2::XMLElement *getBody(const string body_name = "base")
    {
        auto body_link = doc.NewElement("link");
        body_link->SetAttribute("name", (body_name + "_link").c_str());

        // solid mass dims is thinner than collision box
        auto body_solid_dims = body_dims;
        body_solid_dims.Y(0.05);
        pushChild(
            getInertial(body_mass, boxInertia(body_mass, body_solid_dims)),
            body_link);
        pushChild(getBoxCollision(body_dims, body_name), body_link);
        igmath::Pose3d mesh_pose(
            0., 0, body_dims.Z() / 2,
            0., -igmath::Angle::HalfPi.Radian(), igmath::Angle::HalfPi.Radian());
        pushChild(
            getMeshVisual(body_name, body_mesh_name, "Grey", body_mesh_scale, mesh_pose),
            body_link);
        body_link->InsertNewChildElement("velocity_decay");
        return body_link;
    }

    igmath::Vector3d getPropellerPosition(const propeller_type &prop_type)
    {
        return prop_type == CCW ? propeller_distance : propeller_distance * igmath::Vector3d(-1, 1, 1);
    }

    string getPropellerRotateDirection(const propeller_type &prop_type)
    {
        return prop_type == CW ? "cw" : "ccw";
    }

    string getPropellerMeshName(const propeller_type &prop_type)
    {
        return propeller_mesh_name + "_" + getPropellerRotateDirection(prop_type);
    }

    /**
     * @brief Get the Propeller object
     * 
     * @param idx 
     * @param pos : relative to joint
     * @return tinyxml2::XMLElement* 
     */
    tinyxml2::XMLElement *getPropeller(const int &idx, const propeller_type &type, const string &pose_parent_frame)
    {
        const igmath::Vector3d pos = getPropellerPosition(type);
        const string name = "rotor_" + std::to_string(idx);
        auto link = doc.NewElement("link");
        link->SetAttribute("name", (name + "_link").c_str());
        auto pose = link->InsertNewChildElement("pose");
        pose->SetText(toString(igmath::Pose3d(pos, igmath::Quaterniond::Zero)).c_str());
        pose->SetAttribute("frame", pose_parent_frame.c_str());

        link->InsertEndChild(getInertial(propeller_mass, igmath::Vector3d(9.75e-7, 1.66704e-4, 1.66704e-4)));
        link->InsertEndChild(getCylindricalCollision(propeller_thickness, propeller_radius, name));
        link->InsertEndChild(getMeshVisual(name, getPropellerMeshName(type), "White", propeller_mesh_scale_factor));
        return link;
    }

    tinyxml2::XMLElement *getElevon(const int &idx, const string &body_frame)
    {

        const igmath::Vector3d pos = elevon_distance * igmath::Vector3d(std::pow(-1, idx + 1), 1, -1);
        const string name = "elevon_" + std::to_string(idx);
        auto link = doc.NewElement("link");
        link->SetAttribute("name", (name + "_link").c_str());

        auto pose = link->InsertNewChildElement("pose");
        pose->SetText(toString(igmath::Pose3d(pos, igmath::Quaterniond::Zero)).c_str());
        pose->SetAttribute("frame", body_frame.c_str());

        pushChild(getInertial(elevon_mass, boxInertia(elevon_mass, elevon_dims)), link);
        pushChild(getBoxCollision(elevon_dims, name), link);

        igmath::Pose3d mesh_pose(
            0., 0., 0.,
            igmath::Angle::Pi.Radian(), 0., 0.);
        pushChild(getMeshVisual(name, elevon_mesh, "White", elevon_mesh_scale, mesh_pose), link);

        return link;
    }

    tinyxml2::XMLElement *getRevoluteJoint(
        tinyxml2::XMLElement *parent,
        tinyxml2::XMLElement *child,
        const double limit_lower,
        const double limit_upper,
        const double initial_angle,
        const igmath::Vector3i rotation_axis = igmath::Vector3i(0, 0, 1),
        const igmath::Pose3d joint_pose_in_child_frame = igmath::Pose3d::Zero)
    {
        auto joint = doc.NewElement("joint");
        string joint_name = string(parent->Attribute("name")) + "_" + child->Attribute("name") + "_joint";
        joint->SetAttribute("name", joint_name.c_str());
        joint->SetAttribute("type", "revolute");
        joint->InsertNewChildElement("parent")->SetText(parent->Attribute("name"));
        joint->InsertNewChildElement("child")->SetText(child->Attribute("name"));
        joint->InsertNewChildElement("pose")->SetText(toString(joint_pose_in_child_frame).c_str());
        auto axis = joint->InsertNewChildElement("axis");
        // does not works for now: https://github.com/osrf/gazebo/issues/2694
        axis->InsertNewChildElement("initial_position")->SetText(initial_angle);
        axis->InsertNewChildElement("xyz")->SetText(toString(rotation_axis).c_str());
        // friction set to high value to make it stay on set angle
        axis->InsertNewChildElement("dynamics")->InsertNewChildElement("friction")->SetText(1e6);

        auto limit = axis->InsertNewChildElement("limit");
        // todo: can change this to continous and remove limits
        limit->InsertNewChildElement("lower")->SetText(limit_lower);
        limit->InsertNewChildElement("upper")->SetText(limit_upper);
        axis->InsertNewChildElement("use_parent_model_frame")->SetText(1);
        return joint;
    }

    tinyxml2::XMLElement *getContinousRevoluteJoint(
        tinyxml2::XMLElement *parent,
        tinyxml2::XMLElement *child,
        const igmath::Vector3i rotation_axis = igmath::Vector3i(0, 0, 1),
        const igmath::Pose3d joint_pose_in_child_frame = igmath::Pose3d::Zero)
    {
        auto joint = doc.NewElement("joint");
        string joint_name = string(parent->Attribute("name")) + "_" + child->Attribute("name") + "_joint";
        joint->SetAttribute("name", joint_name.c_str());
        joint->SetAttribute("type", "revolute");
        joint->InsertNewChildElement("parent")->SetText(parent->Attribute("name"));
        joint->InsertNewChildElement("child")->SetText(child->Attribute("name"));
        joint->InsertNewChildElement("pose")->SetText(toString(joint_pose_in_child_frame).c_str());
        auto axis = joint->InsertNewChildElement("axis");
        axis->InsertNewChildElement("xyz")->SetText(toString(rotation_axis).c_str());
        auto limit = axis->InsertNewChildElement("limit");
        limit->InsertNewChildElement("lower")->SetText(-1e16);
        limit->InsertNewChildElement("upper")->SetText(1e16);

        // todo: can change this to continous and remove limits
        axis->InsertNewChildElement("use_parent_model_frame")->SetText(1);
        return joint;
    }

    string getPropellerDownwashTopic(const int &motor_number)
    {
        return "motor/" + std::to_string(motor_number) + "/downwash";
    }

    tinyxml2::XMLElement *getMotorPlugin(const char *motor_link_name,
                                         const char *motor_joint_name,
                                         const int &motor_number,
                                         const propeller_type &prop_orientation,
                                         const char *motor_plugin_name = "librotors_gazebo_motor_model.so")
    {
        auto plugin = doc.NewElement("plugin");
        plugin->SetAttribute("name", ("rotor_" + std::to_string(motor_number) + "_dynamics").c_str());
        plugin->SetAttribute("filename", motor_plugin_name);
        plugin->InsertNewChildElement("robotNamespace")->SetText(tailsitter_name.c_str());
        plugin->InsertNewChildElement("linkName")->SetText(motor_link_name);
        plugin->InsertNewChildElement("jointName")->SetText(motor_joint_name);
        plugin->InsertNewChildElement("motorNumber")->SetText(motor_number);
        plugin->InsertNewChildElement("turningDirection")->SetText(getPropellerRotateDirection(prop_orientation).c_str());
        plugin->InsertNewChildElement("commandSubTopic")->SetText("/gazebo/command/motor_speed");
        plugin->InsertNewChildElement("motorSpeedPubTopic")->SetText(("motor_speed/" + std::to_string(motor_number)).c_str());
        plugin->InsertNewChildElement("motorDownwashPubTopic")->SetText(getPropellerDownwashTopic(motor_number).c_str());
        plugin->InsertNewChildElement("propellerDiameter")->SetText(2 * propeller_radius);

        plugin->InsertNewChildElement("motorConstant")->SetText(8e-5);

        return plugin;
    }

    tinyxml2::XMLElement *getAerodynamicsPlugin(const char *body_link_name,
                                                const double &surface_area_wet,
                                                const int &downwash_motor_number = -1,
                                                const char *downwash_motor_joint = "",
                                                const char *motor_plugin_name = "libpigeon_gazebo_flat_plate_aerodynamics.so")
    {
        assert(surface_area_wet >= 0.);
        auto plugin = doc.NewElement("plugin");
        plugin->SetAttribute("name", "flat_plate_aerodynamics");
        plugin->SetAttribute("filename", motor_plugin_name);
        plugin->InsertNewChildElement("lift_surface_link")->SetText(body_link_name);
        plugin->InsertNewChildElement("surface_area_wet")->SetText(surface_area_wet);
        plugin->InsertNewChildElement("surface_normal_axis")->SetText("y");
        // vector from CoG to CoA in body frame
        plugin->InsertNewChildElement("cog_coa")->SetText(toString(body_dims.Z() / 4 * igmath::Vector3d::UnitZ).c_str());
        if (downwash_motor_number >= 0)
        {
            plugin->InsertNewChildElement("downwash_sub_topic")->SetText(getPropellerDownwashTopic(downwash_motor_number).c_str());
            plugin->InsertNewChildElement("propeller_joint")->SetText(downwash_motor_joint);
        }
        return plugin;
    }

    string getTailsitter()
    {
        auto sdf = doc.NewElement("sdf");
        doc.InsertFirstChild(sdf);
        sdf->SetAttribute("version", "1.5");
        auto tailsitter_model = sdf->InsertNewChildElement("model");
        tailsitter_model->SetAttribute("name", tailsitter_name.c_str());
        auto body = getBody();
        pushChild(body, tailsitter_model);

        for (int prop_idx = 0; prop_idx < 2; prop_idx++)
        {
            // add propeller
            auto prop_orientation = prop_idx % 2 == 0 ? CW : CCW;
            auto prop = getPropeller(prop_idx, prop_orientation, body->Attribute("name"));
            pushChild(prop, tailsitter_model);
            auto prop_joint = getContinousRevoluteJoint(body, prop);
            pushChild(prop_joint, tailsitter_model);
            auto motor_dynamics_plugin = getMotorPlugin(
                prop->Attribute("name"),
                prop_joint->Attribute("name"),
                prop_idx,
                prop_orientation);
            pushChild(motor_dynamics_plugin, tailsitter_model);

            // add elevon
            auto elevon = getElevon(prop_idx, body->Attribute("name"));
            pushChild(elevon, tailsitter_model);
            auto elevon_joint = getRevoluteJoint(
                body,
                elevon,
                -igmath::Angle::Pi.Radian() / 6,
                igmath::Angle::Pi.Radian() / 6,
                std::pow(-1, prop_idx) * igmath::Angle::Pi.Radian() / 7,
                igmath::Vector3i(1, 0, 0),
                igmath::Pose3d(0, 0, elevon_dims.Z() / 2, 0, 0, 0));
            pushChild(elevon_joint, tailsitter_model);

            auto aerodynamics_plugin = getAerodynamicsPlugin(
                elevon->Attribute("name"),
                elevon_dims.X() * elevon_dims.Z(), prop_idx, prop_joint->Attribute("name"));
            pushChild(aerodynamics_plugin, tailsitter_model);
        }

        auto aerodynamics_plugin = getAerodynamicsPlugin(
            body->Attribute("name"),
            body_dims.X() * body_dims.Z());
        pushChild(aerodynamics_plugin, tailsitter_model);

        ROS_INFO_STREAM(toString(doc).c_str());
        return toString(doc).c_str();
    }

public:
    TailsitterSpawner(const ros::NodeHandle &_nh) : gazebo_client(_nh) {}
    ~TailsitterSpawner() {}

    void spawnTailsitter()
    {
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 2;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        auto tailsitter_SDF = getTailsitter();
        gazebo_client.deleteModel(tailsitter_name);
        gazebo_client.spawnModel(tailsitter_SDF, pose, tailsitter_name);
    }
};
