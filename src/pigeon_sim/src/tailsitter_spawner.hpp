#include "gazebo_client.hpp"
#include <tinyxml2.h>
#include <ignition/math.hh>

namespace igmath = ignition::math;
using std::string;

enum propeller_type
{
    CW,
    CCW
};

class TailsitterSpawner
{
private:
    const string tailsitter_name = "tailsitter";
    const igmath::Vector3d body_dims = igmath::Vector3d(1.0, 0.4, 0.5);
    const igmath::Vector3d body_mesh_dims = igmath::Vector3d(1000, 400, 500);
    const double body_mass = 1.6;
    const string body_mesh_name = "pacflyer_s100_mockup";

    const double propeller_radius = 0.1, propeller_thickness = 0.005, propeller_mass = 0.005;
    const string propeller_mesh_name = "iris_prop";
    const igmath::Vector3d propeller_mesh_dims = igmath::Vector3d(0.2577, 0.0280, 0.0103);
    const double propeller_mesh_scale_factor = (2 * propeller_radius) / propeller_mesh_dims.X();
    const igmath::Vector3d propeller_distance = igmath::Vector3d(
        body_dims.X() / 3,
        0.0,
        body_dims.Z() / 2 + 2 * propeller_thickness);

    GazeboClient gazebo_client;
    tinyxml2::XMLDocument doc;

    static const string toString(const igmath::Pose3d pose)
    {
        std::stringstream pose_str;
        pose_str << pose;
        ROS_INFO("%s", pose_str.str().c_str());
        return pose_str.str();
    }

    static const string toString(const igmath::Vector3d x)
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

    tinyxml2::XMLElement *getBody(const string name = "base")
    {
        auto base_link = doc.NewElement("link");
        base_link->SetAttribute("name", (name + "_link").c_str());

        // calculate inertia
        // since static, mass and inertial is irrelevant
        const double mass = 1.6;
        const igmath::Vector3d principal_inertia(0.147563, 0.0458929, 0.1977);
        base_link->InsertEndChild(getInertial(mass, principal_inertia));

        // collison box

        auto collision = base_link->InsertNewChildElement("collision");
        collision->SetAttribute("name", string(base_link->Attribute("name")).append("_collision").c_str());
        auto collision_geometry = collision->InsertNewChildElement("geometry");
        collision_geometry->InsertNewChildElement("box")->InsertNewChildElement("size")->SetText(toString(body_dims).c_str());

        auto surface = collision->InsertNewChildElement("surface");
        auto contact_ode = surface->InsertNewChildElement("contact")->InsertNewChildElement("ode");
        contact_ode->InsertNewChildElement("max_vel")->SetText(10);
        contact_ode->InsertNewChildElement("min_depth")->SetText(0.01);
        surface->InsertNewChildElement("friction")->InsertNewChildElement("ode");

        // visual mesh
        auto visual = base_link->InsertNewChildElement("visual");
        visual->SetAttribute("name", string(base_link->Attribute("name")).append("_visual").c_str());

        visual->InsertNewChildElement("pose")->SetText(toString(igmath::Pose3d(0., 0, body_dims.Z() / 2, 0., -igmath::Angle::HalfPi.Radian(), igmath::Angle::HalfPi.Radian())).c_str());
        // visual->InsertNewChildElement("geometry")->InsertNewChildElement("box")->InsertNewChildElement("size")->SetText(toString(igmath::Vector3d(lx, ly, lz)).c_str());
        auto body_mesh = visual->InsertNewChildElement("geometry")->InsertNewChildElement("mesh");
        body_mesh->InsertNewChildElement("scale")->SetText(toString(body_dims / body_mesh_dims).c_str());
        body_mesh->InsertNewChildElement("uri")->SetText(("file://pigeon_sim/meshes/" + body_mesh_name + ".dae").c_str());

        auto material = visual->InsertNewChildElement("material");
        auto material_script = material->InsertNewChildElement("script");
        material_script->InsertNewChildElement("uri")->SetText("file://media/materials/scripts/gazebo.material");
        material_script->InsertNewChildElement("name")->SetText("Gazebo/DarkGrey");

        base_link->InsertNewChildElement("velocity_decay");
        return base_link;
    }

    tinyxml2::XMLElement *getCylindricalCollision(
        const double &length,
        const double &radius, const string &parent_name)
    {
        auto collision = doc.NewElement("collision");
        collision->SetAttribute("name", (parent_name + "_collision").c_str());
        auto collision_geometry = collision->InsertNewChildElement("geometry");
        auto cylinder = collision_geometry->InsertNewChildElement("cylinder");
        cylinder->InsertNewChildElement("length")->SetText(length);
        cylinder->InsertNewChildElement("radius")->SetText(radius);

        auto surface = collision->InsertNewChildElement("surface");
        surface->InsertNewChildElement("contact")->InsertNewChildElement("ode");
        surface->InsertNewChildElement("friction")->InsertNewChildElement("ode");
        return collision;
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
        return visual;
    }

    igmath::Vector3d getPropellerPosition(const propeller_type &prop_type)
    {
        return prop_type == CCW ? propeller_distance : propeller_distance * igmath::Vector3d(-1, 1, 1);
    }

    string getPropellerRotateDirection(const propeller_type &prop_type){
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
        link->InsertEndChild(getMeshVisual(name, getPropellerMeshName(type), "DarkGrey", propeller_mesh_scale_factor));
        return link;
    }

    tinyxml2::XMLElement *getRevoluteJoint(tinyxml2::XMLElement *parent, tinyxml2::XMLElement *child)
    {
        auto joint = doc.NewElement("joint");
        string joint_name = string(parent->Attribute("name")) + "_" + child->Attribute("name") + "_joint";
        joint->SetAttribute("name", joint_name.c_str());
        joint->SetAttribute("type", "revolute");
        joint->InsertNewChildElement("parent")->SetText(parent->Attribute("name"));
        joint->InsertNewChildElement("child")->SetText(child->Attribute("name"));
        auto axis=joint->InsertNewChildElement("axis");
        axis->InsertNewChildElement("xyz")->SetText("0 0 1");
        auto limit = axis->InsertNewChildElement("limit");
        limit->InsertNewChildElement("lower")->SetText(-1e16);
        limit->InsertNewChildElement("upper")->SetText(1e16);
        axis->InsertNewChildElement("use_parent_model_frame")->SetText(1);
        return joint;
    }

    tinyxml2::XMLElement *getMotorPlugin(const char* motor_link_name,
    const char* motor_joint_name,
    const int& motor_number,
    const propeller_type& prop_orientation,
    const char* motor_plugin_name="librotors_gazebo_motor_model.so")
    {
        auto plugin = doc.NewElement("plugin");
        plugin->SetAttribute("name", "rotor_model_dynamics");
        plugin->SetAttribute("filename", motor_plugin_name);
        plugin->InsertNewChildElement("robotNamespace")->SetText(tailsitter_name.c_str());
        plugin->InsertNewChildElement("linkName")->SetText(motor_link_name);
        plugin->InsertNewChildElement("jointName")->SetText(motor_joint_name);
        plugin->InsertNewChildElement("motorNumber")->SetText(motor_number);
        plugin->InsertNewChildElement("turningDirection")->SetText(getPropellerRotateDirection(prop_orientation).c_str());
        plugin->InsertNewChildElement("commandSubTopic")->SetText("/gazebo/command/motor_speed");
        plugin->InsertNewChildElement("motorSpeedPubTopic")->SetText(("/motor_speed/"+std::to_string(motor_number)).c_str());
        
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
            auto prop_orientation = prop_idx % 2 == 0 ? CW : CCW;
            auto prop = getPropeller(prop_idx, prop_orientation, body->Attribute("name"));
            pushChild(prop, tailsitter_model);
            auto prop_joint = getRevoluteJoint(body, prop);
            pushChild(prop_joint, tailsitter_model);
            auto motor_dynamics_plugin = getMotorPlugin(
                prop->Attribute("name"), 
                prop_joint->Attribute("name"),
                prop_idx,
                prop_orientation);
            pushChild(motor_dynamics_plugin, tailsitter_model);
        }

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
