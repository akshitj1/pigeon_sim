#include <gazebo/gazebo.hh>

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

#include "common.h"

namespace gazebo
{
    class GazeboRosTransport
    {
    private:
        // Create temporary "ConnectGazeboToRosTopic" publisher and message
        gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub;

        // Create temporary "ConnectRosToGazeboTopic" publisher and message
        gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub;
        const std::string model_namespace;
        gazebo::transport::NodePtr node_handle;

    public:
        GazeboRosTransport(gazebo::transport::NodePtr &_node_handle, const std::string &_model_namespace)
            : model_namespace(_model_namespace),
              node_handle(_node_handle)
        {
            gz_connect_gazebo_to_ros_topic_pub =
                _node_handle->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                    "~/" + kConnectGazeboToRosSubtopic, 1);

            gz_connect_ros_to_gazebo_topic_pub = _node_handle->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
                "~/" + kConnectRosToGazeboSubtopic, 1);
        }

    private:
        std::string getRosTopic(const std::string &topic)
        {
            return model_namespace + "/" + topic;
        }

        std::string getGazeboTopic(const std::string &topic)
        {
            return "~/" + getRosTopic(topic);
        }

    public:
        template <typename M, typename T>

        gazebo::transport::SubscriberPtr getSubscriber(
            const std::string &topic,
            const gz_std_msgs::ConnectRosToGazeboTopic_MsgType &msg_type,
            void (T::*_callback_fn)(const boost::shared_ptr<M const> &),
            T *_obj)
        {
            auto subscriber = node_handle->Subscribe(
                getGazeboTopic(topic),
                _callback_fn,
                _obj);

            gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
            connect_ros_to_gazebo_topic_msg.set_ros_topic(getRosTopic(topic));
            connect_ros_to_gazebo_topic_msg.set_gazebo_topic(getGazeboTopic(topic));
            connect_ros_to_gazebo_topic_msg.set_msgtype(msg_type);

            gz_connect_ros_to_gazebo_topic_pub->Publish(
                connect_ros_to_gazebo_topic_msg, true);
            return subscriber;
        }

        template <typename T>
        gazebo::transport::PublisherPtr getPublisher(const std::string &topic,
                                                     const gz_std_msgs::ConnectGazeboToRosTopic_MsgType &msg_type)
        {
            auto publisher = node_handle->Advertise<T>(getGazeboTopic(topic), 1);

            gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
            connect_gazebo_to_ros_topic_msg.set_gazebo_topic(getGazeboTopic(topic));
            connect_gazebo_to_ros_topic_msg.set_ros_topic(getRosTopic(topic));
            connect_gazebo_to_ros_topic_msg.set_msgtype(msg_type);
            gz_connect_gazebo_to_ros_topic_pub->Publish(
                connect_gazebo_to_ros_topic_msg, true);
            return publisher;
        }
    };
} // namespace gazebo