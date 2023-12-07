#include "utils.h"

namespace Utils
{
    Eigen::Quaterniond FromMsgQuaternion(const geometry_msgs::msg::Quaternion& msg)
    {
        Eigen::Quaterniond q;
        q.x() = msg.x;
        q.y() = msg.y;
        q.z() = msg.z;
        q.w() = msg.w;
        return q;
    }

    Eigen::Vector3d FromMsgPosition(const geometry_msgs::msg::Point& msg)
    {
        Eigen::Vector3d v{ msg.x, msg.y, msg.z };
        return v;
    }
    Eigen::Vector3d FromMsgPosition(const geometry_msgs::msg::Vector3& msg)
    {
        Eigen::Vector3d v{ msg.x, msg.y, msg.z };
        return v;
    }

    geometry_msgs::msg::Point ToMsgPoint(const Eigen::Vector3d& v)
    {
        geometry_msgs::msg::Point msg;
        msg.x = v.x();
        msg.y = v.y();
        msg.z = v.z();
        return msg;
    }

    geometry_msgs::msg::Quaternion ToMsgQuaternion(const Eigen::Quaterniond& q)
    {
        geometry_msgs::msg::Quaternion msg;
        msg.x = q.x();
        msg.y = q.y();
        msg.z = q.z();
        msg.w = q.w();
        return msg;
    }

    moveit_msgs::msg::CollisionObject CreateBoxCollision(
        const std::string& name,
        const Eigen::Vector3d& dimension,
        const Eigen::Vector3d& location,
        const Eigen::Quaterniond& rot,
        const std::string& ns)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = ns + "/world";
        collision_object.id = name;
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.push_back(dimension.x());
        primitive.dimensions.push_back(dimension.y());
        primitive.dimensions.push_back(dimension.z());
        collision_object.primitives.push_back(primitive);
        geometry_msgs::msg::Pose box_pose;
        box_pose.position = ToMsgPoint(location);
        box_pose.orientation = ToMsgQuaternion(rot);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        return collision_object;
    }

    geometry_msgs::msg::Pose
    GetBoxTargetPose(const Eigen::Vector3f& adress, const geometry_msgs::msg::Pose& palletPose,const Eigen::Vector3d& boxDimension, float separation)
    {
        geometry_msgs::msg::Pose pose;
        pose.position = palletPose.position;

        // take into account the pallet is symmetrical
        const Eigen::Quaterniond idealPalletOrientation {0.7071067690849304, 0.0, 0.0, 0.7071067690849304};
        const Eigen::Quaterniond palletOrientation1 = FromMsgQuaternion(palletPose.orientation);
        const Eigen::Quaterniond palletOrientation2 = palletOrientation1 * Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

        const Eigen::Quaterniond palletOrientation = GetClosestQuaternionFromList(idealPalletOrientation, {palletOrientation1, palletOrientation2});

        Eigen::Vector3d localPalletDirX = palletOrientation * Eigen::Vector3d::UnitX();
        Eigen::Vector3d localPalletDirY = palletOrientation * Eigen::Vector3d::UnitY();
        Eigen::Vector3d localPalletDirZ = palletOrientation * Eigen::Vector3d::UnitZ();

        Eigen::Vector3d p = FromMsgPosition(palletPose.position);
        Eigen::Quaterniond q = FromMsgQuaternion(palletPose.orientation);

        // change local coordinate system for UR10 gipper targe pose in the way to have X+ pointing sky
        // Used blender to find this matrix.
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        rotation << 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;
        p += adress.x() * localPalletDirX * separation * boxDimension.x() + adress.y() * localPalletDirY * separation * boxDimension.y() +
            adress.z() * localPalletDirZ * separation * boxDimension.z();

        pose.position = ToMsgPoint(p);
        pose.orientation = toMsg(Eigen::Quaterniond(q.toRotationMatrix() * rotation));
        return pose;
    }

    Eigen::Quaterniond GetClosestQuaternionFromList(const Eigen::Quaterniond& q1, const std::vector<Eigen::Quaterniond>& qList)
    {
        float smallestAngle = std::numeric_limits<float>::max();
        Eigen::Quaterniond closestQ = qList.front();
        for (const auto& q : qList)
        {
            float angle = std::abs(q1.angularDistance(q));
            if (angle < smallestAngle)
            {
                smallestAngle = angle;
                closestQ = q;
            }
        }
        return closestQ;
    }
} // namespace Utils
