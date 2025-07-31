#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

// Crée une couleur RGBA
std_msgs::ColorRGBA makeColor(float r, float g, float b, float a = 1.0)
{
    std_msgs::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

// Crée un marker représentant une face colorée d’un cube
visualization_msgs::Marker makeFaceMarker(int id, const std::string& frame_id,
                                           float px, float py, float pz,
                                           float sx, float sy, float sz,
                                           const std::string& face,
                                           const std_msgs::ColorRGBA& color)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "colored_cube";
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;

    float thickness = 0.001; // Épaisseur de chaque face

    if (face == "front") {
        m.pose.position.x = px + sx/2;
        m.pose.position.y = py;
        m.pose.position.z = pz;
        m.scale.x = thickness; m.scale.y = sy; m.scale.z = sz;
    } else if (face == "back") {
        m.pose.position.x = px - sx/2;
        m.pose.position.y = py;
        m.pose.position.z = pz;
        m.scale.x = thickness; m.scale.y = sy; m.scale.z = sz;
    } else if (face == "left") {
        m.pose.position.x = px;
        m.pose.position.y = py + sy/2;
        m.pose.position.z = pz;
        m.scale.x = sx; m.scale.y = thickness; m.scale.z = sz;
    } else if (face == "right") {
        m.pose.position.x = px;
        m.pose.position.y = py - sy/2;
        m.pose.position.z = pz;
        m.scale.x = sx; m.scale.y = thickness; m.scale.z = sz;
    } else if (face == "top") {
        m.pose.position.x = px;
        m.pose.position.y = py;
        m.pose.position.z = pz + sz/2;
        m.scale.x = sx; m.scale.y = sy; m.scale.z = thickness;
    } else if (face == "bottom") {
        m.pose.position.x = px;
        m.pose.position.y = py;
        m.pose.position.z = pz - sz/2;
        m.scale.x = sx; m.scale.y = sy; m.scale.z = thickness;
    }

    m.color = color;
    m.lifetime = ros::Duration(0); // Persiste dans RViz
    return m;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_colors");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate rate(10);

    // Paramètres du cube
    float px = 0.5, py = 0.0, pz = 0.078; // position centrée au-dessus de la table
    float sx = 0.05, sy = 0.05, sz = 0.05;  // taille du cube

    std::vector<visualization_msgs::Marker> markers;

    // Création des 6 faces du cube
    markers.push_back(makeFaceMarker(0, "panda_link0", px, py, pz, sx, sy, sz, "front",  makeColor(1, 0, 0)));  // Rouge
    markers.push_back(makeFaceMarker(1, "panda_link0", px, py, pz, sx, sy, sz, "back",   makeColor(0, 1, 0)));  // Vert
    markers.push_back(makeFaceMarker(2, "panda_link0", px, py, pz, sx, sy, sz, "left",   makeColor(0, 0, 1)));  // Bleu
    markers.push_back(makeFaceMarker(3, "panda_link0", px, py, pz, sx, sy, sz, "right",  makeColor(1, 1, 0)));  // Jaune
    markers.push_back(makeFaceMarker(4, "panda_link0", px, py, pz, sx, sy, sz, "top",    makeColor(0, 1, 1)));  // Cyan
    markers.push_back(makeFaceMarker(5, "panda_link0", px, py, pz, sx, sy, sz, "bottom", makeColor(1, 0, 1)));  // Magenta

    // Création de la table
    visualization_msgs::Marker table_marker;
    table_marker.header.frame_id = "panda_link0";
    table_marker.header.stamp = ros::Time::now();
    table_marker.ns = "colored_cube";
    table_marker.id = 100;
    table_marker.type = visualization_msgs::Marker::CUBE;
    table_marker.action = visualization_msgs::Marker::ADD;
    table_marker.pose.position.x = 0.5;
    table_marker.pose.position.y = 0.0;
    table_marker.pose.position.z = 0.025;  // hauteur = moitié de la table
    table_marker.pose.orientation.w = 1.0;
    table_marker.scale.x = 0.1;
    table_marker.scale.y = 0.6;
    table_marker.scale.z = 0.05;
    table_marker.color = makeColor(0.4, 0.25, 0.1);  // Marron
    table_marker.lifetime = ros::Duration(0);

    ROS_INFO("Publishing colored cube markers and table...");

    
    // Stockage structuré des infos dans ROS paramètre serveur
    nh.setParam("/grasp/cube/position/x", px);
    nh.setParam("/grasp/cube/position/y", py);
    nh.setParam("/grasp/cube/position/z", pz);

    nh.setParam("/grasp/cube/size/x", sx);
    nh.setParam("/grasp/cube/size/y", sy);
    nh.setParam("/grasp/cube/size/z", sz);

    // Couleurs des faces
    nh.setParam("/grasp/cube/face_colors/front",   std::vector<double>{1.0, 0.0, 0.0});
    nh.setParam("/grasp/cube/face_colors/back",    std::vector<double>{0.0, 1.0, 0.0});
    nh.setParam("/grasp/cube/face_colors/left",    std::vector<double>{0.0, 0.0, 1.0});
    nh.setParam("/grasp/cube/face_colors/right",   std::vector<double>{1.0, 1.0, 0.0});
    nh.setParam("/grasp/cube/face_colors/top",     std::vector<double>{0.0, 1.0, 1.0});
    nh.setParam("/grasp/cube/face_colors/bottom",  std::vector<double>{1.0, 0.0, 1.0});


    while (ros::ok())
    {
        // Met à jour et publie les faces du cube
        for (auto& m : markers) {
            m.header.stamp = ros::Time::now();
            marker_pub.publish(m);
        }

        // Met à jour et publie la table
        table_marker.header.stamp = ros::Time::now();
        marker_pub.publish(table_marker);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
