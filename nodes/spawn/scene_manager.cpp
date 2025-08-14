class SceneManager {
public:
    void addCube(const geometry_msgs::Pose& pose, const std::vector<double>& size);
    void attachObject(const std::string& object_id, const std::string& link_name);
    void removeObject(const std::string& object_id);
};