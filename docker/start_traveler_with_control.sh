sudo docker start traveler_container
sudo docker exec -it traveler_container bash -c "source /opt/ros/humble/setup.bash && source /home/traveler/dependencies/install/setup.bash && export ROS_DOMAIN_ID=19 && ros2 run traveler_high_controller traveler_high_controller"
