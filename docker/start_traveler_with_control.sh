sudo docker start traveler_container
sudo docker exec -it traveler_container bash -c "source /opt/ros/humble/setup.bash && source /home/traveler/dependencies/install/setup.bash && export ROS_DOMAIN_ID=19 && python3 /home/traveler/highlevel/rasp_main.py"
