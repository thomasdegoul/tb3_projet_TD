#include "ros/init.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose2D.h>
#include <tb3_projet_TD/MoveAction.h>
#include <nav_msgs/Path.h>

using namespace actionlib;
using namespace ros;
using namespace std;
using namespace geometry_msgs;
using namespace tb3_projet_TD;
using namespace nav_msgs;

typedef SimpleActionClient<MoveAction> Client;

Pose2D position;
MoveGoal goal;

/**
###  Vecteur pour y stocker des positions
###  Operations possibles :
    trajectoire.size() : retourne la taille du vecteur
    trajectoire.push_back(position) : insere une positon a la fin du vecteur
    trajectoire.erase(trajectoire.begin()) : supprime le premier element du
                                             vecteur
    trajectoire[0] : permet l'acces au premier element du vecteur (position
                     de type Pose2D)
**/
vector<Pose2D> trajectoire;

void spinThread() { ros::spin(); }

/*
This method is called when the goal completes
*/
void doneCb(const SimpleClientGoalState &s, const MoveResultConstPtr &result) {
  if (s == SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("action se termine avec success ! distance = %f, temps = %f", 
             result->distance_totale, result->temps_ecoule);
  } else
    ROS_INFO("Action is preempted");
}

/*
 This method is called when the goal becomes active
*/
void activeCb() { ROS_INFO("Goal just went active"); }

/*
This method is called whenever there is data present on feedback channel
*/
void feedback_callback(const MoveFeedbackConstPtr &feedback) {
  ROS_INFO("Feedback recu : pos_x = %f, distance restante = %f !", 
           feedback->pos_x, feedback->distance_restante);
}
void trajectoireCallback(const Path &msg) {
  for (size_t i = 0; i < msg.poses.size(); i++) {
    position.x = msg.poses[i].pose.position.x;
    position.y = msg.poses[i].pose.position.y;
    trajectoire.push_back(position);
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  boost::thread spin_thread(&spinThread);

  // A Completer...

  Client client("moving", true); 
  client.waitForServer();
  Subscriber sub = nh.subscribe("/consigne_trajectoire", 10, trajectoireCallback);

  ros::Rate loop_rate(10);
while (ros::ok()) {
  if (trajectoire.size() > 0) {
    goal.target_x = trajectoire[0].x;
    goal.target_y = trajectoire[0].y;
    client.sendGoal(goal, &doneCb, &activeCb, &feedback_callback);
    client.waitForResult();   

    if (client.getState() == SimpleClientGoalState::SUCCEEDED)
      trajectoire.erase(trajectoire.begin());   
    else
      trajectoire.clear();   
  }
  loop_rate.sleep();
}

  ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
  // ... voir slide "les actions"

  spin_thread.join();
  return 0;
}