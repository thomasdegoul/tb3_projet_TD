#include <actionlib/server/simple_action_server.h>
#include <tb3_projet_TD/MoveAction.h>
#include <ros/ros.h>

using namespace tb3_projet_TD;
using namespace actionlib;
using namespace ros;

typedef SimpleActionServer<MoveAction> Server;

void execute(const MoveGoalConstPtr &goal, Server *as) {

  MoveResult res;
  MoveFeedback fb;

  ROS_INFO("goal = target_x: %f, target_y: %f ! ", goal->target_x, goal->target_y);

  int count = 0;
  ros::Rate loop_rate(1); // 1 hz

  bool success = true;
  while (count < goal->target_x) {

    count++;
    fb.pos_x = count;
    fb.pos_y = 0.0;
    fb.vitesse_lineaire = 0.1;
    fb.vitesse_angulaire = 0.0;
    fb.distance_restante = goal->target_x - count;

    ROS_INFO("POS intermediaire x = %f", fb.pos_x);
    as->publishFeedback(fb);

    loop_rate.sleep();
  }

  if (success) {
    res.distance_totale = count;
    res.temps_ecoule = count; // 1 seconde par boucle
    ROS_INFO("objectif atteint !");
    as->setSucceeded(res);
  }

  // A completer (voir slide serveur d'action 2/2)
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "serveur_action");
  ros::NodeHandle nh;

  Server server(nh, "moving", boost::bind(&execute, _1, &server), false);

  server.start();

  ROS_INFO("serveur demarre et en attente d'un goal ! ");

  spin();

  return 0;
}